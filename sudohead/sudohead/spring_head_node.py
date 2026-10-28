#
## spring_head_node.py
#
#  ROS2 node for Sudo's pan-tilt head for TRACKING (gaze controller). Based on spring_controller.py
#  (critically damped spring, same constants), with these changes (see CHANGES.md for the reasons):
#    [C5] publishes the MEASURED head angles (servo present positions) as sensor_msgs/JointState on
#         /joint_states (head_pan_joint, head_tilt_joint; radians, same signs as the degree topics:
#         pan + = left, tilt + = down).
#    [C6] zero calibration BEFORE torque-on, and torque-on holds the present position (driver C4).
#    [C7] the spring state is clamped to the servo limits (no wind-up beyond what the head can reach).
#    [C8] no log line every control tick (20/s); a throttled status line instead.
#    [C9] the hardware settings and spring constants are ROS parameters.
#    [C11] default stiffness k = 64 (c = 16, still critically damped) instead of 25: the head follows a
#          moving guest with about half the lag (closed-loop simulation in CHANGES.md). k=25 c=10 restores
#          the reference behaviour.
#  Topics in: /head/pan_target, /head/tilt_target (std_msgs/Float32, degrees) -- unchanged.
#

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from .HeadDriver import DynamixelDriver


class SpringHeadNode(Node):

    def __init__(self):
        super().__init__("head_node")

        # [C9] parameters
        self.declare_parameter("device_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("pan_id", 54)
        self.declare_parameter("tilt_id", 55)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("k", 64.0)                 # spring stiffness [C11] (reference: 25)
        self.declare_parameter("c", 16.0)                 # damping, 2*sqrt(k) = critically damped (reference: 10)
        self.declare_parameter("max_velocity", 60.0)      # deg/s
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("pan_joint_name", "head_pan_joint")
        self.declare_parameter("tilt_joint_name", "head_tilt_joint")
        self.declare_parameter("measure_every_n", 1)      # read the servos every n control ticks
        self.declare_parameter("log_period_s", 2.0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.control_period = 1.0 / max(1.0, float(g("control_rate_hz")))
        self.k = float(g("k"))
        self.c = float(g("c"))
        self.max_velocity = float(g("max_velocity"))
        self.publish_joint_states = bool(g("publish_joint_states"))
        self.joint_names = [str(g("pan_joint_name")), str(g("tilt_joint_name"))]
        self.measure_every_n = max(1, int(g("measure_every_n")))
        self.log_period = float(g("log_period_s"))

        self.driver = DynamixelDriver(
            device_name=str(g("device_name")),
            baudrate=int(g("baudrate")),
            pan_id=int(g("pan_id")),
            tilt_id=int(g("tilt_id")),
        )

        # [C6] calibrate first (zero = where the head really is), then torque-on holding that position
        self.driver.calibrate_zero()
        self.driver.enable()

        self.pan_limits = (self.driver.PAN_MIN, self.driver.PAN_MAX)
        self.tilt_limits = (self.driver.TILT_MIN, self.driver.TILT_MAX)

        # Current and target states (model of the head, degrees)
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.target_pan = 0.0
        self.target_tilt = 0.0
        self.pan_velocity = 0.0
        self.tilt_velocity = 0.0

        self.measured = None          # (pan, tilt) degrees, last successful read
        self.read_failures = 0
        self.tick = 0
        self.last_log = self.get_clock().now()

        self.create_subscription(Float32, "/head/pan_target", self.pan_callback, 10)
        self.create_subscription(Float32, "/head/tilt_target", self.tilt_callback, 10)
        self.joint_pub = self.create_publisher(JointState, str(g("joint_state_topic")), 10)
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            f"Head node started (critically damped spring, k={self.k}, c={self.c}, "
            f"max {self.max_velocity} deg/s); joint_states {'on' if self.publish_joint_states else 'off'}"
        )

    # Topic Callbacks
    def pan_callback(self, msg):
        self.target_pan = max(self.pan_limits[0], min(self.pan_limits[1], float(msg.data)))

    def tilt_callback(self, msg):
        self.target_tilt = max(self.tilt_limits[0], min(self.tilt_limits[1], float(msg.data)))

    # One spring axis
    def _spring(self, current, velocity, target, limits, dt):
        accel = self.k * (target - current) - self.c * velocity
        velocity += accel * dt
        velocity = max(-self.max_velocity, min(self.max_velocity, velocity))
        current += velocity * dt
        # [C7] clamp the model to what the servo can do
        if current < limits[0] or current > limits[1]:
            current = max(limits[0], min(limits[1], current))
            velocity = 0.0
        return current, velocity

    # Control Loop
    def control_loop(self):
        dt = self.control_period
        self.current_pan, self.pan_velocity = self._spring(
            self.current_pan, self.pan_velocity, self.target_pan, self.pan_limits, dt)
        self.current_tilt, self.tilt_velocity = self._spring(
            self.current_tilt, self.tilt_velocity, self.target_tilt, self.tilt_limits, dt)

        # Send commands to hardware
        self.driver.set_pan(self.current_pan)
        self.driver.set_tilt(self.current_tilt)

        # [C5] measured angles -> /joint_states
        self.tick += 1
        if self.publish_joint_states and self.tick % self.measure_every_n == 0:
            stamp = self.get_clock().now()          # time of the read
            pan, tilt = self.driver.get_pan(), self.driver.get_tilt()
            if pan is None or tilt is None:
                self.read_failures += 1             # skip this sample; never publish a wrong angle
            else:
                self.measured = (pan, tilt)
                js = JointState()
                js.header.stamp = stamp.to_msg()
                js.name = list(self.joint_names)
                js.position = [math.radians(pan), math.radians(tilt)]
                self.joint_pub.publish(js)

        # [C8] throttled status line
        now = self.get_clock().now()
        if (now - self.last_log).nanoseconds * 1e-9 >= self.log_period:
            self.last_log = now
            meas = "n/a" if self.measured is None else f"{self.measured[0]:.1f}/{self.measured[1]:.1f}"
            self.get_logger().info(
                f"target {self.target_pan:.1f}/{self.target_tilt:.1f}  "
                f"cmd {self.current_pan:.1f}/{self.current_tilt:.1f}  measured {meas}  "
                f"read failures {self.read_failures}"
            )

    # Turn off motors and close driver on shutdown
    def destroy_node(self):
        try:
            self.driver.disable()
            self.driver.close()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Error shutting down the Dynamixel driver: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpringHeadNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()