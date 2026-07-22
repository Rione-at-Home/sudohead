#
## head_target_generator.py
#
#  ROS2 test node for evaluating pan-tilt motion controllers.
#  Publishes target trajectories with a 20-second timer per mode.
#

import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class HeadTargetGenerator(Node):

    def __init__(self):
        super().__init__("head_target_generator")

        # Parameters: "sequence" runs all modes sequentially for 20s each.
        # Specific mode (e.g., "sine") runs just that mode for 20s.
        self.declare_parameter("mode", "sequence")
        self.declare_parameter("duration", 20.0)  # seconds per test

        # Publishers
        self.pan_pub = self.create_publisher(Float32, "/head/pan_target", 10)
        self.tilt_pub = self.create_publisher(Float32, "/head/tilt_target", 10)
        self.mode_pub = self.create_publisher(String, "/head/mode", 10)

        # Timing and state
        self.dt = 0.05  # 20 Hz
        self.test_duration = self.get_parameter("duration").get_parameter_value().double_value

        self.sequence_modes = ["step", "ramp", "sine", "random", "noise"]
        self.sequence_index = 0

        self.active_mode = ""
        self.mode_elapsed = 0.0
        self.pan = 0.0
        self.tilt = 0.0

        # Control Loop Timer
        self.timer = self.create_timer(self.dt, self.timer_callback)

        initial_param = self.get_parameter("mode").get_parameter_value().string_value.lower()
        if initial_param == "sequence":
            self.active_mode = self.sequence_modes[0]
        else:
            self.active_mode = initial_param

        self.get_logger().info(
            f"Target generator initialized. Active mode: '{self.active_mode}' "
            f"({self.test_duration}s duration per test)"
        )

    def timer_callback(self):
        self.mode_elapsed += self.dt

        # Check if the 20-second timer expired for the active mode
        if self.mode_elapsed >= self.test_duration:
            param_mode = self.get_parameter("mode").get_parameter_value().string_value.lower()

            if param_mode == "sequence":
                self.sequence_index += 1
                if self.sequence_index < len(self.sequence_modes):
                    self.active_mode = self.sequence_modes[self.sequence_index]
                    self.mode_elapsed = 0.0
                    self.get_logger().info(f"Advancing sequence to mode: '{self.active_mode}'")
                else:
                    self.active_mode = "idle"
                    self.get_logger().info("Sequence complete! Holding at 0°.")
            else:
                self.active_mode = "idle"
                self.get_logger().info(f"Test duration ({self.test_duration}s) reached. Holding at 0°.")

        # Execute current motion pattern
        if self.active_mode == "step":
            self.step_motion()
        elif self.active_mode == "ramp":
            self.ramp_motion()
        elif self.active_mode == "sine":
            self.sine_motion()
        elif self.active_mode == "random":
            self.random_motion()
        elif self.active_mode == "noise":
            self.noise_motion()
        else:
            # Idle state when done
            self.pan = 0.0
            self.tilt = 0.0

        self.publish_targets()

    # --- Motion Patterns (resetting mathematically to t=0 per mode) ---

    def step_motion(self):
        """Step jumps every 5 seconds within the 20s test window."""
        t = self.mode_elapsed
        if t < 5.0:
            self.pan = 0.0
        elif t < 10.0:
            self.pan = 60.0
        elif t < 15.0:
            self.pan = -60.0
        else:
            self.pan = 0.0
        self.tilt = 0.0

    def ramp_motion(self):
        """Constant slope ramp over time."""
        self.pan = -60.0 + (60.0 * (self.mode_elapsed / self.test_duration))
        self.tilt = 0.0

    def sine_motion(self):
        """Smooth sine wave starting from t=0."""
        self.pan = 40.0 * math.sin(0.5 * self.mode_elapsed)
        self.tilt = 0.0

    def random_motion(self):
        """Random walk bounded between -60° and 60°."""
        self.pan += random.uniform(-2.0, 2.0)
        self.pan = max(-60.0, min(60.0, self.pan))
        self.tilt = 0.0

    def noise_motion(self):
        """Jitter noise centered around 20°."""
        self.pan = 20.0 + random.uniform(-2.0, 2.0)
        self.tilt = 0.0

    def publish_targets(self):
        pan_msg = Float32(data=float(self.pan))
        tilt_msg = Float32(data=float(self.tilt))
        mode_msg = String(data=self.active_mode)

        self.pan_pub.publish(pan_msg)
        self.tilt_pub.publish(tilt_msg)
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadTargetGenerator()

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