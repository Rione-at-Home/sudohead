#
## head_target_generator.py
#
#  ROS2 test node for evaluating pan-tilt motion controllers.
#  Publishes deterministic and stochastic target trajectories on
#  /head/pan_target and /head/tilt_target.
#

import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class HeadTargetGenerator(Node):

    def __init__(self):
        super().__init__("head_target_generator")

        self.declare_parameter("mode", "step")

        # Publishers
        self.pan_pub = self.create_publisher(
            Float32,
            "/head/pan_target",
            10
        )

        self.tilt_pub = self.create_publisher(
            Float32,
            "/head/tilt_target",
            10
        )

        # State and timing variables
        self.dt = 0.05  # 20 Hz
        self.time = 0.0

        self.pan = 0.0
        self.tilt = 0.0

        # Control Loop Timer
        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        current_mode = self.get_parameter("mode").get_parameter_value().string_value
        self.get_logger().info(f"Target generator started in mode: '{current_mode}'")

    # Timer Loop
    def timer_callback(self):
        self.time += self.dt

        # Check for mode changes dynamically from ROS params
        mode = self.get_parameter("mode").get_parameter_value().string_value.lower()

        if mode == "step":
            self.step_motion()

        elif mode == "ramp":
            self.ramp_motion()

        elif mode == "sine":
            self.sine_motion()

        elif mode == "random":
            self.random_motion()

        elif mode == "noise":
            self.noise_motion()

        else:
            self.get_logger().warn(f"Unknown mode '{mode}'. Holding at 0°.", once=True)
            self.pan = 0.0
            self.tilt = 0.0

        self.publish_targets()

    # Motion Patterns
    def step_motion(self):
        """Tests overshoot and settling time across large jumps."""

        if self.time < 3.0:
            self.pan = 0.0

        elif self.time < 6.0:
            self.pan = 60.0

        elif self.time < 9.0:
            self.pan = -60.0

        else:
            self.time = 0.0
            self.pan = 0.0

        self.tilt = 0.0

    def ramp_motion(self):
        """Tests constant-velocity tracking and lag."""
        self.pan += 0.5

        if self.pan > 60.0:
            self.pan = -60.0

        self.tilt = 0.0

    def sine_motion(self):
        """Tests smooth continuous acceleration and direction reversal."""
        self.pan = 40.0 * math.sin(0.5 * self.time)
        self.tilt = 0.0

    def random_motion(self):
        """Simulates a wandering subject using a random walk."""
        self.pan += random.uniform(-2.0, 2.0)
        self.pan = max(-60.0, min(60.0, self.pan))
        self.tilt = 0.0

    def noise_motion(self):
        """Simulates visual detector jitter around a stationary subject."""
        self.pan = 20.0 + random.uniform(-2.0, 2.0)
        self.tilt = 0.0

    # Publisher Helper
    def publish_targets(self):
        pan_msg = Float32()
        pan_msg.data = float(self.pan)

        tilt_msg = Float32()
        tilt_msg.data = float(self.tilt)

        self.pan_pub.publish(pan_msg)
        self.tilt_pub.publish(tilt_msg)


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