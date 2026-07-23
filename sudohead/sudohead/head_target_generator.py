#
## head_target_generator.py
#
#  ROS2 test node for evaluating pan-tilt motion controllers.
#  Publishes target trajectories with clean per-mode resets.
#

import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class HeadTargetGenerator(Node):

    def __init__(self):
        super().__init__("head_target_generator")

        # Parameters
        self.declare_parameter("mode", "sequence")
        self.declare_parameter("duration", 20.0)  # Seconds per test
        self.declare_parameter("rate", 20.0)      # Loop rate in Hz

        # Publishers
        self.pan_pub = self.create_publisher(Float32, "/head/pan_target", 10)
        self.tilt_pub = self.create_publisher(Float32, "/head/tilt_target", 10)
        self.mode_pub = self.create_publisher(String, "/head/mode", 10)

        # Timing and state setup
        rate_hz = self.get_parameter("rate").get_parameter_value().double_value
        self.dt = 1.0 / rate_hz if rate_hz > 0 else 0.05

        self.test_duration = self.get_parameter("duration").get_parameter_value().double_value
        self.sequence_modes = ["step", "ramp", "sine", "random", "noise"]
        self.sequence_index = 0

        self.active_mode = ""
        self.mode_elapsed = 0.0
        self.pan = 0.0
        self.tilt = 0.0

        # Initialize starting mode
        initial_param = self.get_parameter("mode").get_parameter_value().string_value.lower()
        if initial_param == "sequence":
            self.switch_to_mode(self.sequence_modes[0])
            
        else:
            self.switch_to_mode(initial_param)

        # Control Loop Timer
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f"Target generator ready. Active mode: '{self.active_mode}' "
            f"({self.test_duration}s per test @ {rate_hz} Hz)"
        )

    def switch_to_mode(self, new_mode: str):
        """Resets trajectory variables when switching modes."""
        self.active_mode = new_mode
        self.mode_elapsed = 0.0
        self.pan = 0.0
        self.tilt = 0.0
        self.get_logger().info(f"Transitioning to mode: '{self.active_mode}'")

    def timer_callback(self):
        self.mode_elapsed += self.dt

        # Check if the per-mode timer expired
        if self.mode_elapsed >= self.test_duration:
            param_mode = self.get_parameter("mode").get_parameter_value().string_value.lower()

            if param_mode == "sequence":
                self.sequence_index += 1

                if self.sequence_index < len(self.sequence_modes):
                    self.switch_to_mode(self.sequence_modes[self.sequence_index])

                else:
                    self.switch_to_mode("idle")
                    self.get_logger().info("Full benchmark sequence complete! Holding position at 0°.")

            else:

                self.switch_to_mode("idle")
                self.get_logger().info(f"Test duration ({self.test_duration}s) reached. Holding position at 0°.")

        # Motion generation
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
            self.pan = 0.0
            self.tilt = 0.0

        self.publish_targets()

    # --- Motion Generators ---

    def step_motion(self):
        """Step jumps every 5 seconds inside the 20s window."""
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
        """Linear ramp sweeping from -60° to +60° over test duration."""
        self.pan = -60.0 + (120.0 * (self.mode_elapsed / self.test_duration))
        self.tilt = 0.0

    def sine_motion(self):
        """Smooth sine wave starting cleanly from t=0."""
        self.pan = 40.0 * math.sin(0.5 * self.mode_elapsed)
        self.tilt = 0.0

    def random_motion(self):
        """Bounded random walk starting from 0°."""
        self.pan += random.uniform(-2.0, 2.0)
        self.pan = max(-60.0, min(60.0, self.pan))
        self.tilt = 0.0

    def noise_motion(self):
        """Jitter noise centered around 20°."""
        self.pan = 20.0 + random.uniform(-2.0, 2.0)
        self.tilt = 0.0

    def publish_targets(self):
        self.pan_pub.publish(Float32(data=float(self.pan)))
        self.tilt_pub.publish(Float32(data=float(self.tilt)))
        self.mode_pub.publish(String(data=self.active_mode))


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