#  ROS2 controller node for Sudo's pan-tilt head.
#  Combines Dynamixel hardware control (with quintic trajectory profiling)
#  and target filtering using the benchmark-validated One Euro Filter.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .filters import OneEuroFilter
from .HeadDriver import DynamixelDriver


class HeadNode(Node):

    def __init__(self):
        super().__init__("head_node")

        # Parameters for 1E filter and motion profiling
        self.declare_parameter("min_cutoff", 1.0)
        self.declare_parameter("beta", 0.05)
        self.declare_parameter("d_cutoff", 1.0)
        self.declare_parameter("motion_profile", "quintic")
        self.declare_parameter("motion_time", 2.5)

        self.min_cutoff = self.get_parameter("min_cutoff").get_parameter_value().double_value
        self.beta = self.get_parameter("beta").get_parameter_value().double_value
        self.d_cutoff = self.get_parameter("d_cutoff").get_parameter_value().double_value
        self.motion_profile = self.get_parameter("motion_profile").get_parameter_value().string_value
        self.motion_time = self.get_parameter("motion_time").get_parameter_value().double_value

        self.control_period = 0.05  # 20 Hz

        # Target Filtering (One Euro Filter)
        self.pan_filter = OneEuroFilter(
            dt=self.control_period,
            min_cutoff=self.min_cutoff,
            beta=self.beta,
            d_cutoff=self.d_cutoff,
        )

        self.tilt_filter = OneEuroFilter(
            dt=self.control_period,
            min_cutoff=self.min_cutoff,
            beta=self.beta,
            d_cutoff=self.d_cutoff,
        )

        self.driver = DynamixelDriver()
        self.driver.enable()
        self.driver.calibrate_zero()

        # --- Motion State Variables ---
        self.raw_pan = 0.0
        self.raw_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0

        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.pan_goal = 0.0
        self.tilt_goal = 0.0

        self.pan_start = 0.0
        self.pan_elapsed = 0.0
        self.tilt_start = 0.0
        self.tilt_elapsed = 0.0

        self.create_subscription(Float32, "/head/pan_target", self.pan_cb, 10)
        self.create_subscription(Float32, "/head/tilt_target", self.tilt_cb, 10)

        self.timer = self.create_timer(self.control_period, self.control_loop)
        self.get_logger().info(
            f"Head node active. Filter: One Euro (min_cutoff={self.min_cutoff}, beta={self.beta}, d_cutoff={self.d_cutoff})"
        )

    def pan_cb(self, msg: Float32):
        self.raw_pan = msg.data

    def tilt_cb(self, msg: Float32):
        self.raw_tilt = msg.data

    def update_target(self):
        """
        Applies One Euro filter to raw targets and updates quintic trajectory goals.
        """
        
        self.filtered_pan = self.pan_filter.update(self.raw_pan)
        self.filtered_tilt = self.tilt_filter.update(self.raw_tilt)

        if abs(self.filtered_pan - self.pan_goal) > 0.5:
            self.pan_start = self.current_pan
            self.pan_goal = self.filtered_pan
            self.pan_elapsed = 0.0

        if abs(self.filtered_tilt - self.tilt_goal) > 0.5:
            self.tilt_start = self.current_tilt
            self.tilt_goal = self.filtered_tilt
            self.tilt_elapsed = 0.0

    def control_loop(self):
        self.update_target()

        # Quintic polynomial trajectory generation

        # PAN
        self.pan_elapsed += self.control_period

        pan_s = min(max(self.pan_elapsed / self.motion_time, 0.0), 1.0)

        pan_blend = 6 * (pan_s ** 5) - 15 * (pan_s ** 4) + 10 * (pan_s ** 3)

        self.current_pan = self.pan_start + pan_blend * (self.pan_goal - self.pan_start)

        # TILT
        self.tilt_elapsed += self.control_period

        tilt_s = min(max(self.tilt_elapsed / self.motion_time, 0.0), 1.0)

        tilt_blend = 6 * (tilt_s ** 5) - 15 * (tilt_s ** 4) + 10 * (tilt_s ** 3)

        self.current_tilt = self.tilt_start + tilt_blend * (self.tilt_goal - self.tilt_start)

        # Hardware Command
        self.driver.set_pan(self.current_pan)
        self.driver.set_tilt(self.current_tilt)

    def destroy_node(self):
        try:
            self.driver.disable()
            self.driver.close()
            if rclpy.ok():
                self.get_logger().info("Dynamixel driver safely disabled and closed.")

        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Error shutting down Dynamixel driver: {e}")


        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadNode()

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