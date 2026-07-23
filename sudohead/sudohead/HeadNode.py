#
## head_node.py
#
#  ROS2 controller & benchmark logger node.
#  Logs raw targets, filtered targets, encoder feedback, and error metrics
#  into structured results directories with embedded CSV metadata headers.
#

import csv
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class HeadNode(Node):

    def __init__(self):
        super().__init__("head_node")

        # Configurable Parameters for Benchmarking
        self.declare_parameter("controller_name", "ema_alpha02")
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("filter_type", "EMA")
        self.declare_parameter("alpha", 0.2)
        self.declare_parameter("motion_profile", "quintic")
        self.declare_parameter("motion_time", 2.5)

        self.controller_name = self.get_parameter("controller_name").get_parameter_value().string_value
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.filter_type = self.get_parameter("filter_type").get_parameter_value().string_value
        self.alpha = self.get_parameter("alpha").get_parameter_value().double_value
        self.motion_profile = self.get_parameter("motion_profile").get_parameter_value().string_value
        self.motion_time = self.get_parameter("motion_time").get_parameter_value().double_value

        self.target_dir = os.path.join(self.output_dir, self.controller_name)
        os.makedirs(self.target_dir, exist_ok=True)

        # Subscriptions
        self.create_subscription(Float32, "/head/pan_target", self.pan_cb, 10)
        self.create_subscription(Float32, "/head/tilt_target", self.tilt_cb, 10)
        self.create_subscription(String, "/head/mode", self.mode_cb, 10)

        # State Variables
        self.raw_pan = 0.0
        self.raw_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0

        self.current_mode = ""
        self.csv_file = None
        self.csv_writer = None
        self.mode_start_time = None

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Head node ready. Writing logs to: {self.target_dir}/")

    def pan_cb(self, msg: Float32):
        self.raw_pan = msg.data

    def tilt_cb(self, msg: Float32):
        self.raw_tilt = msg.data

    def mode_cb(self, msg: String):
        new_mode = msg.data.lower()
        if new_mode != self.current_mode:
            self.close_current_log()
            self.current_mode = new_mode

            if self.current_mode not in ["idle", ""]:
                self.start_new_log(self.current_mode)

    def start_new_log(self, mode_name: str):
        """Creates a new CSV file with metadata header rows."""
        file_path = os.path.join(self.target_dir, f"{mode_name}.csv")
        self.csv_file = open(file_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        # Metadata Header (Comment lines)
        self.csv_writer.writerow([f"# controller={self.controller_name}"])
        self.csv_writer.writerow([f"# filter={self.filter_type}"])
        self.csv_writer.writerow([f"# alpha={self.alpha}"])
        self.csv_writer.writerow([f"# motion_profile={self.motion_profile}"])
        self.csv_writer.writerow([f"# motion_time={self.motion_time}"])

        # Data Column Headers
        self.csv_writer.writerow([
            "time_s",
            "raw_pan", "filtered_pan", "measured_pan", "pan_error",
            "raw_tilt", "filtered_tilt", "measured_tilt", "tilt_error"
        ])

        self.mode_start_time = time.time()
        self.get_logger().info(f"Started logging mode '{mode_name}' -> {file_path}")

    def close_current_log(self):
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.get_logger().info(f"Saved and closed log for mode: '{self.current_mode}'")

    def update_filter(self):
        """Simple Exponential Moving Average (EMA) filter step."""
        self.filtered_pan += self.alpha * (self.raw_pan - self.filtered_pan)
        self.filtered_tilt += self.alpha * (self.raw_tilt - self.filtered_tilt)

    def control_loop(self):
        if not self.csv_file or self.mode_start_time is None:
            return

        self.update_filter()

        measured_pan = self.filtered_pan  # Replace with self.driver.get_pan()
        measured_tilt = self.filtered_tilt  # Replace with self.driver.get_tilt()

        pan_error = self.raw_pan - measured_pan # tracking error
        tilt_error = self.raw_tilt - measured_tilt

        t_elapsed = time.time() - self.mode_start_time
        self.csv_writer.writerow([
            f"{t_elapsed:.4f}",
            f"{self.raw_pan:.3f}", f"{self.filtered_pan:.3f}", f"{measured_pan:.3f}", f"{pan_error:.3f}",
            f"{self.raw_tilt:.3f}", f"{self.filtered_tilt:.3f}", f"{measured_tilt:.3f}", f"{tilt_error:.3f}"
        ])

    def destroy_node(self):
        self.close_current_log()
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