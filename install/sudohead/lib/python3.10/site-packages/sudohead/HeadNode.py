#
## head_node.py
#
#  ROS2 controller & benchmark logger node for Sudo's pan-tilt head.
#  Combines Dynamixel hardware control (with quintic trajectory profiling)
#  and per-mode CSV benchmark logging.
#

import csv
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .HeadDriver import DynamixelDriver


class HeadNode(Node):

    def __init__(self):
        super().__init__("head_node")

        # --- Benchmarking & Param Config ---
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

        # Create output directory: results/<controller_name>/
        self.target_dir = os.path.join(self.output_dir, self.controller_name)
        os.makedirs(self.target_dir, exist_ok=True)

        # --- Hardware Initialization ---
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

        self.control_period = 0.05  # 20 Hz

        # --- Logging State ---
        self.current_mode = ""
        self.csv_file = None
        self.csv_writer = None
        self.mode_start_time = None

        # --- Subscriptions ---
        self.create_subscription(Float32, "/head/pan_target", self.pan_cb, 10)
        self.create_subscription(Float32, "/head/tilt_target", self.tilt_cb, 10)
        self.create_subscription(String, "/head/mode", self.mode_cb, 10)

        # --- Control Loop Timer (20 Hz) ---
        self.timer = self.create_timer(self.control_period, self.control_loop)
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
        """Creates a new CSV file with metadata header rows for the active mode."""
        file_path = os.path.join(self.target_dir, f"{mode_name}.csv")
        self.csv_file = open(file_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        # Embedded Metadata Headers
        self.csv_writer.writerow([f"# controller={self.controller_name}"])
        self.csv_writer.writerow([f"# filter={self.filter_type}"])
        self.csv_writer.writerow([f"# alpha={self.alpha}"])
        self.csv_writer.writerow([f"# motion_profile={self.motion_profile}"])
        self.csv_writer.writerow([f"# motion_time={self.motion_time}"])

        # Data Column Headers (Full signal chain from target to encoder)
        self.csv_writer.writerow([
            "time_s",
            "raw_pan", "filtered_pan", "goal_pan", "current_pan", "measured_pan", "pan_error",
            "raw_tilt", "filtered_tilt", "goal_tilt", "current_tilt", "measured_tilt", "tilt_error"
        ])

        self.mode_start_time = time.time()
        self.get_logger().info(f"Started logging mode '{mode_name}' -> {file_path}")

    def close_current_log(self):
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.get_logger().info(f"Saved and closed log for mode: '{self.current_mode}'")

    def update_target(self):
        """Applies filter to raw targets and updates quintic trajectory goals."""
        # Filter input target
        self.filtered_pan += self.alpha * (self.raw_pan - self.filtered_pan)
        self.filtered_tilt += self.alpha * (self.raw_tilt - self.filtered_tilt)

        # Update trajectory trajectory goals if target changed significantly
        if abs(self.filtered_pan - self.pan_goal) > 0.5:
            self.pan_start = self.current_pan
            self.pan_goal = self.filtered_pan
            self.pan_elapsed = 0.0

        if abs(self.filtered_tilt - self.tilt_goal) > 0.5:
            self.tilt_start = self.current_tilt
            self.tilt_goal = self.filtered_tilt
            self.tilt_elapsed = 0.0

    def control_loop(self):
        # Always update filter and trajectory goals
        self.update_target()

        # Quintic polynomial trajectory generation for Pan
        self.pan_elapsed += self.control_period

        pan_s = min(max(self.pan_elapsed / self.motion_time, 0.0), 1.0)

        pan_blend = 6 * (pan_s ** 5) - 15 * (pan_s ** 4) + 10 * (pan_s ** 3)

        self.current_pan = self.pan_start + pan_blend * (self.pan_goal - self.pan_start)

        # Quintic polynomial trajectory generation for Tilt
        self.tilt_elapsed += self.control_period

        tilt_s = min(max(self.tilt_elapsed / self.motion_time, 0.0), 1.0)

        tilt_blend = 6 * (tilt_s ** 5) - 15 * (tilt_s ** 4) + 10 * (tilt_s ** 3)

        self.current_tilt = self.tilt_start + tilt_blend * (self.tilt_goal - self.tilt_start)

        # --- Hardware Command ---
        self.driver.set_pan(self.current_pan)
        self.driver.set_tilt(self.current_tilt)

        # --- Hardware Feedback ---
        measured_pan = self.driver.get_pan()
        measured_tilt = self.driver.get_tilt()

        # Only record CSV if active test mode is running
        if self.csv_file and self.mode_start_time is not None:
            pan_error = self.raw_pan - measured_pan
            tilt_error = self.raw_tilt - measured_tilt
            t_elapsed = time.time() - self.mode_start_time

            self.csv_writer.writerow([
                f"{t_elapsed:.4f}",
                f"{self.raw_pan:.3f}", f"{self.filtered_pan:.3f}", f"{self.pan_goal:.3f}",
                f"{self.current_pan:.3f}", f"{measured_pan:.3f}", f"{pan_error:.3f}",
                f"{self.raw_tilt:.3f}", f"{self.filtered_tilt:.3f}", f"{self.tilt_goal:.3f}",
                f"{self.current_tilt:.3f}", f"{measured_tilt:.3f}", f"{tilt_error:.3f}"
            ])
            self.csv_file.flush()

    def destroy_node(self):
        self.close_current_log()

        # Clean disable and disconnect of Dynamixel hardware
        try:
            self.driver.disable()
            self.driver.close()
            self.get_logger().info("Dynamixel driver safely disabled and closed.")
        except Exception as e:
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