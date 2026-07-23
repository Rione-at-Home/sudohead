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

from .filters import EMAFilter, KalmanFilter1D, OneEuroFilter, PassThroughFilter
from .HeadDriver import DynamixelDriver


class HeadNode(Node):

    def __init__(self):
        super().__init__("head_node")

        # --- Benchmarking & Param Config ---
        self.declare_parameter("controller_name", "")
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("filter_type", "ONEEURO")  # EMA, KALMAN, ONEEURO, or NONE
        self.declare_parameter("alpha", 0.05)             # EMA Parameter
        self.declare_parameter("process_noise", 0.05)     # Kalman Q
        self.declare_parameter("measurement_noise", 2.0)  # Kalman R
        self.declare_parameter("min_cutoff", 1.0)         # 1€ fc_min
        self.declare_parameter("beta", 0.05)              # 1€ beta
        self.declare_parameter("d_cutoff", 1.0)           # 1€ d_cutoff
        self.declare_parameter("motion_profile", "quintic")
        self.declare_parameter("motion_time", 2.5)

        custom_controller_name = self.get_parameter("controller_name").get_parameter_value().string_value
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.filter_type = self.get_parameter("filter_type").get_parameter_value().string_value.upper()
        self.alpha = self.get_parameter("alpha").get_parameter_value().double_value
        self.process_noise = self.get_parameter("process_noise").get_parameter_value().double_value
        self.measurement_noise = self.get_parameter("measurement_noise").get_parameter_value().double_value
        self.min_cutoff = self.get_parameter("min_cutoff").get_parameter_value().double_value
        self.beta = self.get_parameter("beta").get_parameter_value().double_value
        self.d_cutoff = self.get_parameter("d_cutoff").get_parameter_value().double_value
        self.motion_profile = self.get_parameter("motion_profile").get_parameter_value().string_value
        self.motion_time = self.get_parameter("motion_time").get_parameter_value().double_value

        self.control_period = 0.05  # 20 Hz

        # --- Filter Instantiation & Folder Naming ---
        if self.filter_type == "ONEEURO":
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
            fc_str = str(self.min_cutoff).replace(".", "")
            beta_str = str(self.beta).replace(".", "")
            d_str = str(self.d_cutoff).replace(".", "")
            auto_name = f"oneeuro_fc{fc_str}_b{beta_str}_d{d_str}"

        elif self.filter_type == "KALMAN":
            self.pan_filter = KalmanFilter1D(
                process_noise=self.process_noise,
                measurement_noise=self.measurement_noise,
            )
            self.tilt_filter = KalmanFilter1D(
                process_noise=self.process_noise,
                measurement_noise=self.measurement_noise,
            )
            q_str = str(self.process_noise).replace(".", "")
            r_str = str(self.measurement_noise).replace(".", "")
            auto_name = f"kalman_q{q_str}_r{r_str}"

        elif self.filter_type == "EMA":
            self.pan_filter = EMAFilter(alpha=self.alpha)
            self.tilt_filter = EMAFilter(alpha=self.alpha)
            alpha_str = str(self.alpha).replace(".", "")
            auto_name = f"ema_alpha{alpha_str}"

        else:
            self.pan_filter = PassThroughFilter()
            self.tilt_filter = PassThroughFilter()
            auto_name = "passthrough"

        # Directory path determination
        self.controller_name = custom_controller_name if custom_controller_name else auto_name
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
        self.get_logger().info(f"Head node ready ({self.filter_type} filter). Writing logs to: {self.target_dir}/")

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
        self.csv_writer.writerow([f"# filter_type={self.filter_type}"])
        if self.filter_type == "EMA":
            self.csv_writer.writerow([f"# alpha={self.alpha}"])
        elif self.filter_type == "KALMAN":
            self.csv_writer.writerow([f"# process_noise_Q={self.process_noise}"])
            self.csv_writer.writerow([f"# measurement_noise_R={self.measurement_noise}"])
        elif self.filter_type == "ONEEURO":
            self.csv_writer.writerow([f"# min_cutoff={self.min_cutoff}"])
            self.csv_writer.writerow([f"# beta={self.beta}"])
            self.csv_writer.writerow([f"# d_cutoff={self.d_cutoff}"])

        self.csv_writer.writerow([f"# motion_profile={self.motion_profile}"])
        self.csv_writer.writerow([f"# motion_time={self.motion_time}"])

        # Data Column Headers
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

            if rclpy.ok():
                self.get_logger().info(f"Saved and closed log for mode: '{self.current_mode}'")

    def update_target(self):
        """Applies filter to raw targets and updates quintic trajectory goals."""
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
        self.pan_elapsed += self.control_period
        pan_s = min(max(self.pan_elapsed / self.motion_time, 0.0), 1.0)
        pan_blend = 6 * (pan_s ** 5) - 15 * (pan_s ** 4) + 10 * (pan_s ** 3)
        self.current_pan = self.pan_start + pan_blend * (self.pan_goal - self.pan_start)

        self.tilt_elapsed += self.control_period
        tilt_s = min(max(self.tilt_elapsed / self.motion_time, 0.0), 1.0)
        tilt_blend = 6 * (tilt_s ** 5) - 15 * (tilt_s ** 4) + 10 * (tilt_s ** 3)
        self.current_tilt = self.tilt_start + tilt_blend * (self.tilt_goal - self.tilt_start)

        # Hardware Command
        self.driver.set_pan(self.current_pan)
        self.driver.set_tilt(self.current_tilt)

        # Hardware Feedback
        measured_pan = self.driver.get_pan()
        measured_tilt = self.driver.get_tilt()

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