#
## head_node.py
#
#  ROS2 node for controlling the pan-tilt mechanism on Sudo. This node
#  subscribes to target pan and tilt angles from a tracking node and smoothly 
#  moves the head towards those targets using the DynamixelDriver. This node
#  acts as the interface between ROS2 and the Dynamixel hardware.
#

import csv
import time

from .HeadDriver import DynamixelDriver
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class HeadNode(Node):

    def __init__(self):
        super().__init__('head_node')

        self.driver = DynamixelDriver()
        self.driver.enable()
        self.driver.calibrate_zero()

        # States
        self.raw_pan = 0.0
        self.raw_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.pan_goal = 0.0
        self.tilt_goal = 0.0

        self.control_period = 0.05  
        self.max_velocity = 60.0  # degrees per second

        self.pan_start = 0.0
        self.pan_elapsed = 0.0
        self.tilt_start = 0.0
        self.tilt_elapsed = 0.0
        self.motion_time = 2.5   # seconds

        # Subscriptions
        self.create_subscription(
            Float32,
            '/head/pan_target',
            self.pan_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/head/tilt_target',
            self.tilt_callback,
            10
        )

        self.create_subscription(
            String,
            '/head/mode',
            self.mode_callback,
            10
        )

        # Control Loop Timer
        self.timer = self.create_timer(
            self.control_period,
            self.control_loop
        )

        # --- Logging State ---
        self.is_logging = False
        self.active_mode = None
        self.log_file = None
        self.csv_writer = None
        self.start_time = None

        self.get_logger().info("Head node initialized. Waiting for /head/mode topic to begin logging...")

    def mode_callback(self, msg):
        new_mode = msg.data.lower().strip()

        # Switch log files if mode changes
        if new_mode != self.active_mode:
            self.switch_log_file(new_mode)

    def switch_log_file(self, mode_name: str):
        """Closes old CSV and initializes a new mode-specific log file."""
        if self.log_file and not self.log_file.closed:
            self.log_file.close()

        self.active_mode = mode_name
        self.start_time = time.time()
        
        filename = f"head_log_{mode_name}.csv"
        self.log_file = open(filename, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            "time",
            "raw_pan",
            "filtered_pan",
            "goal_pan",
            "current_pan",
            "measured_pan",
            "raw_tilt",
            "filtered_tilt",
            "goal_tilt",
            "current_tilt",
            "measured_tilt"
        ])

        self.is_logging = True
        self.get_logger().info(f"Switched mode to '{mode_name}'. Logging to '{filename}'")

    def pan_callback(self, msg):
        self.raw_pan = msg.data

    def tilt_callback(self, msg):
        self.raw_tilt = msg.data

    def update_target(self):
        alpha = 0.2

        self.filtered_pan += alpha * (self.raw_pan - self.filtered_pan)
        self.filtered_tilt += alpha * (self.raw_tilt - self.filtered_tilt)

        if abs(self.filtered_pan - self.pan_goal) > 0.5:
            self.pan_start = self.current_pan
            self.pan_goal = self.filtered_pan
            self.pan_elapsed = 0.0

        if abs(self.filtered_tilt - self.tilt_goal) > 0.5:
            self.tilt_start = self.current_tilt
            self.tilt_goal = self.filtered_tilt
            self.tilt_elapsed = 0.0

    def control_loop(self):
        # Update targets and trajectories
        self.update_target()
        
        self.pan_elapsed += self.control_period
        pan_s = min(max(self.pan_elapsed / self.motion_time, 0.0), 1.0)
        pan_blend = 6*pan_s**5 - 15*pan_s**4 + 10*pan_s**3
        self.current_pan = self.pan_start + pan_blend * (self.pan_goal - self.pan_start)

        self.tilt_elapsed += self.control_period
        tilt_s = min(max(self.tilt_elapsed / self.motion_time, 0.0), 1.0)
        tilt_blend = 6*tilt_s**5 - 15*tilt_s**4 + 10*tilt_s**3
        self.current_tilt = self.tilt_start + tilt_blend * (self.tilt_goal - self.tilt_start)
                
        # Send positions to hardware
        self.driver.set_pan(self.current_pan)
        self.driver.set_tilt(self.current_tilt)

        # Read actual positions
        measured_pan = self.driver.get_pan()
        measured_tilt = self.driver.get_tilt()

        # Write to log ONLY if active mode is set
        if self.is_logging:
            t = time.time() - self.start_time

            self.csv_writer.writerow([
                t,
                self.raw_pan,
                self.filtered_pan,
                self.pan_goal,
                self.current_pan,
                measured_pan,
                self.raw_tilt,
                self.filtered_tilt,
                self.tilt_goal,
                self.current_tilt,
                measured_tilt
            ])
            self.log_file.flush()

    def destroy_node(self):
        if self.log_file and not self.log_file.closed:
            self.log_file.close()

        self.driver.disable()
        self.driver.close()

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


if __name__ == '__main__':
    main()