
#
## head_node.py
#
#  ROS2 node for controlling the pan-tilt mechanism on Sudo. This node
#  subscribes to target pan and tilt angles from a tracking node and smoothly 
#  moves the head towards those targets using the DynamixelDriver. This node
#  acts as the interface between ROS2 and the Dynamixel hardware.
#

from .HeadDriver import DynamixelDriver
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32



class HeadNode(Node):

    def __init__(self):
        super().__init__('head_node')

        # Dynamixel Driver Initialization
        self.driver = DynamixelDriver()

        self.driver.enable()
        self.driver.calibrate_zero()

        # Raw Input States
        self.raw_pan = 0.0
        self.raw_tilt = 0.0

        # Current and Target States
        self.current_pan = 0.0
        self.current_tilt = 0.0

        self.target_pan = 0.0
        self.target_tilt = 0.0

        self.control_period = 0.05  
        self.max_velocity = 60.0  # degrees per second

        # Pan trajectory
        self.pan_start = 0.0
        self.pan_goal = 0.0
        self.pan_elapsed = 0.0

        # Tilt trajectory
        self.tilt_start = 0.0
        self.tilt_goal = 0.0
        self.tilt_elapsed = 0.0

        # Trajectory duration
        self.motion_time = 2.5   # seconds

        # Timer period
        self.control_period = 0.05

        # Filtered Output
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0

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

        # Control Loop Timer
        self.timer = self.create_timer(
            0.05,   # 20 Hz
            self.control_loop
        )

        self.get_logger().info(
            "Head node started"
        )

    # Topic Callbacks
    def pan_callback(self, msg):

        self.raw_pan = msg.data

    def tilt_callback(self, msg):

        self.raw_tilt = msg.data

    # Update Target method
    def update_target(self):

        alpha = 0.3

        self.filtered_pan += alpha * (
            self.raw_pan - self.filtered_pan
        )

        self.filtered_tilt += alpha * (
            self.raw_tilt - self.filtered_tilt
        )

        if abs(self.filtered_pan - self.pan_goal) > 0.5:
            self.pan_start = self.current_pan
            self.pan_goal = self.filtered_pan
            self.pan_elapsed = 0.0

        if abs(self.filtered_tilt - self.tilt_goal) > 0.5:
            self.tilt_start = self.current_tilt
            self.tilt_goal = self.filtered_tilt
            self.tilt_elapsed = 0.0

    # Main Control Loop
    def control_loop(self):

        self.get_logger().info(
            f"Raw Pan: {self.raw_pan:.1f}°, "
            f"Raw Tilt: {self.raw_tilt:.1f}°"
        )

        self.get_logger().info(
            f"Filtered Pan: {self.filtered_pan:.1f}°, "
            f"Filtered Tilt: {self.filtered_tilt:.1f}°"
        )

        self.get_logger().info(
            f"Current Pan: {self.current_pan:.1f}°, "
            f"Current Tilt: {self.current_tilt:.1f}°"
        )

        self.get_logger().info(
            f"Target Pan: {self.pan_goal:.1f}°, "
            f"Target Tilt: {self.tilt_goal:.1f}°"
        )

        self.update_target()
        
        self.pan_elapsed += self.control_period

        pan_s = self.pan_elapsed / self.motion_time
        pan_s = min(max(pan_s, 0.0), 1.0)

        pan_blend = (
            6*pan_s**5
            - 15*pan_s**4
            + 10*pan_s**3
        )

        self.current_pan = (
            self.pan_start
            + pan_blend *
            (
                self.pan_goal
                - self.pan_start
            )
        )

        self.tilt_elapsed += self.control_period

        tilt_s = (self.tilt_elapsed / self.motion_time)

        tilt_s = min(max(tilt_s, 0.0), 1.0)

        tilt_blend = (
            6*tilt_s**5
            - 15*tilt_s**4
            + 10*tilt_s**3
        )

        self.current_tilt = (
            self.tilt_start +
            tilt_blend *
            (
                self.tilt_goal -
                self.tilt_start
            )
        )
                
        self.driver.set_pan(
            self.current_pan
        )

        self.driver.set_tilt(
            self.current_tilt
        )

    # Turn off motors and close driver on shutdown
    def destroy_node(self):

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