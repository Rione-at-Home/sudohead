
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

from scipy.interpolate import CubicSpline



class HeadNode(Node):

    def __init__(self):
        super().__init__('head_node')

        # Dynamixel Driver Initialization
        self.driver = DynamixelDriver()

        self.driver.enable()
        self.driver.calibrate_zero()

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
        self.motion_time = 2   # seconds

        # Timer period
        self.control_period = 0.05

        self.pan_spline = None
        self.tilt_spline = None

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

        self.pan_start = self.current_pan
        self.pan_goal = msg.data
        self.pan_elapsed = 0.0

        self.pan_spline = CubicSpline(
            [0.0, self.motion_time],
            [self.pan_start, self.pan_goal],
            bc_type="clamped"
        )

    def tilt_callback(self, msg):

        self.tilt_start = self.current_tilt
        self.tilt_goal = msg.data
        self.tilt_elapsed = 0.0

        self.tilt_spline = CubicSpline(
            [0.0, self.motion_time],
            [self.tilt_start, self.tilt_goal],
            bc_type="clamped"
        )

    # Main Control Loop
    def control_loop(self):

        self.pan_elapsed += self.control_period
        self.tilt_elapsed += self.control_period

        self.pan_elapsed = min(
            self.pan_elapsed,
            self.motion_time
        )

        self.tilt_elapsed = min(
            self.tilt_elapsed,
            self.motion_time
        )

        if self.pan_spline is not None:
            self.current_pan = float(
                self.pan_spline(self.pan_elapsed)
            )

        if self.tilt_spline is not None:
            self.current_tilt = float(
                self.tilt_spline(self.tilt_elapsed)
            )

        self.driver.set_pan(self.current_pan)
        self.driver.set_tilt(self.current_tilt)
        
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