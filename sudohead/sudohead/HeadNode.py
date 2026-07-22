
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

        # Current and Target States
        self.current_pan = 0.0
        self.current_tilt = 0.0

        self.target_pan = 0.0
        self.target_tilt = 0.0

        self.control_period = 0.05  
        self.max_velocity = 60.0  # degrees per second

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

        self.target_pan = msg.data

        self.get_logger().info(
            f"Received pan target: {self.target_pan}"
        )

    def tilt_callback(self, msg):

        self.target_tilt = msg.data

        self.get_logger().info(
            f"Received tilt target: {self.target_tilt}"
        )

    # Main Control Loop
    def control_loop(self):
        
        # Exponential smoothing
        #alpha = 0.15
        #
        #self.current_pan += (self.target_pan - self.current_pan) * alpha
        #
        #self.current_tilt += (self.target_tilt - self.current_tilt) * alpha
        #
        #Linear Velocity Smoothing
        max_step = (
            self.max_velocity *
            self.control_period
        )


        # PAN
        pan_error = self.target_pan - self.current_pan

        if abs(pan_error) <= max_step: # if the target is within one step, just set it directly

            self.current_pan = self.target_pan 

        else: # move towards the target by one step in the correct direction
            
            self.current_pan += (
                max_step *
                (1 if pan_error > 0 else -1)
            )


        # TILT
        tilt_error = (self.target_tilt - self.current_tilt)

        if abs(tilt_error) <= max_step:

            self.current_tilt = self.target_tilt

        else:

            self.current_tilt += (
                max_step
                if tilt_error > 0
                else -max_step
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