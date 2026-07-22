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

        self.driver = DynamixelDriver()

        self.driver.enable()
        self.driver.calibrate_zero()

        # Current and Target States
        self.current_pan = 0.0
        self.current_tilt = 0.0

        self.target_pan = 0.0
        self.target_tilt = 0.0

        # Current Velocities
        self.pan_velocity = 0.0
        self.tilt_velocity = 0.0

        # Control Loop Timing and Limits
        self.control_period = 0.05  
        self.max_velocity = 60.0  # degrees per second

        # Spring Controller Parameters
        self.k = 25.0  # Spring stiffness
        self.c = 10.0  # Damping coefficient (2 * sqrt(k) for critically damped)

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

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop
        )

        self.get_logger().info(
            "Head node started (Critically Damped Spring Controller)"
        )

    # Topic Callbacks
    def pan_callback(self, msg):
        self.target_pan = msg.data

    def tilt_callback(self, msg):
        self.target_tilt = msg.data

    # Control Loop
    def control_loop(self):
        dt = self.control_period

        # Pan Spring Calculation
        pan_error = self.target_pan - self.current_pan
        pan_accel = (self.k * pan_error) - (self.c * self.pan_velocity)
        
        self.pan_velocity += pan_accel * dt
        
        # Clamp maximum pan velocity
        self.pan_velocity = max(
            -self.max_velocity, 
            min(self.max_velocity, self.pan_velocity)
        )
        
        self.current_pan += self.pan_velocity * dt

        # Tilt Spring Calculation
        tilt_error = self.target_tilt - self.current_tilt
        tilt_accel = (self.k * tilt_error) - (self.c * self.tilt_velocity)
        
        # Euler integration for tilt
        self.tilt_velocity += tilt_accel * dt
        
        # Clamp maximum tilt velocity
        self.tilt_velocity = max(
            -self.max_velocity, 
            min(self.max_velocity, self.tilt_velocity)
        )
        
        self.current_tilt += self.tilt_velocity * dt

        # --- Send commands to hardware ---
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