#  ROS2 controller node for Sudo's pan-tilt head.
#  Combines Dynamixel hardware control (with quintic trajectory profiling)
#  and target filtering using the benchmark-validated One Euro Filter.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

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
        self.health_pub = self.create_publisher(
           DiagnosticArray,
           "/head_health_status",
           10
        )
        self.health_timer = self.create_timer(
          5.0,
          self.check_health
        )

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

    def build_motor_diagnostic(self, name, health):

        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = str(health["id"])

        if health["ok"]:
            status.level = DiagnosticStatus.OK
            status.message = "Motor communication healthy"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "Motor communication failure"

        status.values = [
            KeyValue(
                key="id",
                value=str(health["id"])
            ),
            KeyValue(
                key="ping_ok",
                value=str(health["ping_ok"])
            ),
            KeyValue(
                key="position_read_ok",
                value=str(health["position_read_ok"])
            ),
            KeyValue(
                key="position",
                value=str(health["position"])
            ),
            KeyValue(
                key="comm_result",
                value=str(health["comm_result"])
            ),
            KeyValue(
                key="error",
                value=str(health["error"])
            ),
        ]

        return status
    
    def check_health(self):

        pan_health = self.driver.get_motor_health(
            self.driver.pan_id
        )

        tilt_health = self.driver.get_motor_health(
            self.driver.tilt_id
        )

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.status = [
            self.build_motor_diagnostic(
                "head/pan",
                pan_health
            ),
            self.build_motor_diagnostic(
                "head/tilt",
                tilt_health
            ),
        ]

        self.health_pub.publish(msg)
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
        pan_result = self.driver.set_pan(
            self.current_pan
        )

        tilt_result = self.driver.set_tilt(
            self.current_tilt
        )

        if not pan_result["ok"] or not tilt_result["ok"]:
            self.publish_command_fault(
                pan_result,
                tilt_result
            )

    def publish_command_fault(self, pan_result, tilt_result):

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        statuses = []

        pan_status = DiagnosticStatus()
        pan_status.name = "head/pan"
        pan_status.hardware_id = str(self.driver.pan_id)

        if pan_result["ok"]:
            pan_status.level = DiagnosticStatus.OK
            pan_status.message = "Command successful"
        else:
            pan_status.level = DiagnosticStatus.ERROR
            pan_status.message = "Pan command communication failure"

        statuses.append(pan_status)

        tilt_status = DiagnosticStatus()
        tilt_status.name = "head/tilt"
        tilt_status.hardware_id = str(self.driver.tilt_id)

        if tilt_result["ok"]:
            tilt_status.level = DiagnosticStatus.OK
            tilt_status.message = "Command successful"
        else:
            tilt_status.level = DiagnosticStatus.ERROR
            tilt_status.message = "Tilt command communication failure"

        statuses.append(tilt_status)

        msg.status = statuses

        self.health_pub.publish(msg)
    
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