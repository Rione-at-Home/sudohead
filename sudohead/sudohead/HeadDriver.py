#
## HeadDriver.py
#
#  Driver for the Dynamixel pan-tilt mechanism on the cat head.
#
#  Changes against the reference version (see CHANGES.md for the reasons):
#    [C1] try_read_position(): returns None when the read fails (the reference returned whatever the SDK
#         gave back, which is not a position after a communication error).
#    [C2] get_pan() / get_tilt() return None instead of a wrong angle when the read fails.
#    [C3] calibrate_zero() retries and raises if it cannot read a servo (no garbage zero).
#    [C4] enable() writes each servo's present position as its goal (and checks it) BEFORE torque-on,
#         so the head cannot jump to a stale goal at startup.
#  Everything else (control table, limits, angle conversion, API) is unchanged.
#

import time

from dynamixel_sdk import PortHandler, PacketHandler

# AX12 Control Table Addresses
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

PROTOCOL_VERSION = 1.0

TICKS_PER_DEGREE = 1023.0 / 300.0

COMM_SUCCESS = 0


class DynamixelDriver:

    def __init__(
        self,
        device_name="/dev/ttyUSB0",
        baudrate=1000000,
        pan_id=54,
        tilt_id=55,
    ):

        self.pan_id = pan_id
        self.tilt_id = tilt_id

        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError(
                f"Failed to open port {device_name}"
            )

        if not self.port_handler.setBaudRate(baudrate):
            raise RuntimeError(
                f"Failed to set baudrate {baudrate}"
            )

        print(f"Connected to {device_name}")

        self.pan_zero = 0
        self.tilt_zero = 0

        print("Pinging motors...")
        self.ping(self.pan_id)
        self.ping(self.tilt_id)

    # Communication Helpers
    def ping(self, dxl_id):

        model, comm_result, error = (
            self.packet_handler.ping(
                self.port_handler,
                dxl_id
            )
        )

        print(
            f"PING ID={dxl_id} "
            f"MODEL={model} "
            f"COMM={comm_result} "
            f"ERROR={error}"
        )

        return comm_result == COMM_SUCCESS

    # Torque
    def enable_torque(self, dxl_id):

        comm_result, error = (
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE,
            )
        )

        print(
            f"Enable Torque ID={dxl_id} "
            f"COMM={comm_result} "
            f"ERROR={error}"
        )

    def disable_torque(self, dxl_id):

        self.packet_handler.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE,
        )

    # [C4] Write the present position as the goal, read it back, then torque-on.
    def hold_current(self, dxl_id, retries=5):

        for _ in range(retries):
            position = self.try_read_position(dxl_id)
            if position is None:
                continue

            comm_result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_GOAL_POSITION,
                int(position),
            )
            if comm_result == COMM_SUCCESS and error == 0:
                goal, comm_result, error = self.packet_handler.read2ByteTxRx(
                    self.port_handler,
                    dxl_id,
                    ADDR_GOAL_POSITION,
                )
                if comm_result == COMM_SUCCESS and error == 0 and goal == int(position):
                    return
            time.sleep(0.01)

        raise RuntimeError(
            f"Could not set the goal of ID={dxl_id} to its present position; refusing to enable torque"
        )

    def enable(self):

        for dxl_id in (self.pan_id, self.tilt_id):
            self.hold_current(dxl_id)
            self.enable_torque(dxl_id)

    def disable(self):

        self.disable_torque(self.pan_id)
        self.disable_torque(self.tilt_id)

    # Raw Position Access
    def read_position(self, dxl_id):
        """Unchanged reference behaviour (kept for existing callers): may return a wrong value on error."""

        position, comm_result, error = (
            self.packet_handler.read2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_PRESENT_POSITION,
            )
        )

        if comm_result != COMM_SUCCESS:
            print(
                f"Read Error ID={dxl_id} "
                f"COMM={comm_result}"
            )

        return position

    # [C1] Present position in ticks, or None when the read failed.
    def try_read_position(self, dxl_id, retries=1):

        for attempt in range(max(1, retries)):
            position, comm_result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_PRESENT_POSITION,
            )
            if comm_result == COMM_SUCCESS and error == 0 and 0 <= position <= 1023:
                return position
            if attempt + 1 < retries:
                time.sleep(0.005)
        return None

    def write_position(self, dxl_id, position):

        comm_result, error = (
            self.packet_handler.write2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_GOAL_POSITION,
                int(position),
            )
        )

    # Calibration
    # [C3] Retry, and fail loudly instead of storing a garbage zero.
    def calibrate_zero(self, retries=5):

        self.pan_zero = self.try_read_position(self.pan_id, retries)
        self.tilt_zero = self.try_read_position(self.tilt_id, retries)

        if self.pan_zero is None or self.tilt_zero is None:
            raise RuntimeError(
                "Cannot read the pan/tilt position for zero calibration "
                f"(pan={self.pan_zero}, tilt={self.tilt_zero})"
            )

        print()
        print("=== ZERO CALIBRATION ===")
        print(f"Pan Zero  : {self.pan_zero}")
        print(f"Tilt Zero : {self.tilt_zero}")
        print("========================")
        print()

    # Safe Limits
    PAN_MIN = -60
    PAN_MAX = 60

    TILT_MIN = -45
    TILT_MAX = 45

    # Pan Control
    def set_pan(self, angle):

        angle = max(
            self.PAN_MIN,
            min(self.PAN_MAX, angle)
        )

        position = int(
            self.pan_zero +
            angle * TICKS_PER_DEGREE
        )

        self.write_position(
            self.pan_id,
            position
        )

    # [C2] None when the read failed.
    def get_pan(self):

        position = self.try_read_position(self.pan_id)
        if position is None:
            return None

        return (
            position - self.pan_zero
        ) / TICKS_PER_DEGREE

    # Tilt Control
    def set_tilt(self, angle):

        angle = max(
            self.TILT_MIN,
            min(self.TILT_MAX, angle)
        )

        position = int(
            self.tilt_zero +
            angle * TICKS_PER_DEGREE
        )

        self.write_position(
            self.tilt_id,
            position
        )

    # [C2] None when the read failed.
    def get_tilt(self):

        position = self.try_read_position(self.tilt_id)
        if position is None:
            return None

        return (
            position - self.tilt_zero
        ) / TICKS_PER_DEGREE

    # Cleanup
    def close(self):

        self.port_handler.closePort()