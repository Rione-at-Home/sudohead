"""Ping the head's AX-12A servos.

Change against the reference version (see CHANGES.md, C10): AX-12A servos speak Dynamixel Protocol 1.0;
the reference pinged with Protocol 2.0, which always reports a communication failure even when the
servo is fine. The port is now an argument and both IDs are pinged.

    python3 dynamixel_debug.py                 # /dev/ttyUSB0
    python3 dynamixel_debug.py /dev/ttyACM0    # OpenCR
"""

import sys

from dynamixel_sdk import PacketHandler, PortHandler

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 1000000
IDS = (54, 55)          # pan, tilt
AX12_MODEL = 12

port = PortHandler(PORT)
packet = PacketHandler(1.0)     # AX-12A: Protocol 1.0

print("open port:", port.openPort())
print("set baud :", port.setBaudRate(BAUD))

for dxl_id in IDS:
    model, comm, err = packet.ping(port, dxl_id)
    ok = comm == 0
    print(f"ID {dxl_id}: model={model}{' (AX-12A)' if model == AX12_MODEL else ''} "
          f"comm={packet.getTxRxResult(comm)} error={packet.getRxPacketError(err) or 'none'} "
          f"-> {'OK' if ok else 'NO ANSWER'}")

port.closePort()