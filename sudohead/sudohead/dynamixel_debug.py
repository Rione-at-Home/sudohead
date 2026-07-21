from dynamixel_sdk import PortHandler, PacketHandler

PORT = "/dev/ttyACM0"
BAUD = 1000000

port = PortHandler(PORT)
packet = PacketHandler(2.0)

print(port.openPort())
print(port.setBaudRate(BAUD))

model, comm, err = packet.ping(port, 54)

print("model:", model)
print("comm:", comm)
print(packet.getTxRxResult(comm))
print(packet.getRxPacketError(err))