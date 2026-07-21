# sudohead

ROS 2 package for controlling **Sudo's 2-DOF pan-tilt head** using Dynamixel AX-12A servos. The package provides a hardware driver for communicating with the Dynamixels and a ROS 2 node that accepts pan and tilt angle commands, applies motion smoothing, and drives the motors.

---

## Features

* Control of a 2-DOF pan-tilt mechanism
* Supports Dynamixel AX-12A servos (Protocol 1.0)
* Automatic zero-position calibration at startup
* Smooth motion interpolation using a first-order filter
* Angle-based interface (degrees)
* Configurable software motion limits
* Simple ROS 2 topic interface

---

## Hardware

### Servo Configuration

| Joint | Dynamixel ID |
| ----- | -----------: |
| Pan   |           54 |
| Tilt  |           55 |

### Communication

* Interface: OpenCR
* Port: `/dev/ttyACM0`
* Baudrate: `1000000`
* Protocol: Dynamixel Protocol 1.0

---

## Package Structure

```text
sudohead/
├── HeadDriver.py          # Dynamixel hardware driver
├── head_node.py           # ROS2 control node
├── package.xml
├── setup.py
└── README.md
```

---

## Driver

The `DynamixelDriver` class is responsible for all low-level communication with the AX-12A motors.

### Responsibilities

* Opening the serial port
* Pinging the Dynamixels
* Enabling/disabling torque
* Reading motor positions
* Writing goal positions
* Zero calibration
* Angle-to-position conversion
* Software safety limits

### Control Table

| Register         | Address |
| ---------------- | ------: |
| Torque Enable    |      24 |
| Goal Position    |      30 |
| Present Position |      36 |

---

## Zero Calibration

During startup, the current servo positions are stored as the zero reference.

Example:

```text
=== ZERO CALIBRATION ===
Pan Zero  : 479
Tilt Zero : 483
========================
```

All commanded angles are measured relative to this calibrated home position.

---

## Motion Limits

Current software limits are

```python
PAN_MIN  = -60
PAN_MAX  =  60

TILT_MIN = -45
TILT_MAX =  45
```

These limits are applied before commands are sent to the motors to prevent excessive motion.

---

## ROS Node

The `head_node` is the interface between ROS 2 and the Dynamixel driver.

Responsibilities include:

* Receiving target angles
* Applying motion smoothing
* Sending commands to the Dynamixel driver
* Safely disabling torque during shutdown

---

## Published Topics

None.

---

## Subscribed Topics

### `/head/pan_target`

Type:

```text
std_msgs/msg/Float32
```

Description:

Desired pan angle in **degrees**.

Example:

```bash
ros2 topic pub --once /head/pan_target std_msgs/msg/Float32 "{data: 30.0}"
```

---

### `/head/tilt_target`

Type:

```text
std_msgs/msg/Float32
```

Description:

Desired tilt angle in **degrees**.

Example:

```bash
ros2 topic pub --once /head/tilt_target std_msgs/msg/Float32 "{data: -20.0}"
```

---

## Motion Smoothing

The node does not command the motors directly to the requested angle.

Instead, it gradually approaches the target using an exponential smoothing filter:

```python
current += (target - current) * alpha
```

Current configuration:

```python
alpha = 0.15
```

Control frequency:

```
20 Hz
```

This produces smoother head motion and reduces sudden servo movements.

---

## Running

Build the workspace:

```bash
colcon build --packages-select sudohead
```

Source the workspace:

```bash
source install/setup.bash
```

Run the head node:

```bash
ros2 run sudohead head_node
```

---

## Example Commands

Move head left:

```bash
ros2 topic pub --once /head/pan_target std_msgs/msg/Float32 "{data: 45.0}"
```

Move head right:

```bash
ros2 topic pub --once /head/pan_target std_msgs/msg/Float32 "{data: -45.0}"
```

Look up:

```bash
ros2 topic pub --once /head/tilt_target std_msgs/msg/Float32 "{data: -30.0}"
```

Look down:

```bash
ros2 topic pub --once /head/tilt_target std_msgs/msg/Float32 "{data: 30.0}"
```

Return to center:

```bash
ros2 topic pub --once /head/pan_target std_msgs/msg/Float32 "{data: 0.0}"

ros2 topic pub --once /head/tilt_target std_msgs/msg/Float32 "{data: 0.0}"
```

---

## Future Improvements

Potential additions include:

* Face/person tracking
* Joystick teleoperation
* Home position service
* Configurable ROS parameters
* PID-based tracking controller
* Launch files
* RViz visualization
* JointState publisher
* Dynamic motion speed adjustment
* Head trajectory generation

---

## License

This project is intended for research and educational use.
