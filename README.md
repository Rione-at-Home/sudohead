# sudohead

This is the old version. ROS 2 package for controlling **Sudo's 2-DOF pan-tilt head** using Dynamixel AX-12A servos. The package provides a low-level hardware driver for communicating with Dynamixel servos, continuous target signal filtering via an adaptive One Euro Filter in the node execution loop, quintic polynomial trajectory motion profiling, and empirical benchmarking derived from offline Jupyter telemetry notebooks.

---

## Features

* Control of a 2-DOF pan-tilt mechanism (Dynamixel AX-12A, Protocol 1.0)
* Automatic zero-position calibration at startup
* Real-time target filtering in `head_node` using the **One Euro Filter** to suppress servo jitter while maintaining low phase lag
* Empirical filter evaluation based on telemetry analysis (evaluating RMSE, jitter, and phase lag across step, sine, and random signals)
* Smooth trajectory generation using **quintic polynomial profiling**
* Degree-based angle interface with configurable software motion limits
* Dynamic ROS 2 parameters for filter cutoff frequencies and motion timing
* Safe torque disabling and serial port cleanup on shutdown

---

## Hardware

### Servo Configuration

| Joint | Dynamixel ID |
| --- | --- |
| Pan | 54 |
| Tilt | 55 |

### Communication

* Interface: OpenCR
* Port: `/dev/ttyACM0`
* Baudrate: `1000000`
* Protocol: Dynamixel Protocol 1.0

---

## Package Structure

```text
sudohead/
├── dynamixel_driver.py    # Low-level Dynamixel hardware driver
├── filters.py             # Reusable signal filters (EMA, Kalman, One Euro, PassThrough)
├── head_node.py           # ROS 2 control node (One Euro filtering, trajectory generation, ROS interface)
├── notebooks/
│   └── filter_analysis.ipynb # Telemetry logging, grid search, Pareto optimization, and metric analysis
├── package.xml
├── setup.py
└── README.md
```

---

## Driver

The `DynamixelDriver` class (`dynamixel_driver.py`) manages all low-level communication with the AX-12A motors via the Dynamixel SDK.

### Responsibilities

* Opening serial port communication
* Pinging Dynamixel motors during startup
* Enabling and disabling motor torque
* Reading present positions and writing goal positions
* Zero-position calibration
* Angle-to-position conversion (`1023 ticks / 300°`)
* Enforcing software safety limits

### Control Table

| Register | Address |
| --- | --- |
| Torque Enable | 24 |
| Goal Position | 30 |
| Present Position | 36 |

---

## Zero Calibration

During startup, current servo positions are read and stored as the zero-angle reference point:

```text
=== ZERO CALIBRATION ===
Pan Zero  : 479
Tilt Zero : 483
========================
```

All commanded pan and tilt angles are measured relative to this calibrated home position.

---

## Motion Limits

Software safety limits are enforced before commands are sent to the hardware driver:

```python
PAN_MIN  = -60  # degrees
PAN_MAX  =  60  # degrees

TILT_MIN = -45  # degrees
TILT_MAX =  45  # degrees
```

---

## ROS Node & Parameters

The `head_node` links ROS 2 command topics to the Dynamixel driver. While `filters.py` provides implementations for multiple architectures (EMA, Kalman, One Euro), `head_node` actively utilizes the **One Euro Filter** for real-time target smoothing.

### Responsibilities

* Subscribing to target pan and tilt angles
* Filtering raw input targets via a 1D One Euro Filter to suppress high-frequency jitter
* Interpolating trajectory steps using a quintic polynomial motion profile
* Driving the Dynamixel motors at 20 Hz
* Safely disabling motor torque on node destruction

### Configurable ROS 2 Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `min_cutoff` | `double` | `1.0` | Minimum cutoff frequency for One Euro filter ($f_{c,\min}$ in Hz) |
| `beta` | `double` | `0.05` | Speed coefficient ($\beta$) for adaptive cutoff scaling |
| `d_cutoff` | `double` | `1.0` | Cutoff frequency for derivative filtering ($f_{c,d}$ in Hz) |
| `motion_profile` | `string` | `"quintic"` | Trajectory profile type |
| `motion_time` | `double` | `2.5` | Duration of full trajectory motion (seconds) |

---

## Subscribed Topics

### `/head/pan_target`

Type: `std_msgs/msg/Float32`

Description: Desired pan angle in **degrees**.

Example:

```bash
ros2 topic pub --once /head/pan_target std_msgs/msg/Float32 "{data: 30.0}"
```

---

### `/head/tilt_target`

Type: `std_msgs/msg/Float32`

Description: Desired tilt angle in **degrees**.

Example:

```bash
ros2 topic pub --once /head/tilt_target std_msgs/msg/Float32 "{data: -20.0}"
```

---

## Motion Smoothing & Trajectory Generation

`head_node` combines real-time adaptive signal filtering with continuous trajectory profiling to achieve responsive target tracking while protecting the Dynamixel servos from mechanical strain.

### 1. Real-Time Target Filtering (One Euro Filter)

Raw command inputs in `head_node` are filtered dynamically using a **One Euro Filter**. The filter dynamically scales its cutoff frequency $f_c$ based on the magnitude of the target derivative (input velocity):

$$f_c = f_{c,\min} + \beta \cdot \vert{}\dot{x}\vert{}$$

$$\alpha = \frac{1}{1 + \frac{1}{2\pi f_c \Delta t}}$$

$$\hat{x}_k = \alpha x_k + (1 - \alpha) \hat{x}_{k-1}$$

* **Stationary / Low Speed:** $f_c \to f_{c,\min}$, heavily attenuating high-frequency noise to eliminate servo jitter.
* **High Speed:** $f_c$ grows with velocity via parameter $\beta$, reducing phase lag during rapid target tracking.

### 2. Telemetry Benchmarking & Metric Evaluation (Jupyter Notebook)

To determine ideal filter configurations, telemetry log datasets are analyzed offline in the notebook (`filter_analysis.ipynb`) across Step, Sine wave, and Random tracking patterns.

The comparative performance of candidate filter implementations (EMA, Kalman, and One Euro) is quantified using three primary metrics:

* **Root Mean Square Error (RMSE):** Quantifies spatial tracking accuracy relative to target trajectories:

$$\text{RMSE} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} (x_i - \hat{x}_i)^2}$$

* **Velocity Jitter ($J$):** Measures high-frequency derivative variance to quantify mechanical vibration and gear strain mitigation:

$$J = \sqrt{\frac{1}{N-1} \sum_{i=1}^{N-1} (\dot{\hat{x}}_{i+1} - \dot{\hat{x}}_i)^2}$$

* **Phase Lag ($\tau_{\text{lag}}$):** Calculates temporal delay between command input changes and filtered output response.

#### Parameter Grid Search & Pareto Front Optimization

By executing grid-search sweeps over parameter ranges ($f_{c,\min} \in [0.1, 5.0] \text{ Hz}$, $\beta \in [0.001, 0.5]$), Pareto optimal fronts are generated to select filter parameters that minimize phase lag for targeted jitter limits. The default values (`min_cutoff=1.0`, `beta=0.05`) reflect the optimized trade-off point for pan-tilt head response.

### 3. Quintic Polynomial Trajectory Profiling

After passing target commands through the One Euro Filter, motion steps are blended using a quintic polynomial profile to guarantee zero velocity and zero acceleration boundary conditions at start and endpoint:

$$s = \min\left(\max\left(\frac{t_{\text{elapsed}}}{t_{\text{motion}}}, 0.0\right), 1.0\right)$$

$$\text{blend} = 6s^5 - 15s^4 + 10s^3$$

$$\theta_{\text{current}} = \theta_{\text{start}} + \text{blend} \cdot (\theta_{\text{goal}} - \theta_{\text{start}})$$

### Control Loop Execution

```text
20 Hz (0.05 s execution period)
```

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

Run the head node with default parameters:

```bash
ros2 run sudohead head_node
```

Run with custom filter parameters tuned from notebook Pareto analysis:

```bash
ros2 run sudohead head_node --ros-args -p min_cutoff:=0.8 -p beta:=0.01 -p motion_time:=2.0
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

* Face/person tracking integration
* Joystick teleoperation node
* Home position service (`std_srvs/srv/Trigger`)
* Launch files for automated bringup
* `sensor_msgs/msg/JointState` publisher for RViz / URDF visualization
* Automated CSV export pipeline directly from `head_node` into `filter_analysis.ipynb`

---

## License

This project is intended for research and educational use.