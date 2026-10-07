# Robot Arm with Gamepad Control for QNX

This repository provides a complete ROS2 project for controlling a 5-DOF robotic arm using a standard gamepad controller on the QNX SDP 8.0. The project is designed to run on a Raspberry Pi 4B or Pi 5 and demonstrates real-time, intuitive control over the arm's movements.

![Picture of the Robotic Arm with ping pong balls.](./docs/QNX-Robot-Arm-at-CES.jpeg)

The project uses the [Arduino-based Robot Arm Model](https://cults3d.com/en/3d-model/various/arduino-based-robot-arm-howtomechatronics) from [How To Mechatronics](https://howtomechatronics.com/) -- check out their website and other models!

> **Important:**
> The `joy_teleop_hiddi` node requires access to the low-level HIDDI service, and the `arm_controller` node requires access to the I²C bus. The launch script should be run with root privileges.

(Are you trying this project or something based on it? [Come chat with us in the QNX Discord!](https://discord.gg/Jj4EkkrFTT))

***

## Table of Contents

- [Robot Arm with Gamepad Control for QNX](#robot-arm-with-gamepad-control-for-qnx)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Key Features](#key-features)
  - [Hardware Setup](#hardware-setup)
    - [1. I²C Communication Wiring (Pi to PCA9685)](#1-ic-communication-wiring-pi-to-pca9685)
    - [2. Servo Power Wiring](#2-servo-power-wiring)
    - [3. Servo Control Wiring (Servos to PCA9685)](#3-servo-control-wiring-servos-to-pca9685)
  - [Software Requirements](#software-requirements)
  - [How to Build](#how-to-build)
  - [How to Run the Demo](#how-to-run-the-demo)
  - [Configuration \& Tuning](#configuration--tuning)
    - [Manual Calibration](#manual-calibration)
    - [Setting Safe Workspace Limits (`start_robot.sh`)](#setting-safe-workspace-limits-start_robotsh)
      - [Servo Percentage Limits](#servo-percentage-limits)
      - [Cartesian Workspace Limits](#cartesian-workspace-limits)
    - [Controller Tuning (`arm_controller_node.py`)](#controller-tuning-arm_controller_nodepy)
  - [Controls](#controls)
    - [Global Controls](#global-controls)
    - [Joystick mode Controls](#joystick-mode-controls)
    - [Inverse Kinematic controls](#inverse-kinematic-controls)
  - [References](#references)
  - [Acknowledgments](#acknowledgments)

***

## Overview

This project provides a demo for teleoperating a 5-DOF robotic arm. It consists of two primary ROS2 nodes:
1. **joy_teleop_hiddi (C++):** A high-performance node that interfaces directly with the QNX HIDDI service to read raw data from a connected gamepad. It parses this data and publishes it as standard `/joy` messages.
2. **ik_solver (C++):** An inverse kinematics solver node using the Orocos
KDL library's Levenberg-Marquardt (LMA) position solver. It receives
Cartesian velocity commands from the arm controller, integrates them into
a position target, and solves for joint angles. A position-priority weight
matrix `[1,1,1,0,0,0]` is used to handle the under-determined 5-DOF
system by prioritizing end-effector position over orientation.
3. **arm_controller (Python):** An easily configurable node that acts as
the hardware abstraction layer. It subscribes to both `/joy` messages for
direct control and `/Mov` messages for IK-computed joint angles. It
converts all commands to PWM signals and sends them to the servos via a
PCA9685 I²C servo driver at 50Hz with exponential smoothing.


***

## Key Features

* **Velocity Control:** The joystick controls the arm's speed, not its position, allowing it to hold its pose when the joystick is released.
* **Movement Smoothing:** An easing function that makes all arm movements, including homing, smooth and controlled, preventing jerky motions.
* **Homing Function:** A dedicated "Home" button on the gamepad smoothly returns the arm to its neutral, upright position.
* **"Screensaver" Mode:** A toggleable mode that makes the arm perform a continuous, pre-programmed drawing motion (a figure-eight) until the user provides input.
* **Individual Servo Tuning:** All movement speeds, safe limits, and automated poses are easily configurable in the Python script.
* **Direct Joystick Mode:** Each joystick axis directly controls the speed of a corresponding servo joint.
* **Inverse Kinematics Mode:** Joystick controls the end effector position in Cartesian space (X, Y, Z). The IK solver computes the required joint angles automatically.

## Hardware Setup

This project requires a Raspberry Pi 4 or Raspberry Pi 5 to be connected to a PCA9685 16-channel servo driver board. This board is responsible for providing the power and control signals to the six servos of the robot arm. In all, you'll need:

* 1 x Raspberry Pi 4 or Raspberry Pi 5
* 1 x 3D printed body parts (see https://cults3d.com/en/3d-model/various/arduino-based-robot-arm-howtomechatronics)
* 3 x MG996R Servo Motor
* 3 x SG90 Micro Servo Motor
* 1 x 5V 6A+ DC Power Supply (and optionally a DC barrel jack breakout receiver)
* 1 x PCA9685 16-channel servo PWM controller
* Wiring (servo extension wires, power wires, dupont jumper wires to connect PCA9685 to Raspberry Pi)

The setup involves three main sets of connections:
1.  **I²C Communication:** A 4-wire connection between the Raspberry Pi and the PCA9685 for sending control commands.
2.  **Servo Power:** A dedicated, high-current 6V power supply connected directly to the PCA9685 to drive the servos.
3.  **Servo Control:** Connecting the six individual servos to the output channels of the PCA9685 board.

### 1. I²C Communication Wiring (Pi to PCA9685)
This connection allows the Raspberry Pi to tell the servo driver which servos to move. Use four jumper wires to connect the Raspberry Pi's GPIO header to the control pins on the PCA9685 board.

* **PCA9685 `VCC`** → RPi **Pin 1 (3.3V)**
* **PCA9685 `SDA`** → RPi **Pin 3 (GPIO 2)**
* **PCA9685 `SCL`** → RPi **Pin 5 (GPIO 3)**
* **PCA9685 `GND`** → RPi **Pin 6 (Ground)**

### 2. Servo Power Wiring
The servos require more power than the Raspberry Pi can safely provide. You must use a separate **6V power supply** to power the servos.

* Connect the **`+` (positive)** wire from your 6V power supply to the **`V+`** terminal on the green screw-down block of the PCA9685.
* Connect the **`-` (negative/ground)** wire from your power supply to the **`GND`** terminal on the green block.

> ** Warning:** Do **not** attempt to power the servos directly from the Raspberry Pi's 5V GPIO pin. Doing so can draw too much current and permanently damage your Raspberry Pi.

### 3. Servo Control Wiring (Servos to PCA9685)
Plug the six servos from the robot arm into the PWM output channels on the PCA9685 board. The demo is configured to use the following channel mapping:

* **Channel 0:** Base Servo
* **Channel 1:** Shoulder Servo
* **Channel 2:** Elbow Servo
* **Channel 3:** Wrist Roll Servo
* **Channel 4:** Wrist Pitch Servo
* **Channel 5:** Gripper Servo

Ensure the servo plugs are oriented correctly. The signal wire (yellow) should be on the pin row labeled **PWM**, the power/V+ wire (red) on the center row, and the ground wire (black) on the row labeled **GND**.

***

## Software Requirements

* **Operating System:** QNX SDP 8.0 with APK support.
* **ROS2 Jazzy:** A full ROS2 Jazzy installation for QNX `aarch64le` is required.
* **Dependencies:** The ROS2 installation must include `rclpy` and `rclcpp`.
* **Package Installation:** requires `packages` to run nodes; `pip3 install packaging`.
* **APK Dependencies:** Building requires the following APKs; `sudo apk add ros2-jazzy tinyxml2-dev eigen-dev qnx-screen-dev`
***
## How to Build

The project is built using `colcon` on a self hosted QNX target, the standard ROS2 build tool. A convenience script, `build.sh`, to automate this process.

**Run the build script:**
From the root of the project directory, make the script executable and run it:
```bash
chmod +x build.sh
./build.sh
```

The script will automatically locate your ROS2 installation, then build the projects packages. The compiled output will be placed in the `install/` directory.

***

## How to Run the Demo
> The first time the demo is run you are required to manually calibration see Manual Calibration section for more information.

A launch script, `start_robot.sh`, is provided in the `target_scripts` directory to set up the environment and run both nodes simultaneously on the QNX target.

```bash
# Make the target_script executable
chmod +x ./target_scripts/start_robot.sh
# Execute the script with root privileges to grant access to the I²C and HIDDI hardware.
sudo ./target_scripts/start_robot.sh
```

The script will launch `joy_teleop_node`, `ik_solver` and the `arm_controller_node.py` nodes in the background. You can now control the arm with the joystick. To stop both nodes, press `Ctrl+C` in the terminal where you ran the script.

***

## Configuration & Tuning

### Manual Calibration
After the Baby Robot Arm is fully assembled you will need to calibrate each the servos. The easiest way to calibrate servos is by running the `start_robot.sh` script to startup the demo and press Y to move the robot in the idle position before reinstalling each servo from the bottom up to match the image below. Before and after installing each servo it is important to home the servo by pressing Y to ensure it is in the correct position before moving onto the next one.

When homed the servos should be in the provided position
- Base (Servo 0)    - Face towards you.
- Servo 1-4         - Point straight up.
- Gripper (Servo 5) - Be 60% open

### Setting Safe Workspace Limits (`start_robot.sh`)
To prevent the robot arm from colliding with obstacles or damaging its own servos, you can configure software limits directly in the `start_robot.sh` launch script. You do not need to recompile the code to change these boundaries. There are two types of limits:

#### Servo Percentage Limits
These limits are enforced by the arm controller node and apply only in direct joystick mode. They are disabled when the IK solver is active, as Cartesian limits take over instead.
Locate the `SERVO_MIN_LIMITS` and `SERVO_MAX_LIMITS` arrays in the script. The arrays map to the 6 servos in this exact order: `[Base, Shoulder, Elbow, Pitch, Roll, Gripper]`.

* **Values:** Each value represents a percentage of physical rotation from **0.0** to **100.0**, where 50.0 is dead center.
* **How it works:** If you set a minimum limit of 25.0 and a maximum of 75.0, the Python node will mathematically clamp the servo so it cannot move outside of that 50% window, no matter how hard you push the joystick.
* **Safe Centering:** When you press the "Home" button on the gamepad, the script will automatically calculate a safe center pose that falls within your defined limits so the arm never breaks its boundaries.

#### Cartesian Workspace Limits
This is used by the IK solver to clamp the target position before solving, ensuring the solver only receives reachable targets and preventing unexpected arm movements at workspace boundaries. Applies only in IK Mode.
Locate the `CART_MIN_LIMITS` and `CART_MAX_LIMITS` arrays in the script. The arrays define the workspace box in this order: `[X, Y, Z]`.

* **Values:** Each value is a distance in meters from the robot base frame origin.
* **How it Works:**  The IK solver clamps the Cartesian target position to this 3D box before attempting to solve. If the joystick pushes the target outside the box, it is clamped to the nearest box boundary and the arm stops at the workspace edge.
* **Determining Limits:** Use the FK (Forward Kinematics) output logged at startup to find the home position of the gripper. Set the limits as a box around this position based on your desired workspace.

### Controller Tuning (`arm_controller_node.py`)
Additional high-level control logic and tunable parameters are located at the top of the **`arm_controller_node.py`** script. You can tweak the following to change how the arm feels:
* `SERVO_SPEEDS`: Adjust the sensitivity and maximum velocity of each servo's movement.
* `SMOOTHING_FACTOR`: Change how "cushioned" or "snappy" the arm feels when starting or stopping a movement.
* `DRAWING_POSE`: Manually tune the multi-servo pose the arm moves to before starting the screensaver.
* `SCREENSAVER_SPEED`, `SWEEP_WIDTH`: Change the speed and size of the screensaver motion.

After making changes to the Python script, simply use the `transfer.sh` script again to copy the updated file to the target. No recompilation is needed.

***

## Controls
To control the robot a **Logitech F310/F710** in D input mode via io-hid is used. This will both control the global control state, as well as can be used to manipulate individual joints.

### Global Controls
Global Controls use a modifier in order to make it harder to accidentally press. In our case this is the `SELECT`/`BACK` and `START` buttons combined with a face button

`SELECT` (Change input mode)
* `A` -> Joystick (default)
* `B` -> Inverse Kinematic
* `X` -> Direct Joint (not currently supported)
* `Y` -> IK with Direct Joint (not currently supported)

`START`: (Select Screen saver dance)
* `A` -> Figure Eight
* `B` -> Wave
* `X` -> COBRA
* `Y` -> CONDUCTOR

### Joystick mode Controls
* `Left Stick X`     -> Base        (servo 0)
* `Left Stick Y`     -> Shoulder    (servo 1)
* `Right Stick X`    -> Elbow       (servo 2)
* `Right Stick Y`    -> Hand tilt   (servo 4)
* `DPAD X`           -> Hand rotate (servo 3)
* `Shoulder Buttons` -> Gripper     (servo 5)
* `Stick Buttons`    -> Gripper     (servo 5) Note same as Shoulder Buttons
* `Y`                -> Center all servos

### Inverse Kinematic controls
* `Left Stick X`     -> Cartesian X
* `Left Stick Y`     -> Cartesian Y
* `Right Stick X`    -> Cartesian Z
* `Shoulder Buttons` -> Gripper     (servo 5)
* `Stick Buttons`    -> Gripper     (servo 5) Note same as Shoulder Buttons
* `Y`                -> Center all servos

## References

1.  **QNX Ports - Official Build Files Repository**: This is the official GitHub repository where QNX provides ports for open-source projects, including ROS2.
- <https://github.com/qnx-ports/build-files>
2.  **ROS2 Humble Port for QNX**: This is the specific location for the ROS2 Humble port, which contains the necessary patches and build changes to get ROS2 running on the QNX OS.
- <https://github.com/qnx-ports/build-files/tree/main/ports/ros2>

***

## Acknowledgments

* The 3D model for the robotic arm was created by **How To Mechatronics**.
* **Project Page with 3D Models:** [Arduino based Robot Arm](https://cults3d.com/en/3d-model/various/arduino-based-robot-arm-howtomechatronics)
* **Other Projects & 3D Models:** [How To Mechatronics Homepage](https://cults3d.com/en/users/HowToMechatronics/3d-models)
