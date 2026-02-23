# Robot Arm with Gamepad Control for QNX
 
This repository provides a complete ROS2 project for controlling a 6-DOF robotic arm using a standard gamepad controller on the QNX SDP 8.0. The project is designed to run on a Raspberry Pi 4B or Pi 5 and demonstrates real-time, intuitive control over the arm's movements.

The project uses the [Arduino-based Robot Arm Model](https://cults3d.com/en/3d-model/various/arduino-based-robot-arm-howtomechatronics) from [How To Mechatronics](https://howtomechatronics.com/) -- check out their website and other models!

> **Important:**  
> The `joy_teleop_hiddi` node requires access to the low-level HIDDI service, and the `arm_controller` node requires access to the I²C bus. The launch script should be run with root privileges.

(Are you trying this project or something based on it? [Come chat with us in the QNX Discord!](https://discord.gg/Jj4EkkrFTT))

***
 
## Table of Contents
 
- [Overview](#overview)
- [Key Features](#key-features)
- [Hardware Setup](#hardware-setup)
- [Software Requirements](#software-requirements)
- [How to Build](#how-to-build)
- [How to Transfer](#how-to-transfer)
- [How to Run the Demo](#how-to-run-the-demo)
- [Configuration and Tuning](#configuration-and-tuning)
- [References](#references)
- [Acknowledgments](#acknowledgments)
 
***
 
## Overview
 
This project provides a demo for teleoperating a 6-DOF robotic arm. It consists of two primary ROS2 nodes:
1. **joy_teleop_hiddi (C++):** A high-performance node that interfaces directly with the QNX HIDDI service to read raw data from a connected gamepad. It parses this data and publishes it as standard `/joy` messages.
2. **arm_controller (Python):** An easily configurable node that subscribes to the `/joy` messages. It translates joystick input into smooth, incremental movements for the arm's six servos, communicating with them via a `PCA9685 I²C servo driver`.

 
***
 
## Key Features
 
* **Velocity Control:** The joystick controls the arm's speed, not its position, allowing it to hold its pose when the joystick is released.
* **Movement Smoothing:** An easing function that makes all arm movements, including homing, smooth and controlled, preventing jerky motions.
* **Homing Function:** A dedicated "Home" button on the gamepad smoothly returns the arm to its neutral, upright position.
* **"Screensaver" Mode:** A toggleable mode that makes the arm perform a continuous, pre-programmed drawing motion (a figure-eight) until the user provides input.
* **Individual Servo Tuning:** All movement speeds, safe limits, and automated poses are easily configurable in the Python script.
***
 
## Hardware Setup
 
This project requires a Raspberry Pi 4 to be connected to a PCA9685 16-channel servo driver board. This board is responsible for providing the power and control signals to the six servos of the robot arm.
 
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
 
* **Operating System:** QNX SDP 8.0.
* **ROS2 Humble:** A full ROS2 Humble installation cross-compiled for QNX `aarch64le` is required. This is a complex process that involves building the port from source. The official build files and instructions are maintained by QNX and available at the link below.
  * **Official Port:** [QNX ROS2 Build Files](https://github.com/qnx-ports/build-files/tree/main/ports/ros2)
 
* **Dependencies:** The cross-compiled ROS2 installation must include `rclpy` and `rclcpp`.
* **Package Installation:** requires `packages` to run nodes; `pip3 install packaging`.

***
## How to Build
 
The project is built using `colcon`, the standard ROS2 build tool. A convenience script, `build.sh`, is provided to automate the cross-compilation process for the QNX target.
 
1.  **Ensure your QNX environment is set up:**
    ```bash
    source ~/qnx800/qnxsdp-env.sh
    ```
2.  **Run the build script:**
    From the root of the project directory, make the script executable and run it:
    ```bash
    chmod +x build.sh
    ./build.sh
    ```
The script will automatically locate your QNX toolchain and cross-compiled ROS2 installation, then build the `joy_teleop_hiddi` and `arm_controller` packages. The compiled output will be placed in the `install/aarch64le/` directory.
 
***
 
## How to Transfer
 
After a successful build, the `transfer.sh` script is used to copy the compiled nodes to your target Raspberry Pi running QNX.
 
1.  **Configure the Target IP:**
Open the `transfer.sh` script and set the `TARGET_IP_ADDRESS` variable to your Raspberry Pi's IP address.
2.  **Run the transfer script:**
    ```bash
    chmod +x transfer.sh
    ./transfer.sh
    ```
The script will use `scp` to copy the contents of your `install/aarch64le/` directory to the `/data/home/qnxuser/opt/ros/nodes/` directory on the target device.
 
***
 
## How to Run the Demo
 
A launch script, `start_robot.sh`, is provided in the `target_scripts` directory to set up the environment and run both nodes simultaneously on the QNX target.
 
1.  **Transfer the script:**
Make sure `start_robot.sh` has been transferred to the QNX target.
2.  **Make it executable:**
    On the QNX target, run:
    ```bash
    chmod +x start_robot.sh
    ```
3.  **Run the demo:**
    Execute the script with root privileges to grant access to the I²C and HIDDI hardware.
    ```bash
    ./start_robot.sh
    ```
The script will launch both the `joy_teleop_node` and the `arm_controller_node.py` in the background. You can now control the arm with the joystick. To stop both nodes, press `Ctrl+C` in the terminal where you ran the script.
 
***
 
## Configuration & Tuning

### Setting Safe Workspace Limits (`start_robot.sh`)
To prevent the robot arm from colliding with obstacles or damaging its own servos, you can configure software limits directly in the `start_robot.sh` launch script. You do not need to recompile the code to change these boundaries.

Locate the `SERVO_MIN_LIMITS` and `SERVO_MAX_LIMITS` arrays in the script. The arrays map to the 6 servos in this exact order: `[Base, Shoulder, Elbow, Pitch, Roll, Gripper]`.

* **Values:** Each value represents a percentage of physical rotation from **0.0** to **100.0**, where 50.0 is dead center. 
* **How it works:** If you set a minimum limit of 25.0 and a maximum of 75.0, the Python node will mathematically clamp the servo so it cannot move outside of that 50% window, no matter how hard you push the joystick.
* **Safe Centering:** When you press the "Home" button on the gamepad, the script will automatically calculate a safe center pose that falls within your defined limits so the arm never breaks its boundaries.

### Controller Tuning (`arm_controller_node.py`)
Additional high-level control logic and tunable parameters are located at the top of the **`arm_controller_node.py`** script. You can tweak the following to change how the arm feels:
* `SERVO_SPEEDS`: Adjust the sensitivity and maximum velocity of each servo's movement.
* `SMOOTHING_FACTOR`: Change how "cushioned" or "snappy" the arm feels when starting or stopping a movement.
* `DRAWING_POSE`: Manually tune the multi-servo pose the arm moves to before starting the screensaver.
* `SCREENSAVER_SPEED`, `SWEEP_WIDTH`: Change the speed and size of the screensaver motion.
 
After making changes to the Python script, simply use the `transfer.sh` script again to copy the updated file to the target. No recompilation is needed.

***
 
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