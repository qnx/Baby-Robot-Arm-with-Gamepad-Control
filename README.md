# Robot Arm with Gamepad Control for QNX

This repository is a hardened ROS2 reference/demo for controlling a 5-DOF robotic arm with a supported Logitech F310/F710 on QNX SDP 8.0. It targets a Raspberry Pi 4B or Pi 5; production hardware output-disable integration, guarded validation, and a complete measured IK model remain deployment work.

![Picture of the Robotic Arm with ping pong balls.](./docs/QNX-Robot-Arm-at-CES.jpeg)

The project uses the [Arduino-based Robot Arm Model](https://cults3d.com/en/3d-model/various/arduino-based-robot-arm-howtomechatronics) from [How To Mechatronics](https://howtomechatronics.com/) -- check out their website and other models!

> **Important:**
> The `joy_teleop_hiddi` node requires access to the low-level HIDDI service, and the `arm_controller` node requires access to the I²C bus. Run them through a dedicated unprivileged account with only those device permissions. Never run the robot launcher with `sudo`.

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
    - [Controller Tuning](#controller-tuning)
  - [Controls](#controls)
    - [Global Controls](#global-controls)
    - [Joystick mode Controls](#joystick-mode-controls)
    - [Inverse Kinematic controls](#inverse-kinematic-controls)
  - [References](#references)
  - [Acknowledgments](#acknowledgments)

***

## Overview

This project provides a demo for teleoperating a 5-DOF robotic arm with a separate gripper servo. The repository contains three ROS2 nodes; the secured default launcher runs the gamepad and arm-controller nodes, while IK is experimental and opt-in:

1. **joy_teleop_hiddi (C++):** Interfaces with the QNX HIDDI service, accepts one supported length- and framing-checked Logitech report profile, and publishes standard relative `joy` messages. Exclusive attachment and a 250 ms report watchdog prevent a disconnected or displaced controller from replaying stale motion. Removal, report loss, and invalid active reports latch a terminal input fault, publish a final neutral sample, and exit nonzero so supervision stops the complete control unit; recovery requires a fresh supervised start. VID/PID and report-profile checks are not device authentication; full descriptor-property validation remains a target gate pending genuine QNX captures.
2. **ik_solver (C++):** An inverse kinematics solver node using the Orocos
KDL library's Levenberg-Marquardt (LMA) position solver. It receives
Cartesian velocity commands from the arm controller, integrates them into
a position target, and solves for joint angles. A position-priority weight
matrix `[1,1,1,0,0,0]` is used to prioritize end-effector position over
orientation. The hardened solver requires an exact, bounded five-revolute-joint
chain ending at a leaf/tool-center-point link. The checked-in URDF is incomplete,
so it intentionally fails validation before creating ROS publishers or subscribers.
Malformed pre-solve inputs keep IK inhibited; a solver, FK, home-workspace, or
output-invariant failure during active control exits nonzero so the launcher
stops the complete control unit.
3. **arm_controller (Python):** An easily configurable node that acts as
the hardware abstraction layer. It publishes the relative `joint_session`
grant and validates relative `joy`, `joint`, and `mov` commands before they
reach the actuator layer. It
converts all commands to PWM signals and sends them to the servos via a
PCA9685 I²C servo driver at 50 Hz with exponential smoothing. Outputs start
disarmed, calibrated limits remain active in every mode, and command leases
disable output after stale input.


***

## Key Features

* **Velocity Control:** The joystick controls the arm's speed, not its position, allowing it to hold its pose when the joystick is released.
* **Movement Smoothing:** An easing function bounds commanded output changes, including the center command, to reduce jerky motion.
* **Bounded Center Command:** A dedicated button requests the configured neutral pose; this is not sensor-backed homing.
* **Default-disabled Demonstrations:** Autonomous trajectories have a short renewable lease and a 30-second ceiling, but remain unavailable in the secured launcher until the physical joint map is verified.
* **Individual Servo Tuning:** Movement speeds and automated poses are code-configurable; calibrated safe limits are validated launch parameters constrained by compiled actuator bounds.
* **Direct Joystick Mode:** Each joystick axis directly controls the speed of a corresponding servo joint.
* **Experimental Inverse Kinematics Mode:** With an explicitly enabled and fully validated replacement model, the joystick can control the end effector in Cartesian space. It is unavailable with the incomplete checked-in URDF.

## Hardware Setup

This project requires a Raspberry Pi 4 or Raspberry Pi 5 to be connected to a PCA9685 16-channel servo driver board. This board is responsible for providing the power and control signals to the six servos of the robot arm. In all, you'll need:

* 1 x Raspberry Pi 4 or Raspberry Pi 5
* 1 x 3D printed body parts (see https://cults3d.com/en/3d-model/various/arduino-based-robot-arm-howtomechatronics)
* 3 x 270-degree high-torque servo motors for the base, shoulder, and elbow
* 3 x 180-degree micro servo motors for the wrist, hand, and gripper
* 1 x regulated 6V 6A+ DC power supply (and optionally a DC barrel jack breakout receiver)
* 1 x PCA9685 16-channel servo PWM controller
* 1 x physical emergency stop and normally-off PCA9685 OE or servo-power watchdog
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

> **Warning:** Do **not** attempt to power the servos directly from the Raspberry Pi's 5V GPIO pin. Doing so can draw too much current and permanently damage your Raspberry Pi.

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
* **ROS2 dependencies:** The installation must include `rclpy`, `rclcpp`, `rcl_interfaces`, `ament_index_cpp`, `builtin_interfaces`, `sensor_msgs`, `std_msgs`, `orocos_kdl`, `kdl_parser`, and `urdf`.
* **ROS2 security:** Provision an SROS2 keystore with a separate enclave and deny-by-default permissions for each required node.
* **Python dependencies:** The target requires the `smbus` module imported by the controller package.
* **APK Dependencies:** Building requires the following APKs; `sudo apk add ros2-jazzy tinyxml2-dev eigen-dev qnx-screen-dev`
***
## How to Build

The project is built using `colcon` on a self-hosted QNX target, the standard ROS2 build tool. A convenience script, `build.sh`, is provided to automate this process.

**Run the build script:**
From the root of the project directory, make the script executable and run it:
```bash
chmod +x build.sh
./build.sh
```

The script requires the trusted ROS2 Jazzy installation at `/opt/ros/jazzy`, clears inherited Python-startup and colcon/CMake/compiler controls before and after executable setup metadata, ignores ambient CMake environment-package, package-root, and package-registry hints, and refuses non-empty `build/`, `install/`, or `log/` bases. It launches `colcon` through a positive environment allowlist containing only fixed runtime values plus the enumerated QNX/ROS release-manifest inputs. Run `./clean.sh` before each hardened build. It then builds the project packages as `RelWithDebInfo`. A shared CMake policy probes and applies supported stack protection, `_FORTIFY_SOURCE=2`, PIE, and RELRO/NOW to the HID C/C++ and IK C++ executables. Unsupported target flags warn rather than masquerading as enabled; CFI/LTO are not enabled. Inspect the final QNX ELF files before promotion because successful configure-time probes do not prove properties of the shipped binaries. The compiled output is placed in the `install/` directory. Direct standalone CMake/toolchain invocation is outside this hardened entry-point contract and must use an equivalent clean environment and fresh build tree.

***

## How to Run the Demo

> Keep servo power or PCA9685 OE disabled while provisioning and validating the target. Complete the Manual Calibration and safety checks before energized operation.

Deploy the built `install/` tree to a versioned, administrator-owned prefix. Do not run a developer-writable build tree with elevated privileges.

```bash
sudo install -d /opt/baby_robot_arm/releases/1
sudo cp -a install/. /opt/baby_robot_arm/releases/1/
sudo install -m 0755 target_scripts/start_robot.sh /opt/baby_robot_arm/releases/1/start_robot.sh
sudo chown -R root:root /opt/baby_robot_arm/releases/1
sudo chmod -R u=rwX,go=rX /opt/baby_robot_arm/releases/1
sudo ln -sfn /opt/baby_robot_arm/releases/1 /opt/baby_robot_arm/current
```

Create signed SROS2 artifacts for the active enclaves, with deny-by-default topic permissions:

* `/baby_robot_arm/<robot-id>/joy`
* `/baby_robot_arm/<robot-id>/controller`
* `/baby_robot_arm/<robot-id>/ik` only when a verified replacement model enables experimental IK

See the [ROS2 Jazzy keystore guidance](https://docs.ros.org/en/ros2_documentation/jazzy/Tutorials/Advanced/Security/The-Keystore.html). Protect enclave private keys and keep certificate-authority private keys off the robot. An administrator must separately grant the dedicated service account only the QNX HIDDI and I²C permissions it needs.

On every boot, the privileged service manager must first create a private runtime lock directory owned by the dedicated robot account (replace `babyrobot` with that account):

```bash
mkdir -p /var/run/baby_robot_arm
chown babyrobot:babyrobot /var/run/baby_robot_arm
chmod 0700 /var/run/baby_robot_arm
```

The driver validates the directory and lock file owner, mode, type, and link count, then holds a non-blocking POSIX record lock for the entire I²C session. A second controller fails before reaching PWM writes.

Run the launcher as that **unprivileged** account:

```bash
export ROBOT_ID=lab_arm_1
export ROS_DOMAIN_ID=41
export ROS_SECURITY_KEYSTORE=/etc/baby_robot_arm/security
export ROBOT_INSTALL_PREFIX=/opt/baby_robot_arm/current
/opt/baby_robot_arm/current/start_robot.sh
```

The launcher starts Bash in privileged mode to suppress inherited function imports and `BASH_ENV`, rejects raw exported-function entries before any descendant shell can import them, then clears inherited loader, Python-startup, executable, and search-path controls at each setup boundary. It enforces SROS2, confines relative topics to `/baby_robot_arm/<robot-id>`, and treats every active node as one required control unit. While the launcher is alive, any active-node exit—including a terminal HID fault or active IK safety-invariant failure—terminates the others and returns a failure status. Configure the service manager to start it from a minimal allowlisted environment (the equivalent of `env -i` plus the documented variables) and to own/kill the complete launcher process group if the leader dies: dynamic-loader variables are consumed before any script body can unset them, and SIGKILL cannot run shell traps. Do not invoke this script with `sudo`.

The default is `ENABLE_EXPERIMENTAL_IK=0`. Do not set it to `1` with the checked-in URDF: the solver will reject that incomplete model and the supervised control unit will stop. Enabling IK requires a measured replacement model whose `world`-to-tool chain contains exactly the bounded revolute joints `base`, `shoulder`, `elbow`, `wrist`, and `hand`, in that order, plus verified collision geometry and a site-specific SROS2 IK enclave. Autonomous demonstration trajectories are also disabled by the secured launcher until the physical wrist channel order has been verified on the target with power/torque limited.

The secured default also refuses to arm because this repository cannot supply a board-specific PCA9685 OE or servo-power hook. Integrate that normally-off hook before energized use. `ALLOW_SOFTWARE_ONLY_OUTPUT=1` is provided only for supervised bench testing with servo power disconnected; it emits a danger warning and must not be used as a deployment setting.

Read [SECURITY.md](SECURITY.md) before commissioning. The [security remediation guide](docs/security-remediation-guide.html) records each audited issue, vulnerable source excerpt, recommended repair, and risk-reduction rationale. Software shutdown cannot replace a physical E-stop and normally-off OE or servo-power watchdog.

***

## Configuration & Tuning

### Manual Calibration
Calibrate first with servo horns/linkages detached or servo power disabled, the work area clear, and the physical E-stop within reach. Signal-only tests may use the conspicuous software-output override while servo power is disconnected; any energized calibration requires the board-specific normally-off output hook. Start the secured launcher as the unprivileged service account. With every stick neutral, hold `L3` and press the exact `SELECT`+`START`+`A` chord to arm outputs; keep `L3` held while motion is allowed, then press `Y` to request the bounded center pose. Refit and verify one linkage at a time from the base upward; release `L3` or press `SELECT`+`START`+`B` to disarm before touching the mechanism. The first pulse after arming can move a servo because this open-loop controller cannot know its physical position while outputs are disabled.

When centered, the commanded servo positions are:
- Base (Servo 0)    - Face towards you.
- Servo 1-4         - Point straight up.
- Gripper (Servo 5) - Use the configured 50% midpoint command

### Setting Safe Workspace Limits (`start_robot.sh`)
To reduce the range of unexpected motion and protect calibrated servo travel, configure software limits directly in the `start_robot.sh` launch script. These bounds do not detect self-collision, obstacles, people, or swept paths. You do not need to recompile the code to change them. There are two types of limits:

#### Servo Percentage Limits
These calibrated limits are enforced by the arm controller node in every mode, including IK and autonomous demonstrations. Cartesian constraints may narrow the allowed motion but never replace these actuator-layer limits.
Locate the `SERVO_MIN_LIMITS` and `SERVO_MAX_LIMITS` arrays in the script. The arrays map to the 6 servos in this exact order: `[Base, Shoulder, Elbow, Wrist Roll, Wrist Pitch, Gripper]`.

* **Values:** Each value represents a percentage of physical rotation from **0.0** to **100.0**, where 50.0 is dead center.
* **How it works:** If you set a minimum limit of 25.0 and a maximum of 75.0, the Python node will mathematically clamp the servo so it cannot move outside of that 50% window, no matter how hard you push the joystick.
* **Safe Centering:** When you press the "Home" button on the gamepad, the script will automatically calculate a safe center pose that falls within your defined limits so the arm never breaks its boundaries.

#### Cartesian Workspace Limits
This is used by the IK solver to constrain targets in IK mode. It is an additional limit, not a complete collision-safety system.
Locate the `CART_MIN_LIMITS` and `CART_MAX_LIMITS` arrays in the script. The arrays define the workspace box in this order: `[X, Y, Z]`.

* **Values:** Each value is a distance in meters from the robot base frame origin.
* **How it Works:** The IK solver rejects or bounds targets outside this 3D box before publishing a joint solution. Per-joint, finite-value, rate, and slew checks still apply downstream.
* **Determining Limits:** Use a verified full-arm model and measured tool-center point. A point inside this box can still produce a self-collision or strike an obstacle, so use collision-aware path validation and a guarded clear zone.

### Controller Tuning
The tuning constants are split by responsibility:

* `arm_control.py`: `SERVO_SPEEDS`, `SMOOTHING_FACTOR`, pulse widths, angle ranges, and maximum steps.
* `arm_controller_input.py`: `DRAWING_POSE`, `SCREENSAVER_SPEED`, `SCREENSAVER_WIDTH`, and `SCREENSAVER_HEIGHT`.
* `arm_controller_node.py`: immutable launch parameters for command timeouts, autonomous leases, hardware-output policy, I²C, servo limits, and the experimental IK opt-in. Every shutdown path disables output without parking motion.

After changing the Python source, run `./clean.sh` and then `./build.sh` to refresh the package in the `install/` tree, then restart the demo. Hardened builds deliberately reject a non-empty build, install, or log base instead of reusing security-sensitive CMake state. This repository does not include a `transfer.sh` script.

***

## Controls
The HID parser supports a **Logitech F310/F710** in XInput or DirectInput mode through QNX HIDDI. Other controller models are not accepted by the current VID/PID lookup table. Validate captured center/minimum/maximum reports from the exact controller and target stack with servo power disabled before commissioning.

### Global Controls
Outputs start disarmed. `L3` is the held operator-enable control and no longer commands the gripper. Arming requires neutral sticks and the exact chord below. A valid joystick sample that releases `L3` is handled before any arm, mode, or demonstration chord is parsed, so an ambiguous chord cannot postpone output disable:

* Hold `L3` + `SELECT` + `START` + `A` -> Arm outputs; keep `L3` held
* Release `L3`, or use the neutral `SELECT` + `START` + `B` chord -> Immediately disarm and disable PWM

Fault-stop, an actual mode change, autonomous entry or dance change, and autonomous cancellation freeze queued actuator targets at the last commanded output estimate before another producer can renew authority. This prevents re-arming or changing producers from resuming an old slew target.

Mode selection uses the exact held-operator chord `L3` + `SELECT`/`BACK` plus
exactly one of `A`, `B`, or `X`; no other button may be held:

`SELECT` (Change input mode)
* `A` -> Joystick (default)
* `B` -> Experimental inverse kinematics; rejected unless `enable_ik_mode` was enabled at launch
* `X` -> Direct Joint (on every mode entry or re-arm, read the nonzero `std_msgs/UInt64` grant from relative `joint_session`, echo it exactly as `JointState.header.frame_id` `joint-session:N`, and publish the fresh command on relative `joint`; zero revokes the producer session on fault or mode exit, and the source stamp must also be strictly later than the controller's local grant receipt and no later than callback receipt)

The following demonstration mappings describe the implementation, but they are unavailable under the secured launcher. Do not enable autonomous mode until the physical joint/channel order and bounded trajectories have been verified with power and torque limited. When enabled in a validated integration, holding `L3` plus `START` and one face button renews a short motion lease; releasing `L3` disables output and every run has a 30-second ceiling.

`START` (Select demonstration motion)
* `A` -> Figure Eight
* `B` -> Wave
* `X` -> COBRA
* `Y` -> CONDUCTOR

### Joystick mode Controls
* `Left Stick X`     -> Base        (servo 0)
* `Left Stick Y`     -> Shoulder    (servo 1)
* `Right Stick Y`    -> Elbow       (servo 2)
* `Right Stick X`    -> Hand tilt   (servo 4)
* `DPAD X`           -> Hand rotate (servo 3)
* `Shoulder Buttons` -> Gripper     (servo 5)
* `Right Stick Button` -> Open gripper; `L3` remains dedicated to operator enable
* `Y`                -> Center all servos using the controller's calibrated setpoints

### Inverse Kinematic controls
These controls must remain disabled in deployment until experimental IK uses a complete verified model and the integration supplies independently verified homing or position feedback. On every IK entry, forced reset, or re-arm, the controller creates a new exactly representable session epoch. `CurrentPositions` carries `[base, shoulder, elbow, wrist, hand, epoch]`; the solver accepts only a strictly newer epoch and invalidates the previous stream before validating the joint state. `CartesianCmd` carries `[x, y, z, home, epoch]`, and solver output must return the exact `JointState.header.frame_id` `ik-session:N` plus a fresh source stamp before actuator targets can change. The synchronized joints are still only the controller's commanded open-loop estimate, so they cannot attest physical homing or detect slip, stall, or manual displacement. The two `Float64MultiArray` coordination topics are also unstamped and do not provide a per-command sequence or source lease; within one running control-unit lifetime, the epoch prevents reuse across IK sessions, but it is not persistent across a supervised-unit restart and does not prevent forgery by an authorized publisher.

* `Left Stick X`     -> Cartesian X
* `Left Stick Y`     -> Cartesian Y
* `Right Stick Y`    -> Cartesian Z
* `Shoulder Buttons` -> Gripper     (servo 5)
* `Right Stick Button` -> Open gripper; `L3` remains dedicated to operator enable
* `Y`                -> Ask the IK solver to center arm joints 0–4; gripper remains unchanged

## References

1.  **QNX Ports - Official Build Files Repository**: This is the official GitHub repository where QNX provides ports for open-source projects, including ROS2.
- <https://github.com/qnx-ports/build-files>
2.  **ROS2 Port for QNX**: This directory contains the QNX ROS2 port build files. The scripts in this repository expect ROS2 Jazzy under `/opt/ros/jazzy`.
- <https://github.com/qnx-ports/build-files/tree/main/ports/ros2>

***

## Acknowledgments

* The 3D model for the robotic arm was created by **How To Mechatronics**.
* **Project Page with 3D Models:** [Arduino based Robot Arm](https://cults3d.com/en/3d-model/various/arduino-based-robot-arm-howtomechatronics)
* **Other Projects & 3D Models:** [How To Mechatronics Homepage](https://cults3d.com/en/users/HowToMechatronics/3d-models)
