# Baby Robot Arm Security and Safety

This project controls physical actuators. Security failures, stale commands, invalid numeric values, and software crashes can therefore become unexpected motion. Treat command authorization, input validation, calibrated limits, process supervision, and hardware output-disable as one safety boundary.

## Secure deployment requirements

A deployment is not production-ready until all of the following controls are present:

1. Run the ROS graph as a dedicated unprivileged account. Grant only narrowly scoped I2C and HIDDI permissions; do not run start_robot.sh with sudo.
2. Promote tested artifacts to a root-owned, non-writable install prefix. Do not execute a developer install tree or user site packages in the robot service.
3. Create a separate SROS2 identity for the joystick broker, IK solver, arm controller, and any separately deployed command gateway. Use authenticated and encrypted DDS governance with deny-by-default topic permissions. Follow the [ROS 2 Jazzy keystore guidance](https://docs.ros.org/en/ros2_documentation/jazzy/Tutorials/Advanced/Security/The-Keystore.html) and keep certificate-authority private keys off the robot.
4. Assign a unique ROBOT_ID and ROS_DOMAIN_ID. Keep relative topics inside /baby_robot_arm/robot-id and restrict DDS transport to the local host unless an authenticated remote control path is intentionally designed.
5. Keep calibrated per-joint limits enabled in every mode. Cartesian limits, IK constraints, and autonomous trajectories may narrow but never replace the actuator-layer envelope.
6. Outputs start disarmed. Hold `L3` and use the exact neutral `SELECT`+`START`+`A` arming chord; keep `L3` held to renew the command lease. Releasing `L3` or pressing `SELECT`+`START`+`B` disables output. On a schema-valid Joy sample, dead-man release is evaluated before timestamp and chord parsing: replayed or clock-rollback stop input may deny service, but it cannot defer revocation. Any invalid Joy sample received while armed also fault-stops; commands and lease renewals require fresh, ordered timestamps. Timestamped Joy/JointState actuator inputs reject invalid shape, non-finite values, stale or excessively future timestamps, replays, and actuator steps outside the calibrated envelope. Direct-joint entry/re-arm rotates a bounded process-lifetime epoch, announces it on relative `joint_session`, and requires the producer to echo the exact `joint-session:N` frame; zero revokes that producer session on fault or mode exit, and the source stamp must also be strictly after the controller's local grant receipt and no later than callback receipt. Each IK entry/re-arm similarly rotates a bounded, process-lifetime monotonic session epoch carried by `CurrentPositions` and `CartesianCmd`; the solver burns a newer epoch before payload validation, and actuator output must return the exact `ik-session:N` frame plus a fresh stamp. These epochs block delayed prior-session traffic while the supervised unit remains running, but they are not persistent across a complete restart and are ordering tokens rather than publisher authentication. The `Float64MultiArray` coordination messages also remain unstamped and per-command unsequenced; production messages/gateways still need source leases, explicit timestamps, monotonic command sequences, and rate/velocity/acceleration fields.
7. Wire the PCA9685 OE pin or servo power through a normally-off hardware watchdog and provide a physical E-stop. Software cleanup and I2C writes are not substitutes for a fail-off circuit.
8. Operate inside a guarded clear zone while commissioning. Test first with servo power disconnected, then with limited torque and speed.

## Launch contract

target_scripts/start_robot.sh fails closed unless:

- it is run without root privileges;
- ROS_DOMAIN_ID is explicitly assigned;
- an SROS2 keystore exists at ROS_SECURITY_KEYSTORE (default /etc/baby_robot_arm/security);
- a promoted install exists at ROBOT_INSTALL_PREFIX (default /opt/baby_robot_arm/current); and
- an administrator-managed `/var/run/baby_robot_arm` directory exists for the exclusive actuator lock; and
- every node active in the selected launch profile remains alive.

The default profile runs joystick input and the actuator controller but is intentionally unable to energize outputs until a board-specific hardware output hook is bound. `ALLOW_SOFTWARE_ONLY_OUTPUT=1` is a conspicuous bench-only override for servo-power-disconnected testing; it is not an acceptable deployed configuration. Autonomous demonstrations are disabled because the physical wrist channel order cannot be attested from repository data. Experimental IK is disabled by default because the checked-in URDF is incomplete; setting `ENABLE_EXPERIMENTAL_IK=1` with that model deliberately makes the supervised unit fail closed. A replacement IK model must provide the exact bounded revolute-joint chain `base`, `shoulder`, `elbow`, `wrist`, and `hand`, ending at a leaf/tool-center-point link.

The launcher starts Bash in privileged mode so exported functions and `BASH_ENV` cannot run before its body, rejects raw exported-function entries before any descendant shell can import them, then clears loader, Python-startup, interpreter/search-path, and ROS setup selectors at every executable-metadata boundary. It confines canonical install/security paths, enables SROS2 enforcement, uses one namespace per robot, launches installed executables directly, and terminates the complete control unit when any active child exits. The service manager must nevertheless construct a minimal allowlisted environment and own the complete launcher process group/job: the dynamic loader consumes variables such as `LD_PRELOAD` before Bash can unset them, and launcher SIGKILL/crash cannot execute a shell trap to reap orphaned children. Execute a root-owned promoted copy of the launcher, not a developer-writable workspace copy. The service manager or deployment tooling must verify root ownership, non-writable modes, private-key permissions, kill-all-on-leader-loss behavior, and a signed/hash manifest before launch; the portable runtime script does not attest those properties.

## Fault behavior

Faults must disable outputs; they must not initiate centering or parking. Parking is a separate operator-requested operation that requires a clear zone, calibrated limits, bounded speed, and a deadline. Fault-stop freezes every queued target at the last commanded output estimate; actual mode changes and autonomous producer entry/change/cancellation do the same before new authority can renew. A component crash observed by the launcher, controller disconnect, stale command, I2C error, or invalid solver result must revoke command authority and transition to the disabled state. An active-controller removal, invalid report, report-stream loss, or 250 ms report timeout irreversibly latches the HID broker for that process: it publishes a final neutral sample under the publication-order lock, ignores later reports, shuts down, and exits nonzero so launcher supervision stops the complete unit. Malformed IK input before an active solve is rejected and keeps the solver inhibited; an active solver, FK, home-workspace, or publication-invariant failure exits nonzero for the same supervised stop. Launcher loss itself relies on service-manager process-group containment. The software issues PCA9685 FULL_OFF writes, but those writes cannot protect against SIGKILL, process loss, or a failed I2C bus.

## Residual hardware and deployment work

Repository code cannot provide a physical E-stop, validate the deployed wiring or wrist channel order, create site-specific DDS credentials, assign QNX abilities, or certify collision geometry. It exposes an actuator output-disable hook, but no board-specific OE GPIO implementation is bound by default. Those controls must be implemented and tested on the target system before energized operation. Autonomous mode must remain disabled until the physical joint map is verified with power/torque limited. The current URDF must also be replaced and verified against the complete physical linkage and tool-center point before IK can be enabled; even then, this solver does not perform collision or swept-path checking.

The HID broker validates supported report lengths, framing, top-level usage, and active-report identity, but VID/PID and profile bytes are spoofable. Full report-descriptor property fingerprinting still requires genuine-device captures through QNX HIDDI. The QNX release gate must also configure the complete ROS build, run the target tests, and inspect the final ELF files for the compiler/linker hardening requested by the shared CMake policy; successful host probes alone are not evidence about shipped binaries. The supported `./build.sh` boundary launches `colcon` with a positive environment allowlist and fresh outputs. Direct standalone toolchain/build invocation is outside that contract: later build tools can again inherit ambient controls such as `PYTHONPATH` or `QCC_CONF_PATH`, so any such workflow needs an equivalent clean environment and fresh cache. The cross-toolchain necessarily consumes administrator-supplied inputs including `QNX_HOST`, `QNX_TARGET`, `ARCH`, `CPUVAR`, `CPUVARDIR`, optional `NDDSHOME`, and optional external-dependency/ROS roots. CI must provide every such input from an administrator-controlled, versioned manifest and verify the SDK/dependency snapshot rather than accepting developer-selected values as release inputs.

## Verification expectations

Every security-sensitive change should include:

- parser boundary and captured-report tests;
- malformed, oversized, NaN, and infinity message tests;
- controller-removal and stale-command tests;
- process-kill and partial-I2C-failure tests proving bounded output disable;
- launcher-leader kill tests proving that the service manager removes every process-group member;
- joint, velocity, acceleration, path, and parameter-invariant tests; and
- final QNX ELF inspection for effective PIE, RELRO/NOW, stack-canary, and fortify properties; and
- an SBOM and dependency scan for the exact promoted target image.

No static review proves the absence of all defects. Complete dynamic testing, fault injection, QNX target inspection, and guarded hardware validation before release.
