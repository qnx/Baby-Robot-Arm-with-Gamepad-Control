# Joy teleop security notes

The node now publishes the relative topic `joy` with exactly 6 axes and 12
buttons, emits at most the latest valid HID sample once per 20 ms cycle (reports
within a cycle coalesce), accepts one length- and framing-validated controller
report at a time and requires neutral controls on initial activation. A 250 ms
report timeout, active removal, stream overflow/resume, invalid active report,
or exclusive-attachment conflict irreversibly latches a terminal source fault.
That transition establishes a neutral final command state after any earlier
sample, rejects all later fresh reports, shuts the broker down, and returns
failure so the launcher stops the complete control unit. Recovery requires a
fresh supervised start; a
second controller is ignored while one is active and is never auto-promoted.

Fixed-offset parsing accepts only these exact profiles:

| VID:PID | Mode | Callback length | Framing |
| --- | --- | ---: | --- |
| `046d:c21d` | F310 XInput | 20 | report ID `0x00`, size `0x14` (20 decimal) |
| `046d:c21f` | F710 XInput | 20 | report ID `0x00`, size `0x14` (20 decimal) |
| `046d:c216` | F310 DirectInput | 8 | no leading report ID; hat is 0–8 |
| `046d:c219` | F710 DirectInput | 7 | no leading report ID; hat is 0–8 |

XInput bytes 14 through 19 are not consumed by the command mapping. Tests sweep
every value at each byte and assert that axes and buttons are unchanged. Capture
genuine-device reports on the target before deciding whether these bytes can be
required to be zero without rejecting a supported Logitech/QNX combination.
The same command-inert sweep covers F310 DirectInput bytes 6–7 and F710
DirectInput byte 6.
XInput framing also rejects impossible up+down or left+right pairs while
retaining all four adjacent diagonal combinations.
L3 is XInput/DirectInput button bit 6 and is published as ROS button 10, the
actuator-side operator dead-man; bit 7 is R3. XInput signed little-endian Y
values are safely inverted (including saturation for `-32768`) so physical up
and down have the same parser sign as both DirectInput profiles.

These code controls do not authenticate USB hardware or ROS participants:

- VID/PID, descriptors, product strings, and serial strings are spoofable. Lock
  down physical USB ports and QNX HIDDI access. Use a trusted hardware gateway
  if the deployment requires device authentication.
- Full report-descriptor property fingerprinting (including button counts and
  logical ranges) remains a target release gate pending genuine QNX HIDDI
  descriptor captures. The current checks must not be described as device
  authentication or complete descriptor validation.
- Launch each robot in a distinct namespace and enforce DDS/SROS2 identities and
  deny-by-default publish permissions. Namespace and `ROS_DOMAIN_ID` separation
  alone are not authentication.
- The actuator consumer must enforce its own freshness timeout, exclusive
  control lease, motion-rate limits, held dead-man control, and hardware E-stop.
- Validate the 250 ms report timeout and the exact profiles above with captured
  F310/F710 reports on the deployed QNX/HIDDI versions before operating hardware.

Target validation must also exercise exclusive-attach contention; terminal
nonzero exit on unplug, report stall, and HIDDI overflow/resume; callback and
disconnect ordering; proof that no fresh sample follows the final neutral under
forced timer-publish/fault contention; supervisor teardown of the other active
nodes; and the full namespaced ROS publisher-to-actuator path. These behaviors
require the deployed QNX HID service, ROS installation, controllers, and robot
hardware.

The parser tests are host-buildable without QNX headers:

```sh
cmake -S src/joy_teleop_hiddi/test -B /tmp/joy-parser-test
cmake --build /tmp/joy-parser-test
ctest --test-dir /tmp/joy-parser-test --output-on-failure
```

They can also be run without CMake:

```sh
cc -std=c99 -Wall -Wextra -Wpedantic -Werror \
  -Isrc/joy_teleop_hiddi/test/stubs \
  -Isrc/joy_teleop_hiddi/include \
  src/joy_teleop_hiddi/test/test_parser.c \
  src/joy_teleop_hiddi/src/parser.c \
  -o /tmp/joy_parser_unit_tests
/tmp/joy_parser_unit_tests
```
