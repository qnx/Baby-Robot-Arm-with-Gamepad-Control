# physical-ai-ui

`physical-ai-ui` is a standalone QNX SDP 8.0 GTK application for a live camera,
physical hand interaction, and robot control. It contains the former physical
experience only: robot and wrist controls, editable ROI, hand selection,
joint/IK overlays, screensavers, QR flow, and their QPP inputs.

The repository contains only the application and does not depend on
`hailo-joint-ui` or a shared source library.

## Build and run

On QNX, from the project directory:

```sh
make
./physical-ai-ui
make clean
```

## QPP data source

See [docs/JointDevelopment.md](docs/JointDevelopment.md) for the app's data
contracts.

The executable is `physical-ai-ui`, its GTK application ID is
`com.qnx.physical-ai-ui`, and settings are stored in
`.physical-ai-ui-settings.json`. Run `./physical-ai-ui --help` for options.

## License

Apache-2.0. See [LICENSE](LICENSE).
