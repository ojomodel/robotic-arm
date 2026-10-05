# ESP32 firmware

The real controller source is in [`UnityRobotBridge`](UnityRobotBridge/). It receives joint commands from the Unity application over USB serial and drives five servo channels through a PCA9685. The source files are copied unchanged from the robot project.

| File | Purpose |
| --- | --- |
| `UnityRobotBridge.ino` | ESP32 setup, USB serial, read-only calibration loading, I2C and PWM output |
| `BridgeState.h` | CF1 parser, session state, bounds, watchdog and joint command interpolation |
| `TimestampedTrajectory.h` | Timestamped five-channel command buffering and smoothing |
| `StartupHome.h` | This robot's stored first-enable command pose |
| `tests/` | Existing host-side C++ tests and mock hardware interfaces |

## Build

The project was compiled with these installed dependencies:

- Espressif Arduino ESP32 core **3.3.11**, board target `esp32:esp32:esp32`.
- Adafruit PWM Servo Driver Library **3.0.3**.
- Adafruit BusIO **1.17.4**.
- `Wire`, `Preferences` and `esp_system` from the ESP32 core.

Open `UnityRobotBridge/UnityRobotBridge.ino` in Arduino IDE, install those board/library dependencies, and select the matching ESP32 board. With Arduino CLI configured, compile from the repository root:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 --build-path build/firmware firmware/UnityRobotBridge
```

This is a compile command, not an upload command. No compiled binaries or private board settings are included.

## Connection and behavior

- USB serial: **115200 baud, 8N1**, newline-terminated ASCII.
- I2C: **GPIO21 SDA**, **GPIO22 SCL**, PCA9685 address **0x40**.
- Channels **0–4**: base, shoulder, elbow, wrist pitch, gripper.
- PWM: **50 Hz**, with the project's existing **600–2400 microsecond** conversion and **27 MHz** oscillator conversion reference. These are this build's calibration conventions, not a universal servo setup.
- Xbox input is handled by the PC/Unity application. This firmware does not use Bluetooth or Wi-Fi.

The board starts with PWM outputs off. A deliberate `ARM` command enables them. `HOLD` stops the command trajectory and retains holding pulses; `RELEASE` disables PWM. A **500 ms** command watchdog performs a hold. Reconnecting does not automatically arm the robot.

`POSE` and `TPOSE` require session limits. All five proposed positions are checked before acceptance. `TPOSE` adds timestamped buffering with a 40 ms delay and 40 ms smoothing window. The configured speed ceiling is **400 nominal command degrees/second**; it is not a measured mechanical speed, torque, payload or accuracy rating.

The firmware reports commanded angles, **not measured shaft positions**. There are no joint encoders or servo-supply sensors in this implementation.

## Calibration and startup pose

Saved calibration values are read from ESP32 NVS namespace `arm-neutral`, keys `neutral0`–`neutral4`, without writing them. `StartupHome.h` separately holds this arm's first-enable pose: `[90.8, 92.9, 118.5, 60.9, 180]`. The calibration fallback is `[90.8, 92.9, 104.5, 60.9, 180]`. Neither array is a measured physical pose or a suitable default for every assembled robot.

The Unity model uses CAD-relative joint angles and its own model-to-servo mapping. Do not send model angles directly as servo commands. A later user-saved Unity Home-button pose is independent of this firmware startup pose.

Support the arm and keep external servo power off during wiring, upload or reset. Before deliberately arming, check the physical pose and clearance against the displayed command positions. The first `ARM` writes those positions immediately; subsequent rate limiting does not prevent an initial acquisition movement. `HOLD` retains torque and `RELEASE` can make joints loose; neither switches the external power supply.

See [electronics](../electronics/README.md), the [CF1 protocol](UnityRobotBridge/PROTOCOL.md), and [host tests](UnityRobotBridge/tests/README.md).

## Development status

The bridge has been used with the physical robot through Unity. Existing project records document native ESP32 compilation and device command checks with servo power off; those checks do not measure physical accuracy or speed. This repository preparation does not include a new flash or physical test.

During repeated testing, the controller occasionally needed a manual reset when reconnecting. The cause was not conclusively identified. Connection robustness, calibration, coordinated motion quality and repeatability remain development work.
