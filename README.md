# NeoNav

NeoNav is the Euroboot team's experimental differential-drive robot navigation
stack. It started from `linorobot2_hardware` and adds the firmware config,
Raspberry Pi ROS 2 bring-up scripts, Nav2 tuning, and a Windows Python dashboard
used to drive and debug a small ESP32-based test robot.

This repository is meant to be a reproducible phase-one baseline: per-wheel
speed control on the ESP32, wheel odometry over micro-ROS, a Raspberry Pi 4 ROS 2
brain, and a desktop dashboard for waypoint tests.

## Hardware

Current prototype:

- Raspberry Pi 4 running ROS 2 Jazzy
- ESP32 DevKit connected to the Pi over USB serial
- L293D motor driver
- Two N20 quadrature encoder motors
- Differential-drive chassis

Motor and encoder pinout:

| Function | ESP32 GPIO |
| --- | ---: |
| Left encoder A | 5 |
| Left encoder B | 18 |
| Right encoder A | 21 |
| Right encoder B | 19 |
| Left L293D EN/PWM | 25 |
| Left L293D IN1 | 14 |
| Left L293D IN2 | 13 |
| Right L293D EN/PWM | 26 |
| Right L293D IN3 | 33 |
| Right L293D IN4 | 27 |

Firmware geometry is stored in `config/custom/euroboot_esp32_config.h`:

- wheel diameter: `0.04586 m`
- wheel distance / track width: `0.15216 m`
- encoder CPR: `1400`
- configured max wheel speed: `315 rpm`

The current Pi/dashboard runtime odometry scale uses wheel diameter `0.044 m`,
track width `0.15216 m`, encoder CPR `1400`, and max wheel speed `315 rpm`.
That runtime value is what was used for the latest field tuning runs.

## Architecture

```text
Windows dashboard
        |
        | TCP JSON, port 8765
        v
Raspberry Pi 4, ROS 2 Jazzy
        |
        | /cmd_vel, /odom/unfiltered, /tf, Nav2 FollowPath
        v
micro-ROS agent over USB serial, 921600 baud
        |
        v
ESP32 firmware
        |
        | PI wheel speed control + PWM
        v
L293D motor driver -> N20 motors + encoders
```

The ESP32 is the low-level realtime controller. It subscribes to `/cmd_vel`,
converts robot velocity to left/right wheel RPM, runs one PI controller per
wheel, drives the L293D, and publishes `/odom/unfiltered` at about 50 Hz.

The Pi bridge publishes the Nav2-facing `/odom` and `odom -> base_footprint` TF.
By default, `/odom` keeps encoder x/y distance but fuses yaw as `80%` Pixhawk
gyro-integrated yaw and `20%` encoder yaw. MAVLink yaw is converted from the
Pixhawk/NED convention to ROS yaw with sign `-1`. The dashboard can switch back
to encoder-only yaw at runtime.

The Raspberry Pi runs:

- `micro_ros_agent` for the ESP32 serial link
- `tools/euroboot_ros_bridge.py` for dashboard TCP control and `odom -> base_footprint` TF
- a minimal Nav2 stack configured by `tools/nav2_minimal_odom_params.yaml`

The Windows dashboard runs without ROS installed and talks only to the Pi bridge.

## Repository Layout

- `firmware/` - PlatformIO firmware based on Linorobot2 hardware
- `config/custom/euroboot_esp32_config.h` - Euroboot ESP32 motor, encoder, geometry, and PI constants
- `tools/euroboot_dashboard.py` - Windows Tkinter dashboard
- `tools/euroboot_ros_bridge.py` - Pi TCP-to-ROS bridge and mission runner
- `tools/field_test_client.py` - scripted one-shot field test logger
- `tools/mission_tune_runner.py` - scripted 4-waypoint mission tuner/logger
- `tools/start_euroboot_pi.sh` - one-command Pi bring-up helper
- `tools/nav2_minimal_odom_*.py|yaml` - minimal Nav2 launch and tuning
- `tools/esp32_*` - motor/encoder calibration and isolation tests
- `docs/SETUP.md` - setup guide for a fresh PC/Pi/ESP32
- `docs/EUROBOOT_BRINGUP.md` - development notes and tuning history
- `test_results/` and `debug_runs/` - saved tuning runs
- `docs/LINOROBOT2_HARDWARE_README.md` - original upstream README kept for reference

## Quick Start

On the Raspberry Pi:

```bash
~/start_euroboot.sh restart
~/start_euroboot.sh status
~/start_euroboot.sh logs
```

On the Windows PC, from this repository root:

```powershell
python tools\euroboot_dashboard.py
```

In the dashboard, connect to the Pi host on TCP port `8765`.

If the robot was just powered or reset, the ESP32 may take a little while to
appear through micro-ROS. A good Pi-side check is:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /odom/unfiltered
ros2 lifecycle get /controller_server
```

Expected:

- `/odom/unfiltered` publishes near `50 Hz`
- `controller_server` is `active [3]`

## Build And Upload Firmware

Build on Linux, WSL, or the Raspberry Pi. The micro-ROS PlatformIO build is more
reliable there than on native Windows.

```bash
cd ~/euroboot/firmware
pio run -e euroboot_esp32
pio run -e euroboot_esp32 -t upload
```

Then run:

```bash
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -b 921600 -v2
```

If the ESP32 appears as a different device, change `/dev/ttyUSB0` accordingly.

## Documentation

Start here for a clean setup:

- [Setup Guide](docs/SETUP.md)
- [Bring-Up Notes](docs/EUROBOOT_BRINGUP.md)

Pixhawk yaw probe:

```bash
python3 tools/pixhawk_mavlink_yaw_probe.py --port /dev/serial0 --baud 115200
python3 tools/pixhawk_yaw_zero_test.py --port /dev/serial0 --baud 115200 --deadband 0.5 --heartbeat 0
python3 tools/pixhawk_xy_zero_test.py --port /dev/serial0 --baud 115200 --source local --deadband 0.01 --heartbeat 0
python3 tools/pixhawk_accel_distance_test.py --port /dev/serial0 --baud 115200 --calibration 3 --deadband 0.01 --heartbeat 0
```

On the prototype, Pixhawk TELEM2 is readable on the Raspberry Pi UART as
MAVLink 2 at `115200` baud. The probe decodes `ATTITUDE` yaw and
`GLOBAL_POSITION_INT` heading without requiring `pymavlink`. The zero-test
scripts use the first valid sample as the local origin, then print only when
yaw or XY distance changes enough.

The accelerometer distance script is experimental. It double-integrates IMU
acceleration after a short still calibration, so drift is expected and it should
not be used as final odometry without an external correction source.

Useful diagnostics:

```bash
vcgencmd get_throttled
ros2 topic list
ros2 topic hz /odom/unfiltered
ros2 topic echo /tf --once
ros2 lifecycle get /controller_server
```

`throttled=0x50000` means the Pi has seen undervoltage/throttling. Fix the power
supply or cable before trusting navigation tests.

## Current Status

The phase-one stack is working on the small prototype:

- ESP32 motor/encoder firmware runs with micro-ROS serial
- wheel odometry publishes to ROS 2
- dashboard receives live odometry and can clear its local odom origin
- Nav2 FollowPath can execute waypoint missions in a small test area
- debug runs can be saved for later analysis

The current field baseline started from a 15-run 4-waypoint tuning session
(`debug_runs/mission_20260806_155820_turn09_v024_l020_decisive.*`) and was then
tightened with a 0.5 m square corner test
(`debug_runs/mission_20260806_164842_small_square_clean02_tighter_xy.*`). The
tuned Nav2 profile uses `0.24 m/s` desired speed, `0.20 m` lookahead, `0.30 m`
approach scaling distance, `0.02 m` XY tolerance, and Pixhawk gyro yaw fused at
`0.8` with encoder yaw at `0.2`.

The Pi bridge uses an explicit stop-turn-go controller for waypoint heading
changes. Same-waypoint position correction after a final heading turn is
disabled by default because it caused corner recovery arcs. The turn controller
also applies a small encoder-vx balance correction during forced spins to keep
the actual left/right wheel speeds close to equal magnitude.

The prototype is still limited by cheap motors, the L293D driver, wheel slip,
chassis imbalance, and geometry mismatch. Future upgrades should include better
motor drivers, cleaner mechanics, external yaw from Pixhawk or IMU fusion, and
camera-based pose correction with ArUco or AprilTag markers.
