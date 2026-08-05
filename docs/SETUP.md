# NeoNav Setup Guide

This guide describes how to reproduce the current Euroboot phase-one setup on a
fresh Raspberry Pi, ESP32, and Windows dashboard PC.

## 1. Hardware Wiring

### ESP32 To L293D

| Signal | ESP32 GPIO |
| --- | ---: |
| Left EN/PWM | 25 |
| Left IN1 | 14 |
| Left IN2 | 13 |
| Right EN/PWM | 26 |
| Right IN3 | 33 |
| Right IN4 | 27 |

### ESP32 To Encoders

| Signal | ESP32 GPIO |
| --- | ---: |
| Left encoder A | 5 |
| Left encoder B | 18 |
| Right encoder A | 21 |
| Right encoder B | 19 |

Connect all grounds together: ESP32, L293D logic ground, motor supply ground,
and Raspberry Pi ground if using external power.

Use USB between the Raspberry Pi and ESP32 for the first bring-up. It gives the
Pi both the micro-ROS serial link and the ability to upload firmware.

## 2. Raspberry Pi Base Install

Target system used by this project:

- Raspberry Pi 4
- Ubuntu or Raspberry Pi OS capable of ROS 2 Jazzy
- ROS 2 Jazzy
- PlatformIO
- micro-ROS agent workspace installed at `~/microros_ws`

Install common tools:

```bash
sudo apt update
sudo apt install -y git python3-pip python3-serial
```

Install PlatformIO:

```bash
python3 -m pip install --user platformio
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
source ~/.bashrc
```

Install ROS 2 Jazzy using the official ROS documentation for your Pi OS, then
verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

Install/build the micro-ROS agent in `~/microros_ws`. The final setup should
provide:

```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent --help
```

## 3. Clone NeoNav On The Pi

```bash
cd ~
git clone https://github.com/Crostina/NeoNav.git euroboot
cd ~/euroboot
```

Copy the helper script into the Pi home folder:

```bash
cp tools/start_euroboot_pi.sh ~/start_euroboot.sh
chmod +x ~/start_euroboot.sh
```

## 4. Build And Upload ESP32 Firmware

Plug the ESP32 into the Pi by USB and check its device:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

The current scripts assume `/dev/ttyUSB0`. If your ESP32 appears elsewhere,
either adjust the command or run the helper with `ESP32_PORT` set.

Build and upload:

```bash
cd ~/euroboot/firmware
pio run -e euroboot_esp32
pio run -e euroboot_esp32 -t upload
```

If upload fails because the ESP32 is not entering bootloader mode, hold `BOOT`,
tap `EN/RESET`, start upload, then release `BOOT` when PlatformIO begins writing.
Many dev boards upload automatically without button presses.

## 5. Bring Up The Robot Stack

Use the one-command helper:

```bash
~/start_euroboot.sh restart
```

Expected output includes:

```text
/odom/unfiltered publisher is active
/odom/unfiltered samples are flowing
/tf is publishing
controller_server
bt_navigator
```

Useful commands:

```bash
~/start_euroboot.sh status
~/start_euroboot.sh logs
~/start_euroboot.sh stop
```

Manual checks:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic hz /odom/unfiltered
ros2 topic echo /tf --once
ros2 lifecycle get /controller_server
```

Expected:

- `/odom/unfiltered` near `50 Hz`
- `/tf` publishes `odom -> base_footprint`
- `controller_server` reports `active [3]`

## 6. Start The Windows Dashboard

On the PC:

```powershell
cd C:\Users\Golde\Documents\PlatformIO\Projects\EUROBOOT
python tools\euroboot_dashboard.py
```

Set:

- Pi host: the current Raspberry Pi IP address
- TCP port: `8765`

The dashboard can:

- show live odometry
- clear the local dashboard odom origin
- edit robot geometry
- send geometry to the bridge
- add/edit waypoints
- set optional final heading per waypoint
- send a Nav2 mission
- save selected debug data

## 7. Debugging

### ESP32 Does Not Appear

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Try a different USB cable. Some cables are charge-only.

### micro-ROS Agent Runs But No Odom

```bash
tail -120 /tmp/euroboot/micro_ros_agent.log
ros2 topic list
ros2 topic hz /odom/unfiltered
```

If `/odom/unfiltered` appears late, rerun:

```bash
~/start_euroboot.sh restart
```

### Nav2 Does Not Activate

Check TF:

```bash
ros2 topic echo /tf --once
tail -120 /tmp/euroboot/dashboard_bridge.log
tail -120 /tmp/euroboot/nav2.log
```

Nav2 needs `odom -> base_footprint`. The dashboard bridge publishes that TF from
the ESP32 `/odom/unfiltered` messages.

### Pi Randomly Resets

```bash
vcgencmd get_throttled
```

If the output contains `0x50000`, the Pi has seen undervoltage/throttling. Use a
stronger Pi power supply and a better USB-C cable before continuing navigation
tests.

## 8. Tuning Notes

The current robot works, but it is mechanically limited. Navigation quality is
affected by:

- wheel diameter mismatch
- wheel slip
- loose chassis geometry
- cheap N20 motor variation
- L293D voltage drop and weak current delivery
- battery voltage sag

Before increasing speed, first verify:

- both wheels report reasonable encoder direction
- straight motion keeps `y` error small
- in-place turns rotate with both wheels in opposite directions
- `/odom/unfiltered` stays near `50 Hz`
- Pi power is stable

Saved runs in `test_results/` and `debug_runs/` document the tuning process used
for the current baseline.

The repeatable field-tuning mission is scripted:

```powershell
python tools\mission_tune_runner.py --host <pi-ip> --label my_test --timeout 70 --pause-after 8
```

It sends this 4-waypoint path:

```text
1: x=0.754 y=0.733
2: x=1.504 y=0.002
3: x=0.746 y=-0.510
4: x=-0.008 y=-0.002 final yaw=0 deg
```

Each run writes a CSV and JSON into `debug_runs/`, including final error,
cross-track error, stop samples, oscillation count, and the Nav2/IMU parameters
used. The current best baseline is:

```text
runtime wheel_diameter_m=0.04
runtime wheel_base_m=0.15216
desired_linear_vel=0.28
lookahead_dist=0.20
min_lookahead_dist=0.10
max_lookahead_dist=0.40
lookahead_time=0.50
min_approach_linear_velocity=0.05
approach_velocity_scaling_dist=0.30
regulated_linear_scaling_min_radius=0.36
regulated_linear_scaling_min_speed=0.055
max_angular_accel=3.30
xy_goal_tolerance=0.05
yaw_goal_tolerance=6.28
```

## 9. Pixhawk Yaw Input

The prototype Pixhawk is connected from TELEM2 to the Raspberry Pi UART:

| Pixhawk TELEM2 | Raspberry Pi |
| --- | --- |
| TX | RXD / GPIO15 / physical pin 10 |
| RX | TXD / GPIO14 / physical pin 8 |
| GND | GND |

Do not connect Pixhawk 5V to the Raspberry Pi 5V rail unless the power
architecture has been checked. The UART signal level must be 3.3 V.

The Pi UART should be enabled and free from login-console use:

```bash
ls -l /dev/serial0
systemctl is-active serial-getty@ttyAMA0.service serial-getty@serial0.service
```

Expected on the current Pi:

```text
/dev/serial0 -> ttyAMA0
inactive
inactive
```

Probe the Pixhawk:

```bash
python3 tools/pixhawk_mavlink_yaw_probe.py --port /dev/serial0 --baud 115200 --seconds 8
```

Known-good result from the prototype:

```text
ATTITUDE yaw ~= -9.7 deg
GLOBAL_POSITION_INT heading ~= 350.3 deg
ATTITUDE rate ~= 10 Hz
```

This is integrable into odometry, but the yaw must be treated carefully:

- Pixhawk `ATTITUDE.yaw` is an attitude estimate, usually absolute or
  compass-referenced, and it follows the MAVLink/Pixhawk yaw convention rather
  than ROS ENU yaw.
- Wheel odometry yaw is local and starts wherever the robot was when odom was
  cleared.
- The bridge should store a yaw offset when odometry is cleared:
  `local_yaw = normalize(pixhawk_yaw - pixhawk_yaw_at_origin)`.
- Current integration uses Pixhawk yaw only to correct heading in Pi-side
  `/odom` and TF, while keeping ESP32 wheel odometry for `x`, `y`, and linear
  speed.
- The tested dashboard default is Pixhawk gyro-integrated yaw with sign `-1`,
  weighted `0.8`, plus encoder yaw weighted `0.2`. Uncheck `Use Pixhawk yaw` to
  return to encoder-only heading.
- Geometry changes from the dashboard now affect the bridge-published `/odom`
  scaling and TF immediately. ESP32 control constants still require firmware
  rebuild/upload until a runtime firmware config service is added.
- The more standard ROS 2 solution is to publish Pixhawk IMU/yaw and fuse it
  with wheel odometry using `robot_localization` EKF, producing the final
  `/odom` used by Nav2.
