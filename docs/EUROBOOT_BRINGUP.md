# Euroboot Linorobot2 Bring-Up

## Current State

- Base repo: `linorobot/linorobot2_hardware`, branch `jazzy`.
- Active firmware environment: `euroboot_esp32`.
- Custom hardware config: `config/custom/euroboot_esp32_config.h`.
- Local dashboard prototype: `tools/euroboot_dashboard.py`.
- Previous Euroboot firmware/dashboard backup:
  `C:\Users\Golde\Documents\PlatformIO\Projects\EUROBOOT_backup_20260728_151933`.

## Local Dashboard Prototype

The dashboard runs on Windows as a normal Python/Tkinter app. It talks to the
robot through `tools/euroboot_ros_bridge.py`, a small TCP JSON bridge that runs
on the Raspberry Pi and connects to ROS 2/Nav2.

The dashboard does not require ROS 2 on Windows.

### Start The Pi Bridge

The Pi now has a helper script in the maker home folder:

```bash
~/start_euroboot.sh restart
~/start_euroboot.sh status
~/start_euroboot.sh logs
~/start_euroboot.sh stop
```

The script starts the micro-ROS agent first, resets the ESP32 while the agent is
already listening, waits for `/odom/unfiltered`, starts the dashboard bridge,
waits for `odom -> base_footprint` TF, then starts Nav2. This avoids the common
half-started state where the ESP32 is online late but Nav2 has already failed
activation.

Manual bring-up is still possible. First make sure the normal robot stack is
running on the Pi:

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -b 921600 -v2
```

In another terminal, start the dashboard bridge. This bridge now also publishes
the required `odom -> base_footprint` TF, so do not run the old standalone
`odom_to_tf_bridge.py` at the same time:

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
python3 ~/euroboot/tools/euroboot_ros_bridge.py --host 0.0.0.0 --port 8765
```

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ~/euroboot/tools/nav2_minimal_odom_launch.py
```

### Start The Windows Dashboard

Run it from the project root:

```bash
python tools/euroboot_dashboard.py
```

Current features:

- reads default wheel diameter, wheel distance, CPR, and max RPM from
  `config/euroboot_dashboard_settings.json`, falling back to
  `config/custom/euroboot_esp32_config.h`
- draws a 2D odom-frame grid
- draws the robot body, wheels, and forward direction arrow
- shows a heading compass
- receives live odometry from the Pi bridge
- draws the live odometry trail while a mission is running
- lets you click the map to add waypoints
- lets each waypoint leg choose `forward` or `backward` travel
- lets you draw a continuous track and send it as one Nav2 path
- sends waypoint missions to Nav2's `/follow_path` controller action through
  the Pi bridge
- sends stop commands through `/cmd_vel`
- clears dashboard odometry by resetting the bridge's local odom origin
- saves dashboard geometry/settings across app restarts
- can send geometry to the Pi bridge with `Apply To Robot`
- can save opt-in debug logs for selected missions

The Clear Odom button resets the dashboard/bridge local origin. It does not
reset the raw ESP32 `/odom/unfiltered` counter in firmware.

The current dashboard waypoint mode sends straight path segments directly to
Nav2's controller server. This is faster and more stable for phase-one testing,
but it is not obstacle-aware global planning yet.

Before each straight path segment, the Pi bridge now aligns the robot heading
using an in-place turn (`linear.x = 0`, `angular.z != 0`). This commands one
wheel forward and the other wheel backward instead of doing a slow one-wheel
arc, then hands the straight segment to Nav2 FollowPath.

The in-place turn is intentionally stronger than the first version:

- heading tolerance: about `0.03 rad`
- angular command: `max(0.35, min(2.00, abs(error) * 2.8))`
- timeout: `7 s`
- short settle after alignment before starting the straight segment

`Apply To Robot` stores runtime geometry on the Pi at:

```bash
~/euroboot/config/euroboot_runtime_geometry.json
```

This keeps the dashboard and bridge consistent. It does not flash new ESP32
firmware constants; changing the firmware odometry geometry still requires a
firmware rebuild/upload.

When `Save Debug Data` is checked, each dashboard mission writes:

```text
debug_runs/euroboot_debug_YYYYMMDD_HHMMSS_idNNNN.csv
debug_runs/euroboot_debug_YYYYMMDD_HHMMSS_idNNNN.json
```

The CSV contains live odometry, raw odometry, speed, angular speed, and mission
state. The JSON contains geometry, waypoint list, timestamp, and debug ID.

Waypoint editing:

- map clicks add checkpoints
- selecting a checkpoint fills editable `x`, `y`, `Final theta deg`, and
  `Travel mode` fields
- editing `x` or `y` moves the checkpoint on the map
- `Travel mode=backward` keeps the checkpoint target position but drives the
  segment in reverse, with robot heading opposite the travel direction
- `Final theta deg` is optional; leave it empty when final heading does not
  matter
- if `Final theta deg` is set, the bridge performs a final in-place alignment
  after reaching that checkpoint

## ESP32 Motor And Encoder Mapping

Linorobot2 uses `MOTOR1` as the left wheel and `MOTOR2` as the right wheel for a 2WD differential robot.

### Left Wheel / MOTOR1

- Encoder A: GPIO5
- Encoder B: GPIO18
- L293D EN/PWM: GPIO25
- L293D IN_A: GPIO14
- L293D IN_B: GPIO13

### Right Wheel / MOTOR2

- Encoder A: GPIO21
- Encoder B: GPIO19
- L293D EN/PWM: GPIO26
- L293D IN_A: GPIO33
- L293D IN_B: GPIO27

## Raspberry Pi To ESP32 Link

### Recommended For First Bring-Up: USB

Use a normal USB cable from the Raspberry Pi USB port to the ESP32 USB port.

- No GPIO UART wiring is needed.
- The ESP32 appears on the Pi as `/dev/ttyUSB0`, `/dev/ttyUSB1`, or `/dev/ttyACM0`.
- This is safest for upload/debug because Linorobot2 uses `Serial` for micro-ROS.

Agent command on the Pi:

```bash
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -b 921600
```

Change `/dev/ttyUSB0` to the actual ESP32 device if needed.

### Direct GPIO UART Option

Only use this if USB is not practical.

For the current unmodified Linorobot2 firmware, `Serial` is ESP32 UART0:

- Raspberry Pi GPIO14 / TXD, physical pin 8 -> ESP32 RX0 / GPIO3
- Raspberry Pi GPIO15 / RXD, physical pin 10 -> ESP32 TX0 / GPIO1
- Raspberry Pi GND, physical pin 6 -> ESP32 GND
- Do not connect Pi 5V to ESP32 5V unless power architecture is intentional and checked.

Pi setup:

```bash
sudo raspi-config
```

Use Interface Options -> Serial Port:

- login shell over serial: `No`
- hardware serial port: `Yes`

Then run:

```bash
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/serial0 -b 921600
```

UART0 shares ESP32 boot/upload/debug traffic, so USB is still the better first test.

## Build On Raspberry Pi / Linux

The micro-ROS PlatformIO build currently fails on Windows with:

```text
'.' is not recognized as an internal or external command
```

Build this firmware on the Raspberry Pi, WSL, or another Linux machine.

```bash
cd ~/euroboot/firmware
pio run -e euroboot_esp32
```

Upload over USB:

```bash
pio run -e euroboot_esp32 -t upload
```

Run the agent:

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -b 921600
```

Verified on the Raspberry Pi at `192.168.137.225`: build and upload succeeded over `/dev/ttyUSB0`, and the agent established a session with `euroboot_base_node`.

## Phase 1 Control Model

The ESP32 is the low-level controller. The Raspberry Pi publishes ROS 2 `/cmd_vel`, and the ESP32 firmware:

- receives `/cmd_vel` through the micro-ROS agent over USB serial at `921600` baud
- converts linear/angular velocity to left/right target wheel RPM using differential-drive geometry
- reads both quadrature encoders at the 50 Hz control timer
- runs one independent PI loop per wheel
- sends signed PWM to the L293D direction pins and EN pins
- publishes encoder odometry on `/odom/unfiltered`
- stops the motors if no velocity command arrives for 200 ms

Current geometry:

- wheel diameter: `0.04586 m`
- wheel circumference: about `0.14407 m`
- left/right wheel distance: `0.15216 m`
- encoder counts per wheel revolution: `1400`
- configured max target wheel speed: `315 rpm`

Current phase-one controller additions:

- ESP32 still runs Linorobot2's independent per-wheel PI controller.
- Euroboot adds a removable feedforward layer before sending PWM to the L293D.
- Current feedforward values from the 2026-08-04 off-ground sweep:
  - left forward/reverse: `0 + 0.00 * rpm`
  - right forward: `30 + 0.25 * rpm`
  - right reverse: `6 + 0.16 * rpm`
- These are only a starting point. They were tuned off-ground at `0.10 m/s`, so ground tests will need another pass.

Useful command-to-wheel examples:

- `linear.x = 0.05 m/s`, `angular.z = 0.0 rad/s` -> left/right about `20.8 rpm`
- `linear.x = 0.10 m/s`, `angular.z = 0.0 rad/s` -> left/right about `41.6 rpm`
- `linear.x = 0.10 m/s`, `angular.z = 1.0 rad/s` -> left about `10.0 rpm`, right about `73.3 rpm`
- `linear.x = 0.0 m/s`, `angular.z = 1.0 rad/s` -> left about `-31.7 rpm`, right about `31.7 rpm`

## 2026-08-04 Off-Ground Forward Tests

All tests commanded `1.0 m` of encoder odometry at `0.10 m/s`.

| Firmware | Final odom distance | Lateral odom drift | Yaw drift |
| --- | ---: | ---: | ---: |
| no feedforward | `1.0164 m` | `0.3605 m` | `-18.18 deg` |
| first right-forward feedforward | `1.0157 m` | `0.1623 m` | `-11.31 deg` |
| current feedforward | `1.0114 m` | `0.0461 m` | `-6.63 deg` |

Test command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" -r 10
```

## 2026-08-04 Nav2 Controller Tuning

The initial Nav2 setup used DWB. It worked, but the robot corrected most of
the lateral error late in the motion. The current tuned setup uses Nav2's
Regulated Pure Pursuit controller instead.

Current selected controller values in `tools/nav2_minimal_odom_params.yaml`:

- controller plugin: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- desired linear velocity: `0.14 m/s`
- lookahead distance: `0.18 m`
- min/max lookahead: `0.08 m` / `0.32 m`
- approach velocity scaling distance: `0.20 m`
- min approach velocity: `0.05 m/s`
- yaw goal tolerance: `0.20 rad`
- rotate-to-heading: disabled

Ground 1 m relative-goal comparison:

| Setup | Status | Duration | Forward | Final y error | Max y error | Yaw drift |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| DWB initial | succeeded | `13.38 s` | `0.9666 m` | `-0.0170 m` | `0.0751 m` | `2.65 deg` |
| RPP conservative | succeeded | `16.40 s` | `0.9647 m` | `0.0001 m` | `0.0333 m` | `1.01 deg` |
| RPP fast/current | succeeded | `10.92 s` | `0.9737 m` | `-0.0081 m` | `0.0447 m` | `8.44 deg` |

An attempted balanced tune with tighter yaw tolerance and rotate-to-heading
failed before movement because Nav2 timed out waiting for `compute_path_to_pose`.
The active configuration was reverted to the successful RPP fast/current setup.

## 2026-08-04 Off-Ground High-Speed Tuning

For off-ground speed tuning, the dashboard bridge was changed to send missions
through Nav2's `/follow_path` action directly. This avoids planner/BT action
timeouts and tunes the controller behavior itself.

The active off-ground controller values are now:

- desired linear velocity: `0.28 m/s`
- controller frequency: `15 Hz`
- lookahead distance: `0.30 m`
- min/max lookahead: `0.12 m` / `0.55 m`
- lookahead time: `0.75 s`
- min approach velocity: `0.09 m/s`
- approach velocity scaling distance: `0.12 m`
- max angular acceleration: `2.6 rad/s^2`

Off-ground 1 m `/follow_path` comparison:

| Setup | Status | Duration | Forward | Final y error | Max y error | Yaw drift | Avg speed |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| RPP `0.14 m/s` via `NavigateToPose` | succeeded | `13.87 s` | `1.0569 m` | `-0.0242 m` | `0.1240 m` | `-5.29 deg` | `0.090 m/s` |
| RPP `0.21 m/s` via `/follow_path` | succeeded | `5.73 s` | `0.9694 m` | `-0.0042 m` | `0.0080 m` | `-3.76 deg` | `0.178 m/s` |
| RPP `0.28 m/s` via `/follow_path` | succeeded | `4.42 s` | `0.9866 m` | `-0.0057 m` | `0.0099 m` | `0.42 deg` | `0.236 m/s` |

This high-speed tune is validated off-ground only. Re-test cautiously on the
floor before using it as a competition speed.

## 2026-08-04 Dashboard Mission Behavior

The dashboard bridge sends each checkpoint as its own guarded `/follow_path`
goal. An attempted continuous multi-checkpoint path was rejected because Nav2's
controller could report success early on sharp paths, especially when the path
returned close to a previous segment.

After every Nav2 success, the bridge now checks the real odometry distance to
the target waypoint. If Nav2 reports success while the robot is still more than
about `0.08 m` away, the bridge retries that waypoint instead of reporting
`mission complete`.

The heading-turn loop also holds zero command while inside tolerance before it
declares the turn stable. This prevents the bridge from nudging the robot back
out of tolerance during the final settle check.

Pre-turn alignment is non-fatal for normal checkpoints. If it cannot settle, the
bridge logs the problem and lets Nav2 continue, instead of stopping the whole
mission in the middle.

Automatic pre-turning is now reserved for larger heading errors, about
`1.20 rad`, instead of triggering before nearly every close checkpoint. In the
1 m box tests this made short waypoint missions smoother and faster while still
keeping the in-place turn available for large heading changes.

The active tight-box Nav2 tune is:

- desired linear velocity: `0.22 m/s`
- lookahead distance: `0.14 m`
- min/max lookahead: `0.07 m` / `0.28 m`
- lookahead time: `0.45 s`
- min approach velocity: `0.05 m/s`
- approach velocity scaling distance: `0.20 m`
- XY goal tolerance: `0.06 m`
- yaw goal tolerance: `6.28 rad`

The yaw tolerance is intentionally wide because normal dashboard checkpoints do
not require a final heading. Use the waypoint `Final theta deg` field when the
robot must finish with a specific orientation.

Ground dense-waypoint comparison in a small test box:

| Setup | Status | Duration | Worst checkpoint miss | Pre-turn events |
| --- | --- | ---: | ---: | ---: |
| conservative tight tune | done | `34.41 s` | `0.055 m` | `7` |
| less eager pre-turn | done | `31.29 s` | `0.056 m` | `0` |
| faster current tune | done | `21.89 s` | `0.052 m` | `0` |

If a mission stops unexpectedly, enable `Save Debug Data` in the dashboard and
check the generated `debug_runs/euroboot_debug_*_idNNNN.csv` and `.json`.
Mission status rows and the Pi log at `/tmp/euroboot_dashboard_bridge.log`
should identify whether the stop came from a user stop, a turn timeout, a Nav2
goal rejection, or mission completion.

## 2026-08-08 Reverse Legs And Drawn Tracks

The dashboard now has two mission styles:

- `Run` sends the waypoint list as guarded checkpoint legs. Each selected
  waypoint has a `Travel mode` field, so a leg can be `forward` or `backward`.
- `Run Track` sends the purple drawn track as one continuous `/follow_path`
  goal. This is smoother than treating a hand-drawn curve as many checkpoints,
  because Nav2 follows the whole polyline without stopping at every point.

Reverse legs are handled by the Pi bridge with a small direct closed-loop
driver instead of enabling Nav2 global reversing. This keeps normal forward
checkpoint behavior unchanged and makes reverse testing easier to isolate.

Ground validation in the 1 m test box:

| Test | Mode | Status | Final error | Max cross-track |
| --- | --- | --- | ---: | ---: |
| `reverse_feature02` | one backward leg, 0.35 m | done | `0.0127 m` | `0.0095 m` |
| `drawn_track01` | continuous 0.45 m curve | done | `0.0150 m` | `0.0151 m` |
| `mixed_drive_modes01` | forward 0.35 m, backward home | done | `0.0162 m` | `0.0116 m` |

The test runner supports these modes too:

```powershell
python tools\mission_tune_runner.py --host <pi-ip> --track mixed_forward_backward --mode waypoints --label mixed_test
python tools\mission_tune_runner.py --host <pi-ip> --track drawn_curve --mode path --label drawn_test
```

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```
