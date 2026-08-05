#!/usr/bin/env bash
set -euo pipefail

EUROBOOT_HOME="${EUROBOOT_HOME:-/home/maker/euroboot}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
MICROROS_SETUP="${MICROROS_SETUP:-/home/maker/microros_ws/install/local_setup.bash}"
ESP32_PORT="${ESP32_PORT:-/dev/ttyUSB0}"
ESP32_BAUD="${ESP32_BAUD:-921600}"
BRIDGE_HOST="${BRIDGE_HOST:-0.0.0.0}"
BRIDGE_PORT="${BRIDGE_PORT:-8765}"
LOG_DIR="${LOG_DIR:-/tmp/euroboot}"

mkdir -p "$LOG_DIR"

run_background() {
    local name="$1"
    local command="$2"
    local log_file="$LOG_DIR/$name.log"
    nohup bash -lc "$command" > "$log_file" 2>&1 &
    echo "$name pid=$! log=$log_file"
}

reset_esp32() {
    if [[ ! -e "$ESP32_PORT" ]]; then
        return 0
    fi

    echo "resetting ESP32 over $ESP32_PORT..."
    python3 - "$ESP32_PORT" <<'PY' || echo "warning: ESP32 serial reset failed"
import sys
import time

import serial

port = sys.argv[1]
with serial.Serial(port, 115200, timeout=0.1) as ser:
    # ESP32 dev boards usually wire DTR/RTS to GPIO0/EN. Keep GPIO0 released
    # while pulsing EN, otherwise the board stays in serial download mode.
    ser.dtr = False
    ser.rts = True
    time.sleep(0.10)
    ser.rts = False
    ser.dtr = False
    time.sleep(0.80)
PY
}

wait_for_odom() {
    echo "waiting for /odom/unfiltered publisher..."
    for attempt in $(seq 1 60); do
        if bash -lc "source '$ROS_SETUP'; timeout 1 ros2 topic info /odom/unfiltered 2>/dev/null | grep -q 'Publisher count: [1-9]'"; then
            echo "/odom/unfiltered publisher is active"
            break
        fi
        printf "  odom wait %02d/60\r" "$attempt"
        sleep 1
        if [[ "$attempt" == "60" ]]; then
            echo
            echo "warning: /odom/unfiltered has no publisher yet"
            return 1
        fi
    done

    echo "checking /odom/unfiltered data rate..."
    for attempt in $(seq 1 12); do
        if bash -lc "source '$ROS_SETUP'; timeout 4 ros2 topic hz /odom/unfiltered 2>/dev/null | grep -q 'average rate'"; then
            echo "/odom/unfiltered samples are flowing"
            return 0
        fi
        printf "  odom sample wait %02d/12\r" "$attempt"
        sleep 1
    done
    echo
    echo "warning: /odom/unfiltered publisher exists, but samples were not confirmed"
    return 1
}

wait_for_tf() {
    echo "waiting for odom -> base_footprint TF..."
    for attempt in $(seq 1 45); do
        if bash -lc "source '$ROS_SETUP'; timeout 3 ros2 topic echo /tf --once >/dev/null 2>&1"; then
            echo "/tf is publishing"
            return 0
        fi
        printf "  tf wait %02d/45\r" "$attempt"
        sleep 1
    done
    echo
    echo "warning: /tf has no samples yet"
    return 1
}

stop_stack() {
    pkill -f "euroboot_ros_bridge.py" 2>/dev/null || true
    pkill -f "nav2_minimal_odom_launch.py" 2>/dev/null || true
    pkill -f "micro_ros_agent.*serial.*${ESP32_PORT}" 2>/dev/null || true
    pkill -f "controller_server" 2>/dev/null || true
    pkill -f "planner_server" 2>/dev/null || true
    pkill -f "behavior_server" 2>/dev/null || true
    pkill -f "bt_navigator" 2>/dev/null || true
    pkill -f "lifecycle_manager_navigation" 2>/dev/null || true
}

start_stack() {
    if [[ ! -e "$ESP32_PORT" ]]; then
        echo "warning: $ESP32_PORT does not exist; micro-ROS agent may fail"
    fi

    stop_stack
    sleep 0.8

    run_background "micro_ros_agent" \
        "source '$ROS_SETUP'; source '$MICROROS_SETUP'; ros2 run micro_ros_agent micro_ros_agent serial -D '$ESP32_PORT' -b '$ESP32_BAUD' -v2"
    sleep 1
    reset_esp32
    sleep 2

    if ! wait_for_odom; then
        echo "ESP32 odometry is not online. Nav2 was not started."
        echo "Try resetting/replugging the ESP32 USB, then run: $0 restart"
        status_stack
        return 1
    fi

    run_background "dashboard_bridge" \
        "source '$ROS_SETUP'; source '$MICROROS_SETUP'; python3 '$EUROBOOT_HOME/tools/euroboot_ros_bridge.py' --host '$BRIDGE_HOST' --port '$BRIDGE_PORT'"
    sleep 2
    if ! wait_for_tf; then
        echo "Dashboard bridge is online, but it did not publish TF. Nav2 was not started."
        status_stack
        return 1
    fi

    run_background "nav2" \
        "source '$ROS_SETUP'; ros2 launch '$EUROBOOT_HOME/tools/nav2_minimal_odom_launch.py'"
    sleep 6

    status_stack
}

status_stack() {
    echo "== processes =="
    pgrep -af "micro_ros_agent|nav2_minimal_odom_launch.py|euroboot_ros_bridge.py|controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager_navigation" || true
    echo
    echo "== ROS topics =="
    bash -lc "source '$ROS_SETUP'; timeout 4 ros2 topic list 2>/dev/null | sort" || true
}

logs_stack() {
    for log in "$LOG_DIR"/micro_ros_agent.log "$LOG_DIR"/nav2.log "$LOG_DIR"/dashboard_bridge.log; do
        echo "== $log =="
        tail -40 "$log" 2>/dev/null || echo "missing"
    done
}

case "${1:-restart}" in
    start)
        start_stack
        ;;
    stop)
        stop_stack
        ;;
    restart)
        start_stack
        ;;
    status)
        status_stack
        ;;
    logs)
        logs_stack
        ;;
    *)
        echo "usage: $0 {start|stop|restart|status|logs}"
        exit 2
        ;;
esac
