#!/usr/bin/env python3
"""Run one Euroboot bridge field test and save a CSV log."""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import time
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEBUG_DIR = PROJECT_ROOT / "debug_runs"


def parse_key_values(items: list[str]) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"expected key=value, got {item!r}")
        key, raw = item.split("=", 1)
        raw = raw.strip()
        if raw.lower() in {"true", "false"}:
            values[key] = raw.lower() == "true"
            continue
        try:
            values[key] = float(raw)
        except ValueError:
            values[key] = raw
    return values


class Bridge:
    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.settimeout(0.2)
        self.buffer = b""

    def send(self, message: dict[str, Any]) -> None:
        self.sock.sendall((json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8"))

    def recv_available(self, seconds: float) -> list[dict[str, Any]]:
        deadline = time.monotonic() + seconds
        messages: list[dict[str, Any]] = []
        while time.monotonic() < deadline:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            self.buffer += chunk
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                messages.append(json.loads(line.decode("utf-8")))
        return messages

    def close(self) -> None:
        self.sock.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.137.217")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--label", required=True)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--final-yaw-deg", type=float, default=None)
    parser.add_argument("--path-yaw-deg", type=float, default=None)
    parser.add_argument("--timeout", type=float, default=18.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--pause-after", type=float, default=8.0)
    parser.add_argument("--imu", action="append", default=[])
    parser.add_argument("--nav2", action="append", default=[])
    args = parser.parse_args()

    imu = parse_key_values(args.imu)
    nav2 = parse_key_values(args.nav2)
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"field_{stamp}_{args.label}"
    csv_path = DEBUG_DIR / f"{stem}.csv"
    json_path = DEBUG_DIR / f"{stem}.json"

    path_yaw = math.atan2(args.y, args.x) if args.path_yaw_deg is None else math.radians(args.path_yaw_deg)
    final_yaw = None if args.final_yaw_deg is None else math.radians(args.final_yaw_deg)
    waypoint = {"x": args.x, "y": args.y, "yaw": path_yaw, "final_yaw": final_yaw}

    metadata = {
        "label": args.label,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "host": args.host,
        "port": args.port,
        "waypoint": waypoint,
        "imu": imu,
        "nav2": nav2,
    }
    json_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    fieldnames = [
        "local_t",
        "bridge_t",
        "event_type",
        "mission_state",
        "event_message",
        "x_m",
        "y_m",
        "yaw_rad",
        "raw_x_m",
        "raw_y_m",
        "raw_yaw_rad",
        "encoder_yaw_rad",
        "pixhawk_yaw_rad",
        "pixhawk_yaw_ros_rad",
        "pixhawk_gyro_yaw_rad",
        "pixhawk_age_s",
        "yaw_source",
        "use_pixhawk_yaw",
        "pixhawk_weight",
        "encoder_weight",
        "pixhawk_yaw_mode",
        "pixhawk_yaw_sign",
        "linear_x_mps",
        "angular_z_radps",
    ]

    bridge = Bridge(args.host, args.port)
    status = "timeout"
    last_odom: dict[str, Any] | None = None
    rows = 0
    started = time.monotonic()

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        terminal_status: str | None = None

        def write_msg(message: dict[str, Any]) -> None:
            nonlocal last_odom, status, rows, terminal_status
            kind = message.get("type", "")
            if kind == "odom":
                last_odom = message
                writer.writerow(
                    {
                        "local_t": time.monotonic() - started,
                        "bridge_t": message.get("t", ""),
                        "event_type": "odom",
                        "mission_state": status,
                        "event_message": "",
                        "x_m": message.get("x", ""),
                        "y_m": message.get("y", ""),
                        "yaw_rad": message.get("yaw", ""),
                        "raw_x_m": message.get("raw_x", ""),
                        "raw_y_m": message.get("raw_y", ""),
                        "raw_yaw_rad": message.get("raw_yaw", ""),
                        "encoder_yaw_rad": message.get("encoder_yaw", ""),
                        "pixhawk_yaw_rad": message.get("pixhawk_yaw", ""),
                        "pixhawk_yaw_ros_rad": message.get("pixhawk_yaw_ros", ""),
                        "pixhawk_gyro_yaw_rad": message.get("pixhawk_gyro_yaw", ""),
                        "pixhawk_age_s": message.get("pixhawk_age_s", ""),
                        "yaw_source": message.get("yaw_source", ""),
                        "use_pixhawk_yaw": message.get("use_pixhawk_yaw", ""),
                        "pixhawk_weight": message.get("pixhawk_weight", ""),
                        "encoder_weight": message.get("encoder_weight", ""),
                        "pixhawk_yaw_mode": message.get("pixhawk_yaw_mode", ""),
                        "pixhawk_yaw_sign": message.get("pixhawk_yaw_sign", ""),
                        "linear_x_mps": message.get("linear_x", ""),
                        "angular_z_radps": message.get("angular_z", ""),
                    }
                )
                rows += 1
                return
            if kind == "mission_status":
                incoming_status = str(message.get("state", status))
                if terminal_status is None or incoming_status != "stopped":
                    status = incoming_status
                    if status in {"done", "error", "stopped"}:
                        terminal_status = status
                writer.writerow(
                    {
                        "local_t": time.monotonic() - started,
                        "bridge_t": "",
                        "event_type": "mission_status",
                        "mission_state": status,
                        "event_message": message.get("message", ""),
                        "x_m": "" if last_odom is None else last_odom.get("x", ""),
                        "y_m": "" if last_odom is None else last_odom.get("y", ""),
                        "yaw_rad": "" if last_odom is None else last_odom.get("yaw", ""),
                    }
                )

        bridge.send({"type": "hello"})
        for msg in bridge.recv_available(1.0):
            write_msg(msg)
        if imu:
            bridge.send({"type": "set_imu", "imu": imu})
            for msg in bridge.recv_available(0.8):
                write_msg(msg)
        if nav2:
            bridge.send({"type": "set_nav2_params", "params": nav2})
            for msg in bridge.recv_available(1.0):
                write_msg(msg)

        bridge.send({"type": "reset_odom"})
        for msg in bridge.recv_available(0.8):
            write_msg(msg)
        bridge.send({"type": "mission", "waypoints": [waypoint]})

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            for msg in bridge.recv_available(0.3):
                write_msg(msg)
            if status in {"done", "error", "stopped"}:
                break

        if status not in {"done", "error", "stopped"}:
            bridge.send({"type": "stop"})
            for msg in bridge.recv_available(1.0):
                write_msg(msg)
        else:
            bridge.send({"type": "stop"})
            for msg in bridge.recv_available(args.settle):
                if msg.get("type") != "mission_status":
                    write_msg(msg)

    bridge.close()

    metadata.update(
        {
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "status": status,
            "rows": rows,
            "csv": str(csv_path),
            "last_odom": last_odom,
        }
    )
    json_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2, sort_keys=True))
    time.sleep(max(0.0, args.pause_after))
    return 0 if status in {"done", "stopped"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
