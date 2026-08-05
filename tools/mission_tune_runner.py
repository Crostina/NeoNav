#!/usr/bin/env python3
"""Run and score the Euroboot 4-waypoint field tuning mission."""

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
MISSION_POINTS = [
    {"x": 0.754, "y": 0.733, "final_yaw": None},
    {"x": 1.504, "y": 0.002, "final_yaw": None},
    {"x": 0.746, "y": -0.510, "final_yaw": None},
    {"x": -0.008, "y": -0.002, "final_yaw": 0.0},
]


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


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


def mission_payload() -> list[dict[str, Any]]:
    payload: list[dict[str, Any]] = []
    previous = (0.0, 0.0)
    for point in MISSION_POINTS:
        yaw = math.atan2(point["y"] - previous[1], point["x"] - previous[0])
        final_yaw = point["final_yaw"]
        payload.append(
            {
                "x": point["x"],
                "y": point["y"],
                "yaw": normalize_angle(yaw),
                "final_yaw": None if final_yaw is None else normalize_angle(final_yaw),
            }
        )
        previous = (point["x"], point["y"])
    return payload


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


def nearest_path_error(x: float, y: float, path: list[tuple[float, float]]) -> float:
    best = float("inf")
    for (x1, y1), (x2, y2) in zip(path, path[1:]):
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            best = min(best, math.hypot(x - x1, y - y1))
            continue
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / length_sq))
        px = x1 + t * dx
        py = y1 + t * dy
        best = min(best, math.hypot(x - px, y - py))
    return best


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def row_time(row: dict[str, str]) -> float:
    bridge_t = f(row, "bridge_t", float("nan"))
    if math.isfinite(bridge_t) and bridge_t > 0.0:
        return bridge_t
    return f(row, "local_t")


def score_rows(rows: list[dict[str, str]], status: str) -> dict[str, Any]:
    odom = [row for row in rows if row.get("event_type") == "odom"]
    if not odom:
        return {"score": 9999.0, "valid": False, "reason": "no odom rows"}

    start_index = 0
    for idx, row in enumerate(rows):
        if row.get("event_type") == "mission_status" and row.get("mission_state") == "navigating":
            start_index = idx
            break
    mission_odom = [row for row in rows[start_index:] if row.get("event_type") == "odom"]
    if len(mission_odom) < 5:
        mission_odom = odom

    path = [(0.0, 0.0)] + [(p["x"], p["y"]) for p in MISSION_POINTS]
    ctes = [nearest_path_error(f(row, "x_m"), f(row, "y_m"), path) for row in mission_odom]
    last = mission_odom[-1]
    final = MISSION_POINTS[-1]
    final_pos_error = math.hypot(f(last, "x_m") - final["x"], f(last, "y_m") - final["y"])
    final_yaw_error = abs(normalize_angle(f(last, "yaw_rad") - float(final["final_yaw"])))

    wz_values = [f(row, "angular_z_radps") for row in mission_odom]
    vx_values = [f(row, "linear_x_mps") for row in mission_odom]
    sign_changes = 0
    last_sign = 0
    for wz in wz_values:
        sign = 1 if wz > 0.18 else -1 if wz < -0.18 else 0
        if sign and last_sign and sign != last_sign:
            sign_changes += 1
        if sign:
            last_sign = sign
    stop_samples = sum(1 for vx, wz in zip(vx_values[:-8], wz_values[:-8]) if abs(vx) < 0.015 and abs(wz) < 0.05)
    duration = row_time(mission_odom[-1]) - row_time(mission_odom[0])

    jumps = 0
    for a, b in zip(mission_odom, mission_odom[1:]):
        dt = max(0.001, row_time(b) - row_time(a))
        dist = math.hypot(f(b, "x_m") - f(a, "x_m"), f(b, "y_m") - f(a, "y_m"))
        if dist / dt > 1.2:
            jumps += 1

    valid = jumps == 0
    failure_penalty = 0.0 if status == "done" else 120.0
    suspect_penalty = 60.0 if jumps else 0.0
    score = (
        130.0 * final_pos_error
        + 80.0 * max(ctes)
        + 45.0 * (sum(ctes) / max(1, len(ctes)))
        + 0.45 * math.degrees(final_yaw_error)
        + 0.45 * sign_changes
        + 0.08 * stop_samples
        + 0.22 * duration
        + failure_penalty
        + suspect_penalty
    )
    return {
        "score": round(score, 3),
        "valid": valid,
        "status": status,
        "duration_s": round(duration, 3),
        "final_x_m": f(last, "x_m"),
        "final_y_m": f(last, "y_m"),
        "final_yaw_deg": round(math.degrees(f(last, "yaw_rad")), 2),
        "final_pos_error_m": round(final_pos_error, 4),
        "final_yaw_error_deg": round(math.degrees(final_yaw_error), 2),
        "max_cross_track_m": round(max(ctes), 4),
        "mean_cross_track_m": round(sum(ctes) / max(1, len(ctes)), 4),
        "oscillation_sign_changes": sign_changes,
        "stop_samples": stop_samples,
        "manual_intervention_suspected": bool(jumps),
        "jump_samples": jumps,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.137.217")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--label", required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--pause-after", type=float, default=8.0)
    parser.add_argument("--imu", action="append", default=[])
    parser.add_argument("--nav2", action="append", default=[])
    args = parser.parse_args()

    imu = parse_key_values(args.imu)
    nav2 = parse_key_values(args.nav2)
    payload = mission_payload()

    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"mission_{stamp}_{args.label}"
    csv_path = DEBUG_DIR / f"{stem}.csv"
    json_path = DEBUG_DIR / f"{stem}.json"

    metadata: dict[str, Any] = {
        "label": args.label,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "host": args.host,
        "port": args.port,
        "mission": payload,
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
        "nearest_path_error_m",
    ]

    bridge = Bridge(args.host, args.port)
    rows: list[dict[str, str]] = []
    status = "idle"
    terminal_status: str | None = None
    last_odom: dict[str, Any] | None = None
    started = time.monotonic()
    path_for_score = [(0.0, 0.0)] + [(p["x"], p["y"]) for p in MISSION_POINTS]

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        def write_row(row: dict[str, Any]) -> None:
            text_row = {key: row.get(key, "") for key in fieldnames}
            rows.append({key: str(value) for key, value in text_row.items()})
            writer.writerow(text_row)

        def handle(message: dict[str, Any]) -> None:
            nonlocal last_odom, status, terminal_status
            kind = message.get("type", "")
            if kind == "odom":
                last_odom = message
                x = float(message.get("x", 0.0))
                y = float(message.get("y", 0.0))
                write_row(
                    {
                        "local_t": time.monotonic() - started,
                        "bridge_t": message.get("t", ""),
                        "event_type": "odom",
                        "mission_state": status,
                        "event_message": "",
                        "x_m": x,
                        "y_m": y,
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
                        "nearest_path_error_m": nearest_path_error(x, y, path_for_score),
                    }
                )
                return
            if kind == "mission_status":
                incoming = str(message.get("state", status))
                if terminal_status is None or incoming != "stopped":
                    status = incoming
                    if status in {"done", "error", "stopped"}:
                        terminal_status = status
                write_row(
                    {
                        "local_t": time.monotonic() - started,
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
            handle(msg)
        if imu:
            bridge.send({"type": "set_imu", "imu": imu})
            for msg in bridge.recv_available(0.8):
                handle(msg)
        if nav2:
            bridge.send({"type": "set_nav2_params", "params": nav2})
            for msg in bridge.recv_available(1.2):
                handle(msg)
        bridge.send({"type": "reset_odom"})
        for msg in bridge.recv_available(1.0):
            handle(msg)
        bridge.send({"type": "mission", "waypoints": payload})

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            for msg in bridge.recv_available(0.3):
                handle(msg)
            if status in {"done", "error", "stopped"}:
                break

        if status not in {"done", "error", "stopped"}:
            status = "timeout"
        bridge.send({"type": "stop"})
        for msg in bridge.recv_available(args.settle):
            if status in {"done", "error"} and msg.get("type") == "mission_status":
                continue
            handle(msg)

    bridge.close()
    metrics = score_rows(rows, status)
    metadata.update(
        {
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "status": status,
            "csv": str(csv_path),
            "metrics": metrics,
        }
    )
    json_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2, sort_keys=True))
    time.sleep(max(0.0, args.pause_after))
    return 0 if status in {"done", "stopped", "timeout"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
