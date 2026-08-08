#!/usr/bin/env python3
"""Run and score Euroboot waypoint tuning missions.

The script talks to tools/euroboot_ros_bridge.py over TCP. It is intentionally
usable from the Windows PC without ROS installed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEBUG_DIR = PROJECT_ROOT / "debug_runs"

TrackPoint = dict[str, float | str | None]

TRACKS: dict[str, list[TrackPoint]] = {
    "wide_box": [
        {"x": 0.754, "y": 0.733, "final_yaw_deg": None},
        {"x": 1.504, "y": 0.002, "final_yaw_deg": None},
        {"x": 0.746, "y": -0.510, "final_yaw_deg": None},
        {"x": -0.008, "y": -0.002, "final_yaw_deg": 0.0},
    ],
    "turn_box": [
        {"x": 0.746, "y": 0.004, "final_yaw_deg": -90.0},
        {"x": 0.750, "y": -0.596, "final_yaw_deg": None},
        {"x": -0.004, "y": -0.504, "final_yaw_deg": None},
        {"x": 0.000, "y": 0.000, "final_yaw_deg": 0.0},
    ],
    "small_square": [
        {"x": 0.500, "y": 0.000, "final_yaw_deg": -90.0},
        {"x": 0.500, "y": -0.500, "final_yaw_deg": 180.0},
        {"x": 0.000, "y": -0.500, "final_yaw_deg": 90.0},
        {"x": 0.000, "y": 0.000, "final_yaw_deg": 0.0},
    ],
    "backward_line": [
        {"x": -0.350, "y": 0.000, "final_yaw_deg": 0.0, "drive_mode": "backward"},
    ],
    "mixed_forward_backward": [
        {"x": 0.350, "y": 0.000, "final_yaw_deg": 0.0, "drive_mode": "forward"},
        {"x": 0.000, "y": 0.000, "final_yaw_deg": 0.0, "drive_mode": "backward"},
    ],
    "drawn_curve": [
        {"x": 0.150, "y": 0.000, "final_yaw_deg": None},
        {"x": 0.300, "y": 0.080, "final_yaw_deg": None},
        {"x": 0.450, "y": 0.080, "final_yaw_deg": None},
    ],
}

WHEEL_BASE_M = 0.15216


@dataclass
class Projection:
    segment: int
    along_m: float
    segment_progress_m: float
    segment_ratio: float
    distance_m: float
    signed_error_m: float
    x_m: float
    y_m: float
    yaw_rad: float


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_drive_mode(value: Any) -> str:
    text = str(value or "forward").strip().lower()
    if text in {"back", "backward", "reverse", "rev"}:
        return "backward"
    return "forward"


def segment_robot_yaw(start: tuple[float, float], end: tuple[float, float], drive_mode: str) -> float:
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    if normalize_drive_mode(drive_mode) == "backward":
        yaw = normalize_angle(yaw + math.pi)
    return yaw


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        value = row.get(key, "")
        if value in ("", None):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def differential_wheel_speeds(linear_x: float, angular_z: float, wheel_base_m: float = WHEEL_BASE_M) -> tuple[float, float]:
    half_track = wheel_base_m * 0.5
    return linear_x - angular_z * half_track, linear_x + angular_z * half_track


def row_time(row: dict[str, str]) -> float:
    bridge_t = f(row, "bridge_t", float("nan"))
    if math.isfinite(bridge_t) and bridge_t > 0.0:
        return bridge_t
    return f(row, "local_t")


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


def mission_payload(points: list[TrackPoint]) -> list[dict[str, Any]]:
    payload: list[dict[str, Any]] = []
    previous = (0.0, 0.0)
    for index, point in enumerate(points):
        x = float(point["x"])
        y = float(point["y"])
        next_point = points[index + 1] if index + 1 < len(points) else point
        drive_mode = normalize_drive_mode(point.get("drive_mode", "forward"))
        if math.hypot(x - previous[0], y - previous[1]) > 0.02:
            yaw = segment_robot_yaw(previous, (x, y), drive_mode)
        elif math.hypot(float(next_point["x"]) - x, float(next_point["y"]) - y) > 0.02:
            yaw = segment_robot_yaw(
                (x, y),
                (float(next_point["x"]), float(next_point["y"])),
                normalize_drive_mode(next_point.get("drive_mode", "forward")),
            )
        else:
            yaw = 0.0
        final_yaw_deg = point.get("final_yaw_deg")
        final_yaw = None if final_yaw_deg is None else math.radians(float(final_yaw_deg))
        payload.append(
            {
                "x": x,
                "y": y,
                "yaw": normalize_angle(yaw),
                "final_yaw": None if final_yaw is None else normalize_angle(final_yaw),
                "drive_mode": drive_mode,
            }
        )
        previous = (x, y)
    return payload


def planned_path(payload: list[dict[str, Any]]) -> list[tuple[float, float]]:
    return [(0.0, 0.0)] + [(float(point["x"]), float(point["y"])) for point in payload]


def project_to_path(x: float, y: float, path: list[tuple[float, float]]) -> Projection:
    best: Projection | None = None
    total_before = 0.0
    for segment, ((x1, y1), (x2, y2)) in enumerate(zip(path, path[1:]), start=1):
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        ratio = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (length * length)))
        px = x1 + ratio * dx
        py = y1 + ratio * dy
        distance = math.hypot(x - px, y - py)
        signed = (dx * (y - y1) - dy * (x - x1)) / length
        projection = Projection(
            segment=segment,
            along_m=total_before + ratio * length,
            segment_progress_m=ratio * length,
            segment_ratio=ratio,
            distance_m=distance,
            signed_error_m=signed,
            x_m=px,
            y_m=py,
            yaw_rad=math.atan2(dy, dx),
        )
        if best is None or projection.distance_m < best.distance_m:
            best = projection
        total_before += length
    if best is None:
        return Projection(0, 0.0, 0.0, 0.0, math.hypot(x, y), 0.0, 0.0, 0.0, 0.0)
    return best


def project_to_segment(x: float, y: float, path: list[tuple[float, float]], segment: int) -> Projection:
    if segment < 1 or segment >= len(path):
        return project_to_path(x, y, path)
    total_before = 0.0
    for previous in range(1, segment):
        x1, y1 = path[previous - 1]
        x2, y2 = path[previous]
        total_before += math.hypot(x2 - x1, y2 - y1)

    x1, y1 = path[segment - 1]
    x2, y2 = path[segment]
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return Projection(segment, total_before, 0.0, 0.0, math.hypot(x - x1, y - y1), 0.0, x1, y1, 0.0)
    ratio = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (length * length)))
    px = x1 + ratio * dx
    py = y1 + ratio * dy
    return Projection(
        segment=segment,
        along_m=total_before + ratio * length,
        segment_progress_m=ratio * length,
        segment_ratio=ratio,
        distance_m=math.hypot(x - px, y - py),
        signed_error_m=(dx * (y - y1) - dy * (x - x1)) / length,
        x_m=px,
        y_m=py,
        yaw_rad=math.atan2(dy, dx),
    )


def row_projection(row: dict[str, str], path: list[tuple[float, float]], mission_mode: str = "waypoints") -> Projection:
    if mission_mode == "path":
        return project_to_path(f(row, "x_m"), f(row, "y_m"), path)
    segment = int(f(row, "mission_index", 0.0))
    return project_to_segment(f(row, "x_m"), f(row, "y_m"), path, segment)


def projection_robot_yaw(projection: Projection, payload: list[dict[str, Any]], mission_mode: str = "waypoints") -> float:
    if mission_mode == "waypoints" and 1 <= projection.segment <= len(payload):
        if normalize_drive_mode(payload[projection.segment - 1].get("drive_mode")) == "backward":
            return normalize_angle(projection.yaw_rad + math.pi)
    return projection.yaw_rad


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


def build_turn_episodes(odom: list[dict[str, str]]) -> list[dict[str, Any]]:
    episodes: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] = []
    for row in odom:
        cmd_vx = f(row, "cmd_linear_x_mps", float("nan"))
        cmd_wz = f(row, "cmd_angular_z_radps", float("nan"))
        command_turn = math.isfinite(cmd_vx) and math.isfinite(cmd_wz) and abs(cmd_vx) < 0.04 and abs(cmd_wz) > 0.12
        status_turn = row.get("mission_state") == "turning"
        if command_turn or status_turn:
            current.append(row)
            continue
        if len(current) >= 3:
            episodes.append(current)
        current = []
    if len(current) >= 3:
        episodes.append(current)

    metrics: list[dict[str, Any]] = []
    for index, rows in enumerate(episodes, start=1):
        first = rows[0]
        last = rows[-1]
        duration = max(0.0, row_time(last) - row_time(first))
        yaw_change = normalize_angle(f(last, "yaw_rad") - f(first, "yaw_rad"))
        net_translation = math.hypot(f(last, "x_m") - f(first, "x_m"), f(last, "y_m") - f(first, "y_m"))
        path_translation = 0.0
        for a, b in zip(rows, rows[1:]):
            path_translation += math.hypot(f(b, "x_m") - f(a, "x_m"), f(b, "y_m") - f(a, "y_m"))
        actual_vx = [abs(f(row, "linear_x_mps")) for row in rows]
        actual_wz = [abs(f(row, "angular_z_radps")) for row in rows]
        cmd_vx = [abs(f(row, "cmd_linear_x_mps", 0.0)) for row in rows]
        cmd_wz = [abs(f(row, "cmd_angular_z_radps", 0.0)) for row in rows]
        actual_wheels = [
            differential_wheel_speeds(f(row, "linear_x_mps"), f(row, "angular_z_radps"))
            for row in rows
        ]
        cmd_wheels = [
            differential_wheel_speeds(f(row, "cmd_linear_x_mps", 0.0), f(row, "cmd_angular_z_radps", 0.0))
            for row in rows
        ]
        actual_balance = [abs(abs(left) - abs(right)) for left, right in actual_wheels]
        cmd_balance = [abs(abs(left) - abs(right)) for left, right in cmd_wheels]
        leak_ratios = [
            abs(f(row, "linear_x_mps")) / max(0.001, abs(f(row, "angular_z_radps")) * WHEEL_BASE_M * 0.5)
            for row in rows
            if abs(f(row, "angular_z_radps")) > 0.12
        ]
        radius = path_translation / max(0.001, abs(yaw_change))
        target_errors = [abs(f(row, "turn_yaw_error_rad", float("nan"))) for row in rows]
        target_errors = [value for value in target_errors if math.isfinite(value)]
        metrics.append(
            {
                "index": index,
                "start_s": round(row_time(first), 3),
                "duration_s": round(duration, 3),
                "yaw_change_deg": round(math.degrees(yaw_change), 2),
                "net_translation_m": round(net_translation, 4),
                "path_translation_m": round(path_translation, 4),
                "estimated_radius_m": round(radius, 4),
                "mean_abs_actual_vx_mps": round(sum(actual_vx) / max(1, len(actual_vx)), 4),
                "max_abs_actual_vx_mps": round(max(actual_vx, default=0.0), 4),
                "mean_abs_actual_wz_radps": round(sum(actual_wz) / max(1, len(actual_wz)), 4),
                "mean_abs_cmd_vx_mps": round(sum(cmd_vx) / max(1, len(cmd_vx)), 4),
                "mean_abs_cmd_wz_radps": round(sum(cmd_wz) / max(1, len(cmd_wz)), 4),
                "mean_actual_left_mps": round(sum(left for left, _right in actual_wheels) / max(1, len(actual_wheels)), 4),
                "mean_actual_right_mps": round(sum(right for _left, right in actual_wheels) / max(1, len(actual_wheels)), 4),
                "mean_cmd_left_mps": round(sum(left for left, _right in cmd_wheels) / max(1, len(cmd_wheels)), 4),
                "mean_cmd_right_mps": round(sum(right for _left, right in cmd_wheels) / max(1, len(cmd_wheels)), 4),
                "mean_actual_wheel_abs_balance_mps": round(sum(actual_balance) / max(1, len(actual_balance)), 4),
                "mean_cmd_wheel_abs_balance_mps": round(sum(cmd_balance) / max(1, len(cmd_balance)), 4),
                "mean_turn_linear_leak_ratio": round(sum(leak_ratios) / max(1, len(leak_ratios)), 3),
                "final_target_yaw_error_deg": None
                if not target_errors
                else round(math.degrees(target_errors[-1]), 2),
            }
        )
    return metrics


def score_rows(rows: list[dict[str, str]], status: str, payload: list[dict[str, Any]], mission_mode: str = "waypoints") -> dict[str, Any]:
    odom = [row for row in rows if row.get("event_type") == "odom"]
    if not odom:
        return {"score": 9999.0, "valid": False, "reason": "no odom rows"}

    start_index = 0
    for idx, row in enumerate(rows):
        if row.get("event_type") == "mission_status" and row.get("mission_state") in {"turning", "navigating"}:
            start_index = idx
            break
    mission_odom = [row for row in rows[start_index:] if row.get("event_type") == "odom"]
    if len(mission_odom) < 5:
        mission_odom = odom

    path = planned_path(payload)
    projections = [row_projection(row, path, mission_mode) for row in mission_odom]
    ctes = [projection.distance_m for projection in projections]
    signed_ctes = [projection.signed_error_m for projection in projections]
    heading_errors = [
        abs(normalize_angle(projection_robot_yaw(projection, payload, mission_mode) - f(row, "yaw_rad")))
        for row, projection in zip(mission_odom, projections)
        if row.get("mission_state") == "navigating"
    ]
    last = mission_odom[-1]
    final = payload[-1]
    final_pos_error = math.hypot(f(last, "x_m") - float(final["x"]), f(last, "y_m") - float(final["y"]))
    final_yaw = final.get("final_yaw")
    final_yaw_error = 0.0 if final_yaw is None else abs(normalize_angle(f(last, "yaw_rad") - float(final_yaw)))

    wz_values = [f(row, "angular_z_radps") for row in mission_odom]
    vx_values = [f(row, "linear_x_mps") for row in mission_odom]
    cmd_wz_values = [f(row, "cmd_angular_z_radps", float("nan")) for row in mission_odom]
    cmd_vx_values = [f(row, "cmd_linear_x_mps", float("nan")) for row in mission_odom]
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

    progress_reversals = 0
    last_along = projections[0].along_m
    for projection in projections[1:]:
        along = projection.along_m
        if along + 0.025 < last_along:
            progress_reversals += 1
        last_along = max(last_along, along)

    cmd_samples = [
        (cmd_vx, cmd_wz, vx, wz)
        for cmd_vx, cmd_wz, vx, wz, row in zip(cmd_vx_values, cmd_wz_values, vx_values, wz_values, mission_odom)
        if math.isfinite(cmd_vx) and math.isfinite(cmd_wz) and f(row, "cmd_age_s", 99.0) < 0.35
    ]
    linear_cmd_error = (
        sum(abs(vx - cmd_vx) for cmd_vx, _cmd_wz, vx, _wz in cmd_samples) / len(cmd_samples)
        if cmd_samples
        else 0.0
    )
    angular_cmd_error = (
        sum(abs(wz - cmd_wz) for _cmd_vx, cmd_wz, _vx, wz in cmd_samples) / len(cmd_samples)
        if cmd_samples
        else 0.0
    )
    turn_episodes = build_turn_episodes(mission_odom)
    turn_translation_total = sum(float(ep["path_translation_m"]) for ep in turn_episodes)
    turn_translation_max = max((float(ep["path_translation_m"]) for ep in turn_episodes), default=0.0)
    turn_radius_max = max((float(ep["estimated_radius_m"]) for ep in turn_episodes), default=0.0)
    turn_balance_mean = (
        sum(float(ep["mean_actual_wheel_abs_balance_mps"]) for ep in turn_episodes) / len(turn_episodes)
        if turn_episodes
        else 0.0
    )
    turn_leak_mean = (
        sum(float(ep["mean_turn_linear_leak_ratio"]) for ep in turn_episodes) / len(turn_episodes)
        if turn_episodes
        else 0.0
    )
    mean_heading_error_deg = math.degrees(sum(heading_errors) / len(heading_errors)) if heading_errors else 0.0

    waypoint_metrics: list[dict[str, Any]] = []
    for index, point in enumerate(payload, start=1):
        distances = [
            math.hypot(f(row, "x_m") - float(point["x"]), f(row, "y_m") - float(point["y"]))
            for row in mission_odom
        ]
        waypoint_metrics.append(
            {
                "index": index,
                "x": point["x"],
                "y": point["y"],
                "min_distance_m": round(min(distances, default=float("inf")), 4),
                "final_yaw_deg": None if point.get("final_yaw") is None else round(math.degrees(float(point["final_yaw"])), 2),
            }
        )

    valid = jumps == 0
    failure_penalty = 0.0 if status == "done" else 160.0
    suspect_penalty = 80.0 if jumps else 0.0
    score = (
        140.0 * final_pos_error
        + 0.45 * math.degrees(final_yaw_error)
        + 95.0 * max(ctes, default=0.0)
        + 55.0 * (sum(ctes) / max(1, len(ctes)))
        + 0.25 * mean_heading_error_deg
        + 45.0 * turn_translation_total
        + 35.0 * turn_translation_max
        + 10.0 * turn_radius_max
        + 30.0 * turn_balance_mean
        + 2.5 * turn_leak_mean
        + 8.0 * linear_cmd_error
        + 1.5 * angular_cmd_error
        + 0.40 * sign_changes
        + 0.08 * stop_samples
        + 0.12 * progress_reversals
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
        "max_cross_track_m": round(max(ctes, default=0.0), 4),
        "mean_cross_track_m": round(sum(ctes) / max(1, len(ctes)), 4),
        "signed_cross_track_mean_m": round(sum(signed_ctes) / max(1, len(signed_ctes)), 4),
        "mean_heading_error_deg": round(mean_heading_error_deg, 2),
        "oscillation_sign_changes": sign_changes,
        "stop_samples": stop_samples,
        "progress_reversals": progress_reversals,
        "mean_abs_linear_cmd_error_mps": round(linear_cmd_error, 4),
        "mean_abs_angular_cmd_error_radps": round(angular_cmd_error, 4),
        "turn_episode_count": len(turn_episodes),
        "turn_translation_total_m": round(turn_translation_total, 4),
        "turn_translation_max_m": round(turn_translation_max, 4),
        "turn_radius_max_m": round(turn_radius_max, 4),
        "turn_wheel_abs_balance_mean_mps": round(turn_balance_mean, 4),
        "turn_linear_leak_ratio_mean": round(turn_leak_mean, 3),
        "turn_episodes": turn_episodes,
        "waypoints": waypoint_metrics,
        "manual_intervention_suspected": bool(jumps),
        "jump_samples": jumps,
    }


def make_plot(csv_path: Path, payload: list[dict[str, Any]], metrics: dict[str, Any], mission_mode: str = "waypoints") -> str | None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    all_rows = list(csv.DictReader(csv_path.open(newline="", encoding="utf-8")))
    start_index = 0
    for idx, row in enumerate(all_rows):
        if row.get("event_type") == "mission_status" and row.get("mission_state") in {"turning", "navigating"}:
            start_index = idx
            break
    rows = [row for row in all_rows[start_index:] if row.get("event_type") == "odom"]
    if len(rows) < 5:
        rows = [row for row in all_rows if row.get("event_type") == "odom"]
    if not rows:
        return None

    t0 = row_time(rows[0])
    ts = [row_time(row) - t0 for row in rows]
    xs = [f(row, "x_m") for row in rows]
    ys = [f(row, "y_m") for row in rows]
    yaws = [math.degrees(f(row, "yaw_rad")) for row in rows]
    path = planned_path(payload)
    projections = [row_projection(row, path, mission_mode) for row in rows]
    ctes = [projection.signed_error_m for projection in projections]
    actual_vx = [f(row, "linear_x_mps") for row in rows]
    actual_wz = [f(row, "angular_z_radps") for row in rows]
    cmd_vx = [f(row, "cmd_linear_x_mps") for row in rows]
    cmd_wz = [f(row, "cmd_angular_z_radps") for row in rows]
    actual_wheels = [differential_wheel_speeds(vx, wz) for vx, wz in zip(actual_vx, actual_wz)]
    cmd_wheels = [differential_wheel_speeds(vx, wz) for vx, wz in zip(cmd_vx, cmd_wz)]
    turn_balance = [abs(abs(left) - abs(right)) for left, right in actual_wheels]
    turn_leak = [
        abs(vx) / max(0.001, abs(wz) * WHEEL_BASE_M * 0.5)
        if abs(wz) > 0.12
        else 0.0
        for vx, wz in zip(actual_vx, actual_wz)
    ]

    fig, axes = plt.subplots(3, 2, figsize=(12, 11))
    ax = axes[0][0]
    ax.plot([p[0] for p in path], [p[1] for p in path], "k--", label="planned")
    ax.plot(xs, ys, color="#2563eb", label="actual")
    ax.scatter([p["x"] for p in payload], [p["y"] for p in payload], color="#dc2626", s=24)
    ax.set_title("Path")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()

    ax = axes[0][1]
    ax.plot(ts, yaws, label="yaw deg")
    ax.set_title("Yaw")
    ax.set_xlabel("time (s)")
    ax.grid(True, alpha=0.25)
    ax.legend()

    ax = axes[1][0]
    ax.plot(ts, cmd_vx, label="cmd vx")
    ax.plot(ts, actual_vx, label="actual vx")
    ax.plot(ts, cmd_wz, label="cmd wz")
    ax.plot(ts, actual_wz, label="actual wz")
    ax.set_title("Command vs Actual")
    ax.set_xlabel("time (s)")
    ax.grid(True, alpha=0.25)
    ax.legend()

    ax = axes[1][1]
    ax.plot(ts, ctes, label="signed cross-track")
    ax.axhline(0.0, color="k", linewidth=0.7)
    ax.set_title(
        f"Errors: final={metrics.get('final_pos_error_m', 0):.3f}m score={metrics.get('score', 0):.1f}"
    )
    ax.set_xlabel("time (s)")
    ax.set_ylabel("m")
    ax.grid(True, alpha=0.25)
    ax.legend()

    ax = axes[2][0]
    ax.plot(ts, [left for left, _right in actual_wheels], label="actual left")
    ax.plot(ts, [right for _left, right in actual_wheels], label="actual right")
    ax.plot(ts, [left for left, _right in cmd_wheels], "--", label="cmd left")
    ax.plot(ts, [right for _left, right in cmd_wheels], "--", label="cmd right")
    ax.set_title("Inferred Wheel Speeds")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("m/s")
    ax.grid(True, alpha=0.25)
    ax.legend()

    ax = axes[2][1]
    ax.plot(ts, turn_balance, label="abs wheel balance error")
    ax.plot(ts, turn_leak, label="linear leak ratio")
    ax.set_title("Turn Cleanliness")
    ax.set_xlabel("time (s)")
    ax.grid(True, alpha=0.25)
    ax.legend()

    fig.tight_layout()
    plot_path = csv_path.with_suffix(".png")
    fig.savefig(plot_path, dpi=130)
    plt.close(fig)
    return str(plot_path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.137.85")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--track", choices=sorted(TRACKS), default="turn_box")
    parser.add_argument("--mode", choices=["waypoints", "path"], default="waypoints")
    parser.add_argument("--label", required=True)
    parser.add_argument("--timeout", type=float, default=70.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--pause-after", type=float, default=8.0)
    parser.add_argument("--imu", action="append", default=[])
    parser.add_argument("--nav2", action="append", default=[])
    parser.add_argument("--turn", action="append", default=[])
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    imu = parse_key_values(args.imu)
    nav2 = parse_key_values(args.nav2)
    turn = parse_key_values(args.turn)
    payload = mission_payload(TRACKS[args.track])
    path = planned_path(payload)

    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"mission_{stamp}_{args.label}"
    csv_path = DEBUG_DIR / f"{stem}.csv"
    json_path = DEBUG_DIR / f"{stem}.json"

    metadata: dict[str, Any] = {
        "label": args.label,
        "track": args.track,
        "mission_mode": args.mode,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "host": args.host,
        "port": args.port,
        "mission": payload,
        "planned_path": [{"x": x, "y": y} for x, y in path],
        "imu": imu,
        "nav2": nav2,
        "turn": turn,
    }
    json_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    fieldnames = [
        "local_t",
        "bridge_t",
        "event_type",
        "mission_state",
        "mission_index",
        "mission_total",
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
        "cmd_linear_x_mps",
        "cmd_angular_z_radps",
        "cmd_age_s",
        "actual_left_mps",
        "actual_right_mps",
        "cmd_left_mps",
        "cmd_right_mps",
        "turn_abs_balance_error_mps",
        "turn_linear_leak_ratio",
        "nearest_segment",
        "along_track_m",
        "segment_progress_m",
        "segment_ratio",
        "nearest_path_x_m",
        "nearest_path_y_m",
        "nearest_path_yaw_rad",
        "cross_track_error_m",
        "signed_cross_track_error_m",
        "heading_error_rad",
        "distance_to_waypoint_m",
        "turn_kind",
        "turn_target_yaw_rad",
        "turn_yaw_error_rad",
    ]

    bridge = Bridge(args.host, args.port)
    rows: list[dict[str, str]] = []
    runtime_config: dict[str, Any] = {}
    status = "idle"
    terminal_status: str | None = None
    last_odom: dict[str, Any] | None = None
    current_index = 0
    current_total = len(payload)
    turn_kind = ""
    turn_target_yaw: float | None = None
    started = time.monotonic()

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        def write_row(row: dict[str, Any]) -> None:
            text_row = {key: row.get(key, "") for key in fieldnames}
            rows.append({key: str(value) for key, value in text_row.items()})
            writer.writerow(text_row)

        def handle(message: dict[str, Any]) -> None:
            nonlocal last_odom, status, terminal_status, current_index, current_total, turn_kind, turn_target_yaw
            kind = message.get("type", "")
            if kind in {"geometry", "nav2_params", "imu", "turn_params"}:
                runtime_config[kind] = message.get("geometry") or message.get("params") or message.get("imu")
                return
            if kind == "odom":
                last_odom = message
                x = float(message.get("x", 0.0))
                y = float(message.get("y", 0.0))
                yaw = float(message.get("yaw", 0.0))
                linear_x = float(message.get("linear_x", 0.0) or 0.0)
                angular_z = float(message.get("angular_z", 0.0) or 0.0)
                cmd_linear_x = float(message.get("cmd_linear_x", 0.0) or 0.0)
                cmd_angular_z = float(message.get("cmd_angular_z", 0.0) or 0.0)
                actual_left, actual_right = differential_wheel_speeds(linear_x, angular_z)
                cmd_left, cmd_right = differential_wheel_speeds(cmd_linear_x, cmd_angular_z)
                turn_tangent = abs(angular_z) * WHEEL_BASE_M * 0.5
                turn_balance = ""
                turn_leak = ""
                if abs(angular_z) > 0.12:
                    turn_balance = abs(abs(actual_left) - abs(actual_right))
                    turn_leak = abs(linear_x) / max(0.001, turn_tangent)
                if args.mode == "path":
                    projection = project_to_path(x, y, path)
                elif 1 <= current_index < len(path):
                    projection = project_to_segment(x, y, path, current_index)
                else:
                    projection = project_to_path(x, y, path)
                waypoint_distance = ""
                if 1 <= current_index <= len(payload):
                    waypoint = payload[current_index - 1]
                    waypoint_distance = math.hypot(x - float(waypoint["x"]), y - float(waypoint["y"]))
                turn_error = "" if turn_target_yaw is None else normalize_angle(float(turn_target_yaw) - yaw)
                write_row(
                    {
                        "local_t": time.monotonic() - started,
                        "bridge_t": message.get("t", ""),
                        "event_type": "odom",
                        "mission_state": status,
                        "mission_index": current_index or "",
                        "mission_total": current_total or "",
                        "event_message": "",
                        "x_m": x,
                        "y_m": y,
                        "yaw_rad": yaw,
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
                        "linear_x_mps": linear_x,
                        "angular_z_radps": angular_z,
                        "cmd_linear_x_mps": cmd_linear_x,
                        "cmd_angular_z_radps": cmd_angular_z,
                        "cmd_age_s": message.get("cmd_age_s", ""),
                        "actual_left_mps": actual_left,
                        "actual_right_mps": actual_right,
                        "cmd_left_mps": cmd_left,
                        "cmd_right_mps": cmd_right,
                        "turn_abs_balance_error_mps": turn_balance,
                        "turn_linear_leak_ratio": turn_leak,
                        "nearest_segment": projection.segment,
                        "along_track_m": projection.along_m,
                        "segment_progress_m": projection.segment_progress_m,
                        "segment_ratio": projection.segment_ratio,
                        "nearest_path_x_m": projection.x_m,
                        "nearest_path_y_m": projection.y_m,
                        "nearest_path_yaw_rad": projection.yaw_rad,
                        "cross_track_error_m": projection.distance_m,
                        "signed_cross_track_error_m": projection.signed_error_m,
                        "heading_error_rad": normalize_angle(projection_robot_yaw(projection, payload, args.mode) - yaw),
                        "distance_to_waypoint_m": waypoint_distance,
                        "turn_kind": turn_kind,
                        "turn_target_yaw_rad": "" if turn_target_yaw is None else turn_target_yaw,
                        "turn_yaw_error_rad": turn_error,
                    }
                )
                return

            if kind == "mission_status":
                incoming = str(message.get("state", status))
                if terminal_status is None or incoming != "stopped":
                    status = incoming
                    if status in {"done", "error", "stopped"}:
                        terminal_status = status
                try:
                    current_index = int(message.get("index", current_index or 0))
                    current_total = int(message.get("total", current_total or len(payload)))
                except (TypeError, ValueError):
                    pass

                event_message = str(message.get("message", ""))
                if status == "turning" and 1 <= current_index <= len(payload):
                    if "final alignment" in event_message:
                        turn_kind = "final"
                        turn_target_yaw = payload[current_index - 1].get("final_yaw")
                    else:
                        turn_kind = "preturn"
                        turn_target_yaw = payload[current_index - 1].get("yaw")
                elif status not in {"turning"}:
                    turn_kind = ""
                    turn_target_yaw = None

                write_row(
                    {
                        "local_t": time.monotonic() - started,
                        "event_type": "mission_status",
                        "mission_state": status,
                        "mission_index": current_index or "",
                        "mission_total": current_total or "",
                        "event_message": event_message,
                        "x_m": "" if last_odom is None else last_odom.get("x", ""),
                        "y_m": "" if last_odom is None else last_odom.get("y", ""),
                        "yaw_rad": "" if last_odom is None else last_odom.get("yaw", ""),
                        "turn_kind": turn_kind,
                        "turn_target_yaw_rad": "" if turn_target_yaw is None else turn_target_yaw,
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
        if turn:
            bridge.send({"type": "set_turn_params", "params": turn})
            for msg in bridge.recv_available(0.8):
                handle(msg)
        bridge.send({"type": "reset_odom"})
        for msg in bridge.recv_available(1.0):
            handle(msg)
        bridge.send({"type": "mission", "mode": args.mode, "waypoints": payload})

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
    metrics = score_rows(rows, status, payload, args.mode)
    plot_path = None if args.no_plot else make_plot(csv_path, payload, metrics, args.mode)
    metadata.update(
        {
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "status": status,
            "csv": str(csv_path),
            "plot": plot_path,
            "runtime_config": runtime_config,
            "metrics": metrics,
        }
    )
    json_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2, sort_keys=True))
    time.sleep(max(0.0, args.pause_after))
    return 0 if status in {"done", "stopped", "timeout"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
