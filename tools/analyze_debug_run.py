#!/usr/bin/env python3
"""Summarize Euroboot dashboard or mission-tune CSV logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any


DEFAULT_WHEEL_BASE_M = 0.15216


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        value = row.get(key, "")
        if value in ("", None):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def row_time(row: dict[str, str]) -> float:
    bridge_t = f(row, "bridge_t", float("nan"))
    if not math.isfinite(bridge_t):
        bridge_t = f(row, "t_bridge", float("nan"))
    if math.isfinite(bridge_t) and bridge_t > 0.0:
        return bridge_t
    return f(row, "local_t")


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def wheel_speeds(linear_x: float, angular_z: float, wheel_base_m: float) -> tuple[float, float]:
    half_track = wheel_base_m * 0.5
    return linear_x - angular_z * half_track, linear_x + angular_z * half_track


def summarize_block(index: int, state: str, rows: list[dict[str, str]], wheel_base_m: float) -> dict[str, Any]:
    first = rows[0]
    last = rows[-1]
    duration = max(0.0, row_time(last) - row_time(first))
    translation = math.hypot(f(last, "x_m") - f(first, "x_m"), f(last, "y_m") - f(first, "y_m"))
    yaw_change = normalize_angle(f(last, "yaw_rad") - f(first, "yaw_rad"))
    vx = [abs(f(row, "linear_x_mps")) for row in rows]
    wz = [abs(f(row, "angular_z_radps")) for row in rows]
    wheels = [wheel_speeds(f(row, "linear_x_mps"), f(row, "angular_z_radps"), wheel_base_m) for row in rows]
    balance = [abs(abs(left) - abs(right)) for left, right in wheels]
    leak_ratio = [
        abs(f(row, "linear_x_mps")) / max(0.001, abs(f(row, "angular_z_radps")) * wheel_base_m * 0.5)
        for row in rows
        if abs(f(row, "angular_z_radps")) > 0.12
    ]
    return {
        "index": index,
        "state": state,
        "samples": len(rows),
        "duration_s": round(duration, 3),
        "translation_m": round(translation, 4),
        "yaw_change_deg": round(math.degrees(yaw_change), 2),
        "start_pose": [round(f(first, "x_m"), 4), round(f(first, "y_m"), 4), round(math.degrees(f(first, "yaw_rad")), 2)],
        "end_pose": [round(f(last, "x_m"), 4), round(f(last, "y_m"), 4), round(math.degrees(f(last, "yaw_rad")), 2)],
        "mean_abs_vx_mps": round(mean(vx), 4) if vx else 0.0,
        "max_abs_vx_mps": round(max(vx), 4) if vx else 0.0,
        "mean_abs_wz_radps": round(mean(wz), 4) if wz else 0.0,
        "mean_left_mps": round(mean([left for left, _right in wheels]), 4) if wheels else 0.0,
        "mean_right_mps": round(mean([right for _left, right in wheels]), 4) if wheels else 0.0,
        "mean_abs_wheel_balance_mps": round(mean(balance), 4) if balance else 0.0,
        "mean_turn_linear_leak_ratio": round(mean(leak_ratio), 3) if leak_ratio else 0.0,
    }


def analyze(path: Path, wheel_base_m: float) -> dict[str, Any]:
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    odom = [row for row in rows if row.get("event_type") == "odom"]
    events = [row for row in rows if row.get("event_type") != "odom"]
    blocks: list[dict[str, Any]] = []
    current: list[dict[str, str]] = []
    state: str | None = None
    for row in odom:
        row_state = row.get("mission_state", "")
        if current and row_state != state:
            blocks.append(summarize_block(len(blocks), state or "", current, wheel_base_m))
            current = []
        state = row_state
        current.append(row)
    if current:
        blocks.append(summarize_block(len(blocks), state or "", current, wheel_base_m))

    turn_blocks = [block for block in blocks if block["state"] == "turning"]
    return {
        "csv": str(path),
        "odom_samples": len(odom),
        "event_count": len(events),
        "final_pose": blocks[-1]["end_pose"] if blocks else None,
        "turn_count": len(turn_blocks),
        "turn_translation_total_m": round(sum(block["translation_m"] for block in turn_blocks), 4),
        "turn_translation_max_m": round(max((block["translation_m"] for block in turn_blocks), default=0.0), 4),
        "turn_wheel_balance_mean_mps": round(mean([block["mean_abs_wheel_balance_mps"] for block in turn_blocks]), 4)
        if turn_blocks
        else 0.0,
        "turn_linear_leak_ratio_mean": round(mean([block["mean_turn_linear_leak_ratio"] for block in turn_blocks]), 3)
        if turn_blocks
        else 0.0,
        "events": [
            {
                "state": row.get("mission_state", ""),
                "message": row.get("event_message", ""),
                "pose": [round(f(row, "x_m"), 4), round(f(row, "y_m"), 4), round(math.degrees(f(row, "yaw_rad")), 2)],
            }
            for row in events
        ],
        "blocks": blocks,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--wheel-base", type=float, default=DEFAULT_WHEEL_BASE_M)
    args = parser.parse_args()
    print(json.dumps(analyze(args.csv, args.wheel_base), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
