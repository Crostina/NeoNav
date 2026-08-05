#!/usr/bin/env python3
"""Analyze Euroboot Nav2 odometry CSV files."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def load_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            rows.append({key: float(value) for key, value in row.items()})
    if not rows:
        raise ValueError(f"{path} has no samples")
    return rows


def metrics(path: Path) -> dict[str, float | str | int]:
    rows = load_rows(path)
    start = rows[0]
    end = rows[-1]
    c = math.cos(start["yaw_rad"])
    s = math.sin(start["yaw_rad"])

    forwards: list[float] = []
    laterals: list[float] = []
    yaw_deltas: list[float] = []
    positive_vx: list[float] = []
    max_vx = 0.0
    max_wz = 0.0

    for row in rows:
        dx = row["x_m"] - start["x_m"]
        dy = row["y_m"] - start["y_m"]
        forwards.append(c * dx + s * dy)
        laterals.append(-s * dx + c * dy)
        yaw_deltas.append(row["yaw_rad"] - start["yaw_rad"])
        vx = row.get("linear_x_mps", 0.0)
        wz = row.get("angular_z_radps", 0.0)
        max_vx = max(max_vx, vx)
        max_wz = max(max_wz, abs(wz))
        if vx > 0.001:
            positive_vx.append(vx)

    avg_vx = sum(positive_vx) / len(positive_vx) if positive_vx else 0.0
    return {
        "file": path.name,
        "samples": len(rows),
        "duration_s": end["t_s"] - start["t_s"],
        "forward_m": forwards[-1],
        "final_y_m": laterals[-1],
        "max_abs_y_m": max(abs(value) for value in laterals),
        "yaw_deg": math.degrees(yaw_deltas[-1]),
        "max_vx_mps": max_vx,
        "avg_positive_vx_mps": avg_vx,
        "max_abs_wz_radps": max_wz,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_files", nargs="+", type=Path)
    args = parser.parse_args()

    for path in args.csv_files:
        result = metrics(path)
        print(f"file={result['file']}")
        print(
            "  "
            f"samples={result['samples']} "
            f"duration={result['duration_s']:.3f}s "
            f"forward={result['forward_m']:.4f}m "
            f"final_y={result['final_y_m']:.4f}m "
            f"max_y={result['max_abs_y_m']:.4f}m "
            f"yaw={result['yaw_deg']:.2f}deg "
            f"max_vx={result['max_vx_mps']:.3f}m/s "
            f"avg_vx={result['avg_positive_vx_mps']:.3f}m/s "
            f"max_wz={result['max_abs_wz_radps']:.3f}rad/s"
        )


if __name__ == "__main__":
    main()
