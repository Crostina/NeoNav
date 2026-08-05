#!/usr/bin/env python3
"""Zero-relative Pixhawk XY distance test.

The first valid Pixhawk position becomes x=0, y=0. The script then prints
relative x, y, and planar distance only when the position changes by a
configurable deadband.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass

import serial

from pixhawk_mavlink_yaw_probe import parse_frames


EARTH_RADIUS_M = 6378137.0


@dataclass
class PositionSample:
    source: str
    x: float
    y: float
    z: float


def local_position_from_payload(payload: bytes) -> PositionSample | None:
    # MAVLink LOCAL_POSITION_NED, msg id 32:
    # uint32 time_boot_ms; float x; float y; float z; float vx; float vy; float vz
    if len(payload) < 28:
        return None
    x, y, z = struct.unpack_from("<fff", payload, 4)
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    return PositionSample("LOCAL_POSITION_NED", x, y, z)


def global_position_from_payload(payload: bytes, origin: tuple[float, float] | None) -> tuple[PositionSample | None, tuple[float, float] | None]:
    # MAVLink GLOBAL_POSITION_INT, msg id 33:
    # int32 lat/lon in degE7, alt/relative_alt in mm
    if len(payload) < 28:
        return None, origin

    lat_i, lon_i, _alt_mm, rel_alt_mm = struct.unpack_from("<iiii", payload, 4)
    if lat_i == 0 and lon_i == 0:
        return None, origin

    lat = lat_i / 1e7
    lon = lon_i / 1e7
    if origin is None:
        origin = (lat, lon)

    lat0, lon0 = origin
    mean_lat = math.radians((lat + lat0) * 0.5)
    x_north = math.radians(lat - lat0) * EARTH_RADIUS_M
    y_east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(mean_lat)
    z_down = -(rel_alt_mm / 1000.0)
    return PositionSample("GLOBAL_POSITION_INT", x_north, y_east, z_down), origin


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Print Pixhawk x/y/distance relative to launch position."
    )
    parser.add_argument("--port", default="/dev/serial0", help="Pixhawk serial device")
    parser.add_argument("--baud", type=int, default=115200, help="Pixhawk serial baud")
    parser.add_argument(
        "--source",
        choices=("local", "global", "auto"),
        default="local",
        help="Position source. Use local for indoor EKF/local-position testing.",
    )
    parser.add_argument(
        "--deadband",
        type=float,
        default=0.01,
        help="Print only when planar distance changes by at least this many meters",
    )
    parser.add_argument(
        "--heartbeat",
        type=float,
        default=0.0,
        help="Print unchanged position at most once every N seconds; 0 disables",
    )
    args = parser.parse_args()

    buffer = bytearray()
    origin_xy: tuple[float, float] | None = None
    global_origin_ll: tuple[float, float] | None = None
    last_printed_distance: float | None = None
    last_print_t = 0.0

    print(f"Opening Pixhawk MAVLink on {args.port} @ {args.baud} baud...", flush=True)
    print("Hold robot still. First valid position will become x=0.000, y=0.000.", flush=True)

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            while True:
                buffer.extend(ser.read(512))
                for frame in parse_frames(buffer):
                    sample: PositionSample | None = None

                    if args.source in ("local", "auto") and frame.msgid == 32:
                        sample = local_position_from_payload(frame.payload)

                    if sample is None and args.source in ("global", "auto") and frame.msgid == 33:
                        sample, global_origin_ll = global_position_from_payload(
                            frame.payload, global_origin_ll
                        )

                    if sample is None:
                        continue

                    if origin_xy is None:
                        origin_xy = (sample.x, sample.y)
                        last_printed_distance = 0.0
                        last_print_t = time.monotonic()
                        print(
                            f"ZERO set from {sample.source}: "
                            f"raw_x={sample.x:.3f} raw_y={sample.y:.3f} | "
                            f"x=0.000 y=0.000 dist=0.000 m",
                            flush=True,
                        )
                        continue

                    x = sample.x - origin_xy[0]
                    y = sample.y - origin_xy[1]
                    distance = math.hypot(x, y)
                    now = time.monotonic()
                    should_print = (
                        last_printed_distance is None
                        or abs(distance - last_printed_distance) >= args.deadband
                    )
                    heartbeat_due = (
                        args.heartbeat > 0.0 and now - last_print_t >= args.heartbeat
                    )

                    if should_print or heartbeat_due:
                        print(
                            f"x={x:8.3f} m | y={y:8.3f} m | "
                            f"dist={distance:8.3f} m | source={sample.source}",
                            flush=True,
                        )
                        last_printed_distance = distance
                        last_print_t = now

    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
