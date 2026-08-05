#!/usr/bin/env python3
"""Zero-relative Pixhawk yaw test.

Run this on the Raspberry Pi while the Pixhawk TELEM port is connected to the
Pi UART. The first valid Pixhawk ATTITUDE yaw becomes 0 deg, then the script
prints only when yaw changes by a configurable deadband.
"""

from __future__ import annotations

import argparse
import math
import struct
import time

import serial

from pixhawk_mavlink_yaw_probe import parse_frames


def normalize_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def yaw_from_attitude_payload(payload: bytes) -> float | None:
    if len(payload) < 28:
        return None
    yaw_rad = struct.unpack_from("<f", payload, 12)[0]
    return math.degrees(yaw_rad)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Print Pixhawk yaw in degrees relative to launch orientation."
    )
    parser.add_argument("--port", default="/dev/serial0", help="Pixhawk serial device")
    parser.add_argument("--baud", type=int, default=115200, help="Pixhawk serial baud")
    parser.add_argument(
        "--deadband",
        type=float,
        default=0.5,
        help="Print only when yaw changes by at least this many degrees",
    )
    parser.add_argument(
        "--heartbeat",
        type=float,
        default=2.0,
        help="Print unchanged yaw at most once every N seconds; 0 disables",
    )
    args = parser.parse_args()

    buffer = bytearray()
    origin_yaw: float | None = None
    last_printed_yaw: float | None = None
    last_print_t = 0.0

    print(f"Opening Pixhawk MAVLink on {args.port} @ {args.baud} baud...", flush=True)
    print("Hold robot still. First ATTITUDE message will become yaw = 0.0 deg.", flush=True)

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            while True:
                buffer.extend(ser.read(512))
                for frame in parse_frames(buffer):
                    if frame.msgid != 30:  # ATTITUDE
                        continue

                    yaw_deg = yaw_from_attitude_payload(frame.payload)
                    if yaw_deg is None:
                        continue

                    now = time.monotonic()
                    if origin_yaw is None:
                        origin_yaw = yaw_deg
                        last_printed_yaw = 0.0
                        last_print_t = now
                        print(
                            f"ZERO set: raw_yaw={yaw_deg:.2f} deg | yaw=0.00 deg",
                            flush=True,
                        )
                        continue

                    relative_yaw = normalize_deg(yaw_deg - origin_yaw)
                    should_print = (
                        last_printed_yaw is None
                        or abs(normalize_deg(relative_yaw - last_printed_yaw)) >= args.deadband
                    )
                    heartbeat_due = (
                        args.heartbeat > 0.0 and now - last_print_t >= args.heartbeat
                    )

                    if should_print or heartbeat_due:
                        print(
                            f"yaw={relative_yaw:8.2f} deg | raw={yaw_deg:8.2f} deg",
                            flush=True,
                        )
                        last_printed_yaw = relative_yaw
                        last_print_t = now

    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
