#!/usr/bin/env python3
"""Experimental Pixhawk accelerometer distance test.

This script estimates relative x/y distance by double-integrating Pixhawk IMU
acceleration. It is intentionally a test tool, not a navigation source:
accelerometer-only position drifts quickly without GPS, optical flow, vision,
wheel odometry, or another external correction.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass

import serial

from pixhawk_mavlink_yaw_probe import MavFrame, parse_frames


G = 9.80665


@dataclass
class Attitude:
    roll: float
    pitch: float
    yaw: float


@dataclass
class AccelSample:
    source: str
    t: float
    ax: float
    ay: float
    az: float


def normalize_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad <= -math.pi:
        rad += 2.0 * math.pi
    return rad


def decode_attitude(frame: MavFrame) -> Attitude | None:
    if frame.msgid != 30 or len(frame.payload) < 28:
        return None
    _time_boot_ms, roll, pitch, yaw, _rollspeed, _pitchspeed, _yawspeed = struct.unpack_from(
        "<Iffffff", frame.payload
    )
    return Attitude(roll=roll, pitch=pitch, yaw=yaw)


def decode_accel(frame: MavFrame) -> AccelSample | None:
    now = time.monotonic()

    if frame.msgid == 27 and len(frame.payload) >= 14:  # RAW_IMU, accel in milli-g
        _time_usec, xacc, yacc, zacc = struct.unpack_from("<Qhhh", frame.payload)
        scale = G / 1000.0
        return AccelSample("RAW_IMU", now, xacc * scale, yacc * scale, zacc * scale)

    if frame.msgid == 26 and len(frame.payload) >= 10:  # SCALED_IMU, accel in milli-g
        _time_boot_ms, xacc, yacc, zacc = struct.unpack_from("<Ihhh", frame.payload)
        scale = G / 1000.0
        return AccelSample("SCALED_IMU", now, xacc * scale, yacc * scale, zacc * scale)

    if frame.msgid == 105 and len(frame.payload) >= 20:  # HIGHRES_IMU, accel in m/s^2
        _time_usec, xacc, yacc, zacc = struct.unpack_from("<Qfff", frame.payload)
        return AccelSample("HIGHRES_IMU", now, xacc, yacc, zacc)

    return None


def body_to_ned(ax: float, ay: float, az: float, attitude: Attitude) -> tuple[float, float, float]:
    cr = math.cos(attitude.roll)
    sr = math.sin(attitude.roll)
    cp = math.cos(attitude.pitch)
    sp = math.sin(attitude.pitch)
    cy = math.cos(attitude.yaw)
    sy = math.sin(attitude.yaw)

    # Rz(yaw) * Ry(pitch) * Rx(roll), body FRD to NED-ish convention.
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    return (
        r00 * ax + r01 * ay + r02 * az,
        r10 * ax + r11 * ay + r12 * az,
        r20 * ax + r21 * ay + r22 * az,
    )


def ned_to_initial_heading(north: float, east: float, yaw0: float) -> tuple[float, float]:
    c = math.cos(yaw0)
    s = math.sin(yaw0)
    forward = c * north + s * east
    left = -s * north + c * east
    return forward, left


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Estimate relative distance by double-integrating Pixhawk accelerometer data."
    )
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--calibration",
        type=float,
        default=3.0,
        help="Seconds to hold still while estimating acceleration bias",
    )
    parser.add_argument(
        "--deadband",
        type=float,
        default=0.01,
        help="Print only when estimated planar distance changes by this many meters",
    )
    parser.add_argument(
        "--zupt-threshold",
        type=float,
        default=0.08,
        help="Zero velocity when horizontal accel magnitude stays below this m/s^2",
    )
    parser.add_argument(
        "--zupt-time",
        type=float,
        default=0.35,
        help="Seconds of low accel before zeroing velocity",
    )
    parser.add_argument(
        "--heartbeat",
        type=float,
        default=0.0,
        help="Print unchanged estimate every N seconds; 0 disables",
    )
    args = parser.parse_args()

    print(f"Opening Pixhawk MAVLink on {args.port} @ {args.baud} baud...", flush=True)
    print(
        f"Hold robot still for {args.calibration:.1f} s for accelerometer bias calibration.",
        flush=True,
    )

    buffer = bytearray()
    attitude: Attitude | None = None
    yaw0: float | None = None
    calibration_samples: list[tuple[float, float, float]] = []
    calibration_deadline: float | None = None
    bias_n = 0.0
    bias_e = 0.0
    bias_d = 0.0
    calibrated = False

    last_accel_t: float | None = None
    still_since: float | None = None
    vn = ve = 0.0
    pn = pe = 0.0
    last_printed_distance: float | None = None
    last_print_t = 0.0
    accel_count = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            while True:
                buffer.extend(ser.read(512))
                for frame in parse_frames(buffer):
                    decoded_attitude = decode_attitude(frame)
                    if decoded_attitude is not None:
                        attitude = decoded_attitude
                        if yaw0 is None:
                            yaw0 = attitude.yaw
                        continue

                    sample = decode_accel(frame)
                    if sample is None or attitude is None or yaw0 is None:
                        continue

                    an, ae, ad = body_to_ned(sample.ax, sample.ay, sample.az, attitude)
                    now = sample.t
                    accel_count += 1

                    if calibration_deadline is None:
                        calibration_deadline = now + args.calibration

                    if not calibrated:
                        calibration_samples.append((an, ae, ad))
                        if now >= calibration_deadline and calibration_samples:
                            bias_n = sum(v[0] for v in calibration_samples) / len(calibration_samples)
                            bias_e = sum(v[1] for v in calibration_samples) / len(calibration_samples)
                            bias_d = sum(v[2] for v in calibration_samples) / len(calibration_samples)
                            calibrated = True
                            last_accel_t = now
                            last_printed_distance = 0.0
                            last_print_t = now
                            print(
                                f"ZERO set from {sample.source}: "
                                f"yaw0={math.degrees(yaw0):.2f} deg | "
                                f"bias_n={bias_n:.4f} bias_e={bias_e:.4f} bias_d={bias_d:.4f} m/s^2 | "
                                f"samples={len(calibration_samples)}",
                                flush=True,
                            )
                        continue

                    if last_accel_t is None:
                        last_accel_t = now
                        continue

                    dt = now - last_accel_t
                    last_accel_t = now
                    if dt <= 0.0 or dt > 0.5:
                        continue

                    an -= bias_n
                    ae -= bias_e
                    ad -= bias_d
                    horizontal_accel = math.hypot(an, ae)

                    if horizontal_accel < args.zupt_threshold:
                        if still_since is None:
                            still_since = now
                        elif now - still_since >= args.zupt_time:
                            vn = 0.0
                            ve = 0.0
                    else:
                        still_since = None

                    vn += an * dt
                    ve += ae * dt
                    pn += vn * dt
                    pe += ve * dt

                    x, y = ned_to_initial_heading(pn, pe, yaw0)
                    vx, vy = ned_to_initial_heading(vn, ve, yaw0)
                    distance = math.hypot(x, y)

                    should_print = (
                        last_printed_distance is None
                        or abs(distance - last_printed_distance) >= args.deadband
                    )
                    heartbeat_due = (
                        args.heartbeat > 0.0 and now - last_print_t >= args.heartbeat
                    )
                    if should_print or heartbeat_due:
                        print(
                            f"x={x:8.3f} m | y={y:8.3f} m | dist={distance:8.3f} m | "
                            f"vx={vx:7.3f} vy={vy:7.3f} m/s | "
                            f"acc={horizontal_accel:6.3f} m/s^2 | src={sample.source}",
                            flush=True,
                        )
                        last_printed_distance = distance
                        last_print_t = now

    except KeyboardInterrupt:
        print(f"\nStopped. accel_samples={accel_count}", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
