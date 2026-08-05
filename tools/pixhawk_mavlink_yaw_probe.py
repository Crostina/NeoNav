#!/usr/bin/env python3
"""Minimal MAVLink yaw probe for a Pixhawk serial link.

This intentionally avoids pymavlink so it can run on a fresh Raspberry Pi.
It decodes enough MAVLink v1/v2 framing to print yaw/heading from common
ArduPilot messages.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass

import serial


@dataclass
class MavFrame:
    msgid: int
    sysid: int
    compid: int
    payload: bytes


def parse_frames(buffer: bytearray) -> list[MavFrame]:
    frames: list[MavFrame] = []
    i = 0
    while i < len(buffer):
        magic = buffer[i]
        if magic not in (0xFE, 0xFD):
            i += 1
            continue

        if magic == 0xFE:
            header_len = 6
            if i + header_len > len(buffer):
                break
            payload_len = buffer[i + 1]
            frame_len = header_len + payload_len + 2
            if i + frame_len > len(buffer):
                break
            sysid = buffer[i + 3]
            compid = buffer[i + 4]
            msgid = buffer[i + 5]
            payload = bytes(buffer[i + 6 : i + 6 + payload_len])
        else:
            header_len = 10
            if i + header_len > len(buffer):
                break
            payload_len = buffer[i + 1]
            incompat_flags = buffer[i + 2]
            signature_len = 13 if incompat_flags & 0x01 else 0
            frame_len = header_len + payload_len + 2 + signature_len
            if i + frame_len > len(buffer):
                break
            sysid = buffer[i + 5]
            compid = buffer[i + 6]
            msgid = buffer[i + 7] | (buffer[i + 8] << 8) | (buffer[i + 9] << 16)
            payload = bytes(buffer[i + 10 : i + 10 + payload_len])

        frames.append(MavFrame(msgid=msgid, sysid=sysid, compid=compid, payload=payload))
        i += frame_len

    if i:
        del buffer[:i]
    return frames


def deg(rad: float) -> float:
    return math.degrees(rad)


def decode_yaw(frame: MavFrame) -> str | None:
    if frame.msgid == 30 and len(frame.payload) >= 28:  # ATTITUDE
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed = struct.unpack_from(
            "<Iffffff", frame.payload
        )
        return (
            f"ATTITUDE sys={frame.sysid} comp={frame.compid} "
            f"t={time_boot_ms} yaw={deg(yaw):8.3f} deg "
            f"roll={deg(roll):7.3f} pitch={deg(pitch):7.3f} "
            f"yawspeed={deg(yawspeed):7.3f} deg/s"
        )

    if frame.msgid == 74 and len(frame.payload) >= 20:  # VFR_HUD
        heading = struct.unpack_from("<h", frame.payload, 8)[0]
        return f"VFR_HUD sys={frame.sysid} comp={frame.compid} heading={heading:4d} deg"

    if frame.msgid == 33 and len(frame.payload) >= 28:  # GLOBAL_POSITION_INT
        hdg = struct.unpack_from("<H", frame.payload, 26)[0]
        if hdg != 65535:
            return f"GLOBAL_POSITION_INT sys={frame.sysid} comp={frame.compid} heading={hdg / 100.0:8.3f} deg"

    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--max-lines", type=int, default=40)
    args = parser.parse_args()

    buffer = bytearray()
    counts: dict[int, int] = {}
    printed = 0
    deadline = time.monotonic() + args.seconds

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        while time.monotonic() < deadline:
            buffer.extend(ser.read(512))
            for frame in parse_frames(buffer):
                counts[frame.msgid] = counts.get(frame.msgid, 0) + 1
                line = decode_yaw(frame)
                if line and printed < args.max_lines:
                    print(line)
                    printed += 1

    print("message_counts", " ".join(f"{msgid}:{count}" for msgid, count in sorted(counts.items())))
    return 0 if printed else 1


if __name__ == "__main__":
    raise SystemExit(main())
