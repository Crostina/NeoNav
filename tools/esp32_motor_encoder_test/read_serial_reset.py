#!/usr/bin/env python3
import argparse
import sys
import time

import serial


def reset_esp32(ser: serial.Serial) -> None:
    # CP210x/ESP32 dev boards commonly wire RTS to EN and DTR to GPIO0.
    # Keep GPIO0 high, pulse EN low, then release EN to boot the application.
    ser.dtr = False
    ser.rts = True
    time.sleep(0.12)
    ser.rts = False
    time.sleep(0.2)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=24.0)
    parser.add_argument("--no-reset", action="store_true")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        if not args.no_reset:
            reset_esp32(ser)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            data = ser.read(4096)
            if data:
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
