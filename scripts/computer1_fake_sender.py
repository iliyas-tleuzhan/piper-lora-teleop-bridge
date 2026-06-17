#!/usr/bin/env python3
"""Fake Piper master sender for the first LoRa transport test."""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time

import serial


DEADMAN_ENABLED = 0x01


def checksum16(payload: str) -> int:
    """Checksum shared with the Arduino sketches."""
    c = 0x1234
    for byte in payload.encode("ascii"):
        c = ((c << 5) | (c >> 11)) & 0xFFFF
        c ^= byte
    return c & 0xFFFF


def build_packet(seq: int, start_time: float) -> str:
    now = time.monotonic()
    elapsed = now - start_time
    time_ms = int(elapsed * 1000)

    # Smooth fake joint targets in degrees. These are not Piper limits.
    q_deg = [
        25.0 * math.sin(elapsed * 0.55),
        18.0 * math.sin(elapsed * 0.43 + 0.7),
        30.0 * math.sin(elapsed * 0.37 + 1.4),
        45.0 * math.sin(elapsed * 0.31 + 2.1),
        20.0 * math.sin(elapsed * 0.49 + 2.8),
        60.0 * math.sin(elapsed * 0.27 + 3.5),
    ]
    q_cd = [int(round(value * 100.0)) for value in q_deg]

    gripper_percent = 50.0 + 35.0 * math.sin(elapsed * 0.25)
    gripper_p100 = int(round(max(0.0, min(100.0, gripper_percent)) * 100.0))

    fields = [
        "PIPER",
        str(seq),
        str(time_ms),
        *(str(value) for value in q_cd),
        str(gripper_p100),
        str(DEADMAN_ENABLED),
    ]
    payload = ",".join(fields)
    return f"{payload},{checksum16(payload)}\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate fake Piper joint targets and send them to Board A over serial."
    )
    parser.add_argument("--port", required=True, help="Serial port, for example COM9 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--rate", type=float, default=5.0, help="Packets per second")
    parser.add_argument("--duration", type=float, default=None, help="Optional run time in seconds")
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=3.0,
        help="Seconds to wait after opening serial before sending. ESP32-S3 often resets on open.",
    )
    parser.add_argument(
        "--write-timeout",
        type=float,
        default=None,
        help="Serial write timeout in seconds. Default is None, meaning block until written.",
    )
    parser.add_argument(
        "--no-flush",
        action="store_true",
        help="Do not call serial flush() after each packet. Use this if writes block.",
    )
    return parser.parse_args()


def drain_board_debug(ser: serial.Serial, stop_event: threading.Event) -> None:
    """Continuously read and discard Board A debug output."""
    while not stop_event.is_set():
        try:
            ser.readline()
        except serial.SerialException:
            return


def main() -> int:
    args = parse_args()
    if args.rate <= 0:
        print("--rate must be greater than 0", file=sys.stderr)
        return 2
    if args.startup_delay < 0:
        print("--startup-delay must be 0 or greater", file=sys.stderr)
        return 2
    if args.write_timeout is not None and args.write_timeout < 0:
        print("--write-timeout must be 0 or greater", file=sys.stderr)
        return 2

    stop_requested = False
    reader_stop = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)

    period = 1.0 / args.rate
    seq = 0
    print(f"Opening {args.port} at {args.baud} baud")
    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=0.2,
            write_timeout=args.write_timeout,
        ) as ser:
            reader = threading.Thread(
                target=drain_board_debug,
                args=(ser, reader_stop),
                daemon=True,
            )
            reader.start()

            print(
                f"Waiting {args.startup_delay:.1f} seconds for ESP32-S3 serial reset/startup..."
            )
            time.sleep(args.startup_delay)

            start_time = time.monotonic()
            next_send = start_time
            print(f"Sending fake PIPER packets at {args.rate:.2f} Hz. Press Ctrl+C to stop.")
            while not stop_requested:
                now = time.monotonic()
                if args.duration is not None and now - start_time >= args.duration:
                    break

                if now < next_send:
                    time.sleep(min(0.02, next_send - now))
                    continue

                line = build_packet(seq, start_time)
                try:
                    ser.write(line.encode("ascii"))
                    if not args.no_flush:
                        ser.flush()
                except serial.SerialTimeoutException as exc:
                    print(
                        "Serial write timed out. Check that Board A is running the "
                        "BoardA_SerialToLoRa sketch, the --port value is correct, "
                        "Arduino Serial Monitor is closed, and the board has been reset. "
                        f"Original error: {exc}",
                        file=sys.stderr,
                    )
                    return 1
                print(f"TX {line.strip()}")

                seq += 1
                next_send += period
    except serial.SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 1
    finally:
        reader_stop.set()

    print("Sender stopped cleanly.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
