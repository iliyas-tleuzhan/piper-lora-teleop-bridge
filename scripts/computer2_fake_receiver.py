#!/usr/bin/env python3
"""Fake Piper slave receiver for the first LoRa transport test."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import serial


DEADMAN_ENABLED = 0x01
STALE_REPEAT_SECONDS = 3.0


@dataclass
class PiperPacket:
    seq: int
    sender_time_ms: int
    q_cd: list[int]
    gripper_p100: int
    flags: int


def checksum16(payload: str) -> int:
    c = 0x1234
    for byte in payload.encode("ascii"):
        c = ((c << 5) | (c >> 11)) & 0xFFFF
        c ^= byte
    return c & 0xFFFF


def parse_piper_line(line: str) -> PiperPacket:
    parts = line.split(",")
    if len(parts) != 12:
        raise ValueError(f"expected 12 comma-separated fields, got {len(parts)}")
    if parts[0] != "PIPER":
        raise ValueError("missing PIPER header")

    payload = ",".join(parts[:-1])
    expected = checksum16(payload)
    received = int(parts[-1])
    if received != expected:
        raise ValueError(f"checksum mismatch received={received} expected={expected}")

    seq = int(parts[1])
    sender_time_ms = int(parts[2])
    q_cd = [int(value) for value in parts[3:9]]
    gripper_p100 = int(parts[9])
    flags = int(parts[10])

    return PiperPacket(seq, sender_time_ms, q_cd, gripper_p100, flags)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read Board B serial output and print fake slave Piper CAN commands."
    )
    parser.add_argument("--port", required=True, help="Serial port, for example COM10 or /dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=1.0,
        help="Seconds without a valid packet before declaring stale",
    )
    return parser.parse_args()


def print_packet(packet: PiperPacket, local_start: float, now: float) -> None:
    q_deg = [value / 100.0 for value in packet.q_cd]
    gripper_percent = packet.gripper_p100 / 100.0
    local_elapsed_ms = int((now - local_start) * 1000.0)
    age_ms = local_elapsed_ms - packet.sender_time_ms
    age_text = f"{age_ms} ms" if age_ms >= 0 else "unknown"

    deadman_ok = bool(packet.flags & DEADMAN_ENABLED)
    deadman_text = "enabled" if deadman_ok else "DISABLED"
    q_text = ", ".join(f"q{i + 1}={value:7.2f} deg" for i, value in enumerate(q_deg))

    print(
        f"RX seq={packet.seq} age={age_text} deadman={deadman_text} "
        f"{q_text}, gripper={gripper_percent:6.2f}%"
    )
    if deadman_ok:
        print("Would send CAN command to slave Piper")
    else:
        print("Deadman disabled: fake slave would stop/freeze now.")


def main() -> int:
    args = parse_args()
    if args.stale_timeout <= 0:
        print("--stale-timeout must be greater than 0", file=sys.stderr)
        return 2

    last_valid_packet_at: float | None = None
    last_stale_print_at = 0.0
    stale = False
    local_start = time.monotonic()

    print(f"Opening {args.port} at {args.baud} baud")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
            print("Waiting for valid PIPER packets from Board B. Press Ctrl+C to stop.")
            while True:
                now = time.monotonic()
                raw = ser.readline()
                if raw:
                    text = raw.decode("ascii", errors="replace").strip()
                    if not text or text.startswith("#"):
                        continue
                    if not text.startswith("PIPER,"):
                        print(f"Ignoring non-PIPER line: {text}")
                        continue

                    try:
                        packet = parse_piper_line(text)
                    except ValueError as exc:
                        print(f"Invalid PIPER packet ignored: {exc}; line={text}")
                        continue

                    last_valid_packet_at = now
                    stale = False
                    print_packet(packet, local_start, now)

                now = time.monotonic()
                if last_valid_packet_at is None:
                    stale_age = now - local_start
                else:
                    stale_age = now - last_valid_packet_at

                if stale_age > args.stale_timeout:
                    if (not stale) or (now - last_stale_print_at >= STALE_REPEAT_SECONDS):
                        print("STALE: fake slave would stop/freeze now.")
                        last_stale_print_at = now
                    stale = True
    except KeyboardInterrupt:
        print("\nReceiver stopped cleanly.")
        return 0
    except serial.SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
