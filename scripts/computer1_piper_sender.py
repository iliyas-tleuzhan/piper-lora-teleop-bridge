#!/usr/bin/env python3
"""Computer 1: raw master Piper CAN commands -> Board A serial -> LoRa."""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time

import serial

from piper_lora_protocol import build_piper_teleop_line
from piper_teleop_core import (
    MASTER_CAN_IDS,
    MasterCommandState,
    RateLimitedPrinter,
    decode_master_frame,
    raw_to_deg,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read master Piper SocketCAN command frames and send raw joint targets "
            "to Board A over serial for LoRa transport."
        )
    )
    parser.add_argument("--port", required=True, help="Board A serial port, for example /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Board A serial baud rate")
    parser.add_argument("--can", default="can0", help="Master Piper SocketCAN interface")
    parser.add_argument("--rate", type=float, default=5.0, help="LoRa target packets per second")
    parser.add_argument("--deadman", action="store_true", help="Set packet deadman=true")
    parser.add_argument(
        "--can-timeout",
        type=float,
        default=0.02,
        help="SocketCAN receive timeout in seconds",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=2.0,
        help="Human-readable status print rate in Hz",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=3.0,
        help="Seconds to wait after opening serial before sending.",
    )
    parser.add_argument("--duration", type=float, default=None, help="Optional run time in seconds")
    parser.add_argument("--write-timeout", type=float, default=None, help="Serial write timeout")
    parser.add_argument("--no-flush", action="store_true", help="Do not serial flush after each packet")
    parser.add_argument("--verbose-packets", action="store_true", help="Print every LoRa packet line")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> int:
    if args.rate <= 0:
        print("--rate must be greater than 0", file=sys.stderr)
        return 2
    if args.can_timeout < 0:
        print("--can-timeout must be 0 or greater", file=sys.stderr)
        return 2
    if args.status_rate < 0:
        print("--status-rate must be 0 or greater", file=sys.stderr)
        return 2
    if args.startup_delay < 0:
        print("--startup-delay must be 0 or greater", file=sys.stderr)
        return 2
    if args.write_timeout is not None and args.write_timeout < 0:
        print("--write-timeout must be 0 or greater", file=sys.stderr)
        return 2
    return 0


def drain_board_debug(ser: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            ser.readline()
        except serial.SerialException:
            return


def open_can_bus(can_interface: str):
    try:
        import can
    except ImportError as exc:
        raise RuntimeError(
            "python-can is not installed. Run `pip install -r requirements.txt`."
        ) from exc

    return can.interface.Bus(channel=can_interface, interface="socketcan")


def main() -> int:
    args = parse_args()
    if exit_code := validate_args(args):
        return exit_code

    stop_requested = False
    reader_stop = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)

    try:
        bus = open_can_bus(args.can)
    except (RuntimeError, OSError) as exc:
        print(f"CAN setup failed on {args.can}: {exc}", file=sys.stderr)
        return 1

    period = 1.0 / args.rate
    seq = 0
    state = MasterCommandState()
    status = RateLimitedPrinter(args.status_rate)
    start_time = time.monotonic()
    next_send = start_time

    print(f"Opening Board A serial {args.port} at {args.baud} baud")
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
            print(f"Waiting {args.startup_delay:.1f} seconds for ESP32 serial startup...")
            time.sleep(args.startup_delay)

            print(f"[MASTER] Reading Piper command CAN frames from {args.can}")
            print(f"[MASTER] Sending raw LoRa teleop packets to {args.port} at {args.rate:.2f} Hz")
            print(f"[MASTER] deadman={args.deadman}; receiver will ignore packets unless this is true")
            print("[MASTER] Waiting for complete 0x155/0x156/0x157 joint target set")

            while not stop_requested:
                now = time.monotonic()
                if args.duration is not None and now - start_time >= args.duration:
                    break

                try:
                    message = bus.recv(timeout=args.can_timeout)
                except OSError as exc:
                    print(f"CAN read failed on {args.can}: {exc}", file=sys.stderr)
                    return 1

                if message is not None and int(message.arbitration_id) in MASTER_CAN_IDS:
                    decode_master_frame(message, state)

                now = time.monotonic()
                if now < next_send:
                    continue

                if not state.has_full_joint_target():
                    status.print("[MASTER] Waiting for complete joint target frames")
                    next_send = now + period
                    continue

                sender_time_ms = int((now - start_time) * 1000.0)
                line = build_piper_teleop_line(
                    seq,
                    sender_time_ms,
                    state.joints_raw(),
                    deadman=args.deadman,
                    gripper=state.gripper,
                )
                try:
                    ser.write(line.encode("ascii"))
                    if not args.no_flush:
                        ser.flush()
                except serial.SerialTimeoutException as exc:
                    print(f"Serial write timed out: {exc}", file=sys.stderr)
                    return 1

                if args.verbose_packets:
                    print(f"[MASTER] TX {line.strip()}", flush=True)
                else:
                    status.print(
                        f"[MASTER] seq={seq} deadman={args.deadman} "
                        f"deg={[round(raw_to_deg(value), 3) for value in state.joints_raw()]} "
                        f"gripper={state.gripper}"
                    )

                seq += 1
                next_send = now + period
    except serial.SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 1
    finally:
        reader_stop.set()
        shutdown = getattr(bus, "shutdown", None)
        if callable(shutdown):
            shutdown()

    print("Real Piper sender stopped cleanly.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
