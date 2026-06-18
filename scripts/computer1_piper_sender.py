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
    MASTER_FEEDBACK_CAN_IDS,
    MasterCommandState,
    RateLimitedPrinter,
    decode_master_feedback_frame,
    raw_to_deg,
)

DEFAULT_SEND_RATE_HZ = 8.0
SERIAL_BAUD = 115200
CAN_RECV_TIMEOUT_S = 0.005
STATUS_RATE_HZ = 2.0
STARTUP_DELAY_S = 3.0
BOARD_TX_RECOVERY_S = 0.6
GRIPPER_REFRESH_S = 1.0
SOURCE_FRAME_TIMEOUT_S = 0.4


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read master Piper SocketCAN feedback frames and send raw joint targets "
            "to Board A over serial for LoRa transport."
        )
    )
    parser.add_argument("--port", required=True, help="Board A serial port, for example /dev/ttyACM0")
    parser.add_argument("--can", default="can0", help="Master Piper SocketCAN interface")
    parser.add_argument("--rate", type=float, default=DEFAULT_SEND_RATE_HZ, help=argparse.SUPPRESS)
    parser.add_argument("--deadman", action="store_true", default=True, help=argparse.SUPPRESS)
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help=argparse.SUPPRESS)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> int:
    if args.rate <= 0:
        print("--rate must be greater than 0", file=sys.stderr)
        return 2
    return 0


def drain_board_debug(
    ser: serial.Serial,
    stop_event: threading.Event,
    tx_ready: threading.Event,
) -> None:
    while not stop_event.is_set():
        try:
            text = ser.readline().decode("ascii", errors="replace").strip()
        except serial.SerialException:
            return
        if text in {"TX done", "TX timeout"}:
            tx_ready.set()


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
    tx_ready = threading.Event()
    tx_ready.set()

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
    status = RateLimitedPrinter(STATUS_RATE_HZ)
    start_time = time.monotonic()
    next_send = start_time
    last_send_at = 0.0
    last_sent_gripper: dict[str, int] | None = None
    last_gripper_sent_at = 0.0

    print(f"Opening Board A serial {args.port} at {args.baud} baud")
    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=0.2,
            write_timeout=None,
        ) as ser:
            reader = threading.Thread(
                target=drain_board_debug,
                args=(ser, reader_stop, tx_ready),
                daemon=True,
            )
            reader.start()
            print(f"Waiting {STARTUP_DELAY_S:.1f} seconds for ESP32 serial startup...")
            time.sleep(STARTUP_DELAY_S)

            print(f"[MASTER] Reading live Piper feedback CAN frames from {args.can}")
            print(f"[MASTER] Sending raw LoRa teleop packets to {args.port} at {args.rate:.2f} Hz")
            print("[MASTER] Waiting for fresh 0x2A5/0x2A6/0x2A7 joint feedback set")

            while not stop_requested:
                now = time.monotonic()

                try:
                    message = bus.recv(timeout=CAN_RECV_TIMEOUT_S)
                except OSError as exc:
                    print(f"CAN read failed on {args.can}: {exc}", file=sys.stderr)
                    return 1

                if message is not None and int(message.arbitration_id) in MASTER_FEEDBACK_CAN_IDS:
                    decode_master_feedback_frame(message, state)

                now = time.monotonic()
                if now < next_send:
                    continue

                if not tx_ready.is_set():
                    if now - last_send_at > BOARD_TX_RECOVERY_S:
                        tx_ready.set()
                    else:
                        continue

                if not state.has_fresh_joint_target(now, SOURCE_FRAME_TIMEOUT_S):
                    status.print("[MASTER] Waiting for fresh joint feedback frames")
                    next_send = now + period
                    continue

                sender_time_ms = int((now - start_time) * 1000.0)
                gripper_to_send = None
                if state.gripper is not None:
                    gripper_changed = state.gripper != last_sent_gripper
                    gripper_refresh_due = now - last_gripper_sent_at >= GRIPPER_REFRESH_S
                    if gripper_changed or gripper_refresh_due:
                        gripper_to_send = state.gripper

                line = build_piper_teleop_line(
                    seq,
                    sender_time_ms,
                    state.joints_raw(),
                    deadman=True,
                    gripper=gripper_to_send,
                )
                try:
                    tx_ready.clear()
                    ser.write(line.encode("ascii"))
                    ser.flush()
                except serial.SerialTimeoutException as exc:
                    print(f"Serial write timed out: {exc}", file=sys.stderr)
                    return 1

                last_send_at = now
                if gripper_to_send is not None:
                    last_sent_gripper = dict(gripper_to_send)
                    last_gripper_sent_at = now

                status.print(
                    f"[MASTER] seq={seq} "
                    f"deg={[round(raw_to_deg(value), 3) for value in state.joints_raw()]} "
                    f"gripper={'sent' if gripper_to_send is not None else 'unchanged'}"
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
