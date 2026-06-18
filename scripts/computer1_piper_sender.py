#!/usr/bin/env python3
"""Real Piper master sender: CAN state -> Board A serial -> LoRa."""

from __future__ import annotations

import argparse
import subprocess
import signal
import sys
import threading
import time

import serial

from piper_lora_protocol import DEADMAN_ENABLED, build_piper_line, mdeg_to_cd
from piper_sdk_adapter import PiperArm, PiperSdkUnavailable, gripper_um_to_p100


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read a Piper master arm over CAN and send compact joint targets to Board A."
    )
    parser.add_argument("--port", required=True, help="Board A serial port, for example COM9")
    parser.add_argument("--baud", type=int, default=115200, help="Board A serial baud rate")
    parser.add_argument("--can", default="can0", help="Piper CAN interface name")
    parser.add_argument("--rate", type=float, default=5.0, help="LoRa target packets per second")
    parser.add_argument(
        "--source",
        choices=("control", "feedback"),
        default="control",
        help="Use control frames for a master arm, or feedback frames for a slave/normal arm.",
    )
    parser.add_argument(
        "--configure-master",
        action="store_true",
        help="Send MasterSlaveConfig(0xFA, 0, 0, 0) before reading.",
    )
    parser.add_argument(
        "--gripper-max-mm",
        type=float,
        default=70.0,
        help="Gripper travel that maps to 100 percent in the LoRa packet.",
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
    parser.add_argument(
        "--can-ok-timeout",
        type=float,
        default=0.0,
        help=(
            "Seconds to wait for piper_sdk isOk() before exiting. "
            "0 means wait forever and print diagnostics periodically."
        ),
    )
    parser.add_argument(
        "--ignore-can-ok",
        action="store_true",
        help="Send packets even when piper_sdk isOk() is false. Use only after verifying CAN reads work.",
    )
    return parser.parse_args()


def drain_board_debug(ser: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            ser.readline()
        except serial.SerialException:
            return


def validate_args(args: argparse.Namespace) -> int:
    if args.rate <= 0:
        print("--rate must be greater than 0", file=sys.stderr)
        return 2
    if args.gripper_max_mm <= 0:
        print("--gripper-max-mm must be greater than 0", file=sys.stderr)
        return 2
    if args.startup_delay < 0:
        print("--startup-delay must be 0 or greater", file=sys.stderr)
        return 2
    if args.write_timeout is not None and args.write_timeout < 0:
        print("--write-timeout must be 0 or greater", file=sys.stderr)
        return 2
    if args.can_ok_timeout < 0:
        print("--can-ok-timeout must be 0 or greater", file=sys.stderr)
        return 2
    return 0


def can_link_diagnostics(can_name: str) -> str:
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", can_name],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return f"Could not run `ip -details link show {can_name}`: {exc}"

    output = (result.stdout or result.stderr).strip()
    if not output:
        output = f"`ip -details link show {can_name}` returned no output"
    return output


def print_can_not_ok(can_name: str, *, elapsed: float, timeout: float) -> None:
    timeout_text = "no timeout" if timeout == 0 else f"timeout={timeout:.1f}s"
    print(
        f"Piper SDK CAN reader is not OK yet "
        f"(waited {elapsed:.1f}s, {timeout_text}). No LoRa packet sent.",
        file=sys.stderr,
    )
    print(can_link_diagnostics(can_name), file=sys.stderr)
    print(
        "Check: CAN adapter is connected, Piper is powered, interface is UP at "
        "1000000 bitrate, and the source mode is correct. For a master arm, try "
        "`--configure-master`; for a normal/slave arm, try `--source feedback`.",
        file=sys.stderr,
    )


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
        arm = PiperArm(args.can)
        arm.connect()
        if args.configure_master:
            arm.configure_master_input()
    except (PiperSdkUnavailable, RuntimeError, OSError) as exc:
        print(f"Piper CAN setup failed: {exc}", file=sys.stderr)
        return 1

    period = 1.0 / args.rate
    seq = 0
    start_time = time.monotonic()
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

            next_send = time.monotonic()
            print(
                f"Streaming real Piper {args.source} state from {args.can} "
                f"to {args.port} at {args.rate:.2f} Hz. Press Ctrl+C to stop."
            )
            can_wait_started = time.monotonic()
            last_can_warning_at = 0.0
            while not stop_requested:
                now = time.monotonic()
                if args.duration is not None and now - start_time >= args.duration:
                    break
                if now < next_send:
                    time.sleep(min(0.02, next_send - now))
                    continue

                if (not args.ignore_can_ok) and (not arm.is_ok()):
                    elapsed = now - can_wait_started
                    if now - last_can_warning_at >= 3.0:
                        print_can_not_ok(
                            args.can,
                            elapsed=elapsed,
                            timeout=args.can_ok_timeout,
                        )
                        last_can_warning_at = now
                    if args.can_ok_timeout > 0 and elapsed >= args.can_ok_timeout:
                        return 1
                    time.sleep(0.1)
                    continue

                state = (
                    arm.read_control_state()
                    if args.source == "control"
                    else arm.read_feedback_state()
                )
                time_ms = int((now - start_time) * 1000.0)
                line = build_piper_line(
                    seq,
                    time_ms,
                    mdeg_to_cd(state.q_mdeg),
                    gripper_um_to_p100(state.gripper_um, args.gripper_max_mm),
                    DEADMAN_ENABLED,
                )
                try:
                    ser.write(line.encode("ascii"))
                    if not args.no_flush:
                        ser.flush()
                except serial.SerialTimeoutException as exc:
                    print(f"Serial write timed out: {exc}", file=sys.stderr)
                    return 1

                q_deg = [value / 1000.0 for value in state.q_mdeg]
                print(
                    f"TX seq={seq} "
                    + " ".join(f"q{i + 1}={value:.2f}" for i, value in enumerate(q_deg))
                    + f" gripper={state.gripper_um / 1000.0:.1f}mm"
                )
                seq += 1
                next_send += period
    except serial.SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 1
    finally:
        reader_stop.set()
        arm.disconnect()

    print("Real Piper sender stopped cleanly.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
