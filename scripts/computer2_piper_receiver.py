#!/usr/bin/env python3
"""Computer 2: Board B serial LoRa packets -> raw Piper slave CAN commands."""

from __future__ import annotations

import argparse
import sys
import time

import serial

from piper_lora_protocol import parse_piper_teleop_line
from piper_sdk_adapter import PiperArm, PiperSdkUnavailable
from piper_teleop_core import (
    RateLimitedPrinter,
    SlavePacketTracker,
    clamp_joints_raw,
    raw_to_deg,
)

SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 0.005
STALE_TIMEOUT_S = 0.5
STATUS_RATE_HZ = 2.0
CONTROL_MODE = 0x01
MOVE_MODE = 0x01
SPEED_PERCENT = 100
FOLLOW_MODE = 0xAD
GRIPPER_DEFAULT_EFFORT = 1000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read raw Piper teleop packets from Board B serial and command the "
            "slave Piper over CAN."
        )
    )
    parser.add_argument("--port", required=True, help="Board B serial port, for example /dev/ttyACM1")
    parser.add_argument("--can", default="can0", help="Slave Piper CAN interface")
    parser.add_argument("--confirm", default="", help="Must be MOVE to allow robot motion")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate packets and print commands without connecting to or moving Piper.",
    )
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help=argparse.SUPPRESS)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> int:
    if args.confirm != "MOVE" and not args.dry_run:
        print("Refusing to move robot. Re-run with --confirm MOVE.", file=sys.stderr)
        return 2
    return 0


def warn_if_receiver_timeout(
    tracker: SlavePacketTracker,
    status: RateLimitedPrinter,
) -> None:
    now = time.monotonic()
    if not tracker.timeout_expired(now, STALE_TIMEOUT_S):
        return
    idle_s = tracker.seconds_since_valid_packet(now)
    idle_text = STALE_TIMEOUT_S if idle_s is None else idle_s
    status.print(f"[SLAVE] No valid packets for {idle_text:.2f}s; holding last command")


def connect_piper(args: argparse.Namespace) -> PiperArm | None:
    if args.dry_run:
        return None

    arm = PiperArm(args.can)
    arm.connect()
    arm.enable_all()
    arm.configure_motion(
        control_mode=CONTROL_MODE,
        move_mode=MOVE_MODE,
        speed_percent=SPEED_PERCENT,
        follow_mode=FOLLOW_MODE,
    )
    return arm


def main() -> int:
    args = parse_args()
    if exit_code := validate_args(args):
        return exit_code

    status = RateLimitedPrinter(STATUS_RATE_HZ)
    tracker = SlavePacketTracker()

    try:
        arm = connect_piper(args)
    except (PiperSdkUnavailable, RuntimeError, OSError, AttributeError) as exc:
        print(f"Piper CAN setup failed: {exc}", file=sys.stderr)
        return 1

    print(f"Opening Board B serial {args.port} at {args.baud} baud")
    if args.dry_run:
        print("[SLAVE] dry-run: not connecting to Piper and not writing CAN commands")
    else:
        print(f"[SLAVE] Arm enabled on {args.can}; motion mode configured")

    try:
        with serial.Serial(args.port, args.baud, timeout=SERIAL_TIMEOUT_S) as ser:
            print("[SLAVE] Waiting for raw Piper LoRa teleop packets. Press Ctrl+C to stop.")
            while True:
                raw = ser.readline()
                if not raw:
                    warn_if_receiver_timeout(tracker, status)
                    continue

                text = raw.decode("ascii", errors="replace").strip()
                if not text or text.startswith("#"):
                    warn_if_receiver_timeout(tracker, status)
                    continue
                if not text.startswith("PIPER,"):
                    status.print(f"[SLAVE] Ignoring non-PIPER line: {text}")
                    warn_if_receiver_timeout(tracker, status)
                    continue

                receiver_time_s = time.monotonic()
                try:
                    packet = parse_piper_teleop_line(text)
                except ValueError as exc:
                    status.print(f"[SLAVE] Ignoring malformed LoRa packet: {exc}")
                    warn_if_receiver_timeout(tracker, status)
                    continue

                decision = tracker.process_packet(packet, receiver_time_s)
                if decision.warning:
                    status.print(f"[SLAVE] {decision.warning}")
                if not decision.accepted:
                    status.print(f"[SLAVE] Ignoring packet: {decision.reason}")
                    warn_if_receiver_timeout(tracker, status)
                    continue

                target_joints = decision.target_joints
                if target_joints is None:
                    status.print("[SLAVE] Ignoring packet: missing joints")
                    continue

                next_joints = clamp_joints_raw(target_joints)

                if arm is not None:
                    arm.write_joints_raw(next_joints, dry_run=args.dry_run)
                    if decision.gripper is not None:
                        arm.write_gripper_raw(
                            decision.gripper,
                            default_effort=GRIPPER_DEFAULT_EFFORT,
                            dry_run=args.dry_run,
                        )

                command_rate_hz = tracker.command_rate_hz(time.monotonic())
                command_rate_text = "unknown" if command_rate_hz is None else f"{command_rate_hz:.1f}Hz"
                status.print(
                    f"[SLAVE] accepted seq={decision.sequence} "
                    f"dropped={decision.dropped} total_dropped={decision.total_dropped} "
                    f"cmd_rate={command_rate_text} "
                    f"target_deg={[round(raw_to_deg(value), 3) for value in target_joints]} "
                    f"cmd_deg={[round(raw_to_deg(value), 3) for value in next_joints]} "
                    f"gripper={decision.gripper}"
                )
    except KeyboardInterrupt:
        print("\n[SLAVE] stopped")
        return 0
    except serial.SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 1
    finally:
        if arm is not None:
            arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
