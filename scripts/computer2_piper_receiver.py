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
    deg_to_raw,
    limit_step_raw,
    raw_to_deg,
)


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read raw Piper teleop packets from Board B serial and command the "
            "slave Piper over CAN."
        )
    )
    parser.add_argument("--port", required=True, help="Board B serial port, for example /dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200, help="Board B serial baud rate")
    parser.add_argument("--can", default="can0", help="Slave Piper CAN interface")
    parser.add_argument("--confirm", default="", help="Must be MOVE to allow robot motion")
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.5,
        help="Seconds without a valid live packet before warning and holding last command.",
    )
    parser.add_argument("--status-rate", type=float, default=2.0, help="Status print rate in Hz")
    parser.add_argument("--control-mode", type=parse_int, default=0x01, help="Piper control mode")
    parser.add_argument("--move-mode", type=parse_int, default=0x01, help="Piper move mode")
    parser.add_argument("--speed-percent", type=int, default=100, help="Piper speed percent")
    parser.add_argument(
        "--follow-mode",
        type=parse_int,
        default=0xAD,
        help="Piper follow/high-follow mode, default 0xAD",
    )
    parser.add_argument(
        "--enable-slew-limit",
        action="store_true",
        help="Limit each accepted command step. Disabled by default for direct teleop feel.",
    )
    parser.add_argument("--max-step-deg", type=float, default=3.0, help="Slew-limit max step")
    parser.add_argument(
        "--gripper-default-effort",
        type=int,
        default=1000,
        help="Fallback gripper effort if a packet omits effort.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate packets and print commands without connecting to or moving Piper.",
    )
    parser.add_argument(
        "--disable-on-exit",
        action="store_true",
        help="Disable all Piper motors when the script exits.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> int:
    if args.confirm != "MOVE" and not args.dry_run:
        print("Refusing to move robot. Re-run with --confirm MOVE.", file=sys.stderr)
        return 2
    if args.stale_timeout <= 0:
        print("--stale-timeout must be greater than 0", file=sys.stderr)
        return 2
    if args.status_rate < 0:
        print("--status-rate must be 0 or greater", file=sys.stderr)
        return 2
    if not 0 <= args.speed_percent <= 100:
        print("--speed-percent must be between 0 and 100", file=sys.stderr)
        return 2
    if args.max_step_deg < 0:
        print("--max-step-deg must be 0 or greater", file=sys.stderr)
        return 2
    if args.gripper_default_effort < 0:
        print("--gripper-default-effort must be 0 or greater", file=sys.stderr)
        return 2
    return 0


def choose_command_joints(
    *,
    last_commanded_joints: list[int] | None,
    target_joints: list[int],
    enable_slew_limit: bool,
    max_step_deg: float,
) -> list[int]:
    if not enable_slew_limit or last_commanded_joints is None:
        return clamp_joints_raw(target_joints)

    return clamp_joints_raw(
        limit_step_raw(last_commanded_joints, target_joints, deg_to_raw(max_step_deg))
    )


def warn_if_receiver_timeout(
    tracker: SlavePacketTracker,
    status: RateLimitedPrinter,
    receiver_timeout_s: float,
) -> None:
    now = time.monotonic()
    if not tracker.timeout_expired(now, receiver_timeout_s):
        return
    idle_s = tracker.seconds_since_valid_packet(now)
    idle_text = receiver_timeout_s if idle_s is None else idle_s
    status.print(f"[SLAVE] No valid packets for {idle_text:.2f}s; holding last command")


def connect_piper(args: argparse.Namespace) -> PiperArm | None:
    if args.dry_run:
        return None

    arm = PiperArm(args.can)
    arm.connect()
    arm.enable_all()
    arm.configure_motion(
        control_mode=args.control_mode,
        move_mode=args.move_mode,
        speed_percent=args.speed_percent,
        follow_mode=args.follow_mode,
    )
    return arm


def main() -> int:
    args = parse_args()
    if exit_code := validate_args(args):
        return exit_code

    status = RateLimitedPrinter(args.status_rate)
    tracker = SlavePacketTracker()
    last_commanded_joints: list[int] | None = None

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
        with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
            print("[SLAVE] Waiting for raw Piper LoRa teleop packets. Press Ctrl+C to stop.")
            while True:
                raw = ser.readline()
                if not raw:
                    warn_if_receiver_timeout(tracker, status, args.stale_timeout)
                    continue

                text = raw.decode("ascii", errors="replace").strip()
                if not text or text.startswith("#"):
                    warn_if_receiver_timeout(tracker, status, args.stale_timeout)
                    continue
                if not text.startswith("PIPER,"):
                    status.print(f"[SLAVE] Ignoring non-PIPER line: {text}")
                    warn_if_receiver_timeout(tracker, status, args.stale_timeout)
                    continue

                receiver_time_s = time.monotonic()
                try:
                    packet = parse_piper_teleop_line(text)
                except ValueError as exc:
                    status.print(f"[SLAVE] Ignoring malformed LoRa packet: {exc}")
                    warn_if_receiver_timeout(tracker, status, args.stale_timeout)
                    continue

                decision = tracker.process_packet(packet, receiver_time_s)
                if decision.warning:
                    status.print(f"[SLAVE] {decision.warning}")
                if not decision.accepted:
                    status.print(f"[SLAVE] Ignoring packet: {decision.reason}")
                    warn_if_receiver_timeout(tracker, status, args.stale_timeout)
                    continue

                target_joints = decision.target_joints
                if target_joints is None:
                    status.print("[SLAVE] Ignoring packet: missing joints")
                    continue

                next_joints = choose_command_joints(
                    last_commanded_joints=last_commanded_joints,
                    target_joints=target_joints,
                    enable_slew_limit=args.enable_slew_limit,
                    max_step_deg=args.max_step_deg,
                )

                if arm is not None:
                    arm.write_joints_raw(next_joints, dry_run=args.dry_run)
                    if decision.gripper is not None:
                        arm.write_gripper_raw(
                            decision.gripper,
                            default_effort=args.gripper_default_effort,
                            dry_run=args.dry_run,
                        )

                last_commanded_joints = next_joints
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
        if args.disable_on_exit and arm is not None:
            try:
                arm.disable_all()
            except Exception as exc:  # noqa: BLE001
                print(f"Warning: failed to disable Piper on exit: {exc}", file=sys.stderr)
        if arm is not None:
            arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
