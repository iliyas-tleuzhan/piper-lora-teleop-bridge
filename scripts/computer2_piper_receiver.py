#!/usr/bin/env python3
"""Real Piper slave receiver: Board B serial -> LoRa target -> CAN commands."""

from __future__ import annotations

import argparse
import sys
import time

import serial

from piper_lora_protocol import (
    DEADMAN_ENABLED,
    PiperPacket,
    cd_to_degrees,
    cd_to_mdeg,
    parse_piper_line,
)
from piper_sdk_adapter import PiperArm, PiperSdkUnavailable, PiperState, gripper_p100_to_um


STALE_REPEAT_SECONDS = 3.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read Board B LoRa packets and command a real slave Piper over CAN."
    )
    parser.add_argument("--port", required=True, help="Board B serial port, for example COM10")
    parser.add_argument("--baud", type=int, default=115200, help="Board B serial baud rate")
    parser.add_argument("--can", default="can0", help="Slave Piper CAN interface name")
    parser.add_argument("--command-rate", type=float, default=20.0, help="CAN command rate in Hz")
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=1.0,
        help="Seconds without a valid live packet before stopping commands.",
    )
    parser.add_argument(
        "--smoothing",
        type=float,
        default=0.35,
        help="Low-pass factor from 0 to 1. Higher follows incoming packets faster.",
    )
    parser.add_argument("--speed-percent", type=int, default=50, help="Piper joint mode speed percent")
    parser.add_argument(
        "--high-follow",
        action="store_true",
        help="Use Piper high-follow mode for slave output.",
    )
    parser.add_argument(
        "--configure-slave",
        action="store_true",
        help="Send MasterSlaveConfig(0xFC, 0, 0, 0) before controlling.",
    )
    parser.add_argument(
        "--enable-arm",
        action="store_true",
        help="Enable all Piper motors on startup. Without this, commands are sent but the script does not enable motors.",
    )
    parser.add_argument(
        "--disable-on-stale",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Disable all motors when packets become stale or deadman is off.",
    )
    parser.add_argument(
        "--disable-on-exit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Disable all motors when the script exits.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate packets and print target commands without writing CAN commands.",
    )
    parser.add_argument(
        "--gripper-max-mm",
        type=float,
        default=70.0,
        help="Gripper travel that maps to 100 percent in the LoRa packet.",
    )
    parser.add_argument(
        "--gripper-effort",
        type=int,
        default=1000,
        help="Gripper effort in 0.001 N/m.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> int:
    if args.command_rate <= 0:
        print("--command-rate must be greater than 0", file=sys.stderr)
        return 2
    if args.stale_timeout <= 0:
        print("--stale-timeout must be greater than 0", file=sys.stderr)
        return 2
    if not 0.0 <= args.smoothing <= 1.0:
        print("--smoothing must be between 0 and 1", file=sys.stderr)
        return 2
    if not 0 <= args.speed_percent <= 100:
        print("--speed-percent must be between 0 and 100", file=sys.stderr)
        return 2
    if args.gripper_max_mm <= 0:
        print("--gripper-max-mm must be greater than 0", file=sys.stderr)
        return 2
    if args.gripper_effort < 0:
        print("--gripper-effort must be 0 or greater", file=sys.stderr)
        return 2
    return 0


def packet_to_state(packet: PiperPacket, gripper_max_mm: float) -> PiperState:
    return PiperState(
        q_mdeg=cd_to_mdeg(packet.q_cd),
        gripper_um=gripper_p100_to_um(packet.gripper_p100, gripper_max_mm),
    )


def smooth_state(current: PiperState | None, target: PiperState, factor: float) -> PiperState:
    if current is None or factor >= 1.0:
        return target
    return PiperState(
        q_mdeg=[
            int(round(old + (new - old) * factor))
            for old, new in zip(current.q_mdeg, target.q_mdeg, strict=True)
        ],
        gripper_um=int(round(current.gripper_um + (target.gripper_um - current.gripper_um) * factor)),
    )


def print_packet(packet: PiperPacket) -> None:
    q_text = " ".join(
        f"q{i + 1}={value:.2f}" for i, value in enumerate(cd_to_degrees(packet.q_cd))
    )
    print(
        f"RX seq={packet.seq} deadman="
        f"{'enabled' if packet.flags & DEADMAN_ENABLED else 'DISABLED'} "
        f"{q_text} gripper={packet.gripper_p100 / 100.0:.1f}%"
    )


def main() -> int:
    args = parse_args()
    if exit_code := validate_args(args):
        return exit_code

    try:
        arm = PiperArm(args.can)
        arm.connect()
        if args.configure_slave:
            arm.configure_slave_output()
        arm.configure_joint_control(
            speed_percent=args.speed_percent,
            high_follow=args.high_follow,
        )
        if args.enable_arm and not args.dry_run:
            arm.enable_all()
    except (PiperSdkUnavailable, RuntimeError, OSError) as exc:
        print(f"Piper CAN setup failed: {exc}", file=sys.stderr)
        return 1

    target: PiperState | None = None
    current: PiperState | None = None
    last_valid_packet_at: float | None = None
    last_stale_print_at = 0.0
    stale = False
    disabled_for_stale = False
    local_start = time.monotonic()
    command_period = 1.0 / args.command_rate
    next_command = local_start

    print(f"Opening Board B serial {args.port} at {args.baud} baud")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
            print(
                f"Commanding real Piper on {args.can} at {args.command_rate:.1f} Hz. "
                "Press Ctrl+C to stop."
            )
            while True:
                now = time.monotonic()
                raw = ser.readline()
                if raw:
                    text = raw.decode("ascii", errors="replace").strip()
                    if text and not text.startswith("#"):
                        if not text.startswith("PIPER,"):
                            print(f"Ignoring non-PIPER line: {text}")
                        else:
                            try:
                                packet = parse_piper_line(text)
                            except ValueError as exc:
                                print(f"Invalid PIPER packet ignored: {exc}; line={text}")
                            else:
                                print_packet(packet)
                                if packet.flags & DEADMAN_ENABLED:
                                    target = packet_to_state(packet, args.gripper_max_mm)
                                    last_valid_packet_at = now
                                    stale = False
                                    disabled_for_stale = False
                                else:
                                    target = None
                                    last_valid_packet_at = None
                                    stale = True
                                    if args.disable_on_stale and not args.dry_run:
                                        arm.disable_all()
                                        disabled_for_stale = True
                                    print("Deadman disabled: slave commands stopped.")

                now = time.monotonic()
                stale_age = now - (last_valid_packet_at if last_valid_packet_at is not None else local_start)
                if stale_age > args.stale_timeout:
                    if (not stale) or (now - last_stale_print_at >= STALE_REPEAT_SECONDS):
                        print("STALE: no valid live packet; slave commands stopped.")
                        last_stale_print_at = now
                    stale = True
                    target = None
                    if args.disable_on_stale and not disabled_for_stale and not args.dry_run:
                        arm.disable_all()
                        disabled_for_stale = True

                if now >= next_command:
                    if target is not None and not stale:
                        if not arm.is_ok():
                            print("Piper SDK CAN reader is not OK", file=sys.stderr)
                            return 1
                        current = smooth_state(current, target, args.smoothing)
                        arm.write_state(
                            current,
                            gripper_effort=args.gripper_effort,
                            dry_run=args.dry_run,
                        )
                    next_command += command_period
                    if next_command < now - command_period:
                        next_command = now + command_period
    except KeyboardInterrupt:
        print("\nReal Piper receiver stopped cleanly.")
        return 0
    except serial.SerialException as exc:
        print(f"Serial error on {args.port}: {exc}", file=sys.stderr)
        return 1
    finally:
        if args.disable_on_exit and not args.dry_run:
            try:
                arm.disable_all()
            except Exception as exc:  # noqa: BLE001
                print(f"Warning: failed to disable Piper on exit: {exc}", file=sys.stderr)
        arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

