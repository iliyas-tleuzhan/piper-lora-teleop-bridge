#!/usr/bin/env python3
"""Computer 2: Board B serial LoRa packets -> raw Piper slave CAN commands."""

from __future__ import annotations

import argparse
import sys
import time

import serial

from piper_lora_protocol import PiperTeleopPacket, extract_piper_teleop_packets
from piper_sdk_adapter import PiperArm, PiperSdkUnavailable
from piper_teleop_core import (
    RateLimitedPrinter,
    SlaveMotionFilter,
    SlavePacketTracker,
    clamp_joints_raw,
    raw_to_deg,
)

SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 0.005
SERIAL_REOPEN_DELAY_S = 1.0
SERIAL_STARTUP_DELAY_S = 1.5
STALE_TIMEOUT_S = 0.5
STATUS_RATE_HZ = 2.0
CONTROL_MODE = 0x01
MOVE_MODE = 0x01
SPEED_PERCENT = 100
FOLLOW_MODE = 0xAD
GRIPPER_DEFAULT_EFFORT = 1000
INITIAL_FEEDBACK_TIMEOUT_S = 3.0
FEEDBACK_POLL_S = 0.05


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read raw Piper teleop packets from Board B serial and command the "
            "slave Piper over CAN."
        )
    )
    parser.add_argument("--port", required=True, help="Board B serial port, for example /dev/ttyACM0")
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


def wait_for_initial_feedback(arm: PiperArm) -> list[int]:
    deadline = time.monotonic() + INITIAL_FEEDBACK_TIMEOUT_S
    last_error: Exception | None = None

    while time.monotonic() < deadline:
        try:
            state = arm.read_joint_feedback_state()
        except (AttributeError, RuntimeError, OSError, ValueError) as exc:
            last_error = exc
            time.sleep(FEEDBACK_POLL_S)
            continue

        if state.joint_hz is None or state.joint_hz > 0.0:
            return clamp_joints_raw(list(state.q_mdeg))
        time.sleep(FEEDBACK_POLL_S)

    if last_error is not None:
        raise RuntimeError(f"no live slave joint feedback: {last_error}") from last_error
    raise RuntimeError("no live slave joint feedback; refusing to enable motion")


def connect_piper(args: argparse.Namespace) -> tuple[PiperArm | None, list[int] | None]:
    if args.dry_run:
        return None, None

    arm = PiperArm(args.can)
    arm.connect()
    initial_joints = wait_for_initial_feedback(arm)
    arm.enable_all()
    arm.configure_motion(
        control_mode=CONTROL_MODE,
        move_mode=MOVE_MODE,
        speed_percent=SPEED_PERCENT,
        follow_mode=FOLLOW_MODE,
    )
    arm.write_joints_raw(initial_joints, dry_run=False)
    return arm, initial_joints


def open_board_serial(port: str, baud: int) -> serial.Serial:
    try:
        return serial.Serial(port, baud, timeout=SERIAL_TIMEOUT_S, exclusive=True)
    except TypeError:
        return serial.Serial(port, baud, timeout=SERIAL_TIMEOUT_S)


def handle_packet(
    *,
    packet: PiperTeleopPacket,
    tracker: SlavePacketTracker,
    motion_filter: SlaveMotionFilter,
    status: RateLimitedPrinter,
    arm: PiperArm | None,
    args: argparse.Namespace,
) -> None:
    receiver_time_s = time.monotonic()
    decision = tracker.process_packet(packet, receiver_time_s)
    if decision.warning:
        status.print(f"[SLAVE] {decision.warning}")
    if not decision.accepted:
        status.print(f"[SLAVE] Ignoring packet: {decision.reason}")
        warn_if_receiver_timeout(tracker, status)
        return

    target_joints = decision.target_joints
    if target_joints is None:
        status.print("[SLAVE] Ignoring packet: missing joints")
        return

    motion = motion_filter.update(target_joints)
    next_joints = motion.command_joints

    if motion.initialized:
        status.print(
            "[SLAVE] startup sync: holding current slave pose and using incoming target "
            "as relative baseline",
            force=True,
        )
    elif motion.rebased:
        status.print(
            "[SLAVE] source target jumped "
            f"{raw_to_deg(motion.source_jump_raw):.1f}deg in one packet; rebasing to prevent jerk",
            force=True,
        )

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
        f"filtered_target_deg={[round(raw_to_deg(value), 3) for value in motion.adjusted_target_joints]} "
        f"cmd_deg={[round(raw_to_deg(value), 3) for value in next_joints]} "
        f"gripper={decision.gripper}"
    )


def main() -> int:
    args = parse_args()
    if exit_code := validate_args(args):
        return exit_code

    status = RateLimitedPrinter(STATUS_RATE_HZ)
    tracker = SlavePacketTracker()

    try:
        arm, initial_joints = connect_piper(args)
    except (PiperSdkUnavailable, RuntimeError, OSError, AttributeError) as exc:
        print(f"Piper CAN setup failed: {exc}", file=sys.stderr)
        return 1

    if args.dry_run:
        print("[SLAVE] dry-run: not connecting to Piper and not writing CAN commands")
        motion_filter = SlaveMotionFilter()
    else:
        print(f"[SLAVE] Arm enabled on {args.can}; motion mode configured")
        print(
            "[SLAVE] Startup pose locked at "
            f"{[round(raw_to_deg(value), 3) for value in initial_joints or []]} deg"
        )
        motion_filter = SlaveMotionFilter(initial_joints)

    try:
        while True:
            print(f"Opening Board B serial {args.port} at {args.baud} baud")
            try:
                with open_board_serial(args.port, args.baud) as ser:
                    print(
                        f"[SLAVE] Waiting {SERIAL_STARTUP_DELAY_S:.1f}s for Board B serial startup"
                    )
                    time.sleep(SERIAL_STARTUP_DELAY_S)
                    try:
                        ser.reset_input_buffer()
                    except serial.SerialException:
                        pass

                    serial_buffer = bytearray()
                    print("[SLAVE] Waiting for raw Piper LoRa teleop packets. Press Ctrl+C to stop.")
                    while True:
                        try:
                            waiting = getattr(ser, "in_waiting", 0)
                            raw = ser.read(max(1, min(waiting or 1, 256)))
                        except serial.SerialException as exc:
                            print(
                                f"[SLAVE] Board B serial lost: {exc}. Reopening...",
                                file=sys.stderr,
                            )
                            break

                        if not raw:
                            warn_if_receiver_timeout(tracker, status)
                            continue

                        serial_buffer.extend(raw)
                        packets = extract_piper_teleop_packets(serial_buffer)
                        if not packets:
                            warn_if_receiver_timeout(tracker, status)
                            continue

                        for packet in packets:
                            handle_packet(
                                packet=packet,
                                tracker=tracker,
                                motion_filter=motion_filter,
                                status=status,
                                arm=arm,
                                args=args,
                            )
            except serial.SerialException as exc:
                print(
                    f"[SLAVE] Cannot open Board B serial {args.port}: {exc}. "
                    f"Retrying in {SERIAL_REOPEN_DELAY_S:.1f}s...",
                    file=sys.stderr,
                )
            time.sleep(SERIAL_REOPEN_DELAY_S)
    except KeyboardInterrupt:
        print("\n[SLAVE] stopped")
        return 0
    finally:
        if arm is not None:
            arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
