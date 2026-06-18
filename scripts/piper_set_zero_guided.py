#!/usr/bin/env python3
"""Guided Piper joint zero calibration for matching two arms."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from piper_teleop_core import raw_to_deg


ZERO_CONFIRM = "SET_ZERO"
SET_ZERO_CODE = 0xAE


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Interactively set Piper joint zero positions. This writes persistent "
            "calibration data to the arm."
        )
    )
    parser.add_argument("--can", default="can0", help="Piper CAN interface")
    parser.add_argument(
        "--joint",
        type=int,
        choices=range(1, 8),
        default=7,
        metavar="1-7",
        help="Joint to calibrate; 7 means joints 1-6 sequentially",
    )
    parser.add_argument(
        "--confirm",
        default="",
        help=f"Must be {ZERO_CONFIRM} to write zero calibration",
    )
    return parser.parse_args()


def import_piper_interface() -> Any:
    try:
        import piper_sdk  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "piper_sdk is not installed. Activate the conda env and run "
            "`pip install -r requirements.txt`."
        ) from exc

    interface_cls = getattr(piper_sdk, "C_PiperInterface_V2", None)
    if interface_cls is None:
        interface_cls = getattr(piper_sdk, "C_PiperInterface", None)
    if interface_cls is None:
        raise RuntimeError("piper_sdk does not expose C_PiperInterface_V2")
    return interface_cls


def message_from_sdk_result(result: Any) -> Any:
    if isinstance(result, tuple):
        if len(result) >= 3:
            return result[2]
        if result:
            return result[-1]
    return result


def unwrap_message(obj: Any, names: tuple[str, ...]) -> Any:
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return obj


def read_joint_feedback(piper: Any) -> list[int] | None:
    getter = getattr(piper, "GetArmJointMsgs", None)
    if not callable(getter):
        return None
    message = message_from_sdk_result(getter())
    joints = unwrap_message(
        message,
        ("joint_state", "arm_joint_feedback", "joint_ctrl", "arm_joint_ctrl"),
    )
    values: list[int] = []
    for index in range(1, 7):
        for name in (f"joint_{index}", f"joint_{index}_angle"):
            if hasattr(joints, name):
                values.append(int(getattr(joints, name)))
                break
        else:
            return None
    return values


def connect_piper(can_name: str) -> Any:
    interface_cls = import_piper_interface()
    try:
        piper = interface_cls(can_name)
    except TypeError:
        piper = interface_cls(can_name=can_name)
    piper.ConnectPort()
    time.sleep(0.2)
    return piper


def enable_arm(piper: Any, joint: int) -> None:
    enable = getattr(piper, "EnableArm", None)
    if callable(enable):
        enable(joint)


def disable_arm(piper: Any, joint: int) -> None:
    disable = getattr(piper, "DisableArm", None)
    if not callable(disable):
        raise RuntimeError("piper_sdk does not expose DisableArm")
    disable(joint)


def set_joint_zero(piper: Any, joint: int) -> None:
    joint_config = getattr(piper, "JointConfig", None)
    if not callable(joint_config):
        raise RuntimeError("piper_sdk does not expose JointConfig")
    joint_config(joint, SET_ZERO_CODE)
    time.sleep(0.5)


def print_feedback(piper: Any) -> None:
    joints = read_joint_feedback(piper)
    if joints is None:
        print("Current joint feedback: unavailable")
        return
    print(f"Current joint feedback deg: {[round(raw_to_deg(value), 3) for value in joints]}")


def prompt_continue(message: str) -> bool:
    answer = input(f"{message} Press Enter to continue, or type q to stop: ").strip().lower()
    return answer != "q"


def calibrate_joint(piper: Any, joint: int) -> bool:
    print(f"\nJoint {joint}")
    print("Support the arm before disabling this motor.")
    if not prompt_continue(f"Ready to disable joint {joint}?"):
        return False

    disable_arm(piper, joint)
    print(f"Joint {joint} disabled. Move it to the shared physical neutral/zero mark.")
    if not prompt_continue(f"Ready to set joint {joint} current position as zero?"):
        enable_arm(piper, joint)
        return False

    set_joint_zero(piper, joint)
    enable_arm(piper, joint)
    print(f"Joint {joint} zero command sent and joint re-enabled.")
    print_feedback(piper)
    return True


def main() -> int:
    args = parse_args()
    if args.confirm != ZERO_CONFIRM:
        print(f"Refusing to write zero calibration. Re-run with `--confirm {ZERO_CONFIRM}`.")
        return 2

    try:
        piper = connect_piper(args.can)
        print("Connected. This procedure writes persistent joint zero calibration.")
        print("Use the same physical neutral pose on the master and slave arms.")
        print_feedback(piper)

        joints = range(1, 7) if args.joint == 7 else (args.joint,)
        for joint in joints:
            if not calibrate_joint(piper, int(joint)):
                print("Calibration stopped by user.")
                return 1

        print("\nZero calibration sequence finished. Power-cycle the arm before teleop testing.")
        return 0
    except (RuntimeError, OSError, AttributeError, ValueError) as exc:
        print(f"Zero calibration failed: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nZero calibration stopped")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
