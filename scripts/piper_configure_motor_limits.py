#!/usr/bin/env python3
"""Read or apply the Piper motor angle/speed limit profile used by teleop."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from piper_teleop_core import JOINT_LIMITS_RAW, raw_to_deg


MAX_JOINT_SPEED_RAW = 3000
QUERY_SETTLE_S = 0.3
WRITE_SETTLE_S = 0.1


def target_limit_rows() -> list[tuple[int, int, int, int]]:
    rows: list[tuple[int, int, int, int]] = []
    for motor_num, (low_raw, high_raw) in enumerate(JOINT_LIMITS_RAW, start=1):
        rows.append((motor_num, high_raw // 100, low_raw // 100, MAX_JOINT_SPEED_RAW))
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Query the Piper firmware motor limits, or apply the full-motion "
            "teleop profile to one arm."
        )
    )
    parser.add_argument("--can", default="can0", help="Piper CAN interface")
    parser.add_argument(
        "--confirm",
        default="",
        help="Use WRITE_LIMITS to persistently write the teleop limit profile",
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


def connect_piper(can_name: str) -> Any:
    interface_cls = import_piper_interface()
    try:
        piper = interface_cls(can_name)
    except TypeError:
        piper = interface_cls(can_name=can_name)
    piper.ConnectPort()
    return piper


def enable_arm_if_possible(piper: Any) -> None:
    enable_arm = getattr(piper, "EnableArm", None)
    if callable(enable_arm):
        enable_arm(7)

    enable_piper = getattr(piper, "EnablePiper", None)
    if callable(enable_piper):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if enable_piper():
                return
            time.sleep(0.01)


def read_limits(piper: Any) -> list[tuple[int, int, int, int]]:
    search = getattr(piper, "SearchAllMotorMaxAngleSpd", None)
    if callable(search):
        search()
        time.sleep(QUERY_SETTLE_S)

    getter = getattr(piper, "GetAllMotorAngleLimitMaxSpd", None)
    if not callable(getter):
        raise RuntimeError("piper_sdk does not expose GetAllMotorAngleLimitMaxSpd")

    message = message_from_sdk_result(getter())
    motors = getattr(message, "motor", None)
    if motors is None:
        raise RuntimeError(f"unexpected motor-limit response type: {type(message).__name__}")

    rows: list[tuple[int, int, int, int]] = []
    for index in range(1, 7):
        motor = motors[index]
        motor_num = int(getattr(motor, "motor_num", index))
        max_angle = int(getattr(motor, "max_angle_limit"))
        min_angle = int(getattr(motor, "min_angle_limit"))
        max_speed = int(getattr(motor, "max_joint_spd"))
        rows.append((motor_num, max_angle, min_angle, max_speed))
    return rows


def write_limits(piper: Any) -> None:
    setter = getattr(piper, "MotorAngleLimitMaxSpdSet", None)
    if not callable(setter):
        raise RuntimeError("piper_sdk does not expose MotorAngleLimitMaxSpdSet")

    enable_arm_if_possible(piper)
    for motor_num, max_angle, min_angle, max_speed in target_limit_rows():
        setter(motor_num, max_angle, min_angle, max_speed)
        time.sleep(WRITE_SETTLE_S)


def print_rows(title: str, rows: list[tuple[int, int, int, int]]) -> None:
    print(title)
    print("motor  min_deg  max_deg  max_speed_rad_s")
    for motor_num, max_angle, min_angle, max_speed in rows:
        print(
            f"{motor_num:>5}  "
            f"{min_angle * 0.1:>7.1f}  "
            f"{max_angle * 0.1:>7.1f}  "
            f"{max_speed / 1000.0:>15.3f}"
        )


def print_software_profile() -> None:
    print("Teleop software clamp profile")
    print("joint  min_deg  max_deg")
    for index, (low_raw, high_raw) in enumerate(JOINT_LIMITS_RAW, start=1):
        print(f"{index:>5}  {raw_to_deg(low_raw):>7.1f}  {raw_to_deg(high_raw):>7.1f}")


def main() -> int:
    args = parse_args()
    print_software_profile()

    try:
        piper = connect_piper(args.can)
        before_rows = read_limits(piper)
        print_rows("\nCurrent firmware limits", before_rows)

        if args.confirm != "WRITE_LIMITS":
            print("\nRead-only mode. Re-run with `--confirm WRITE_LIMITS` to apply this profile.")
            return 0

        print("\nWriting teleop full-motion profile to firmware...")
        write_limits(piper)
        after_rows = read_limits(piper)
        print_rows("\nFirmware limits after write", after_rows)
        print("\nApply the same command on the other Piper arm, then power-cycle both arms.")
        return 0
    except (RuntimeError, OSError, AttributeError, ValueError) as exc:
        print(f"Motor-limit configuration failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
