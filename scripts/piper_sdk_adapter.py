#!/usr/bin/env python3
"""Small compatibility wrapper around AgileX piper_sdk."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class PiperState:
    q_mdeg: list[int]
    gripper_um: int


class PiperSdkUnavailable(RuntimeError):
    pass


def _import_sdk() -> tuple[Any, Any]:
    try:
        import piper_sdk  # type: ignore
    except ImportError as exc:
        raise PiperSdkUnavailable(
            "piper_sdk is not installed. Install it on the Piper computers with "
            "`pip install piper_sdk` after activating the CAN environment."
        ) from exc

    interface_cls = getattr(piper_sdk, "C_PiperInterface_V2", None)
    if interface_cls is None:
        interface_cls = getattr(piper_sdk, "C_PiperInterface")
    log_level = getattr(piper_sdk, "LogLevel", None)
    return interface_cls, log_level


def _message_from_sdk_result(result: Any) -> Any:
    if isinstance(result, tuple):
        if len(result) >= 3:
            return result[2]
        if result:
            return result[-1]
    return result


def _unwrap_message(obj: Any, names: tuple[str, ...]) -> Any:
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return obj


def _public_fields(obj: Any) -> list[str]:
    if hasattr(obj, "__dict__"):
        return sorted(name for name in vars(obj) if not name.startswith("_"))
    return sorted(name for name in dir(obj) if not name.startswith("_"))


def _field(obj: Any, names: tuple[str, ...]) -> int:
    for name in names:
        if hasattr(obj, name):
            return int(getattr(obj, name))
    fields = ", ".join(_public_fields(obj))
    raise AttributeError(
        f"none of these fields exist on {type(obj).__name__}: {names}. "
        f"Available public fields: {fields}"
    )


def _joint_fields(obj: Any) -> list[int]:
    return [
        _field(obj, (f"joint_{index}", f"joint_{index}_angle"))
        for index in range(1, 7)
    ]


class PiperArm:
    def __init__(
        self,
        can_name: str,
        *,
        dh_is_offset: int = 1,
        judge_flag: bool = False,
        can_auto_init: bool = True,
        sdk_joint_limit: bool = True,
        sdk_gripper_limit: bool = True,
    ) -> None:
        C_PiperInterface, LogLevel = _import_sdk()
        kwargs: dict[str, Any] = {
            "can_name": can_name,
            "judge_flag": judge_flag,
            "can_auto_init": can_auto_init,
            "dh_is_offset": dh_is_offset,
            "start_sdk_joint_limit": sdk_joint_limit,
            "start_sdk_gripper_limit": sdk_gripper_limit,
            "log_to_file": False,
            "log_file_path": None,
        }
        if LogLevel is not None:
            kwargs["logger_level"] = LogLevel.WARNING
        try:
            self._piper = C_PiperInterface(**kwargs)
        except TypeError:
            self._piper = C_PiperInterface(can_name)

    def connect(self) -> None:
        self._piper.ConnectPort()

    def disconnect(self) -> None:
        if hasattr(self._piper, "DisconnectPort"):
            self._piper.DisconnectPort()

    def is_ok(self) -> bool:
        if not hasattr(self._piper, "isOk"):
            return True
        return bool(self._piper.isOk())

    def configure_master_input(self) -> None:
        self._piper.MasterSlaveConfig(0xFA, 0, 0, 0)

    def configure_slave_output(self) -> None:
        self._piper.MasterSlaveConfig(0xFC, 0, 0, 0)

    def configure_joint_control(self, *, speed_percent: int, high_follow: bool) -> None:
        mit_mode = 0xAD if high_follow else 0x00
        self.configure_motion(
            control_mode=0x01,
            move_mode=0x01,
            speed_percent=speed_percent,
            follow_mode=mit_mode,
        )

    def configure_motion(
        self,
        *,
        control_mode: int,
        move_mode: int,
        speed_percent: int,
        follow_mode: int,
    ) -> None:
        if hasattr(self._piper, "MotionCtrl_2"):
            self._piper.MotionCtrl_2(control_mode, move_mode, speed_percent, follow_mode)
        else:
            self._piper.ModeCtrl(control_mode, move_mode, speed_percent, follow_mode)

    def enable_all(self) -> None:
        if hasattr(self._piper, "EnableArm"):
            self._piper.EnableArm(7)
        elif hasattr(self._piper, "EnableArmStandbyMode"):
            self._piper.EnableArmStandbyMode(7)
        else:
            raise AttributeError("Piper SDK does not expose an arm enable method")

    def disable_all(self) -> None:
        self._piper.DisableArm(7)

    def read_control_state(self) -> PiperState:
        joints = _unwrap_message(
            _message_from_sdk_result(self._piper.GetArmJointCtrl()),
            ("joint_ctrl", "arm_joint_ctrl", "joint_state", "arm_joint_feedback"),
        )
        gripper = _unwrap_message(
            _message_from_sdk_result(self._piper.GetArmGripperCtrl()),
            ("gripper_ctrl", "arm_gripper_ctrl", "gripper_state", "arm_gripper_feedback"),
        )
        return PiperState(
            q_mdeg=_joint_fields(joints),
            gripper_um=max(0, _field(gripper, ("grippers_angle", "gripper_angle"))),
        )

    def read_feedback_state(self) -> PiperState:
        joints = _unwrap_message(
            _message_from_sdk_result(self._piper.GetArmJointMsgs()),
            ("joint_state", "arm_joint_feedback", "joint_ctrl", "arm_joint_ctrl"),
        )
        gripper = _unwrap_message(
            _message_from_sdk_result(self._piper.GetArmGripperMsgs()),
            ("gripper_state", "arm_gripper_feedback", "gripper_ctrl", "arm_gripper_ctrl"),
        )
        return PiperState(
            q_mdeg=_joint_fields(joints),
            gripper_um=max(0, _field(gripper, ("grippers_angle", "gripper_angle"))),
        )

    def write_state(self, state: PiperState, *, gripper_effort: int, dry_run: bool) -> None:
        if dry_run:
            return
        self._piper.JointCtrl(*state.q_mdeg)
        self._piper.GripperCtrl(max(0, state.gripper_um), gripper_effort, 0x01, 0)

    def write_joints_raw(self, joints_raw: list[int], *, dry_run: bool) -> None:
        if len(joints_raw) != 6:
            raise ValueError("JointCtrl requires exactly 6 joint values")
        if dry_run:
            return
        self._piper.JointCtrl(*[int(value) for value in joints_raw])

    def write_gripper_raw(self, gripper: dict[str, int], *, default_effort: int, dry_run: bool) -> None:
        if dry_run:
            return
        angle = int(gripper.get("angle", 0))
        effort = int(gripper.get("effort", default_effort))
        code = int(gripper.get("code", 1))
        self._piper.GripperCtrl(angle, effort, code, 0)


def gripper_um_to_p100(gripper_um: int, max_mm: float) -> int:
    max_um = max_mm * 1000.0
    if max_um <= 0:
        raise ValueError("gripper max must be greater than 0")
    percent = max(0.0, min(100.0, gripper_um * 100.0 / max_um))
    return int(round(percent * 100.0))


def gripper_p100_to_um(gripper_p100: int, max_mm: float) -> int:
    max_um = max_mm * 1000.0
    if max_um <= 0:
        raise ValueError("gripper max must be greater than 0")
    percent = max(0.0, min(100.0, gripper_p100 / 100.0))
    return int(round(max_um * percent / 100.0))
