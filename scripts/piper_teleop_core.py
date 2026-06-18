#!/usr/bin/env python3
"""Core raw Piper teleoperation helpers shared by the LoRa scripts."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol

from piper_lora_protocol import PiperTeleopPacket


MASTER_COMMAND_CAN_IDS = {0x151, 0x155, 0x156, 0x157, 0x159}
MASTER_FEEDBACK_CAN_IDS = {0x2A5, 0x2A6, 0x2A7, 0x2A8}
RAW_UNITS_PER_DEGREE = 1000
SEQUENCE_RESET_AFTER_S = 1.0
JOINT_LIMITS_RAW: tuple[tuple[int, int], ...] = (
    (-150000, 150000),
    (0, 180000),
    (-170000, 0),
    (-100000, 100000),
    (-70000, 70000),
    (-120000, 120000),
)


class CanMessage(Protocol):
    arbitration_id: int
    data: bytes


def raw_to_deg(value: int | float) -> float:
    return float(value) / RAW_UNITS_PER_DEGREE


def deg_to_raw(value: int | float) -> int:
    return int(round(float(value) * RAW_UNITS_PER_DEGREE))


def decode_i32_be(data: bytes | bytearray | memoryview) -> int:
    if len(data) != 4:
        raise ValueError("expected exactly 4 bytes for int32")
    return int.from_bytes(bytes(data), byteorder="big", signed=True)


@dataclass
class MasterSourceState:
    joints: list[int | None] = field(default_factory=lambda: [None] * 6)
    joint_updated_at: list[float | None] = field(default_factory=lambda: [None] * 6)
    gripper: dict[str, int] | None = None
    mode_frame: list[int] | None = None

    def has_full_joint_target(self) -> bool:
        return all(value is not None for value in self.joints)

    def joints_raw(self) -> list[int]:
        if not self.has_full_joint_target():
            raise ValueError("full joint target is not available yet")
        return [int(value) for value in self.joints]

    def has_fresh_joint_target(self, now_s: float, max_age_s: float) -> bool:
        if not self.has_full_joint_target():
            return False
        return all(
            updated_at is not None and now_s - updated_at <= max_age_s
            for updated_at in self.joint_updated_at
        )

    def update_joint_pair(self, first_index: int, first_value: int, second_value: int) -> None:
        now_s = time.monotonic()
        self.joints[first_index] = first_value
        self.joints[first_index + 1] = second_value
        self.joint_updated_at[first_index] = now_s
        self.joint_updated_at[first_index + 1] = now_s


def decode_master_command_frame(message: CanMessage, state: MasterSourceState) -> bool:
    arbitration_id = int(message.arbitration_id)
    data = bytes(message.data)
    if arbitration_id not in MASTER_COMMAND_CAN_IDS:
        return False

    if arbitration_id == 0x151 and len(data) == 8:
        state.mode_frame = list(data)
        return True

    if arbitration_id == 0x155 and len(data) == 8:
        state.update_joint_pair(0, decode_i32_be(data[0:4]), decode_i32_be(data[4:8]))
        return True

    if arbitration_id == 0x156 and len(data) == 8:
        state.update_joint_pair(2, decode_i32_be(data[0:4]), decode_i32_be(data[4:8]))
        return True

    if arbitration_id == 0x157 and len(data) == 8:
        state.update_joint_pair(4, decode_i32_be(data[0:4]), decode_i32_be(data[4:8]))
        return True

    if arbitration_id == 0x159 and len(data) == 8:
        state.gripper = {
            "angle": decode_i32_be(data[0:4]),
            "effort": int.from_bytes(data[4:6], byteorder="big", signed=False),
            "code": data[6],
        }
        return True

    return False


def decode_master_feedback_frame(message: CanMessage, state: MasterSourceState) -> bool:
    arbitration_id = int(message.arbitration_id)
    data = bytes(message.data)
    if arbitration_id not in MASTER_FEEDBACK_CAN_IDS:
        return False

    if arbitration_id == 0x2A5 and len(data) == 8:
        state.update_joint_pair(0, decode_i32_be(data[0:4]), decode_i32_be(data[4:8]))
        return True

    if arbitration_id == 0x2A6 and len(data) == 8:
        state.update_joint_pair(2, decode_i32_be(data[0:4]), decode_i32_be(data[4:8]))
        return True

    if arbitration_id == 0x2A7 and len(data) == 8:
        state.update_joint_pair(4, decode_i32_be(data[0:4]), decode_i32_be(data[4:8]))
        return True

    if arbitration_id == 0x2A8 and len(data) == 8:
        state.gripper = {
            "angle": decode_i32_be(data[0:4]),
            "effort": 1000,
            "code": 1,
        }
        return True

    return False


def validate_joints_raw(joints: list[object]) -> None:
    if len(joints) != 6:
        raise ValueError("joints must contain exactly 6 values")
    for value in joints:
        if not isinstance(value, int):
            raise ValueError("joint values must be integers in Piper raw units")


def clamp_joints_raw(joints: list[int]) -> list[int]:
    validate_joints_raw(list(joints))
    clamped: list[int] = []
    for value, (low, high) in zip(joints, JOINT_LIMITS_RAW, strict=True):
        clamped.append(max(low, min(high, int(value))))
    return clamped


def limit_step_raw(current: list[int], target: list[int], max_step_raw: int) -> list[int]:
    validate_joints_raw(list(current))
    validate_joints_raw(list(target))
    if max_step_raw < 0:
        raise ValueError("max_step_raw must be non-negative")

    next_joints: list[int] = []
    for current_value, target_value in zip(current, target, strict=True):
        delta = int(target_value) - int(current_value)
        if abs(delta) <= max_step_raw:
            next_joints.append(int(target_value))
        else:
            step = max_step_raw if delta > 0 else -max_step_raw
            next_joints.append(int(current_value) + step)
    return next_joints


@dataclass(frozen=True)
class PacketDecision:
    accepted: bool
    reason: str | None = None
    warning: str | None = None
    target_joints: list[int] | None = None
    gripper: dict[str, int] | None = None
    sequence: int | None = None
    dropped: int = 0
    total_dropped: int = 0
    receiver_time_s: float | None = None
    sender_time_ms: int | None = None


class SlavePacketTracker:
    def __init__(self) -> None:
        self.last_seq: int | None = None
        self.total_dropped = 0
        self.last_valid_rx_time_s: float | None = None
        self.first_valid_rx_time_s: float | None = None
        self.valid_packet_count = 0

    def reset_sequence(self) -> None:
        self.last_seq = None
        self.total_dropped = 0
        self.first_valid_rx_time_s = None
        self.valid_packet_count = 0

    def process_packet(
        self,
        packet: PiperTeleopPacket,
        receiver_time_s: float,
    ) -> PacketDecision:
        try:
            target_joints = clamp_joints_raw(packet.joints_raw)
        except (TypeError, ValueError) as exc:
            return PacketDecision(accepted=False, reason=f"malformed packet: {exc}")

        if not packet.deadman:
            return PacketDecision(accepted=False, reason="deadman=false", sequence=packet.seq)

        if self.last_seq is not None and packet.seq <= self.last_seq:
            stale_gap_s = (
                None
                if self.last_valid_rx_time_s is None
                else receiver_time_s - self.last_valid_rx_time_s
            )
            if stale_gap_s is not None and stale_gap_s > SEQUENCE_RESET_AFTER_S:
                self.reset_sequence()
            else:
                return PacketDecision(
                    accepted=False,
                    reason=f"duplicate/out-of-order packet seq={packet.seq} last_seq={self.last_seq}",
                    sequence=packet.seq,
                    total_dropped=self.total_dropped,
                )

        dropped = 0
        if self.last_seq is not None and packet.seq > self.last_seq + 1:
            dropped = packet.seq - self.last_seq - 1
            self.total_dropped += dropped
        self.last_seq = packet.seq

        self.last_valid_rx_time_s = receiver_time_s
        if self.first_valid_rx_time_s is None:
            self.first_valid_rx_time_s = receiver_time_s
        self.valid_packet_count += 1

        gripper = None
        if packet.has_gripper:
            gripper = {
                "angle": packet.gripper_angle,
                "effort": packet.gripper_effort,
                "code": packet.gripper_code,
            }

        return PacketDecision(
            accepted=True,
            target_joints=target_joints,
            gripper=gripper,
            sequence=packet.seq,
            dropped=dropped,
            total_dropped=self.total_dropped,
            receiver_time_s=receiver_time_s,
            sender_time_ms=packet.sender_time_ms,
        )

    def timeout_expired(self, now_s: float, receiver_timeout_s: float) -> bool:
        if self.last_valid_rx_time_s is None:
            return False
        return now_s - self.last_valid_rx_time_s > receiver_timeout_s

    def seconds_since_valid_packet(self, now_s: float) -> float | None:
        if self.last_valid_rx_time_s is None:
            return None
        return max(0.0, now_s - self.last_valid_rx_time_s)

    def command_rate_hz(self, now_s: float) -> float | None:
        if self.first_valid_rx_time_s is None or self.valid_packet_count < 2:
            return None
        elapsed_s = now_s - self.first_valid_rx_time_s
        if elapsed_s <= 0:
            return None
        return (self.valid_packet_count - 1) / elapsed_s


class RateLimitedPrinter:
    def __init__(self, rate_hz: float) -> None:
        self.interval_s = 1.0 / rate_hz if rate_hz > 0 else float("inf")
        self._last_print = 0.0

    def print(self, message: str, *, force: bool = False) -> None:
        now = time.monotonic()
        if force or now - self._last_print >= self.interval_s:
            print(message, flush=True)
            self._last_print = now
