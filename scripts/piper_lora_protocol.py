#!/usr/bin/env python3
"""Shared LoRa packet helpers for Piper teleoperation."""

from __future__ import annotations

from dataclasses import dataclass


DEADMAN_ENABLED = 0x01
GRIPPER_PRESENT = 0x02
PIPER_MAGIC = "PIPER"


@dataclass(frozen=True)
class PiperPacket:
    seq: int
    sender_time_ms: int
    q_cd: list[int]
    gripper_p100: int
    flags: int


@dataclass(frozen=True)
class PiperTeleopPacket:
    seq: int
    sender_time_ms: int
    joints_raw: list[int]
    gripper_angle: int
    gripper_effort: int
    gripper_code: int
    flags: int

    @property
    def deadman(self) -> bool:
        return bool(self.flags & DEADMAN_ENABLED)

    @property
    def has_gripper(self) -> bool:
        return bool(self.flags & GRIPPER_PRESENT)


def checksum16(payload: str) -> int:
    c = 0x1234
    for byte in payload.encode("ascii"):
        c = ((c << 5) | (c >> 11)) & 0xFFFF
        c ^= byte
    return c & 0xFFFF


def build_piper_line(
    seq: int,
    sender_time_ms: int,
    q_cd: list[int],
    gripper_p100: int,
    flags: int,
) -> str:
    if len(q_cd) != 6:
        raise ValueError(f"expected 6 joint values, got {len(q_cd)}")

    fields = [
        PIPER_MAGIC,
        str(seq),
        str(sender_time_ms),
        *(str(value) for value in q_cd),
        str(gripper_p100),
        str(flags),
    ]
    payload = ",".join(fields)
    return f"{payload},{checksum16(payload)}\n"


def build_piper_teleop_line(
    seq: int,
    sender_time_ms: int,
    joints_raw: list[int],
    *,
    deadman: bool,
    gripper: dict[str, int] | None = None,
) -> str:
    """Build a raw Piper teleop packet for LoRa transport.

    Joint values are Piper SDK/CAN raw units: 0.001 degrees. Gripper values are
    the raw Piper gripper command fields from CAN ID 0x159.
    """

    if len(joints_raw) != 6:
        raise ValueError(f"expected 6 joint values, got {len(joints_raw)}")

    flags = DEADMAN_ENABLED if deadman else 0
    if gripper is not None:
        flags |= GRIPPER_PRESENT
    fields = [
        PIPER_MAGIC,
        str(seq),
        str(sender_time_ms),
        *(str(int(value)) for value in joints_raw),
    ]
    if gripper is not None:
        fields.extend(
            [
                str(int(gripper.get("angle", 0))),
                str(int(gripper.get("effort", 0))),
                str(int(gripper.get("code", 0))),
            ]
        )
    fields.append(str(flags))
    payload = ",".join(fields)
    return f"{payload},{checksum16(payload)}\n"


def _validated_parts(line: str) -> list[str]:
    parts = line.strip().split(",")
    if not parts or parts[0] != PIPER_MAGIC:
        raise ValueError("missing PIPER header")

    payload = ",".join(parts[:-1])
    expected = checksum16(payload)
    received = int(parts[-1])
    if received != expected:
        raise ValueError(f"checksum mismatch received={received} expected={expected}")
    return parts


def parse_piper_line(line: str) -> PiperPacket:
    parts = _validated_parts(line)
    if len(parts) != 12:
        raise ValueError(f"expected 12 comma-separated fields, got {len(parts)}")

    return PiperPacket(
        seq=int(parts[1]),
        sender_time_ms=int(parts[2]),
        q_cd=[int(value) for value in parts[3:9]],
        gripper_p100=int(parts[9]),
        flags=int(parts[10]),
    )


def parse_piper_teleop_line(line: str) -> PiperTeleopPacket:
    parts = _validated_parts(line)
    if len(parts) == 11:
        return PiperTeleopPacket(
            seq=int(parts[1]),
            sender_time_ms=int(parts[2]),
            joints_raw=[int(value) for value in parts[3:9]],
            gripper_angle=0,
            gripper_effort=0,
            gripper_code=0,
            flags=int(parts[9]),
        )
    if len(parts) != 14:
        raise ValueError(f"expected 11 or 14 comma-separated fields, got {len(parts)}")

    return PiperTeleopPacket(
        seq=int(parts[1]),
        sender_time_ms=int(parts[2]),
        joints_raw=[int(value) for value in parts[3:9]],
        gripper_angle=int(parts[9]),
        gripper_effort=int(parts[10]),
        gripper_code=int(parts[11]),
        flags=int(parts[12]),
    )


def cd_to_degrees(q_cd: list[int]) -> list[float]:
    return [value / 100.0 for value in q_cd]


def degrees_to_cd(q_deg: list[float]) -> list[int]:
    if len(q_deg) != 6:
        raise ValueError(f"expected 6 joint values, got {len(q_deg)}")
    return [int(round(value * 100.0)) for value in q_deg]


def cd_to_mdeg(q_cd: list[int]) -> list[int]:
    if len(q_cd) != 6:
        raise ValueError(f"expected 6 joint values, got {len(q_cd)}")
    return [value * 10 for value in q_cd]


def mdeg_to_cd(q_mdeg: list[int]) -> list[int]:
    if len(q_mdeg) != 6:
        raise ValueError(f"expected 6 joint values, got {len(q_mdeg)}")
    return [int(round(value / 10.0)) for value in q_mdeg]

