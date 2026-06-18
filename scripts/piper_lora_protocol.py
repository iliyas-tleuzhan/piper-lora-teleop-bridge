#!/usr/bin/env python3
"""Shared LoRa packet helpers for Piper teleoperation."""

from __future__ import annotations

from dataclasses import dataclass


DEADMAN_ENABLED = 0x01
PIPER_MAGIC = "PIPER"


@dataclass(frozen=True)
class PiperPacket:
    seq: int
    sender_time_ms: int
    q_cd: list[int]
    gripper_p100: int
    flags: int


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


def parse_piper_line(line: str) -> PiperPacket:
    parts = line.strip().split(",")
    if len(parts) != 12:
        raise ValueError(f"expected 12 comma-separated fields, got {len(parts)}")
    if parts[0] != PIPER_MAGIC:
        raise ValueError("missing PIPER header")

    payload = ",".join(parts[:-1])
    expected = checksum16(payload)
    received = int(parts[-1])
    if received != expected:
        raise ValueError(f"checksum mismatch received={received} expected={expected}")

    return PiperPacket(
        seq=int(parts[1]),
        sender_time_ms=int(parts[2]),
        q_cd=[int(value) for value in parts[3:9]],
        gripper_p100=int(parts[9]),
        flags=int(parts[10]),
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

