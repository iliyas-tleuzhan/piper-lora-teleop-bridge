#!/usr/bin/env python3
"""Shared binary LoRa packet helpers for Piper teleoperation."""

from __future__ import annotations

import struct
from dataclasses import dataclass


DEADMAN_ENABLED = 0x01
GRIPPER_PRESENT = 0x02
PIPER_BINARY_MAGIC = b"PLT1"
_BINARY_PACKET_WITHOUT_CRC = struct.Struct("<4sBII6iiHBB")
_BINARY_PACKET = struct.Struct("<4sBII6iiHBBH")
BINARY_PACKET_SIZE = _BINARY_PACKET.size


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


def crc16_ccitt(payload: bytes) -> int:
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_piper_teleop_packet(
    seq: int,
    sender_time_ms: int,
    joints_raw: list[int],
    *,
    deadman: bool,
    gripper: dict[str, int] | None = None,
) -> bytes:
    """Build the compact binary packet used by the real LoRa teleop path."""

    if len(joints_raw) != 6:
        raise ValueError(f"expected 6 joint values, got {len(joints_raw)}")

    flags = DEADMAN_ENABLED if deadman else 0
    gripper_angle = 0
    gripper_effort = 0
    gripper_code = 0
    if gripper is not None:
        flags |= GRIPPER_PRESENT
        gripper_angle = int(gripper.get("angle", 0))
        gripper_effort = int(gripper.get("effort", 0))
        gripper_code = int(gripper.get("code", 0))

    payload = _BINARY_PACKET_WITHOUT_CRC.pack(
        PIPER_BINARY_MAGIC,
        flags,
        int(seq) & 0xFFFFFFFF,
        int(sender_time_ms) & 0xFFFFFFFF,
        *(int(value) for value in joints_raw),
        gripper_angle,
        gripper_effort & 0xFFFF,
        gripper_code & 0xFF,
        0,
    )
    return payload + struct.pack("<H", crc16_ccitt(payload))


def parse_piper_teleop_packet(packet: bytes | bytearray | memoryview) -> PiperTeleopPacket:
    data = bytes(packet)
    if len(data) != BINARY_PACKET_SIZE:
        raise ValueError(f"expected {BINARY_PACKET_SIZE} binary bytes, got {len(data)}")
    if data[:4] != PIPER_BINARY_MAGIC:
        raise ValueError("missing PLT1 binary header")

    received_crc = int.from_bytes(data[-2:], byteorder="little", signed=False)
    expected_crc = crc16_ccitt(data[:-2])
    if received_crc != expected_crc:
        raise ValueError(f"binary CRC mismatch received={received_crc} expected={expected_crc}")

    (
        _magic,
        flags,
        seq,
        sender_time_ms,
        j1,
        j2,
        j3,
        j4,
        j5,
        j6,
        gripper_angle,
        gripper_effort,
        gripper_code,
        _reserved,
        _crc,
    ) = _BINARY_PACKET.unpack(data)

    return PiperTeleopPacket(
        seq=seq,
        sender_time_ms=sender_time_ms,
        joints_raw=[j1, j2, j3, j4, j5, j6],
        gripper_angle=gripper_angle,
        gripper_effort=gripper_effort,
        gripper_code=gripper_code,
        flags=flags,
    )


def _prefix_suffix_len(buffer: bytearray) -> int:
    max_len = min(len(buffer), len(PIPER_BINARY_MAGIC) - 1)
    for keep in range(max_len, 0, -1):
        if PIPER_BINARY_MAGIC.startswith(bytes(buffer[-keep:])):
            return keep
    return 0


def extract_piper_teleop_packets(buffer: bytearray) -> list[PiperTeleopPacket]:
    """Extract complete binary packets from a noisy serial stream."""

    packets: list[PiperTeleopPacket] = []

    while buffer:
        binary_at = buffer.find(PIPER_BINARY_MAGIC)
        if binary_at == -1:
            keep = _prefix_suffix_len(buffer)
            if keep:
                del buffer[:-keep]
            else:
                buffer.clear()
            break

        if binary_at > 0:
            del buffer[:binary_at]
        if len(buffer) < BINARY_PACKET_SIZE:
            break

        packet = bytes(buffer[:BINARY_PACKET_SIZE])
        try:
            packets.append(parse_piper_teleop_packet(packet))
            del buffer[:BINARY_PACKET_SIZE]
        except ValueError:
            del buffer[0]

    return packets
