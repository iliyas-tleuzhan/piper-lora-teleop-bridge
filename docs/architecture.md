# Architecture

## Real System

The system bridges live joint targets between two AgileX Piper arms:

```text
Master Piper arm
  -> CAN
  -> Computer 1 / Orin / Linux laptop
  -> USB Serial
  -> Heltec ESP32 Board A
  -> LoRa node-to-node, 923.2 MHz
  -> Heltec ESP32 Board B
  -> USB Serial
  -> Computer 2 / Linux laptop
  -> CAN
  -> Slave Piper arm
```

## Responsibilities

The ESP32 boards are radio modems. They do not read or write Piper CAN.

Computer 1:

- Reads SocketCAN target frames from the master Piper.
- Prefers feedback frames `0x2A5`, `0x2A6`, and `0x2A7` for joints 1-6 when present.
- Falls back to UDP-compatible command frames `0x155`, `0x156`, and `0x157` when feedback frames are not present.
- Uses `0x2A8` or `0x159` for optional gripper data.
- Waits for a fresh full joint set before sending.
- Writes compact 47-byte binary `PLT1` packets to Board A.
- Waits for Board A `TX done` before sending another packet.

Board A:

- Reads fixed binary teleop packets from USB serial.
- Validates magic and CRC.
- Transmits valid packets over LoRa at 923.2 MHz, BW250, SF7.

Board B:

- Receives LoRa packets.
- Validates magic and CRC.
- Writes valid binary packets to USB serial for Computer 2.
- Displays packet/stale status on the OLED.

Computer 2:

- Scans the Board B serial stream for valid binary teleop packets.
- Rejects corrupt, stale, deadman-off, duplicate, and out-of-order packets.
- Reads slave joint feedback at startup and locks the initial slave pose.
- Treats the first incoming master target as a relative baseline instead of an immediate absolute command.
- Rebases if the source target jumps suddenly, preventing a single stale packet from jerking the slave.
- Smooths and step-limits commands before writing CAN.
- Clamps raw Piper joint targets to known Piper joint limits.
- Writes the slave Piper with `JointCtrl()` and `GripperCtrl()`.

## Source Selection

Different Piper setups expose different useful master-side frames. The sender uses live feedback frames when they exist, but your known-good UDP teleop used command frames, so the LoRa sender falls back to those same IDs automatically.

## Safety Behavior

- The packet deadman flag must be enabled before the receiver commands the slave.
- The receiver refuses to move unless started with `--confirm MOVE`.
- The receiver refuses to start real motion if it cannot read live slave joint feedback.
- If no valid live packet arrives for more than the receiver timeout, the receiver warns and holds the last command.
- `--dry-run` on the receiver validates the real LoRa stream without writing CAN motion commands.
