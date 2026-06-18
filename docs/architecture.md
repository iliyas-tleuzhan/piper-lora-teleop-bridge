# Architecture

## Final Real Goal

The final real system is intended to bridge joint targets between two AgileX Piper arms:

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

## Fake Test

The fake transport test proves:

```text
Computer 1 fake packet
  -> Serial
  -> Board A
  -> LoRa
  -> Board B
  -> Serial
  -> Computer 2 fake CAN output
```

There is no Piper control and no CAN connection in this version. The Python sender generates smooth fake joint targets. Board A forwards valid `PIPER` serial lines over LoRa. Board B validates LoRa packets and forwards valid `PIPER` lines over serial. The Computer 2 receiver validates the packet again and prints what it would send to the slave Piper.

## Real Teleoperation

The real teleoperation scripts keep the ESP32 boards as simple serial radio modems:

```text
Computer 1:
  python-can raw master command reader
  -> raw compact PIPER target packet
  -> Board A serial

Computer 2:
  Board B serial
  -> packet validation
  -> stale/deadman gate
  -> optional slew limit
  -> piper_sdk CAN JointCtrl/GripperCtrl
```

`scripts/computer1_piper_sender.py` matches the `piper-wireless-teleop` master sender. It reads raw SocketCAN command frames from the master Piper:

- `0x155`: joints 1 and 2
- `0x156`: joints 3 and 4
- `0x157`: joints 5 and 6
- `0x159`: optional gripper command

It stores the latest complete target and sends that target over LoRa at the requested packet rate. The sender does not use `piper_sdk`.

`scripts/computer2_piper_receiver.py` validates LoRa packets, drops corrupt packets, rejects deadman-off and out-of-order packets, clamps raw Piper joint targets to known Piper joint limits, and writes the slave Piper with `JointCtrl()` and `GripperCtrl()`. It follows the latest valid target directly by default, with optional slew limiting available through `--enable-slew-limit`.

## Why The ESP32 Boards Stay Simple

The ESP32 boards act as radio modems. They do not understand Piper CAN. This keeps the embedded code small and reduces risk:

- Board A reads a compact ASCII packet from USB serial and transmits it over LoRa.
- Board B receives a LoRa packet and prints it over USB serial.
- Both boards validate the checksum so corrupted packets are dropped early.
- Board B detects stale traffic and displays/prints a fake stop warning.

## Safety Behavior

- The packet deadman flag must be enabled before the receiver commands the slave.
- The receiver refuses to move unless started with `--confirm MOVE`.
- If no valid live packet arrives for more than the receiver `--stale-timeout`, the receiver warns and holds the last command.
- `--dry-run` on the receiver validates the real LoRa stream without writing CAN motion commands.
