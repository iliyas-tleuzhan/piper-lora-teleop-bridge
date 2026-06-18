# Protocol

## Real Teleop Packet Format

Real teleoperation uses a fixed 47-byte binary packet. The packet is sent:

```text
Computer 1 Python -> Board A USB serial -> LoRa -> Board B USB serial -> Computer 2 Python
```

Packet layout, little-endian except the magic bytes:

| Field | Size | Meaning |
| --- | ---: | --- |
| `magic` | 4 | ASCII `PLT1` |
| `flags` | 1 | Deadman and gripper-present bits |
| `seq` | 4 | Unsigned packet sequence |
| `time_ms` | 4 | Sender monotonic milliseconds since script start |
| `j1_raw` to `j6_raw` | 24 | Six signed int32 Piper joint targets |
| `gripper_angle` | 4 | Signed int32 Piper gripper travel |
| `gripper_effort` | 2 | Unsigned int16 Piper gripper effort |
| `gripper_code` | 1 | Uint8 Piper gripper command code |
| `reserved` | 1 | Currently zero |
| `crc16` | 2 | CRC-16/CCITT-FALSE over the first 45 bytes |

Joint values are Piper raw units in `0.001 degrees`. Gripper angle is Piper raw gripper travel from feedback frame `0x2A8`.

## Flags

`flags` bit 0 is the teleoperation deadman:

- `1`: deadman enabled, receiver may command the slave Piper.
- `0`: deadman disabled, receiver treats the packet as a stop condition.

`flags` bit 1 means a gripper command is present:

- `1`: receiver sends `GripperCtrl()`.
- `0`: receiver sends only `JointCtrl()`.

## CRC

The binary packet uses CRC-16/CCITT-FALSE:

- Initial value: `0xFFFF`
- Polynomial: `0x1021`
- Reflected input/output: no
- Final XOR: `0x0000`
- Stored little-endian in the last two bytes

## Source CAN Frames

Computer 1 builds each target from one complete master Piper joint source. It prefers feedback frames when they are available:

- `0x2A5`: joints 1 and 2
- `0x2A6`: joints 3 and 4
- `0x2A7`: joints 5 and 6
- `0x2A8`: optional gripper feedback

If feedback frames are not present, it uses the same command frames as the working UDP bridge:

- `0x155`: joints 1 and 2
- `0x156`: joints 3 and 4
- `0x157`: joints 5 and 6
- `0x159`: optional gripper command

The sender refuses to transmit until it has a fresh full joint set from one of those sources.

## Stale Behavior

Board B declares stale when no valid LoRa packet has arrived for more than one second. Computer 2 warns when no valid live packet has been read for more than `0.5` second and holds the last command.

## Rate Limits

LoRa is low-bandwidth. Do not forward raw high-rate CAN frames over LoRa.

The sender is flow-controlled by Board A `TX done`, so it sends the newest target only when the radio is ready. The fixed binary packet always contains gripper fields, but the receiver only uses them when the gripper-present flag is set.
