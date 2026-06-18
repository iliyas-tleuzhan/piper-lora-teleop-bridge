# Protocol

## Packet Format

Each fake-test packet is one ASCII line ending in `\n`:

```text
PIPER,<seq>,<time_ms>,<q1_cd>,<q2_cd>,<q3_cd>,<q4_cd>,<q5_cd>,<q6_cd>,<gripper_p100>,<flags>,<checksum>
```

Example:

```text
PIPER,42,123456,1000,-500,2500,0,1500,300,5500,1,38921
```

Fields:

- `PIPER`: magic header.
- `seq`: integer packet sequence.
- `time_ms`: sender monotonic time in milliseconds since the sender script started.
- `q1_cd` to `q6_cd`: joint target angles in centi-degrees. `12.34 deg` is sent as `1234`.
- `gripper_p100`: gripper travel percent multiplied by 100. `55.25%` is sent as `5525`.
- `flags`: integer bitfield.
- `checksum`: unsigned 16-bit checksum.

## Real Teleop Packet Format

Real teleoperation uses a raw Piper packet in the same `PIPER` envelope:

```text
PIPER,<seq>,<time_ms>,<j1_raw>,<j2_raw>,<j3_raw>,<j4_raw>,<j5_raw>,<j6_raw>,<flags>,<checksum>
```

When the master gripper command changes, the packet includes gripper fields:

```text
PIPER,<seq>,<time_ms>,<j1_raw>,<j2_raw>,<j3_raw>,<j4_raw>,<j5_raw>,<j6_raw>,<gripper_angle>,<gripper_effort>,<gripper_code>,<flags>,<checksum>
```

Example:

```text
PIPER,42,123456,10000,20000,-30000,0,5000,-6000,35000,1000,1,3,20049
```

Fields:

- `j1_raw` to `j6_raw`: raw Piper joint targets in `0.001 degrees`, decoded from live master feedback frames `0x2A5`, `0x2A6`, and `0x2A7`.
- `gripper_angle`: optional raw gripper travel from feedback frame `0x2A8`, in `0.001 mm`.
- `gripper_effort`: raw gripper effort sent to the slave.
- `gripper_code`: gripper command code sent to the slave.
- `flags`: integer bitfield.
- `checksum`: unsigned 16-bit checksum.

## Flags

`flags` bit 0 is the teleoperation deadman:

- `1`: deadman enabled, receiver may command the slave Piper.
- `0`: deadman disabled, receiver treats the packet as a stop condition.

`flags` bit 1 means a gripper command is present:

- `1`: receiver sends `GripperCtrl()`.
- `0`: receiver sends only `JointCtrl()`.

## Checksum

The checksum is calculated over the packet string before the final comma and checksum field:

```text
PIPER,<seq>,<time_ms>,<...payload fields...>,<flags>
```

Algorithm:

1. Start with `uint16 c = 0x1234`.
2. For every ASCII byte in the payload:
   - rotate `c` left by 5 bits.
   - XOR `c` with the byte.
   - keep `c` masked to 16 bits.
3. Append the final decimal checksum as `,<checksum>\n`.

Python reference:

```python
def checksum16(payload: str) -> int:
    c = 0x1234
    for byte in payload.encode("ascii"):
        c = ((c << 5) | (c >> 11)) & 0xFFFF
        c ^= byte
    return c & 0xFFFF
```

## Stale Behavior

Board B declares stale when no valid LoRa packet has arrived for more than one second. It updates the OLED and prints:

```text
# STALE: no valid LoRa packet for >1s, fake slave would stop/freeze
```

Computer 2 also declares stale when no valid live packet has been read for more than `0.5` second. The real receiver warns and holds the last command, matching the UDP teleop reference.

## Rate Limits

LoRa is low-bandwidth. Do not forward raw high-rate CAN frames over LoRa.

The sender is flow-controlled by Board A `TX done`, so it sends the newest target only when the radio is ready. Gripper fields are omitted from normal joint packets unless the gripper command changes.

## Real Piper Unit Mapping

The Python real teleoperation scripts map units as follows:

- Piper CAN joint feedback frames: `0.001 degrees`.
- LoRa packet real teleop joint fields: `0.001 degrees`.
- Piper SDK gripper: `0.001 mm`.
- LoRa packet real teleop gripper field: derived from Piper `0x2A8` gripper feedback.
