# Protocol

## Packet Format

Each fake test packet is one ASCII line ending in `\n`:

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
- `q1_cd` to `q6_cd`: joint angles in centi-degrees. `12.34 deg` is sent as `1234`.
- `gripper_p100`: gripper percent multiplied by 100. `55.25%` is sent as `5525`.
- `flags`: integer bitfield.
- `checksum`: unsigned 16-bit checksum.

## Flags

`flags` bit 0 is the fake deadman:

- `1`: deadman enabled, fake receiver prints `Would send CAN command to slave Piper`.
- `0`: deadman disabled, fake receiver treats the packet as a stop/freeze condition.

## Checksum

The checksum is calculated over the packet string before the final comma and checksum field:

```text
PIPER,<seq>,<time_ms>,<q1_cd>,<q2_cd>,<q3_cd>,<q4_cd>,<q5_cd>,<q6_cd>,<gripper_p100>,<flags>
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

Computer 2 also declares stale when no valid packet has been read for more than `--stale-timeout`, default `1.0` second:

```text
STALE: fake slave would stop/freeze now.
```

## Rate Limits

LoRa is low-bandwidth. Do not forward raw high-rate CAN frames over LoRa.

For a real Piper LoRa demo, send compact joint targets at 2-5 Hz and let Computer 2 smooth/interpolate locally. Keep packets short and drop corrupted or stale packets.
