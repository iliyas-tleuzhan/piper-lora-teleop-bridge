# Piper LoRa Teleop Operation Guide

This guide is the short operating procedure for the two-arm LoRa teleoperation setup.

## Pre-Run Safety

- Keep the slave arm power cutoff or E-stop within reach.
- Attach antennas before powering either LoRa board.
- Keep the master and slave arms on separate CAN buses.
- Start with both arms in similar safe poses.
- Keep people and loose objects outside the slave arm workspace.

## Bring Up CAN

Run on both computers:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

Verify CAN traffic:

```bash
candump can0
```

Computer 1 must see either feedback frames `0x2A5/0x2A6/0x2A7` or command frames `0x155/0x156/0x157`.

## Match Firmware Limits

Read limits on each arm:

```bash
python scripts/piper_configure_motor_limits.py --can can0
```

Apply the teleop profile only if an arm reports different limits:

```bash
python scripts/piper_configure_motor_limits.py --can can0 --confirm WRITE_LIMITS
```

Power-cycle the arm after writing limits. Apply the same profile on both arms.

## Optional Zero Calibration

Use this only when both arms have matching limits but the same command angles still produce different physical poses.

1. Put one arm in the shared physical zero/neutral reference pose.
2. Support the arm.
3. Run:

   ```bash
   python scripts/piper_set_zero_guided.py --can can0 --confirm SET_ZERO
   ```

4. Repeat on the other arm using the same physical reference pose.
5. Power-cycle both arms.
6. Recheck that the same physical pose gives the same feedback angles on both arms.

For a single visibly offset joint:

```bash
python scripts/piper_set_zero_guided.py --can can0 --joint 6 --confirm SET_ZERO
```

## Start Teleoperation

Start Computer 2 first:

```bash
conda activate piper-lora-teleop
cd ~/Iliyas/piper-lora-teleop-bridge
python scripts/computer2_piper_receiver.py --port /dev/ttyACM0 --can can0 --confirm MOVE
```

Wait for:

```text
[SLAVE] Startup pose locked at [...] deg
[SLAVE] Waiting for raw Piper LoRa teleop packets. Press Ctrl+C to stop.
```

Start Computer 1 second:

```bash
conda activate piper-lora-teleop
cd ~/Iliyas/piper-lora-teleop-bridge
python scripts/computer1_piper_sender.py --port /dev/ttyACM0 --can can0
```

Computer 1 should print `source=feedback` or `source=command`. `source=feedback` is the best source because it represents live master joint feedback. `source=command` follows the same command frame stream as the previous UDP bridge.

After Computer 2 prints startup sync, move the master arm slightly to arm absolute tracking. The first packet should not move the slave.

## Stop Teleoperation

1. Stop Computer 1 with `Ctrl+C`.
2. Stop Computer 2 with `Ctrl+C`.
3. Power off or disable the slave arm after motion has stopped.

## Quick Diagnosis

Only gripper moves:

- Computer 1 is not seeing a full six-joint source. Check `candump can0`.

Slave jumps at startup:

- Pull the latest repo on both computers.
- Start Computer 2 before Computer 1.
- Confirm Computer 2 prints `Startup pose locked`.

Motion is delayed or jittery:

- Confirm only one sender is running.
- Confirm Board B is not repeatedly disconnecting.
- Confirm Computer 2 `cmd_rate` is steady and not bursty.

Arms are consistently offset:

- Match firmware limits on both arms.
- If the offset remains, run guided zero calibration using the same physical zero reference on both arms.
