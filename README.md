# Piper LoRa Teleop Bridge

Real LoRa teleoperation between two AgileX Piper robot arms using two Heltec ESP32 WiFi LoRa 32 V4 boards.

This project mirrors the working `piper-wireless-teleop` UDP behavior, but replaces UDP/Wi-Fi with:

```text
Computer 1 -> USB Serial -> ESP32 Board A -> LoRa -> ESP32 Board B -> USB Serial -> Computer 2
```

The ESP32 boards are simple serial LoRa modems. They do not understand Piper CAN. Computer 1 reads master Piper SocketCAN target frames, sends compact 47-byte binary `PLT1` packets over LoRa, and Computer 2 validates those packets before commanding the slave Piper with `piper_sdk`.

For the short day-to-day runbook, see [docs/operation.md](docs/operation.md).

## Hardware

Use two separate Piper CAN buses:

- Master Piper arm connected only to Computer 1.
- Slave Piper arm connected only to Computer 2.
- Board A connected by USB to Computer 1.
- Board B connected by USB to Computer 2.
- LoRa antennas attached to both Heltec boards before any transmit.
- Slave arm power cutoff or E-stop within reach.

Do not connect both Piper arms to the same CAN bus for wireless teleop.

## Step 1: Clean Clone And Create Conda Env

Run on both computers. If you are resetting from an old checkout, remove the old folders first:

```bash
cd ~/Iliyas
rm -rf piper-lora-teleop-bridge piper-lora-teleop-bridge-main
git clone https://github.com/iliyas-tleuzhan/piper-lora-teleop-bridge.git
cd piper-lora-teleop-bridge
conda create -n piper-lora-teleop python=3.11
conda activate piper-lora-teleop
pip install -r requirements.txt
```

Use this conda environment for all commands below:

```bash
conda activate piper-lora-teleop
```

## Step 2: Configure CAN On Both Computers

Install CAN tools:

```bash
sudo apt update
sudo apt install -y can-utils iproute2 net-tools
```

Bring `can0` up at Piper bitrate:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

If `can0` does not exist, check your CAN adapter driver and the actual interface name:

```bash
ip link
```

To restart CAN after unplugging the adapter or changing wiring:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

Check that the master arm is producing Piper frames on Computer 1:

```bash
candump can0
```

For teleop, Computer 1 must see one complete joint source. The sender prefers live feedback IDs if available:

- `0x2A5`: joints 1 and 2
- `0x2A6`: joints 3 and 4
- `0x2A7`: joints 5 and 6
- `0x2A8`: optional gripper feedback

If those are not present, the sender uses the same command IDs as the working UDP bridge:

- `0x155`: joints 1 and 2
- `0x156`: joints 3 and 4
- `0x157`: joints 5 and 6
- `0x159`: optional gripper command

If Computer 1 prints `Waiting for complete joint frames`, run `candump can0` and verify either `0x2A5/0x2A6/0x2A7` or `0x155/0x156/0x157` are present.

## Step 3: Match Piper Motor Limits

Run this on both Piper computers before serious teleoperation testing. First read the current firmware limits:

```bash
python scripts/piper_configure_motor_limits.py --can can0
```

The bridge uses this full-motion teleop profile:

| Joint | Min deg | Max deg | Max speed |
| --- | ---: | ---: | ---: |
| J1 | -150 | 150 | 3.0 rad/s |
| J2 | 0 | 180 | 3.0 rad/s |
| J3 | -170 | 0 | 3.0 rad/s |
| J4 | -100 | 100 | 3.0 rad/s |
| J5 | -70 | 70 | 3.0 rad/s |
| J6 | -170 | 170 | 3.0 rad/s |

If either arm reports different firmware limits, apply the same profile to that arm:

```bash
python scripts/piper_configure_motor_limits.py --can can0 --confirm WRITE_LIMITS
```

Do this once on the master arm computer and once on the slave arm computer. The command writes persistent driver settings, so power-cycle both arms afterwards.

The official SDK `JointCtrl()` documentation still lists joint 6 as `-120` to `+120` degrees, but the current SDK motor-limit demo writes joint 6 as `-170` to `+170` degrees. This repo follows the writable SDK demo profile so both arms can use the same available wrist-roll range. If your arm firmware rejects this or reports angle-limit errors, use the read command above and keep both arms on the same lower reported range.

## Step 4: Optional Zero Calibration

Use this only if both arms have matching limits but still hold slightly different physical poses for the same joint angles. Zero calibration writes persistent motor calibration.

1. Put both arms in the same physical zero/neutral reference pose. Do not use a random comfortable pose; this pose becomes the firmware zero.
2. Support the arm before disabling any joint.
3. Run the guided script on one arm at a time:

   ```bash
   python scripts/piper_set_zero_guided.py --can can0 --confirm SET_ZERO
   ```

4. Repeat the same procedure on the other arm.
5. Power-cycle both arms.
6. Recheck that the same physical pose produces the same feedback angles on both arms.

If only one joint is visibly offset, calibrate just that joint:

```bash
python scripts/piper_set_zero_guided.py --can can0 --joint 6 --confirm SET_ZERO
```

Zero calibration can reduce static offsets between arms. It will not fix packet loss, LoRa latency, loose hardware, gravity sag, or a sender that is using stale command frames.

## Step 5: Upload ESP32 Firmware

Arduino IDE setup:

1. Install Arduino IDE.
2. Add this Additional Boards Manager URL:

   ```text
   https://resource.heltec.cn/download/package_heltec_esp32_index.json
   ```

3. Install the Heltec ESP32 board package.
4. Install the `Heltec ESP32 Dev-Boards` library.
5. Select:

   ```text
   Board: Heltec WiFi LoRa 32(V4)
   USB CDC On Boot: Enabled
   Upload Mode: USB-OTG-CDC (TinyUSB)
   Upload Speed: 115200
   ```

Upload order:

1. Upload `arduino/BoardB_LoRaToSerial/BoardB_LoRaToSerial.ino` to Board B.
2. Leave Board B connected to Computer 2.
3. Upload `arduino/BoardA_SerialToLoRa/BoardA_SerialToLoRa.ino` to Board A.
4. Leave Board A connected to Computer 1.

Board A should show `Board A` / `Serial->LoRa`. Board B should show stale until packets arrive.

## Step 6: Find ESP32 Serial Ports

Run on each computer:

```bash
python scripts/list_serial_ports.py
```

Use the actual ports printed by the script. On separate Linux computers, each board is often `/dev/ttyACM0`. Close Arduino Serial Monitor before running Python.

## Step 7: Start Computer 2 First

On Computer 2, connected to the slave Piper and Board B:

```bash
conda activate piper-lora-teleop
cd ~/Iliyas/piper-lora-teleop-bridge
python scripts/computer2_piper_receiver.py --port /dev/ttyACM0 --can can0 --confirm MOVE
```

`--confirm MOVE` is required. Without it, the receiver refuses to command the robot.

Optional receiver check:

- `--dry-run`: validate LoRa packets without connecting to or moving Piper.

At startup the receiver reads the slave arm's current joint feedback and commands that current pose once. The first incoming master packet only arms startup sync and does not move the slave. After the master target moves, the receiver tracks the master's absolute joint targets with a small jump guard and tiny deadband. This keeps startup safe without preserving a permanent offset between the two arms.

## Step 8: Start Computer 1

Before starting, put the master and slave arms in similar safe poses.

On Computer 1, connected to the master Piper and Board A:

```bash
conda activate piper-lora-teleop
cd ~/Iliyas/piper-lora-teleop-bridge
python scripts/computer1_piper_sender.py --port /dev/ttyACM0 --can can0
```

The sender prefers live feedback frames when available. If the master CAN bus does not expose them, it automatically uses the UDP-compatible command frames `0x155`, `0x156`, and `0x157`. It requests 50 Hz updates, but sends the newest target only when Board A reports `TX done`, so the actual speed is the maximum the LoRa link can sustain.

## Normal Shutdown

1. Stop Computer 1 sender with `Ctrl+C`.
2. Stop Computer 2 receiver with `Ctrl+C`.
3. Power off or disable the slave arm only after motion has stopped.
4. If CAN or USB was unplugged, restart CAN before the next run:

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000 restart-ms 100
   sudo ip link set can0 up
   ```

## Expected Output

Computer 1 should print:

```text
[MASTER] Sending 47-byte LoRa teleop packets to /dev/ttyACM0 at 50.00 Hz
[MASTER] seq=12 source=feedback deg=[...] gripper=unchanged
```

Computer 2 should print:

```text
[SLAVE] Startup pose locked at [...] deg
[SLAVE] startup sync: holding current slave pose; absolute tracking starts when the master target moves
[SLAVE] accepted seq=12 dropped=0 total_dropped=0 cmd_rate=...
```

If Computer 2 prints no accepted packets:

1. Confirm Board A and Board B are powered and have antennas.
2. Confirm both ESP32 sketches were re-uploaded from this repo.
3. Confirm both ESP32 sketches use `923200000` Hz and `LORA_BANDWIDTH 1`.
4. Confirm the serial ports are correct and not open in Arduino Serial Monitor.
5. Confirm Computer 1 sees either `0x2A5/0x2A6/0x2A7` or `0x155/0x156/0x157` with `candump can0`.

## Hong Kong LoRa Settings

The firmware is configured for Hong Kong operation in the 920-925 MHz band:

- Frequency: `923200000` Hz
- TX power: `10` dBm
- Bandwidth: `250 kHz`
- Spreading factor: `SF7`
- Coding rate: `4/5`
- Preamble length: `8`
- IQ inversion: off
- CRC: enabled

`923.2 MHz` with `250 kHz` bandwidth stays inside 920-925 MHz. The lower-latency transport comes from two changes: the real packets are now fixed 47-byte binary frames, and the radio bandwidth is now 250 kHz instead of 125 kHz.

## Safety Checklist

Before enabling motion:

- Slave E-stop or power cutoff is reachable.
- Both arms start in similar poses.
- Master and slave are on separate CAN buses.
- `candump can0` works on both computers.
- Computer 2 receiver is started before Computer 1 sender.
- Computer 1 sees either `0x2A5/0x2A6/0x2A7` or `0x155/0x156/0x157` before you expect motion.
- Computer 2 prints `Startup pose locked` before Computer 1 starts sending.

## Troubleshooting

CAN interface missing:

```bash
ip link
```

CAN interface down:

```bash
sudo ip link set can0 up
ip -details link show can0
```

Reset CAN interface:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

No master joint frames:

```bash
candump can0
```

Look for either `0x2A5/0x2A6/0x2A7` or `0x155/0x156/0x157`. If both sets are missing, the sender cannot build a complete teleop target.

Jitter, lag, or poor matching:

The receiver now tracks absolute master joint targets after startup. If it still does not match, check the Computer 1 `source=...` log. `source=feedback` is the real master pose. `source=command` matches the old UDP bridge, but it can only be as accurate as the command frames being published on the master CAN bus. Also confirm only one Computer 1 sender is running and that Board B is not repeatedly disconnecting.

Wrist roll stops early:

The bridge allows joint 6 commands up to `-170` to `+170` degrees. If the wrist still stops earlier, read the firmware limits:

```bash
python scripts/piper_configure_motor_limits.py --can can0
```

If one arm has a lower joint 6 range, apply the same limit profile on both arms with `--confirm WRITE_LIMITS`, power-cycle, and test again.

Serial port busy:

```bash
python scripts/list_serial_ports.py
```

Close Arduino Serial Monitor and any other process using `/dev/ttyACM0` or the port shown by `list_serial_ports.py`.

Board B serial disconnect/reopen messages:

The receiver automatically reopens Board B serial if the ESP32 USB CDC port resets or briefly disconnects. If it keeps printing reopen messages, unplug/replug Board B, check the USB cable, close Arduino Serial Monitor, and rerun `python scripts/list_serial_ports.py` to confirm the port name did not change.
