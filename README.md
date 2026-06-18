# Piper LoRa Teleop Bridge

Real LoRa teleoperation between two AgileX Piper robot arms using two Heltec ESP32 WiFi LoRa 32 V4 boards.

This project mirrors the working `piper-wireless-teleop` UDP behavior, but replaces UDP/Wi-Fi with:

```text
Computer 1 -> USB Serial -> ESP32 Board A -> LoRa -> ESP32 Board B -> USB Serial -> Computer 2
```

The ESP32 boards are simple serial LoRa modems. They do not understand Piper CAN. Computer 1 reads live master Piper SocketCAN feedback frames, sends compact 47-byte binary `PLT1` packets over LoRa, and Computer 2 validates those packets before commanding the slave Piper with `piper_sdk`.

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

Check that the master arm is producing live feedback frames on Computer 1:

```bash
candump can0
```

For teleop, Computer 1 must see these master feedback IDs:

- `0x2A5`: joints 1 and 2
- `0x2A6`: joints 3 and 4
- `0x2A7`: joints 5 and 6
- `0x2A8`: optional gripper feedback

If Computer 1 prints `Waiting for fresh joint feedback frames`, run `candump can0` and verify `0x2A5`, `0x2A6`, and `0x2A7` are present and changing when you move the master arm. If those IDs are missing, Computer 1 is not seeing the live physical master pose.

## Step 3: Upload ESP32 Firmware

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

## Step 4: Find ESP32 Serial Ports

Run on each computer:

```bash
python scripts/list_serial_ports.py
```

Use the actual ports printed by the script. On separate Linux computers, each board is often `/dev/ttyACM0`. Close Arduino Serial Monitor before running Python.

## Step 5: Start Computer 2 First

On Computer 2, connected to the slave Piper and Board B:

```bash
conda activate piper-lora-teleop
cd ~/Iliyas/piper-lora-teleop-bridge
python scripts/computer2_piper_receiver.py --port /dev/ttyACM0 --can can0 --confirm MOVE
```

`--confirm MOVE` is required. Without it, the receiver refuses to command the robot.

Optional receiver check:

- `--dry-run`: validate LoRa packets without connecting to or moving Piper.

## Step 6: Start Computer 1

Before starting, put the master and slave arms in similar safe poses.

On Computer 1, connected to the master Piper and Board A:

```bash
conda activate piper-lora-teleop
cd ~/Iliyas/piper-lora-teleop-bridge
python scripts/computer1_piper_sender.py --port /dev/ttyACM0 --can can0
```

The sender uses live feedback frames, not latched command frames. It waits for a fresh full set of `0x2A5`, `0x2A6`, and `0x2A7`, then sends the newest target only when Board A reports `TX done`.

## Expected Output

Computer 1 should print:

```text
[MASTER] Sending 47-byte LoRa teleop packets to /dev/ttyACM0 at 15.00 Hz
[MASTER] seq=12 deg=[...] gripper=unchanged
```

Computer 2 should print:

```text
[SLAVE] accepted seq=12 dropped=0 total_dropped=0 cmd_rate=...
```

If Computer 2 prints no accepted packets:

1. Confirm Board A and Board B are powered and have antennas.
2. Confirm both ESP32 sketches were re-uploaded from this repo.
3. Confirm both ESP32 sketches use `923200000` Hz and `LORA_BANDWIDTH 1`.
4. Confirm the serial ports are correct and not open in Arduino Serial Monitor.
5. Confirm Computer 1 sees `0x2A5`, `0x2A6`, and `0x2A7` with `candump can0`.

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
- Computer 1 sees `0x2A5`, `0x2A6`, and `0x2A7` before you expect motion.

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

No master feedback frames:

```bash
candump can0
```

Look for `0x2A5`, `0x2A6`, and `0x2A7`. If they are missing, the sender cannot build a complete live teleop target.

Serial port busy:

```bash
python scripts/list_serial_ports.py
```

Close Arduino Serial Monitor and any other process using `/dev/ttyACM0` or the port shown by `list_serial_ports.py`.

Board B serial disconnect/reopen messages:

The receiver automatically reopens Board B serial if the ESP32 USB CDC port resets or briefly disconnects. If it keeps printing reopen messages, unplug/replug Board B, check the USB cable, close Arduino Serial Monitor, and rerun `python scripts/list_serial_ports.py` to confirm the port name did not change.
