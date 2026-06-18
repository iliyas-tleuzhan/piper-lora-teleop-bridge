# Piper LoRa Teleop Bridge

Real LoRa teleoperation between two AgileX Piper robot arms using two Heltec ESP32 WiFi LoRa 32 V4 boards.

This project mirrors the `piper-wireless-teleop` UDP control behavior, but replaces UDP/Wi-Fi with:

```text
Computer 1 -> USB Serial -> ESP32 Board A -> LoRa -> ESP32 Board B -> USB Serial -> Computer 2
```

The ESP32 boards are serial LoRa modems. They do not understand Piper CAN. Computer 1 reads raw master Piper SocketCAN command frames, sends compact `PIPER` packets over LoRa, and Computer 2 validates those packets before commanding the slave Piper with `piper_sdk`.

## Hardware

Use two separate Piper CAN buses:

- Master Piper arm connected only to Computer 1.
- Slave Piper arm connected only to Computer 2.
- Board A connected by USB to Computer 1.
- Board B connected by USB to Computer 2.
- LoRa antennas attached to both Heltec boards before any transmit.
- Slave arm power cutoff or E-stop within reach.

Do not connect both Piper arms to the same CAN bus for wireless teleop.

## Step 1: Clone And Create Conda Env

Run on both computers:

```bash
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

Check that the master arm is producing command frames on Computer 1:

```bash
candump can0
```

For teleop, Computer 1 must see these master command IDs:

- `0x155`: joints 1 and 2
- `0x156`: joints 3 and 4
- `0x157`: joints 5 and 6
- `0x159`: optional gripper command

If Computer 1 later prints `Waiting for complete joint target frames`, run `candump can0` and verify `0x155`, `0x156`, and `0x157` are present.

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

Common Linux ports:

- Board A on Computer 1: `/dev/ttyACM0`
- Board B on Computer 2: `/dev/ttyACM1`

Use the actual ports printed by the script. Close Arduino Serial Monitor before running Python.

## Step 5: Start Computer 2 First

On Computer 2, connected to the slave Piper and Board B:

```bash
conda activate piper-lora-teleop
cd piper-lora-teleop-bridge
python scripts/computer2_piper_receiver.py --port /dev/ttyACM1 --can can0 --confirm MOVE
```

`--confirm MOVE` is required. Without it, the receiver refuses to command the robot.

Useful receiver options:

- `--dry-run`: validate LoRa packets without connecting to or moving Piper.
- `--stale-timeout 0.5`: warn and hold the last command if packets stop.
- `--enable-slew-limit --max-step-deg 3`: optional step limiting. Disabled by default for direct teleop feel.
- `--disable-on-exit`: disable all motors when the receiver exits.
- `--speed-percent 100 --follow-mode 0xAD`: default Piper motion settings.

## Step 6: Start Computer 1

On Computer 1, connected to the master Piper and Board A:

```bash
conda activate piper-lora-teleop
cd piper-lora-teleop-bridge
python scripts/computer1_piper_sender.py --port /dev/ttyACM0 --can can0 --rate 5 --deadman
```

`--deadman` is required. The receiver ignores packets with deadman disabled.

Useful sender options:

- `--rate 5`: LoRa packet rate. Keep this modest; LoRa is low bandwidth.
- `--verbose-packets`: print every raw packet line sent to Board A.
- `--can-timeout 0.02`: SocketCAN receive timeout.

## Expected Output

Computer 1 should eventually print status like:

```text
[MASTER] seq=12 deadman=True deg=[...] gripper={...}
```

Computer 2 should print status like:

```text
[SLAVE] accepted seq=12 dropped=0 total_dropped=0 cmd_rate=...
```

If Computer 2 prints no accepted packets:

1. Confirm Computer 1 is running with `--deadman`.
2. Confirm Board A and Board B are powered and have antennas.
3. Confirm both ESP32 sketches use `923200000` Hz.
4. Confirm the serial ports are correct and not open in Arduino Serial Monitor.

## LoRa Settings

- Frequency: `923200000` Hz
- TX power: `10` dBm
- Bandwidth: `125 kHz`
- Spreading factor: `SF7`
- Coding rate: `4/5`
- Preamble length: `8`
- IQ inversion: off
- CRC: enabled

## Safety Checklist

Before enabling motion:

- Slave E-stop or power cutoff is reachable.
- Both arms start in similar poses.
- Master and slave are on separate CAN buses.
- `candump can0` works on both computers.
- Computer 2 receiver is started before Computer 1 sender.
- Computer 1 sender is started with `--deadman`.
- Start at low LoRa rate such as `--rate 2` or `--rate 5`.

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

No master command frames:

```bash
candump can0
```

Look for `0x155`, `0x156`, and `0x157`. If they are missing, the sender cannot build a complete teleop target.

Serial port busy:

```bash
python scripts/list_serial_ports.py
```

Close Arduino Serial Monitor and any other process using `/dev/ttyACM0` or `/dev/ttyACM1`.
