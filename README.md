# Piper LoRa Teleop Bridge

LoRa-based wireless teleoperation between two AgileX Piper robot arms using two Heltec ESP32 WiFi LoRa 32 V4 boards.

The repository includes both:

- A fake transport test that proves serial -> LoRa -> serial.
- Real Piper teleoperation scripts that bridge Piper CAN state through the same LoRa packet.

Fake test path:

```text
Computer 1 fake sender
  -> USB Serial
  -> ESP32 Board A Serial->LoRa
  -> LoRa at 923.2 MHz
  -> ESP32 Board B LoRa->Serial
  -> USB Serial
  -> Computer 2 fake receiver
```

The fake sender generates smooth joint targets for `q1` to `q6` and `gripper`. The fake receiver validates packets and prints `Would send CAN command to slave Piper`.

Real teleoperation path:

```text
Master Piper arm
  -> CAN
  -> Computer 1 real Piper sender
  -> USB Serial
  -> ESP32 Board A Serial->LoRa
  -> LoRa at 923.2 MHz
  -> ESP32 Board B LoRa->Serial
  -> USB Serial
  -> Computer 2 real Piper receiver
  -> CAN
  -> Slave Piper arm
```

## Files

```text
README.md
requirements.txt
docs/
  architecture.md
  protocol.md
  troubleshooting.md
arduino/
  BoardA_SerialToLoRa/
    BoardA_SerialToLoRa.ino
  BoardB_LoRaToSerial/
    BoardB_LoRaToSerial.ino
scripts/
  computer1_fake_sender.py
  computer2_fake_receiver.py
  computer1_piper_sender.py
  computer2_piper_receiver.py
  piper_lora_protocol.py
  piper_sdk_adapter.py
  list_serial_ports.py
```

## Hardware

- 2 Heltec ESP32 WiFi LoRa 32 V4 boards
- 2 LoRa antennas
- 2 USB cables
- Computer 1 for Board A, and Computer 2 for Board B

Attach antennas to both boards before transmitting. Do not run LoRa TX without an antenna.

## Arduino IDE Setup

1. Install Arduino IDE.
2. On Windows, set the Arduino sketchbook location to `C:\Arduino`.
   This avoids OneDrive and Cyrillic path problems with some ESP32 tools.
3. Open Arduino IDE preferences.
4. Add this Additional Boards Manager URL:

   ```text
   https://resource.heltec.cn/download/package_heltec_esp32_index.json
   ```

5. Open Boards Manager and install the Heltec ESP32 board package.
6. Open Library Manager and install `Heltec ESP32 Dev-Boards`.
7. Select these board settings:

   ```text
   Board: Heltec WiFi LoRa 32(V4)
   USB CDC On Boot: Enabled
   Upload Mode: USB-OTG-CDC (TinyUSB)
   Upload Speed: 115200
   ```

## Upload Order

1. Open `arduino/BoardB_LoRaToSerial/BoardB_LoRaToSerial.ino`.
2. Upload it to Board B first.
3. Leave Board B connected to Computer 2 or powered.
4. Open `arduino/BoardA_SerialToLoRa/BoardA_SerialToLoRa.ino`.
5. Upload it to Board A.

Board A OLED should show `Board A` and `Serial->LoRa`.
Board B OLED should show stale until packets arrive.

## Python Setup With Conda

Use Python 3.10 or newer.

Create and activate a Conda environment:

```bash
conda create -n piper-lora-teleop python=3.10
conda activate piper-lora-teleop
pip install -r requirements.txt
```

If you prefer a newer Python available in Conda, Python 3.11 or 3.12 is also fine:

```bash
conda create -n piper-lora-teleop python=3.11
conda activate piper-lora-teleop
pip install -r requirements.txt
```

Optional non-Conda fallback:

Windows:

```powershell
py -3 -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

For real Piper control, run the real scripts on Linux computers with the Piper CAN adapters configured. The official AgileX SDK expects the CAN interface to be up at `1000000` bitrate, for example `can0`. Follow the Piper SDK CAN activation instructions for your adapter before starting these scripts.

List serial ports:

```bash
python scripts/list_serial_ports.py
```

Example ports:

- Board A on Computer 1: `COM9` or `/dev/ttyACM0`
- Board B on Computer 2: `COM10` or `/dev/ttyACM1`

## Run The Fake Test

On Computer 2, start the fake receiver first:

```bash
python scripts/computer2_fake_receiver.py --port COM10
```

On Computer 1, start the fake sender:

```bash
python scripts/computer1_fake_sender.py --port COM9 --rate 5
```

Use the real serial ports reported by `list_serial_ports.py`.

Expected sender output:

```text
Opening COM9 at 115200 baud
Sending fake PIPER packets at 5.00 Hz. Press Ctrl+C to stop.
TX PIPER,0,1000,1307,1279,2957,3195,628,-2454,5087,1,23592
```

Expected receiver output:

```text
Opening COM10 at 115200 baud
Waiting for valid PIPER packets from Board B. Press Ctrl+C to stop.
RX seq=0 age=12 ms deadman=enabled q1=  13.07 deg, q2=  12.79 deg, q3=  29.57 deg, q4=  31.95 deg, q5=   6.28 deg, q6= -24.54 deg, gripper= 50.87%
Would send CAN command to slave Piper
```

If packets stop for more than one second, Computer 2 prints:

```text
STALE: fake slave would stop/freeze now.
```

Board B also prints a debug line:

```text
# STALE: no valid LoRa packet for >1s, fake slave would stop/freeze
```

## Run Real Teleoperation

Run the fake test successfully before trying real robot motion. Keep the ESP32 sketches unchanged; they are serial radio modems.

On Computer 2, start the real receiver first:

```bash
python scripts/computer2_piper_receiver.py --port COM10 --can can0 --enable-arm
```

On Computer 1, start the real sender:

```bash
python scripts/computer1_piper_sender.py --port COM9 --can can0 --rate 5
```

Use the real serial ports from `scripts/list_serial_ports.py` and the real CAN names from `ip link show` / the Piper SDK CAN setup.

Common Linux example:

```bash
python scripts/computer2_piper_receiver.py --port /dev/ttyACM1 --can can0 --enable-arm
python scripts/computer1_piper_sender.py --port /dev/ttyACM0 --can can0 --rate 5
```

Useful options:

- `computer1_piper_sender.py --source control` reads master-arm control frames. This is the default and is usually what you want for a Piper master arm in master/slave mode.
- `computer1_piper_sender.py --source feedback` reads feedback frames instead.
- `computer1_piper_sender.py --configure-master` sends `MasterSlaveConfig(0xFA, 0, 0, 0)` before reading.
- `computer1_piper_sender.py --can-ok-timeout 10` waits up to 10 seconds for the Piper SDK CAN reader to become healthy, then exits.
- `computer1_piper_sender.py --ignore-can-ok` bypasses the SDK `isOk()` guard. Use this only after verifying the script is reading real Piper joint values.
- `computer2_piper_receiver.py --configure-slave` sends `MasterSlaveConfig(0xFC, 0, 0, 0)` before controlling.
- `computer2_piper_receiver.py --dry-run` validates packets and prints targets without writing Piper CAN motion commands.
- `computer2_piper_receiver.py --smoothing 0.35 --command-rate 20` controls interpolation between low-rate LoRa updates.
- `computer2_piper_receiver.py --no-disable-on-exit` leaves the arm enabled when the script exits. The default is to disable on exit.

If packets become stale for more than `--stale-timeout` seconds, or the deadman flag is off, the real receiver stops sending commands and disables the slave arm by default.

If Computer 1 prints `Piper SDK CAN reader is not OK yet`, no LoRa packets are being sent. Check that `can0` is up at `1000000` bitrate and that the master arm is publishing the frame type selected by `--source`. For a master arm, try:

```bash
python scripts/computer1_piper_sender.py --port /dev/ttyACM0 --can can0 --rate 1 --configure-master
```

For a normal/slave arm used as a source, try:

```bash
python scripts/computer1_piper_sender.py --port /dev/ttyACM0 --can can0 --rate 1 --source feedback
```

## LoRa Settings

- Frequency: `923200000` Hz
- TX power: `10` dBm
- Bandwidth: `125 kHz`
- Spreading factor: `SF7`
- Coding rate: `4/5`
- Preamble length: `8`
- IQ inversion: off
- CRC: enabled in the Heltec radio config

## Piper SDK Notes

The real scripts use AgileX `piper_sdk`:

- Computer 1 reads `GetArmJointCtrl()` / `GetArmGripperCtrl()` by default.
- Computer 2 sends `JointCtrl()` / `GripperCtrl()` after setting joint control mode.
- Joint units are converted between SDK `0.001 degrees` and LoRa packet centi-degrees.
- Gripper position is converted between SDK `0.001 mm` and packet percent using `--gripper-max-mm`, default `70.0`.
