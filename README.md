# Piper LoRa Teleop Bridge

Fake transport test for LoRa-based wireless teleoperation between two AgileX Piper robot arms using two Heltec ESP32 WiFi LoRa 32 V4 boards.

This first version does not control Piper arms and does not connect to CAN. It only proves this path:

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

## Python Setup

Use Python 3.10 or newer.

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

## LoRa Settings

- Frequency: `923200000` Hz
- TX power: `10` dBm
- Bandwidth: `125 kHz`
- Spreading factor: `SF7`
- Coding rate: `4/5`
- Preamble length: `8`
- IQ inversion: off
- CRC: enabled in the Heltec radio config

## Next Step After The Fake Test Works

Keep the ESP32 sketches as mostly unchanged serial radio modems. Replace `scripts/computer1_fake_sender.py` with a master Piper CAN reader, and replace the fake print in `scripts/computer2_fake_receiver.py` with a slave Piper CAN writer. Keep the LoRa packet compact, send joint targets at about 2-5 Hz, and smooth/interpolate on Computer 2.
