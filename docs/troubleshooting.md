# Troubleshooting

## `LoRaWan_APP.h` Missing

Install the Heltec ESP32 board package and the `Heltec ESP32 Dev-Boards` library in Arduino IDE. Confirm the Additional Boards Manager URL is:

```text
https://resource.heltec.cn/download/package_heltec_esp32_index.json
```

Restart Arduino IDE after installing.

## Wrong Arduino Sketchbook Location

On Windows, set the Arduino sketchbook to `C:\Arduino`. Some ESP32 tools fail when builds happen inside OneDrive paths or paths containing Cyrillic characters.

## OneDrive Or Cyrillic Path Issues

If compile or upload tools fail with strange path errors, copy the `arduino` folder to `C:\Arduino\piper-lora-teleop-bridge\arduino` and open the sketches from there.

## Failed Upload To ESP32-S3

Use these settings:

```text
Board: Heltec WiFi LoRa 32(V4)
USB CDC On Boot: Enabled
Upload Mode: USB-OTG-CDC (TinyUSB)
Upload Speed: 115200
```

Close Serial Monitor and any Python script before uploading. Only one program can use the serial port at a time.

## Bootloader Sequence

If upload does not start:

1. Hold `PRG`.
2. Tap `RST`.
3. Release `PRG`.
4. Start upload again.

The COM port can change when the board enters bootloader mode. Recheck the selected port.

## No Serial Data Received

- Make sure `USB CDC On Boot` is enabled.
- Make sure you selected the normal runtime COM port, not the bootloader COM port.
- Close Arduino Serial Monitor before starting Python.
- Run `python scripts/list_serial_ports.py` again after unplugging/replugging the board.

## Serial Monitor Blocks Python

Arduino Serial Monitor and Python cannot both open the same serial port. Close Serial Monitor before running:

```bash
python scripts/computer2_piper_receiver.py --port /dev/ttyACM0 --can can0 --dry-run
python scripts/computer1_piper_sender.py --port /dev/ttyACM0 --can can0
```

## Antennas Not Attached

Attach antennas to both Heltec boards before transmitting. Transmitting without an antenna can damage the radio.

## Board A Says `TX done` But Board B Receives Nothing

Check:

- Both boards have antennas attached.
- Board B sketch was uploaded and is powered.
- Both sketches use `RF_FREQUENCY 923200000`.
- Both sketches use `LORA_BANDWIDTH 1`.
- Both sketches use the same spreading factor, coding rate, preamble, IQ inversion, and CRC settings.
- Boards are not too far apart for the first test. Start a few meters apart.

## Mismatched Frequency Or Settings

The sketches must match:

```text
Frequency: 923200000
Bandwidth: 250 kHz
Spreading factor: SF7
Coding rate: 4/5
Preamble length: 8
IQ inversion: off
CRC: on
```

## Invalid Binary Packet

If Board A prints `WARN: dropping invalid binary packet`, Computer 1 and Board A are not using the same binary packet format. Pull the latest repo on Computer 1 and re-upload Board A from the same repo.

If Board B prints `# Dropping invalid binary packet`, Board A and Board B are running mismatched firmware or the LoRa payload was corrupted. Re-upload both sketches from the same repo.

## Stale Packet Warning

Stale means no valid packet has arrived for more than one second.

Common causes:

- Computer 1 sender is not running.
- Board A is not connected to Computer 1.
- Wrong serial port was used.
- Board A is dropping packets due to invalid binary packets.
- Board B is not receiving LoRa packets.
- Frequency or LoRa settings do not match.

## Slave Does Not Follow Master Joints

On Computer 1, check live master feedback:

```bash
candump can0
```

You must see `0x2A5`, `0x2A6`, and `0x2A7`, and the data should change when you move the master arm. If only gripper-related frames change, the slave will only appear to follow the gripper.

## Slave Jumps To An Old Pose On Startup

This should not happen with the current sender because it uses live feedback frames only. If it happens, confirm you pulled the latest repo on Computer 1 and that the sender prints:

```text
[MASTER] Waiting for fresh 0x2A5/0x2A6/0x2A7 joint feedback set
```
