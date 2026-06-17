# Architecture

## Final Real Goal

The final real system is intended to bridge joint targets between two AgileX Piper arms:

```text
Master Piper arm
  -> CAN
  -> Computer 1 / Orin / Linux laptop
  -> USB Serial
  -> Heltec ESP32 Board A
  -> LoRa node-to-node, 923.2 MHz
  -> Heltec ESP32 Board B
  -> USB Serial
  -> Computer 2 / Linux laptop
  -> CAN
  -> Slave Piper arm
```

## First Fake Test

This repository implements only the fake transport test:

```text
Computer 1 fake packet
  -> Serial
  -> Board A
  -> LoRa
  -> Board B
  -> Serial
  -> Computer 2 fake CAN output
```

There is no Piper control and no CAN connection in this version. The Python sender generates smooth fake joint targets. Board A forwards valid `PIPER` serial lines over LoRa. Board B validates LoRa packets and forwards valid `PIPER` lines over serial. The Computer 2 receiver validates the packet again and prints what it would send to the slave Piper.

## Why The ESP32 Boards Stay Simple

The ESP32 boards act as radio modems. They do not understand Piper CAN. This keeps the embedded code small and reduces risk:

- Board A reads a compact ASCII packet from USB serial and transmits it over LoRa.
- Board B receives a LoRa packet and prints it over USB serial.
- Both boards validate the checksum so corrupted packets are dropped early.
- Board B detects stale traffic and displays/prints a fake stop warning.

## Future Real Upgrade

After the fake test works:

- Replace `scripts/computer1_fake_sender.py` with a master Piper CAN reader.
- Replace the fake CAN print in `scripts/computer2_fake_receiver.py` with a slave Piper CAN writer.
- Keep the ESP32 sketches mostly unchanged because they are just serial-to-LoRa and LoRa-to-serial modems.
- Send compact joint targets rather than raw high-rate CAN frames.
