#!/usr/bin/env python3
"""List serial ports visible to pyserial."""

import sys

from serial.tools import list_ports


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 0

    for port in ports:
        print(f"{port.device}\t{port.description}\t{port.hwid}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
