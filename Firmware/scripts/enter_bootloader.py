#!/usr/bin/env python3
"""Send the VCU bootloader request over serial."""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is required. Install with: python3 -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


COMMAND = b"$BOOT"
ACK = b"BOOT:ACK"
DEFAULT_BAUD = 115200


def candidate_ports() -> list[str]:
    ports = []
    preferred_tokens = (
        "usb",
        "uart",
        "serial",
        "stm",
        "stlink",
        "wch",
        "ch340",
        "cp210",
        "ftdi",
        "jlink",
    )

    for port in list_ports.comports():
        haystack = " ".join(
            str(part).lower()
            for part in (port.device, port.description, port.manufacturer, port.hwid)
            if part
        )
        score = sum(token in haystack for token in preferred_tokens)
        if score:
            ports.append((score, port.device))

    ports.sort(reverse=True)
    return [device for _, device in ports]


def send_boot(port: str, baud: int, timeout: float) -> bool:
    with serial.Serial(port, baudrate=baud, timeout=0.1, write_timeout=1) as ser:
        ser.reset_input_buffer()
        ser.write(COMMAND)
        ser.flush()

        deadline = time.monotonic() + timeout
        response = bytearray()
        while time.monotonic() < deadline:
            response.extend(ser.read(64))
            if ACK in response:
                return True
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-p", "--port", help="Serial port to use. If omitted, common USB UART ports are probed.")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate, default {DEFAULT_BAUD}.")
    parser.add_argument("-t", "--timeout", type=float, default=1.5, help="Seconds to wait for BOOT:ACK.")
    args = parser.parse_args()

    ports = [args.port] if args.port else candidate_ports()
    if not ports:
        print("No likely VCU serial port found. Connect the VCU USB/UART adapter, then retry or pass --port.", file=sys.stderr)
        return 1

    for port in ports:
        try:
            if send_boot(port, args.baud, args.timeout):
                print(f"{port}: received BOOT:ACK; VCU should now be in STM ROM bootloader.")
                return 0
            print(f"{port}: no ACK")
        except (OSError, serial.SerialException) as exc:
            print(f"{port}: {exc}", file=sys.stderr)

    print("No VCU acknowledged $BOOT. Reset/power-cycle the VCU and run this script during startup.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
