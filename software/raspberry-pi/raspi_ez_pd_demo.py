#!/usr/bin/env python3
"""Minimal EZ-PD BCR status poller for Raspberry Pi."""

import time

from smbus2 import SMBus


EZ_PD_DEFAULT_ADDRESS = 0x08
STATUS_REG_1 = 0x05


def read_status(bus: SMBus, address: int) -> int:
    """Return the current value of Status Register 1."""
    bus.write_byte(address, STATUS_REG_1)
    return bus.read_byte(address)


def main() -> None:
    bus_number = 1  # Raspberry Pi uses I2C bus 1 on the 40-pin header
    address = EZ_PD_DEFAULT_ADDRESS

    with SMBus(bus_number) as bus:
        while True:
            value = read_status(bus, address)
            print(f"Status Register 1 (0x{STATUS_REG_1:02X}): 0b{value:08b}")
            time.sleep(1.0)


if __name__ == "__main__":
    main()
