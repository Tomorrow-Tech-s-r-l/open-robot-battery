"""Backend for the Infineon EZ-PD BCR USB-C sink controller."""

from __future__ import annotations

import time
from typing import Optional, Sequence, Tuple

from .i2c_backend import I2CBackend, I2CBackendError

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:  # pragma: no cover - dependency not available during unit tests
    SMBus = None  # type: ignore[assignment]
    i2c_msg = None  # type: ignore[assignment]


def _normalize_bus_id(bus_id: str) -> int:
    """Convert various bus identifiers into an integer SMBus index."""
    bus_id = bus_id.strip()
    if bus_id.startswith("/dev/i2c-"):
        return int(bus_id.split("-")[-1])
    if bus_id.lower().startswith("i2c-"):
        return int(bus_id.split("-")[-1])
    return int(bus_id)


class EZPDBCRI2CBackend(I2CBackend):
    """Hardware backend that talks to the EZ-PD BCR controller over I2C."""

    def __init__(self) -> None:
        super().__init__()
        self._bus: Optional[SMBus] = None
        self._bus_number: Optional[int] = None
        self._default_address: int = 0x08

    def configure(self, *, bus_id: str, configuration: Optional[dict] = None) -> None:
        if SMBus is None or i2c_msg is None:
            raise I2CBackendError(
                "The 'smbus2' package is required for the EZ-PD BCR backend. "
                "Install it with 'pip install smbus2'."
            )

        self._bus_id = bus_id
        self._bus_number = _normalize_bus_id(bus_id)
        self._default_address = 0x08

        configuration = configuration or {}
        if "default_address" in configuration:
            self._default_address = int(configuration["default_address"]) & 0x7F

        if self._bus:
            self._bus.close()
        self._bus = SMBus(self._bus_number)

        init_sequence = configuration.get("init_sequence", [])
        for entry in init_sequence:
            self._process_init_entry(entry)

    def _process_init_entry(self, entry: dict) -> None:
        try:
            target_address = int(entry.get("address", self._default_address)) & 0x7F
            payload = entry["write_data"]
            if not isinstance(payload, list) or not payload:
                raise TypeError("write_data must be a non-empty list of integers")
            delay_ms = float(entry.get("delay_ms", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise I2CBackendError(f"Invalid init_sequence entry: {entry}") from exc

        self._write_only(target_address, payload)
        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)

    def _ensure_bus(self) -> SMBus:
        if self._bus is None or self._bus_number is None:
            raise I2CBackendError("I2C bus not configured")
        return self._bus

    def _write_only(self, address: int, data: Sequence[int]) -> None:
        bus = self._ensure_bus()
        try:
            write_msg = i2c_msg.write(address, bytes([x & 0xFF for x in data]))
            bus.i2c_rdwr(write_msg)
        except OSError as exc:
            raise I2CBackendError(f"Write to address 0x{address:02X} failed: {exc}") from exc

    def perform_transaction(
        self,
        address: int,
        write_data: Sequence[int],
        read_length: int,
        timeout: float,
    ) -> Tuple[bool, bytes, str]:
        if address == 0:
            address = self._default_address

        bus = self._ensure_bus()
        write_buffer = bytes([x & 0xFF for x in write_data])
        read_length = max(0, int(read_length))

        try:
            if write_buffer and read_length > 0:
                write_msg = i2c_msg.write(address, write_buffer)
                read_msg = i2c_msg.read(address, read_length)
                bus.i2c_rdwr(write_msg, read_msg)
                return True, bytes(read_msg), ""

            if write_buffer:
                write_msg = i2c_msg.write(address, write_buffer)
                bus.i2c_rdwr(write_msg)
                return True, b"", ""

            if read_length > 0:
                read_msg = i2c_msg.read(address, read_length)
                bus.i2c_rdwr(read_msg)
                return True, bytes(read_msg), ""

            return True, b"", ""
        except OSError as exc:
            return False, b"", f"I2C communication error: {exc}"

    def close(self) -> None:
        if self._bus:
            self._bus.close()
            self._bus = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:  # pragma: no cover - best effort cleanup
            pass
