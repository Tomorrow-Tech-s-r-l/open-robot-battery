"""Backend abstractions used by the I2C manager node."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Sequence, Tuple


class I2CBackendError(RuntimeError):
    """Exception raised when the backend cannot fulfil a transaction."""


class I2CBackend(ABC):
    """Abstract interface implemented by concrete I2C backends."""

    def __init__(self) -> None:
        self._bus_id: Optional[str] = None

    @abstractmethod
    def configure(self, *, bus_id: str, configuration: Optional[dict] = None) -> None:
        """Configure the backend for a specific bus."""

    @abstractmethod
    def perform_transaction(
        self,
        address: int,
        write_data: Sequence[int],
        read_length: int,
        timeout: float,
    ) -> Tuple[bool, bytes, str]:
        """Perform a single I2C transaction."""

    @property
    def bus_id(self) -> Optional[str]:
        """Return the identifier of the configured bus."""
        return self._bus_id


@dataclass
class SimulatedDevice:
    """Simple in-memory representation of an I2C peripheral."""

    address: int
    memory: bytearray

    def read(self, count: int) -> bytes:
        """Return up to ``count`` bytes from the simulated memory."""
        if count <= 0:
            return b""
        if not self.memory:
            return bytes([0] * count)
        result = bytes(self.memory[:count])
        if len(result) < count:
            padding = bytes([self.memory[-1]]) * (count - len(result))
            return result + padding
        return result

    def write(self, values: Iterable[int]) -> None:
        """Replace the internal memory with the provided values."""
        self.memory = bytearray(values)


class SimulatedI2CBackend(I2CBackend):
    """In-memory backend used until the physical interface becomes available."""

    def __init__(self) -> None:
        super().__init__()
        self._devices: Dict[int, SimulatedDevice] = {}
        self._latency_s: float = 0.0

    def configure(self, *, bus_id: str, configuration: Optional[dict] = None) -> None:
        self._bus_id = bus_id
        self._devices.clear()
        self._latency_s = 0.0

        configuration = configuration or {}
        latency_ms = configuration.get("simulated_latency_ms")
        if isinstance(latency_ms, (int, float)):
            self._latency_s = max(float(latency_ms) / 1000.0, 0.0)

        for entry in configuration.get("devices", []):
            try:
                address = int(entry["address"])
                data = entry.get("memory", [])
                device = SimulatedDevice(address=address, memory=bytearray(data))
                self._devices[address] = device
            except (KeyError, ValueError, TypeError) as exc:
                raise I2CBackendError(f"Invalid simulated device entry: {entry}") from exc

    def perform_transaction(
        self,
        address: int,
        write_data: Sequence[int],
        read_length: int,
        timeout: float,
    ) -> Tuple[bool, bytes, str]:
        write_buffer = list(write_data)
        device = self._devices.setdefault(address, SimulatedDevice(address, bytearray()))

        if self._latency_s:
            deadline = time.monotonic() + timeout if timeout > 0 else None
            time.sleep(self._latency_s)
            if deadline is not None and time.monotonic() > deadline:
                return False, b"", "timeout expired during simulated latency"

        if write_buffer:
            device.write(write_buffer)

        if read_length > 0:
            read_back = device.read(read_length)
        else:
            read_back = b""

        return True, read_back, ""
