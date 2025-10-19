"""Helpers for constructing I2C backend instances."""

from __future__ import annotations

import importlib
from typing import Optional, Type

from .i2c_backend import I2CBackend, SimulatedI2CBackend
from .ez_pd_bcr_backend import EZPDBCRI2CBackend

_REGISTRY = {
    "simulation": SimulatedI2CBackend,
    "ez_pd_bcr": EZPDBCRI2CBackend,
}


def _load_from_string(path: str) -> Type[I2CBackend]:
    if ":" not in path:
        raise ValueError(
            "Backend plugin path must use the 'module:ClassName' format, "
            f"received '{path}'."
        )
    module_name, class_name = path.split(":", 1)
    module = importlib.import_module(module_name)
    backend_cls = getattr(module, class_name)
    if not issubclass(backend_cls, I2CBackend):
        raise TypeError(f"{backend_cls} is not a subclass of I2CBackend")
    return backend_cls


def create_backend(name: str, plugin_path: Optional[str] = None) -> I2CBackend:
    """Return a backend instance based on the requested name or plugin."""
    if plugin_path:
        backend_cls = _load_from_string(plugin_path)
        return backend_cls()

    backend_cls = _REGISTRY.get(name)
    if backend_cls is None:
        available = ", ".join(sorted(_REGISTRY))
        raise KeyError(f"Unknown backend '{name}'. Available: {available or 'none'}")

    return backend_cls()
