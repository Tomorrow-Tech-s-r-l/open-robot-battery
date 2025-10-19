import pytest

from orb_i2c_support.backend_factory import create_backend
from orb_i2c_support.ez_pd_bcr_backend import EZPDBCRI2CBackend, SMBus
from orb_i2c_support.i2c_backend import I2CBackendError, SimulatedI2CBackend


def test_backend_factory_returns_simulation_backend() -> None:
    backend = create_backend("simulation")
    assert isinstance(backend, SimulatedI2CBackend)


def test_simulated_backend_read_write_cycle() -> None:
    backend = SimulatedI2CBackend()
    backend.configure(
        bus_id="i2c-1",
        configuration={"devices": [{"address": 0x30, "memory": [1, 2, 3]}]},
    )

    success, data, error = backend.perform_transaction(
        address=0x30,
        write_data=[9, 8],
        read_length=3,
        timeout=0.1,
    )

    assert success is True
    assert error == ""
    assert data == bytes([9, 8, 8])


def test_ez_pd_backend_requires_smbus_dependency() -> None:
    backend = EZPDBCRI2CBackend()
    if SMBus is None:
        with pytest.raises(I2CBackendError):
            backend.configure(bus_id="1")
    else:
        pytest.skip("SMBus available on this platform; backend requires hardware to test.")
