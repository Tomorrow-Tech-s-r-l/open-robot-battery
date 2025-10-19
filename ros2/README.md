# ORB ROS 2 Workspace

This directory contains the ROS 2 workspace that hosts software support for the Open Robot Battery (ORB).

## Packages

- `orb_i2c_interfaces`: Service definitions shared across the I2C stack.
- `orb_i2c_support`: Python implementation of the I2C management node with a pluggable backend, including both a simulation layer and a driver for the Infineon EZ-PD BCR USB-C sink controller.

## Building

```bash
cd ros2
colcon build
source install/setup.bash
```

> If you only want to build the packages added here, append `--packages-up-to orb_i2c_support`.

## Running the I2C manager

```bash
ros2 launch orb_i2c_support i2c_manager.launch.py
```

By default the node runs in simulation mode. All transactions are served from an in-memory backend that mimics a trivial I2C device.

### Useful Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `backend` | `simulation` | Backend name registered in `orb_i2c_support.backend_factory`. |
| `bus_id` | `i2c-1` | Logical identifier for the I2C bus; forwarded to the backend. |
| `backend_plugin` | *empty* | Optional `module:ClassName` string to load a custom backend. |
| `backend_configuration` | `{}` | JSON configuration passed to the backend. |
| `status_interval_sec` | `1.0` | Interval in seconds for status publication on `orb_i2c/status`. |

Example launch overriding the backend configuration:

```bash
ros2 launch orb_i2c_support i2c_manager.launch.py \
  backend_configuration:='{"devices": [{"address": 16, "memory": [1, 2, 3]}]}'
```

### Calling the service

Once the node is running:

```bash
ros2 service call /i2c/transaction orb_i2c_interfaces/srv/I2CTransaction \
  "{address: 16, write_data: [170, 187], read_length: 4, timeout: 0.1}"
```

The response will contain the simulated read buffer. When the real hardware backend becomes available, drop it in as a plugin and reuse the same service interface.

### Using the EZ-PD BCR backend

The Infineon EZ-PD BCR controller exposes its register map over I2C. The provided backend wraps the Linux `smbus2` stack to perform combined write/read transactions and optional power-on configuration sequences.

```bash
ros2 launch orb_i2c_support i2c_manager.launch.py \
  backend:=ez_pd_bcr \
  bus_id:=/dev/i2c-1 \
  backend_configuration:='{
    "default_address": 8,
    "init_sequence": [
      {"write_data": [0x00, 0x08]},  # Example: pull default register map
      {"write_data": [0x06, 0x01], "delay_ms": 5}
    ]
  }'
```

- `default_address` (optional) overrides the default 7-bit address (`0x08` in the datasheet).
- `init_sequence` (optional) allows writing configuration registers at launch. Each entry contains `write_data` (first byte is treated as the register offset) and an optional `delay_ms` for settling time between writes.

Typical tasks enabled by this backend:
- Read the status registers that report VBUS negotiation state, CC pin status, and fault indicators.
- Update configuration registers to adjust PDO selection, discharge behavior, or the sink power profile.
- Trigger firmware utilities such as soft resets or load complete configuration templates as documented in the datasheet.

Example register read (fetch the Status Register 1 at offset `0x05`):

```bash
ros2 service call /i2c/transaction orb_i2c_interfaces/srv/I2CTransaction \
  "{address: 8, write_data: [0x05], read_length: 1, timeout: 0.1}"
```

The first byte in `write_data` is interpreted by the controller as the register pointer; the returned byte shows the live status flags defined in section 6.3 of the EZ-PD BCR datasheet.

### Standalone Raspberry Pi smoke test

If you want to validate the hardware before running ROS:

```bash
pip install smbus2
python3 ros2/scripts/raspi_ez_pd_demo.py
```

The script opens `/dev/i2c-1`, reads Status Register 1 once per second, and prints the bit field as a binary string. A changing value indicates the Pi can reach the EZ-PD BCR device over the I²C bus.

## Tests

Run the unit tests with:

```bash
colcon test --packages-select orb_i2c_support
```

> The test suite depends on `pytest`. Install it in your environment (e.g., `pip install pytest`) if it is not already available.
