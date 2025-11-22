# ORB Controller ROS Package

## Overview

The `orb_controller` package provides a ROS interface for controlling the Infineon EZ-PD BCR (CYPD3177) USB Type-C Port Controller via I2C. This package enables monitoring and control of USB Power Delivery (PD) negotiation for robotic battery management systems.

## Features

- **I2C Communication:** Direct hardware interface to EZ-PD BCR chip
- **Power Monitoring:** Real-time voltage, current, and power status
- **PD Control:** Request specific voltage profiles (5V, 9V, 12V, 15V, 20V)
- **Status Publishing:** Continuous monitoring via ROS topics
- **Service Interface:** On-demand control via ROS services
- **Safety Features:** Voltage/current limits and fault monitoring

## Hardware Requirements

- Infineon EZ-PD BCR (CYPD3177) chip
- I2C connection to host system
- USB Type-C connector with PD support
- Appropriate power source/sink capabilities

## Dependencies

### System Dependencies
```bash
sudo apt-get install -y \
    i2c-tools \
    libi2c-dev \
    python3-smbus
```

### ROS Dependencies
- ROS Noetic (or compatible version)
- roscpp
- std_msgs
- message_generation
- message_runtime

## Installation

1. Clone the repository to your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone <repository-url>
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Configure I2C permissions (if needed):
```bash
sudo usermod -a -G i2c $USER
# Log out and back in for changes to take effect
```

## Usage

### Launch the Controller Node

Basic launch:
```bash
roslaunch orb_controller orb_controller.launch
```

With custom parameters:
```bash
roslaunch orb_controller orb_controller.launch i2c_bus:=1 i2c_address:=0x08 publish_rate:=10.0
```

### Published Topics

- `/orb/power_status` ([orb_controller/PowerStatus])
  - Real-time power measurements
  - Published at configured rate (default: 10 Hz)

- `/orb/pd_status` ([orb_controller/PDStatus])
  - USB-C PD negotiation status
  - Published at configured rate (default: 10 Hz)

### Services

- `/orb_controller/get_status` ([orb_controller/GetStatus])
  - Get current voltage, current, and PD status

- `/orb_controller/set_voltage_current` ([orb_controller/SetVoltageCurrent])
  - Request specific voltage and current limits

- `/orb_controller/request_pd_role` ([orb_controller/RequestPDRole])
  - Request specific PD profile (1-5)

### Parameters

Configuration parameters can be set in `config/orb_controller_params.yaml`:

- `i2c_bus`: I2C bus number (default: 1)
- `i2c_address`: I2C device address (default: 0x08)
- `publish_rate`: Status publishing rate in Hz (default: 10.0)
- `max_voltage_mv`: Maximum allowed voltage in mV (default: 20000)
- `max_current_ma`: Maximum allowed current in mA (default: 5000)

## Testing

### ROS Test Script
```bash
rosrun orb_controller test_orb_controller.py
```

Monitor mode:
```bash
rosrun orb_controller test_orb_controller.py monitor
```

### Direct I2C Test (bypasses ROS)
```bash
# Basic test
python3 scripts/test_i2c_direct.py

# Custom bus and address
python3 scripts/test_i2c_direct.py --bus 1 --addr 0x08

# Scan all registers
python3 scripts/test_i2c_direct.py --scan

# Monitor for 30 seconds
python3 scripts/test_i2c_direct.py --monitor 30
```

## Examples

### Request 12V Profile
```bash
rosservice call /orb_controller/request_pd_role "profile: 3"
```

### Set Specific Voltage and Current
```bash
rosservice call /orb_controller/set_voltage_current "voltage_mv: 9000
current_ma: 2000"
```

### Get Current Status
```bash
rosservice call /orb_controller/get_status
```

### Monitor Power Status
```bash
rostopic echo /orb/power_status
```

## Message Definitions

### PowerStatus.msg
```
std_msgs/Header header
uint16 voltage_mv        # Voltage in millivolts
uint16 current_ma        # Current in milliamps
uint32 power_mw          # Power in milliwatts
bool pd_active           # PD negotiation active
uint8 pd_profile         # Current PD profile (1-5)
```

### PDStatus.msg
```
std_msgs/Header header
uint8 pd_status_raw      # Raw PD status byte
uint8 port_status_raw    # Raw port status byte
bool type_c_connected    # USB-C cable connected
bool pd_contract_active  # PD contract established
uint8 current_profile    # Active profile number
uint16 negotiated_voltage_mv  # Negotiated voltage
uint16 negotiated_current_ma  # Negotiated current
```

## Troubleshooting

### I2C Connection Issues
```bash
# Check if I2C device is detected
i2cdetect -y 1

# Verify I2C permissions
ls -l /dev/i2c-*

# Test I2C communication
i2cget -y 1 0x08 0x00
```

### Node Not Starting
- Check ROS master is running: `roscore`
- Verify I2C device permissions
- Check correct I2C bus and address in parameters
- Review log output: `rosrun orb_controller orb_controller_node`

### No Power Delivery
- Verify USB-C cable supports PD
- Check power source capabilities
- Monitor PD status: `rostopic echo /orb/pd_status`
- Review fault status in logs

## Register Documentation

See [docs/REGISTER_MAP.md](docs/REGISTER_MAP.md) for detailed register descriptions and bit definitions.

**Important:** The register addresses in this package are based on typical EZ-PD BCR implementations. You must verify these with your specific CYPD3177 datasheet and update the following files accordingly:
- `include/orb_controller/ezpd_bcr_i2c.h` - Register address definitions
- `src/ezpd_bcr_i2c.cpp` - Register access implementations
- `src/orb_controller_node.cpp` - Status bit interpretations

## Safety Notes

⚠️ **WARNING:** Improper voltage/current settings can damage connected devices.

- Always verify device voltage tolerances before changing profiles
- Monitor fault status regularly
- Implement appropriate error handling in production code
- Test with current-limited supplies during development
- Never exceed maximum ratings specified in datasheets

## License

This package is released under the CC-BY-NC-SA license as specified in the package.xml file.

## Support

For issues and questions, please contact: sviluppo@tomorrowtech.it