# EZ-PD BCR (CYPD3177) Register Map Documentation

## Overview
This document describes the I2C register map for the Infineon EZ-PD BCR (CYPD3177) USB Type-C Port Controller.

**NOTE:** The register addresses and bit definitions below are based on typical EZ-PD BCR implementations. 
You should verify these with the actual CYPD3177 datasheet for your specific chip revision.

## I2C Interface Details

- **Default I2C Address:** 0x08 (can be modified via ADDR pins)
- **I2C Speed:** Standard mode (100 kHz) or Fast mode (400 kHz)
- **Data Format:** Big-endian for multi-byte registers

## Register Map

### Device Information Registers (0x00-0x0F)

| Address | Register Name | Access | Description |
|---------|--------------|--------|-------------|
| 0x00 | DEVICE_ID | R | Device identification |
| 0x01 | DEVICE_MODE | R/W | Device operation mode |
| 0x02 | INTERRUPT_CLEAR | W | Clear interrupt flags |
| 0x03 | INTERRUPT_MASK | R/W | Interrupt enable mask |
| 0x04 | PD_STATUS | R | Power Delivery status |
| 0x05 | TYPE_C_STATUS | R | USB Type-C connection status |
| 0x06-0x07 | BUS_VOLTAGE | R | VBUS voltage (16-bit) |
| 0x08-0x09 | CURRENT_PDO | R | Current Power Data Object |
| 0x0A-0x0B | VOLTAGE_STATUS | R | Negotiated voltage (16-bit) |
| 0x0C-0x0D | CURRENT_STATUS | R | Negotiated current (16-bit) |
| 0x0E | POWER_STATUS | R | Power status summary |
| 0x0F | FAULT_STATUS | R | Fault status flags |

### Control Registers (0x10-0x1F)

| Address | Register Name | Access | Description |
|---------|--------------|--------|-------------|
| 0x10 | PD_CONTROL | R/W | PD control and profile selection |
| 0x11 | PORT_CONTROL | R/W | Port enable/disable control |
| 0x12 | VBUS_CONTROL | R/W | VBUS control settings |
| 0x13 | CURRENT_LIMIT | R/W | Current limit configuration |
| 0x14-0x15 | VOLTAGE_REQUEST | R/W | Requested voltage (16-bit) |
| 0x16-0x17 | CURRENT_REQUEST | R/W | Requested current (16-bit) |
| 0x18 | SINK_CONTROL | R/W | Sink operation control |
| 0x19 | SOURCE_CONTROL | R/W | Source operation control |
| 0x1A | CABLE_DETECT | R | Cable detection status |
| 0x1B-0x1F | RESERVED | - | Reserved for future use |

### PDO Configuration Registers (0x20-0x3F)

| Address | Register Name | Access | Description |
|---------|--------------|--------|-------------|
| 0x20-0x23 | SINK_PDO_1 | R/W | Sink PDO #1 (5V profile) |
| 0x24-0x27 | SINK_PDO_2 | R/W | Sink PDO #2 (9V profile) |
| 0x28-0x2B | SINK_PDO_3 | R/W | Sink PDO #3 (12V profile) |
| 0x2C-0x2F | SINK_PDO_4 | R/W | Sink PDO #4 (15V profile) |
| 0x30-0x33 | SINK_PDO_5 | R/W | Sink PDO #5 (20V profile) |
| 0x34-0x37 | SINK_PDO_6 | R/W | Sink PDO #6 (PPS profile) |
| 0x38-0x3F | RESERVED | - | Reserved |

## Register Bit Definitions

### PD_STATUS Register (0x04)

| Bit | Name | Description |
|-----|------|-------------|
| 7 | PD_READY | PD negotiation complete |
| 6 | CONTRACT_EXIST | Valid PD contract exists |
| 5 | EXPLICIT_CONTRACT | Explicit contract negotiated |
| 4 | DATA_ROLE | 0=UFP, 1=DFP |
| 3 | POWER_ROLE | 0=Sink, 1=Source |
| 2 | VCONN_SOURCE | Device is VCONN source |
| 1 | PD_ENABLED | PD messaging enabled |
| 0 | PD_CONNECTED | PD capable device connected |

### TYPE_C_STATUS Register (0x05)

| Bit | Name | Description |
|-----|------|-------------|
| 7:6 | ATTACH_STATE | 00=Unattached, 01=AttachWait, 10=Attached, 11=Debug |
| 5 | ORIENTATION | 0=CC1, 1=CC2 |
| 4 | VBUS_PRESENT | VBUS voltage detected |
| 3 | VBUS_VALID | VBUS voltage in valid range |
| 2 | VCONN_PRESENT | VCONN supplied to cable |
| 1 | CC_STATUS | CC line status |
| 0 | CONNECTED | Cable connected |

### PD_CONTROL Register (0x10)

| Bit | Name | Description |
|-----|------|-------------|
| 7:5 | RESERVED | Reserved |
| 4:0 | PROFILE_SELECT | Select PDO profile (1-5) |

### FAULT_STATUS Register (0x0F)

| Bit | Name | Description |
|-----|------|-------------|
| 7 | OCP | Over-current protection triggered |
| 6 | OVP | Over-voltage protection triggered |
| 5 | OTP | Over-temperature protection triggered |
| 4 | VBUS_SHORT | VBUS short circuit detected |
| 3 | VCONN_OCP | VCONN over-current |
| 2 | HARD_RESET | Hard reset occurred |
| 1 | SOFT_RESET | Soft reset occurred |
| 0 | GENERAL_FAULT | General fault condition |

## Voltage and Current Scaling

### Voltage Registers
- **Unit:** 50mV per LSB
- **Range:** 0-20V (0x0000-0x0190)
- **Example:** 0x64 = 100 * 50mV = 5000mV = 5V

### Current Registers
- **Unit:** 10mA per LSB for standard current
- **Unit:** 50mA per LSB for high current (>3A)
- **Range:** 0-5A (0x0000-0x01F4)
- **Example:** 0xC8 = 200 * 10mA = 2000mA = 2A

## Standard PDO Profiles

| Profile | Voltage | Max Current | Power |
|---------|---------|-------------|-------|
| 1 | 5V | 3A | 15W |
| 2 | 9V | 3A | 27W |
| 3 | 12V | 3A | 36W |
| 4 | 15V | 3A | 45W |
| 5 | 20V | 5A | 100W |

## I2C Transaction Examples

### Read Voltage Status
```
Write: [0x0A]       // Register address
Read:  [MSB] [LSB]  // 16-bit voltage value
```

### Set PD Profile
```
Write: [0x10] [0x03]  // Write 0x03 to PD_CONTROL for 12V profile
```

### Read PD Status
```
Write: [0x04]       // Register address
Read:  [Status]     // 8-bit status value
```

## Notes

1. Always verify register addresses and bit definitions with the actual CYPD3177 datasheet
2. Some registers may require specific sequences or timing for proper operation
3. Writing to read-only registers will be ignored
4. Some control registers may require device to be in specific state
5. Multi-byte registers typically use big-endian format, but verify with datasheet

## Safety Considerations

- Always check voltage and current capabilities before requesting higher profiles
- Monitor fault status register regularly
- Implement proper error handling for I2C communication failures
- Use appropriate delays between configuration changes
- Verify cable and device capabilities before negotiating high power profiles