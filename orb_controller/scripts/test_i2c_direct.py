#!/usr/bin/env python3
"""
Direct I2C test script for EZ-PD BCR (CYPD3177) chip
This script bypasses ROS and directly communicates with the chip for testing
"""

import smbus2
import time
import sys
import argparse

class EZPD_BCR_I2C_Test:
    # Register addresses based on CYPD3177 datasheet
    # Note: These addresses should be updated based on the actual datasheet
    REG_DEVICE_ID = 0x00        # Device ID register
    REG_DEVICE_MODE = 0x01      # Device mode register
    REG_INTERRUPT_CLEAR = 0x02  # Interrupt clear register
    REG_INTERRUPT_MASK = 0x03   # Interrupt mask register
    REG_PD_STATUS = 0x04        # PD status register
    REG_TYPE_C_STATUS = 0x05    # Type-C status register
    REG_BUS_VOLTAGE = 0x06      # Bus voltage register (2 bytes)
    REG_CURRENT_PDO = 0x08      # Current PDO register
    REG_VOLTAGE_STATUS = 0x0A   # Voltage status (2 bytes)
    REG_CURRENT_STATUS = 0x0C   # Current status (2 bytes)
    REG_POWER_STATUS = 0x0E     # Power status
    REG_FAULT_STATUS = 0x0F     # Fault status register
    
    def __init__(self, bus_number=1, device_address=0x08):
        """Initialize I2C connection"""
        self.bus_number = bus_number
        self.device_address = device_address
        
        try:
            self.bus = smbus2.SMBus(bus_number)
            print(f"Connected to I2C bus {bus_number}, device 0x{device_address:02X}")
        except Exception as e:
            print(f"Failed to connect to I2C bus: {e}")
            sys.exit(1)
    
    def read_register(self, register):
        """Read a single byte register"""
        try:
            value = self.bus.read_byte_data(self.device_address, register)
            return value
        except Exception as e:
            print(f"Error reading register 0x{register:02X}: {e}")
            return None
    
    def write_register(self, register, value):
        """Write a single byte to a register"""
        try:
            self.bus.write_byte_data(self.device_address, register, value)
            return True
        except Exception as e:
            print(f"Error writing register 0x{register:02X}: {e}")
            return False
    
    def read_word_register(self, register):
        """Read a 2-byte register"""
        try:
            # Read 2 bytes starting from the register
            data = self.bus.read_i2c_block_data(self.device_address, register, 2)
            # Combine bytes (little-endian or big-endian depends on chip)
            value = (data[1] << 8) | data[0]  # Assuming little-endian
            return value
        except Exception as e:
            print(f"Error reading word register 0x{register:02X}: {e}")
            return None
    
    def scan_registers(self):
        """Scan and display all readable registers"""
        print("\n=== Register Scan ===")
        print("Addr | Value (Hex) | Value (Dec) | Value (Bin)")
        print("-" * 50)
        
        for reg in range(0x00, 0x20):  # Scan first 32 registers
            value = self.read_register(reg)
            if value is not None:
                print(f"0x{reg:02X} | 0x{value:02X}       | {value:3d}        | 0b{value:08b}")
            else:
                print(f"0x{reg:02X} | --          | --          | --")
            time.sleep(0.01)  # Small delay between reads
    
    def test_device_id(self):
        """Read and display device ID"""
        print("\n=== Device Identification ===")
        device_id = self.read_register(self.REG_DEVICE_ID)
        if device_id is not None:
            print(f"Device ID: 0x{device_id:02X}")
            # CYPD3177 should return a specific ID
            if device_id == 0x77:  # Example - check datasheet for actual value
                print("Device identified as CYPD3177")
            else:
                print("Unknown device ID")
    
    def test_pd_status(self):
        """Read and display PD status"""
        print("\n=== Power Delivery Status ===")
        
        pd_status = self.read_register(self.REG_PD_STATUS)
        if pd_status is not None:
            print(f"PD Status Register: 0x{pd_status:02X}")
            print(f"  - PD Contract Active: {bool(pd_status & 0x01)}")
            print(f"  - PD Enabled: {bool(pd_status & 0x02)}")
            print(f"  - Data Role: {'DFP' if pd_status & 0x04 else 'UFP'}")
            print(f"  - Power Role: {'Source' if pd_status & 0x08 else 'Sink'}")
        
        type_c_status = self.read_register(self.REG_TYPE_C_STATUS)
        if type_c_status is not None:
            print(f"\nType-C Status Register: 0x{type_c_status:02X}")
            print(f"  - Cable Connected: {bool(type_c_status & 0x01)}")
            print(f"  - Orientation: {'CC2' if type_c_status & 0x02 else 'CC1'}")
            print(f"  - VBUS Present: {bool(type_c_status & 0x04)}")
    
    def test_voltage_current(self):
        """Read and display voltage and current"""
        print("\n=== Voltage and Current Status ===")
        
        voltage = self.read_word_register(self.REG_VOLTAGE_STATUS)
        if voltage is not None:
            # Assuming voltage is in 50mV or 100mV units - check datasheet
            voltage_mv = voltage * 50  # Example scaling
            print(f"Voltage: {voltage_mv} mV ({voltage_mv/1000:.2f} V)")
        
        current = self.read_word_register(self.REG_CURRENT_STATUS)
        if current is not None:
            # Assuming current is in 10mA or 50mA units - check datasheet
            current_ma = current * 10  # Example scaling
            print(f"Current: {current_ma} mA ({current_ma/1000:.2f} A)")
            
            if voltage is not None:
                power_w = (voltage_mv * current_ma) / 1000000
                print(f"Power: {power_w:.2f} W")
    
    def monitor_status(self, duration=10):
        """Monitor status for a specified duration"""
        print(f"\n=== Monitoring Status for {duration} seconds ===")
        print("Time | Voltage | Current | PD Status | Type-C Status")
        print("-" * 60)
        
        start_time = time.time()
        while time.time() - start_time < duration:
            voltage = self.read_word_register(self.REG_VOLTAGE_STATUS)
            current = self.read_word_register(self.REG_CURRENT_STATUS)
            pd_status = self.read_register(self.REG_PD_STATUS)
            type_c_status = self.read_register(self.REG_TYPE_C_STATUS)
            
            timestamp = time.time() - start_time
            
            voltage_str = f"{voltage*50:5d}mV" if voltage is not None else "----mV"
            current_str = f"{current*10:5d}mA" if current is not None else "----mA"
            pd_str = f"0x{pd_status:02X}" if pd_status is not None else "----"
            tc_str = f"0x{type_c_status:02X}" if type_c_status is not None else "----"
            
            print(f"{timestamp:4.1f}s | {voltage_str} | {current_str} | {pd_str}     | {tc_str}")
            
            time.sleep(1)
    
    def run_full_test(self):
        """Run all tests"""
        print("="*60)
        print("EZ-PD BCR (CYPD3177) I2C Test")
        print(f"I2C Bus: {self.bus_number}, Address: 0x{self.device_address:02X}")
        print("="*60)
        
        self.test_device_id()
        self.test_pd_status()
        self.test_voltage_current()
        self.scan_registers()
        self.monitor_status(5)
        
        print("\n" + "="*60)
        print("Test Complete")
        print("="*60)

def main():
    parser = argparse.ArgumentParser(description='Test EZ-PD BCR chip via I2C')
    parser.add_argument('--bus', type=int, default=1, help='I2C bus number (default: 1)')
    parser.add_argument('--addr', type=lambda x: int(x,0), default=0x08, 
                        help='I2C device address (default: 0x08)')
    parser.add_argument('--scan', action='store_true', help='Only scan registers')
    parser.add_argument('--monitor', type=int, metavar='SECONDS', 
                        help='Monitor status for N seconds')
    
    args = parser.parse_args()
    
    tester = EZPD_BCR_I2C_Test(args.bus, args.addr)
    
    if args.scan:
        tester.scan_registers()
    elif args.monitor:
        tester.monitor_status(args.monitor)
    else:
        tester.run_full_test()

if __name__ == '__main__':
    main()