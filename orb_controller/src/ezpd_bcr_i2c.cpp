#include "orb_controller/ezpd_bcr_i2c.h"
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iostream>

EZPD_BCR_I2C::EZPD_BCR_I2C(int i2c_bus, uint8_t i2c_addr)
    : i2c_bus_(i2c_bus), i2c_fd_(-1), i2c_addr_(i2c_addr), connected_(false) {
}

EZPD_BCR_I2C::~EZPD_BCR_I2C() {
    closeI2CDevice();
}

bool EZPD_BCR_I2C::initialize() {
    if (connected_) {
        return true;
    }
    
    i2c_fd_ = openI2CDevice();
    if (i2c_fd_ < 0) {
        return false;
    }
    
    // Try to read from device to verify connection
    uint8_t test_reg = 0;
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        setError("Failed to set I2C slave address: " + std::string(strerror(errno)));
        closeI2CDevice();
        return false;
    }
    
    connected_ = true;
    return true;
}

bool EZPD_BCR_I2C::isConnected() const {
    return connected_ && i2c_fd_ >= 0;
}

int EZPD_BCR_I2C::openI2CDevice() {
    std::ostringstream device_path;
    device_path << "/dev/i2c-" << i2c_bus_;
    
    int fd = open(device_path.str().c_str(), O_RDWR);
    if (fd < 0) {
        setError("Failed to open I2C device " + device_path.str() + ": " + std::string(strerror(errno)));
        return -1;
    }
    
    if (ioctl(fd, I2C_SLAVE, i2c_addr_) < 0) {
        setError("Failed to set I2C slave address: " + std::string(strerror(errno)));
        close(fd);
        return -1;
    }
    
    return fd;
}

void EZPD_BCR_I2C::closeI2CDevice() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
    connected_ = false;
}

bool EZPD_BCR_I2C::readRegister(uint8_t reg_addr, uint8_t& value) {
    if (!isConnected()) {
        setError("I2C not connected");
        return false;
    }
    
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        setError("Failed to set I2C slave address: " + std::string(strerror(errno)));
        return false;
    }
    
    // Write register address
    if (write(i2c_fd_, &reg_addr, 1) != 1) {
        setError("Failed to write register address: " + std::string(strerror(errno)));
        return false;
    }
    
    // Read register value
    if (read(i2c_fd_, &value, 1) != 1) {
        setError("Failed to read register value: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

bool EZPD_BCR_I2C::writeRegister(uint8_t reg_addr, uint8_t value) {
    if (!isConnected()) {
        setError("I2C not connected");
        return false;
    }
    
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        setError("Failed to set I2C slave address: " + std::string(strerror(errno)));
        return false;
    }
    
    uint8_t buffer[2] = {reg_addr, value};
    if (write(i2c_fd_, buffer, 2) != 2) {
        setError("Failed to write register: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

bool EZPD_BCR_I2C::readRegisters(uint8_t reg_addr, uint8_t* data, size_t length) {
    if (!isConnected()) {
        setError("I2C not connected");
        return false;
    }
    
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        setError("Failed to set I2C slave address: " + std::string(strerror(errno)));
        return false;
    }
    
    // Write starting register address
    if (write(i2c_fd_, &reg_addr, 1) != 1) {
        setError("Failed to write register address: " + std::string(strerror(errno)));
        return false;
    }
    
    // Read multiple bytes
    if (read(i2c_fd_, data, length) != static_cast<ssize_t>(length)) {
        setError("Failed to read registers: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

bool EZPD_BCR_I2C::writeRegisters(uint8_t reg_addr, const uint8_t* data, size_t length) {
    if (!isConnected()) {
        setError("I2C not connected");
        return false;
    }
    
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        setError("Failed to set I2C slave address: " + std::string(strerror(errno)));
        return false;
    }
    
    uint8_t* buffer = new uint8_t[length + 1];
    buffer[0] = reg_addr;
    memcpy(buffer + 1, data, length);
    
    ssize_t written = write(i2c_fd_, buffer, length + 1);
    delete[] buffer;
    
    if (written != static_cast<ssize_t>(length + 1)) {
        setError("Failed to write registers: " + std::string(strerror(errno)));
        return false;
    }
    
    return true;
}

bool EZPD_BCR_I2C::getPDStatus(uint8_t& pd_status) {
    return readRegister(REG_PD_STATUS, pd_status);
}

bool EZPD_BCR_I2C::getPortStatus(uint8_t& port_status) {
    return readRegister(REG_PORT_STATUS, port_status);
}

bool EZPD_BCR_I2C::getVoltageStatus(uint16_t& voltage_mv) {
    uint8_t data[2];
    if (!readRegisters(REG_VOLTAGE_STATUS, data, 2)) {
        return false;
    }
    
    // Combine two bytes (adjust byte order based on datasheet)
    voltage_mv = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return true;
}

bool EZPD_BCR_I2C::getCurrentStatus(uint16_t& current_ma) {
    uint8_t data[2];
    if (!readRegisters(REG_CURRENT_STATUS, data, 2)) {
        return false;
    }
    
    // Combine two bytes (adjust byte order based on datasheet)
    current_ma = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return true;
}

bool EZPD_BCR_I2C::requestPDRole(uint8_t profile) {
    // Write profile to PD control register
    // Adjust based on actual CYPD3177 register map
    if (profile < 1 || profile > 5) {
        setError("Invalid PD profile. Must be 1-5");
        return false;
    }
    
    // Map profile to register value (adjust based on datasheet)
    uint8_t control_value = profile; // Simplified - check datasheet for actual mapping
    return writeRegister(REG_PD_CONTROL, control_value);
}

bool EZPD_BCR_I2C::setVoltageCurrent(uint16_t voltage_mv, uint16_t current_ma) {
    // This is a simplified implementation
    // The actual implementation depends on the CYPD3177 register map
    // You may need to write to multiple registers or use a command structure
    
    // Map voltage to PD profile
    uint8_t profile = 0;
    if (voltage_mv <= 5000) {
        profile = 1; // 5V
    } else if (voltage_mv <= 9000) {
        profile = 2; // 9V
    } else if (voltage_mv <= 12000) {
        profile = 3; // 12V
    } else if (voltage_mv <= 15000) {
        profile = 4; // 15V
    } else if (voltage_mv <= 20000) {
        profile = 5; // 20V
    } else {
        setError("Voltage out of range (max 20V)");
        return false;
    }
    
    if (!requestPDRole(profile)) {
        return false;
    }
    
    // TODO: Set current limit if supported by registers
    // This may require additional register writes based on the datasheet
    
    return true;
}

std::string EZPD_BCR_I2C::getLastError() const {
    return last_error_;
}

void EZPD_BCR_I2C::setError(const std::string& error) {
    last_error_ = error;
    std::cerr << "[EZPD_BCR_I2C] Error: " << error << std::endl;
}