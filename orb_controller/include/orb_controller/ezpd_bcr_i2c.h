#ifndef EZPD_BCR_I2C_H
#define EZPD_BCR_I2C_H

#include <string>
#include <cstdint>
#include <vector>

/**
 * @brief I2C interface driver for Infineon EZ-PD BCR (CYPD3177) chip
 * 
 * This class provides low-level I2C communication with the EZ-PD BCR chip
 * to read status registers and control the USB-C Power Delivery controller.
 */
class EZPD_BCR_I2C {
public:
    // EZ-PD BCR Register Addresses (based on CYPD3177 datasheet)
    // Note: These are typical register addresses - verify with actual datasheet
    static constexpr uint8_t REG_PD_STATUS = 0x00;
    static constexpr uint8_t REG_PORT_STATUS = 0x01;
    static constexpr uint8_t REG_VOLTAGE_STATUS = 0x02;
    static constexpr uint8_t REG_CURRENT_STATUS = 0x03;
    static constexpr uint8_t REG_PD_CONTROL = 0x10;
    static constexpr uint8_t REG_PD_CTRL_0 = 0x20;
    static constexpr uint8_t REG_PD_CTRL_1 = 0x21;
    static constexpr uint8_t REG_PD_CTRL_2 = 0x22;
    static constexpr uint8_t REG_TYPE_C_CTRL = 0x30;
    
    // I2C Address (default, can be changed via ADDR pins)
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x08; // Typical address, verify from datasheet
    
    /**
     * @brief Constructor
     * @param i2c_bus I2C bus number (e.g., 1 for /dev/i2c-1)
     * @param i2c_addr I2C device address (default: DEFAULT_I2C_ADDR)
     */
    EZPD_BCR_I2C(int i2c_bus = 1, uint8_t i2c_addr = DEFAULT_I2C_ADDR);
    
    /**
     * @brief Destructor
     */
    ~EZPD_BCR_I2C();
    
    /**
     * @brief Initialize I2C connection
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Check if I2C connection is valid
     * @return true if connected, false otherwise
     */
    bool isConnected() const;
    
    /**
     * @brief Read a single register
     * @param reg_addr Register address
     * @param value Output value
     * @return true if successful, false otherwise
     */
    bool readRegister(uint8_t reg_addr, uint8_t& value);
    
    /**
     * @brief Write a single register
     * @param reg_addr Register address
     * @param value Value to write
     * @return true if successful, false otherwise
     */
    bool writeRegister(uint8_t reg_addr, uint8_t value);
    
    /**
     * @brief Read multiple consecutive registers
     * @param reg_addr Starting register address
     * @param data Output buffer
     * @param length Number of bytes to read
     * @return true if successful, false otherwise
     */
    bool readRegisters(uint8_t reg_addr, uint8_t* data, size_t length);
    
    /**
     * @brief Write multiple consecutive registers
     * @param reg_addr Starting register address
     * @param data Input buffer
     * @param length Number of bytes to write
     * @return true if successful, false otherwise
     */
    bool writeRegisters(uint8_t reg_addr, const uint8_t* data, size_t length);
    
    /**
     * @brief Get PD (Power Delivery) status
     * @param pd_status Output PD status byte
     * @return true if successful, false otherwise
     */
    bool getPDStatus(uint8_t& pd_status);
    
    /**
     * @brief Get port status
     * @param port_status Output port status byte
     * @return true if successful, false otherwise
     */
    bool getPortStatus(uint8_t& port_status);
    
    /**
     * @brief Get voltage status (in mV)
     * @param voltage_mv Output voltage in millivolts
     * @return true if successful, false otherwise
     */
    bool getVoltageStatus(uint16_t& voltage_mv);
    
    /**
     * @brief Get current status (in mA)
     * @param current_ma Output current in milliamps
     * @return true if successful, false otherwise
     */
    bool getCurrentStatus(uint16_t& current_ma);
    
    /**
     * @brief Request specific PD role and profile
     * @param profile PD profile number (e.g., 1 = 5V, 2 = 9V, 3 = 12V, 4 = 15V, 5 = 20V)
     * @return true if successful, false otherwise
     */
    bool requestPDRole(uint8_t profile);
    
    /**
     * @brief Set output voltage and current limit
     * @param voltage_mv Desired voltage in millivolts
     * @param current_ma Desired current limit in milliamps
     * @return true if successful, false otherwise
     */
    bool setVoltageCurrent(uint16_t voltage_mv, uint16_t current_ma);
    
    /**
     * @brief Get error string for last operation
     * @return Error message string
     */
    std::string getLastError() const;

private:
    int i2c_bus_;
    int i2c_fd_;
    uint8_t i2c_addr_;
    std::string last_error_;
    bool connected_;
    
    /**
     * @brief Open I2C device
     * @return File descriptor if successful, -1 otherwise
     */
    int openI2CDevice();
    
    /**
     * @brief Close I2C device
     */
    void closeI2CDevice();
    
    /**
     * @brief Set error message
     * @param error Error message
     */
    void setError(const std::string& error);
};

#endif // EZPD_BCR_I2C_H