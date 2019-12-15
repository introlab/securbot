/**
 * @file i2c.hpp
 * @author Cedric Godin
 * @brief I2C driver
 * @version 0.1
 * @date 2019-10-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/i2c.h>

#include "defines.hpp"

/**
 * @brief I2C driver class.
 * Wrapper around ESP IDF API
 * Allow thread-safe read and write operation to both i2c bus
 */
class I2C
{
public:
    /**
     * @brief Get an i2c instance.
     * Return the i2c instance. Create it if itdoesn't exist.
     * @param bus_num i2c bus number
     * @return I2C* pointer to the i2c object
     */
    static I2C* instance(i2c_port_t bus_num);

    /**
     * @brief Initialize the i2c driver.
     * Initialize the i2c driver. Must be called before any read or write operation.
     * @return esp_err_t driver installation result. Check against ESP_OK
     */
    esp_err_t begin();

    /**
     * @brief Read byte(s) from an i2c slave.
     * Perform a full read from an i2c slave.
     * Thread-safe
     * @param address i2c slave address
     * @param data buffer where to store the data
     * @param size how many byte to read
     * @return esp_err_t read opeation success. Check against ESP_OK
     */
    esp_err_t read(uint8_t address, uint8_t data[], size_t size);

    /**
     * @brief Read byte(s) from a smbus slave.
     * Perform a full read from a smbus slave.
     * Preceeds the read with the command and repeated start condition.
     * Thread-safe
     * @param address smbus slave address
     * @param cmd command byte
     * @param data buffer where to store the data
     * @param size how many byte to read
     * @return esp_err_t operation success. Check against ESP_OK
     */
    esp_err_t smread(uint8_t address, uint8_t cmd, uint8_t data[], size_t size);

    /**
     * @brief Write byte(s) to an i2c slave.
     * Perform a full write to an i2c slave.
     * Thread-safe
     * @param address i2c slave address
     * @param data data buffer from which to retrieve the data
     * @param size how many byte to write
     * @return esp_err_t write operation success. Check against ESP_OK
     */
    esp_err_t write(uint8_t address, uint8_t data[], size_t size);

    /**
     * @brief Write byte(s) to a smbus slave.
     * Perform a full write opertion to a smbus slave.
     * Preceeds the write with the command.
     * Thread safe
     * @param address smbus slave address
     * @param cmd command byte
     * @param data buffer containing data to be sent
     * @param size how many byte to write
     * @return esp_err_t operation success. Check against ESP_OK
     */
    esp_err_t smwrite(uint8_t address, uint8_t cmd, uint8_t data[], size_t size);

private:
    /**
     * @brief Construct a new I2C object.
     * Construct a new I2C object which use the specified bus number
     * Is called automatically when an instance is requested
     * @param bus_num i2c bus number
     */
    I2C(i2c_port_t bus_num);

    /**
     * @brief Pointer to I2C object instance.
     * I2C object pointer array. There is one object with its own pointer for each bus.
     * All object methods are called on the same object.
     */
    static I2C* _instance[I2C_NUM_MAX];

    /**
     * @brief i2c bus number.
     * The i2c bus used by this instance of I2C bus
     */
    i2c_port_t _bus_num;

    /**
     * @brief i2c bus mutex.
     * i2c mutex to make read and write operations thread-safe
     */
    SemaphoreHandle_t _mutex;
};