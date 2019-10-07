/**
 * @file i2c.cpp
 * @author Cedric Godin
 * @brief I2C driver
 * @version 0.1
 * @date 2019-10-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "hal/i2c.hpp"

I2C* I2C::_instance[I2C_NUM_MAX] = {NULL};

I2C* I2C::instance(i2c_port_t bus_num)
{
    // Create an instance if it is the first call
    if (I2C::_instance[bus_num] == NULL)
    {
        I2C::_instance[bus_num] = new I2C(bus_num);
    }
    return I2C::_instance[bus_num];
}

esp_err_t I2C::begin()
{
    // Prepare configuration
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;

    if (_bus_num == I2C_NUM_0)  // We are initializing first driver
    {
        config.sda_io_num = (gpio_num_t)I2C_NUM_0_SDA_PIN;
        config.scl_io_num = (gpio_num_t)I2C_NUM_0_SCL_PIN;
    }
    else if (_bus_num == I2C_NUM_1) // We are initializing second driver
    {
        config.sda_io_num = (gpio_num_t)I2C_NUM_1_SDA_PIN;
        config.scl_io_num = (gpio_num_t)I2C_NUM_1_SCL_PIN;
    }

    // There is already pull ups on the board
    config.sda_pullup_en = GPIO_PULLUP_DISABLE;
    config.scl_pullup_en = GPIO_PULLUP_DISABLE;

    config.master.clk_speed = I2C_CLK_SPEED;
    i2c_param_config(_bus_num, &config);    // Fill config with remaining default

    // Actually start the driver using config
    return i2c_driver_install(_bus_num, config.mode, 0, 0, 0);
}

esp_err_t I2C::read(uint8_t address, uint8_t data[], size_t size)
{
    if (size == 0)  // No read required
    {
        return ESP_OK;
    }

    xSemaphoreTake(_mutex, portMAX_DELAY);

    // Create transaction handle
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Tell handle we will add steps
    i2c_master_start(cmd);

    // Write device address and read write bit
    i2c_master_write_byte(cmd, (address << 1 ) | I2C_MASTER_READ, true);
    if (size > 1) { // More then one byte to read
        i2c_master_read(cmd, data, size-1, I2C_MASTER_ACK);
    }

    // Read the last byte. No aknowledge on the last read.
    i2c_master_read_byte(cmd, data+size-1, I2C_MASTER_NACK);

    // Tell handle there is no more steps
    i2c_master_stop(cmd);

    // Perform the transaction then delete handle
    esp_err_t ret = i2c_master_cmd_begin(_bus_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t I2C::write(uint8_t address, uint8_t data[], size_t size)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    // Create transaction handle
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Tell handle we will add steps
    i2c_master_start(cmd);

    // Write device address and read write bit
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);

    // Write data and check for aknowledge bit
    i2c_master_write(cmd, data, size, true);

    // Tell handle there is no more steps
    i2c_master_stop(cmd);

    // Perform the transaction then delete handle
    esp_err_t ret = i2c_master_cmd_begin(_bus_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(_mutex);

    xSemaphoreGive(_mutex);

    return ret;
}

I2C::I2C(i2c_port_t bus_num)
{
    _bus_num = bus_num;
    _mutex = xSemaphoreCreateMutex();
}