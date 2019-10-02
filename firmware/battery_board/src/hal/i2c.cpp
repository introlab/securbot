#include "hal/i2c.hpp"

I2C* I2C::_instance[I2C_NUM_MAX] = {NULL};

I2C* I2C::instance(i2c_port_t bus_num)
{
    if (I2C::_instance[bus_num] == NULL)
    {
        I2C::_instance[bus_num] = new I2C(bus_num);
    }
    return I2C::_instance[bus_num];
}

esp_err_t I2C::begin()
{
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;

    if (_bus_num == I2C_NUM_0)
    {
        config.sda_io_num = (gpio_num_t)I2C_NUM_0_SDA_PIN;
        config.scl_io_num = (gpio_num_t)I2C_NUM_0_SCL_PIN;
    }
    else if (_bus_num == I2C_NUM_1)
    {
        config.sda_io_num = (gpio_num_t)I2C_NUM_1_SDA_PIN;
        config.scl_io_num = (gpio_num_t)I2C_NUM_1_SCL_PIN;
    }
    config.sda_pullup_en = GPIO_PULLUP_DISABLE;
    config.scl_pullup_en = GPIO_PULLUP_DISABLE;

    config.master.clk_speed = I2C_CLK_SPEED;
    i2c_param_config(_bus_num, &config);

    return i2c_driver_install(_bus_num, config.mode, 0, 0, 0);
}

esp_err_t I2C::read(uint8_t address, uint8_t data[], size_t size, bool send_register_address, uint8_t register_address)
{
    if (size == 0)
    {
        return ESP_OK;
    }

    xSemaphoreTake(_mutex, portMAX_DELAY);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (address << 1 ) | I2C_MASTER_READ, true);
    if (send_register_address)
    {
        i2c_master_write_byte(cmd, register_address, true);
    }
    if (size > 1) {
        i2c_master_read(cmd, data, size-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data+size-1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_bus_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t I2C::write(uint8_t address, uint8_t data[], size_t size, bool send_register_address, uint8_t register_address)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    if (send_register_address)
    {
        i2c_master_write_byte(cmd, register_address, true);
    }
    i2c_master_write(cmd, data, size, true);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_bus_num, cmd, 1000 / portTICK_RATE_MS);

    xSemaphoreGive(_mutex);

    return ret;
}

I2C::I2C(i2c_port_t bus_num)
{
    _bus_num = bus_num;
    _mutex = xSemaphoreCreateMutex();
}