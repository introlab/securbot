/**
 * @file BQ24725A.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Driver for the BQ24725A chip
 * @version 0.1
 * @date 2019-10-09
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "chip/BQ24725A.hpp"

#define CHIP_ADR 0b0001001      // Base chip address

#define CHARGE_OPTION_CMD 0x12  // access charge option register
#define INPUT_CURRENT_CMD 0x3F  // access input current register
#define CHARGE_VOLTAGE_CMD 0x15 // access charge voltage register
#define CHARGE_CURRENT_CMD 0x14 // access charge current register

BQ24725A::BQ24725A(i2c_port_t i2c_bus)
{
    // bus instance
    _i2c = I2C::instance(i2c_bus);

    // chip config
    _chg_opt.fields.CHARGE_INHIBIT = 1;
}

esp_err_t BQ24725A::configure()
{
    return _i2c->smwrite(CHIP_ADR, CHARGE_OPTION_CMD, _chg_opt.bytes, 2);
}

esp_err_t BQ24725A::setChargeInhibit(uint8_t inhibit)
{
    // 1 bit max
    if (inhibit >> 1)
    {
        return ESP_ERR_INVALID_ARG;
    }

    _chg_opt.fields.CHARGE_INHIBIT = inhibit;
    return configure();
}

esp_err_t BQ24725A::setChargeVoltage(uint16_t voltage)
{
    // 11 bits max
    if (voltage >> 11)
    {
        return ESP_ERR_INVALID_ARG;
    }

    charge_voltage_register_map map;
    map.fields.DACV = voltage;

    return _i2c->smwrite(CHIP_ADR, CHARGE_VOLTAGE_CMD, map.bytes, 2);
}

esp_err_t BQ24725A::setChargeCurrent(uint8_t current)
{
    // 7 bits max
    if (current >> 7)
    {
        return ESP_ERR_INVALID_ARG;
    }

    charge_current_register_map map;
    map.fields.DACICHG = current;

    return _i2c->smwrite(CHIP_ADR, CHARGE_CURRENT_CMD, map.bytes, 2);
}

esp_err_t BQ24725A::setInputCurrent(uint8_t current)
{
    // 6 bits max
    if (current >> 6)
    {
        return ESP_ERR_INVALID_ARG;
    }

    input_current_register_map map;
    map.fields.DACIIN = current;

    return _i2c->smwrite(CHIP_ADR, INPUT_CURRENT_CMD, map.bytes, 2);
}