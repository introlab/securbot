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
#define DEVICE_ID_CMD 0xFF      // access device id register

BQ24725A::BQ24725A(i2c_port_t i2c_bus)
{
    // bus instance
    _i2c = I2C::instance(i2c_bus);

    // chip config
    _chg_opt.fields.ACOK = 1;           // 1.3s ACOK pin deglitch time
    _chg_opt.fields.WATCHDOG = 0b11;    // 175s WATCHDOG timer
    _chg_opt.fields.BAT = 0;            // 59.19% of voltage regulation limit (~2.486V/cell)
    _chg_opt.fields.EMI_FREQ = 0;       // Reduce PWM frequency by 18%
    _chg_opt.fields.EMI_EN = 0;         // Disable PWM frequency adjustment
    _chg_opt.fields.IFAULT_HI = 0;      // disabled
    _chg_opt.fields.IFAULT_LOW = 0;     // 135mv
    _chg_opt.fields.LEARN = 0;          // Disable learn cycle
    _chg_opt.fields.IOUT = 1;           // 20x charge current
    _chg_opt.fields.ACOC = 1;           // 3.33x input current regulation limit
    _chg_opt.fields.CHARGE_INHIBIT = 1; // disable charge
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

esp_err_t BQ24725A::selectIOUT(uint8_t iout)
{
    // 1 bit max
    if (iout >> 1)
    {
        return ESP_ERR_INVALID_ARG;
    }

    _chg_opt.fields.IOUT = iout;

    return configure();
}

esp_err_t BQ24725A::getChipId(uint16_t &id)
{
    esp_err_t ret;

    union id_map
    {
        uint16_t id;
        uint8_t bytes[2];
    } map;

    ret =  _i2c->smread(CHIP_ADR, DEVICE_ID_CMD, map.bytes, 2);
    id = map.id;

    return ret;
}