/**
 * @file BQ76925.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Driver for the BQ76925 analog front end chip
 * @version 0.1
 * @date 2019-10-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "chip/BQ76925.hpp"

#define CHIP_ADR 1          // chip i2c address

#define STATUS_REG_ADR 0    // status register address
#define CELL_CTL_REG_ADR 1  // cell ctl register address
#define BAL_CTL_REG_ADR 2   // bal ctl register address
#define CONFIG_1_REG_ADR 3  // config 1 register address
#define CONFIG_2_REG_ADR 4  // config 2 register address
#define POWER_CTL_REG_ADR 5 // power ctl register address
#define CHIP_ID_REG_ADR 7   // chip id register address

#define VCOUT_VSS 0
#define VCOUT_VCn 1
#define VCOUT_05VREF 2
#define VCOUT_085VREF 3

BQ76925::BQ76925(i2c_port_t i2c_bus)
{
    _i2c = I2C::instance(i2c_bus);

    _config1.fields.I_THRESH = 0x7;
    _config1.fields.I_COMP_POL = 0;
    _config1.fields.I_AMP_CAL = 0;
    _config1.fields.I_GAIN = 0;

    _config2.fields.CRC_EN = 0;
    _config2.fields.REF_SEL = 1;

    _power.fields.REF_EN = 1;       // reference should always be active
    _power.fields.SLEEP_DIS = 1;    // we never want to sleep

    _power.fields.I_AMP_EN = 1;
    _power.fields.I_COMP_EN = 1;
    _power.fields.VC_AMP_EN = 1;
    _power.fields.VTB_EN = 1;
}

esp_err_t BQ76925::selectCell(uint8_t cell_number)
{
    cell_ctl_register_map map;

    map.fields.CELL_SEL = cell_number;  // Specify cell
    map.fields.VCOUT_SEL = VCOUT_VCn;   // Set VCOUT to output selected cell voltage

    return writeRegister(CELL_CTL_REG_ADR, map.bytes); // Write to chip
}

esp_err_t BQ76925::balanceCell(BQ76925::bal_ctl_register_map balance)
{
    return writeRegister(BAL_CTL_REG_ADR, balance.bytes);
}

esp_err_t BQ76925::getStatus(BQ76925::status_register_map &status)
{
    return readRegister(STATUS_REG_ADR, status.bytes);
}

esp_err_t BQ76925::setMonitorMode(uint8_t monitor, uint8_t charge)
{
    esp_err_t ret;

    // Set polarity
    _config1.fields.I_COMP_POL = charge;

    // Write polarity
    ret = writeRegister(CONFIG_1_REG_ADR, _config1.bytes);
    if (ret != ESP_OK)  // Polarity change failed we abort
    {
        return ret;
    }

    // Enable peripheral for monitoring
    _power.fields.I_AMP_EN = monitor;
    _power.fields.I_COMP_EN = monitor;
    _power.fields.VC_AMP_EN = monitor;
    _power.fields.VTB_EN = monitor;

    // Write to power register
    return writeRegister(POWER_CTL_REG_ADR, _power.bytes);
}

esp_err_t BQ76925::configure()
{
    esp_err_t ret;

    // write config 1
    ret = writeRegister(CONFIG_1_REG_ADR, _config1.bytes);
    if (ret != ESP_OK)  // this failed so we abort the rest
    {
        return ret;
    }

    // write config 2
    ret = writeRegister(CONFIG_2_REG_ADR, _config2.bytes);
    if (ret != ESP_OK)  // this failed so we abort the rest
    {
        return ret;
    }

    // write power
    ret = writeRegister(POWER_CTL_REG_ADR, _power.bytes);
    return ret; // we must return status wether success or failure
}

esp_err_t BQ76925::writeRegister(uint8_t address, uint8_t value[1])
{
    address_map adr;
    adr.fields.CHIP = CHIP_ADR;     // chip address
    adr.fields.REGISTER = address;  // register address

    // actual write operation
    return _i2c->write(adr.address, value, 1);
}

esp_err_t BQ76925::readRegister(uint8_t address, uint8_t value[1])
{
    address_map adr;
    adr.fields.CHIP = CHIP_ADR;     // chip address
    adr.fields.REGISTER = address;  // register address

    // actual read operation
    return _i2c->read(adr.address, value, 1);
}

esp_err_t BQ76925::readChipId(uint8_t &id)
{
    address_map adr;
    adr.fields.CHIP = CHIP_ADR;                // chip address
    adr.fields.REGISTER = CHIP_ID_REG_ADR;  // register address

    // actual read operation
    return _i2c->read(adr.address, &id, 1);
}