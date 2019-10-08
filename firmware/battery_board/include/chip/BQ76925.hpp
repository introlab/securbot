/**
 * @file BQ76925.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Driver for the BQ76925 analog front end chip
 * @version 0.1
 * @date 2019-10-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include "hal/i2c.hpp"

/**
 * @brief Interface class for the BQ76925 chip.
 * Allow configuration and usage of a single BQ76925 chip
 */
class BQ76925
{
public:
    /**
     * @brief address mapping.
     * chip and target register address mapping
     */
    union address_map
    {
        /**
         * @brief address fields.
         * address fields for direct value access
         */
        struct
        {
            /**
             * @brief register address.
             * specifiy which register to access
             */
            uint8_t REGISTER:5;
            /**
             * @brief chip address.
             * prefix the register so that the chip respond
             */
            uint8_t CHIP:2;
            /**
             * @brief reserved.
             * discarded when i2c bitshift left to add READ/WRITE bit
             */
            uint8_t RESERVED:1;
        } fields;
        /**
         * @brief complete address.
         * complete chip address pointing to the required register
         */
        uint8_t address;
    };

    /**
     * @brief status register mapping.
     * register reflecting current chip status
     */
    union status_register_map
    {
        /**
         * @brief status register bitfields.
         * status register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief Power on reset flag.
             * Set on each power-up and wake-up from sleepé May be cleared by writing with 0
             */
            uint8_t POR:1;
            /**
             * @brief CRC error status.
             * Updated on every i2c write packet when CRC_EN=1. 1 = CRC error
             */
            uint8_t CRC_ERR:1;
            /**
             * @brief Over-current alert.
             * Reflects the state of the over-current comparator. 1 = over-current
             */
            uint8_t ALERT:1;
            /**
             * @brief Reserved bits.
             * Always 0
             */
            uint8_t RESERVED:5;
        } fields;
        /**
         * @brief status register value as byte.
         * status register value as byte for chip communication
         */
        uint8_t bytes[1];
    };

    /**
     * @brief cell ctl register map.
     * Register where you can express which output you want on VCOUT
     */
    union cell_ctl_register_map
    {
        /**
         * @brief cell ctl register bitfields.
         * cell ctl register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief Cell selection.
             * Select the cell voltage to ouput on VCOUT
             */
            uint8_t CELL_SEL:3;
            /**
             * @brief reserved bit.
             * always 0
             */
            uint8_t RESERVED0:1;
            /**
             * @brief VCOUT pin function.
             * Select the function of the VCOUT pin
             */
            uint8_t VCOUT_SEL:2;
            /**
             * @brief reserved bits.
             * always 0
             */
            uint8_t RESERVED1:2;
        } fields;
        /**
         * @brief cell ctl register value as byte.
         * cell ctl register value as byte for chip communication
         */
        uint8_t bytes[1];
    };
    
    /**
     * @brief bal ctl register map.
     * This register is in charge to bypass batteries when charging
     */
    union bal_ctl_register_map
    {
        /**
         * @brief bal ctl register bitfield.
         * bal ctl register bitfields for direct values access
         */
        struct
        {
            /**
             * @brief balance cell 1.
             * turn on cell 1 balancing resistor
             */
            uint8_t BAL_1:1;
            /**
             * @brief balance cell 2.
             * turn on cell 2 balancing resistor
             */
            uint8_t BAL_2:1;
            /**
             * @brief balance cell 3.
             * turn on cell 3 balancing resistor
             */
            uint8_t BAL_3:1;
            /**
             * @brief balance cell 4.
             * turn on cell 4 balancing resistor
             */
            uint8_t BAL_4:1;
            /**
             * @brief balance cell 5.
             * turn on cell 5 balancing resistor
             */
            uint8_t BAL_5:1;
            /**
             * @brief balance cell 6.
             * turn on cell 6 balancing resistor
             */
            uint8_t BAL_6:1;
            /**
             * @brief reserved bits.
             * always 0
             */
            uint8_t RESERVED:2;
        } fields;
        /**
         * @brief bal ctl register value as byte.
         * bal ctl register value as byte for  chip communication
         */
        uint8_t bytes[1];
    };

    /**
     * @brief config 1 register mapping.
     * This register is able to monitor the current while charging and discharging and send a flag if something is not normal
     */
    union config_1_register_map
    {
        /**
         * @brief config 1 register bitfields.
         * config 1 register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief current amplifier gain.
             * Sets the nominal gain of the current amplifier
             */
            uint8_t I_GAIN:1;
            /**
             * @brief reserved bit.
             * always 0
             */
            uint8_t RESERVED:1;
            /**
             * @brief current amplifier calibration.
             * When 0, current amplifier reports SENSEN with respect to VSS
             * When 1, current amplifier reports SENSEP with respect to VSS
             */
            uint8_t I_AMP_CAL:1;
            /**
             * @brief Current comparator polarity select.
             * When 0, trips on discharge current (SENSEP > SENSEN)
             * When 1, trips on charge current (SENSEP < SENSEN)
             */
            uint8_t I_COMP_POL:1;
            /**
             * @brief Current comparator threshhold.
             * Sets the threshold of the current comparator
             */
            uint8_t I_THRESH:4;
        } fields;
        /**
         * @brief config 1 register value as byte.
         * config 1 register value as byte for chip communication
         */
        uint8_t bytes[1];
    };

    /**
     * @brief config 2 register mapping.
     * This register contains additionnal configuration options
     */
    union config_2_register_map
    {
        /**
         * @brief config 2 register fields.
         * config 2 register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief Reference selection.
             * Sets VREF VCOUT and VIOUT
             */
            uint8_t REF_SEL:1;
            /**
             * @brief Reserved bits.
             * Always 0
             */
            uint8_t RESERVED:6;
            /**
             * @brief CRC enable.
             * Enables the CRC communication check on writes
             */
            uint8_t CRC_EN:1;
        } fields;
        /**
         * @brief config 2 register value as byte.
         * config 2 register value as byte for chip communication
         */
        uint8_t bytes[1];
    };

    /**
     * @brief power ctl register mapping.
     * This register allows to turn on and off chip features
     */
    union power_ctl_register_map
    {
        /**
         * @brief power ctl register fields.
         * power ctl register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief Voltage reference enable.
             * When 1 the 1.5/3 V reference is enabled. Diable to save power
             */
            uint8_t REF_EN:1;
            /**
             * @brief Thermistor bias enable.
             * When 1, the VTB pin is internally switched to the V3P3 voltage
             */
            uint8_t VTB_EN:1;
            /**
             * @brief Cell amplifier enable.
             * When 1, the cell amplifier is enabled. Disable to save power.
             */
            uint8_t VC_AMP_EN:1;
            /**
             * @brief Current amplifier enable.
             * When 1, the current amplifier is enabled. Disable to save power.
             */
            uint8_t I_AMP_EN:1;
            /**
             * @brief Current comparator enable.
             * When 1, current comparator is enable. Disable to save power.
             */
            uint8_t I_COMP_EN:1;
            /**
             * @brief Reserved bit.
             * Always 0
             */
            uint8_t RESERVED:1;
            /**
             * @brief Sleep mode disable.
             * When 1, disable the sleep mode
             */
            uint8_t SLEEP_DIS:1;
            /**
             * @brief Sleep control.
             * Set to 1 to put the device to sleep
             */
            uint8_t SLEEP:1;
        } fields;
        /**
         * @brief power ctl register value as byte.
         * power ctl register value as byte for chip communication
         */
        uint8_t bytes[1];
    };

    /**
     * @brief Construct a new BQ76925 object.
     * Construct a BQ76952 connected to the specified i2c bus
     * @param i2c_bus i2c bus number to which the chip is connected
     */
    BQ76925(i2c_port_t i2c_bus);

    /**
     * @brief Select VCOUT cell.
     * Select which cell voltage should appear on the VCOUT pin
     * @param cell_number cell number (0-5)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t selectCell(uint8_t cell_number);

    /**
     * @brief Control balance resistor status.
     * Apply balance mapping to the balance resistor
     * @param balance desired balance mapping
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t balanceCell(bal_ctl_register_map balance);

    /**
     * @brief Get the chip status.
     * Fill up a status map with current chip status
     * @param status status map to be filled
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t getStatus(status_register_map &status);

    /**
     * @brief Set monitoring mode.
     * Enable chip peripheral and set comparator polarity according to required operation mode
     * @param monitor 1 for monitoring, 0 for standby
     * @param charge 1 for charge, 0 for discharge
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setMonitorMode(uint8_t monitor, uint8_t charge);

    /**
     * @brief Write configuration to the chip.
     * Write the configuration prepared when building the object to the chip
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t configure();

private:
    /**
     * @brief Write to chip register.
     * Write a value to one the BQ76925 register
     * @param address Register address
     * @param value value to be written
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t writeRegister(uint8_t address, uint8_t value[1]);

    /**
     * @brief Read from chip register.
     * Read a value from one of the BQ76925 register
     * @param address register address
     * @param value where to store the red value
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t readRegister(uint8_t address, uint8_t value[1]);

    /**
     * @brief i2c bus instance.
     * i2c bus instance used to communicate with the chip
     */
    I2C* _i2c;

    /**
     * @brief config 1 mapping.
     * configuration to be written in config 1 register on configure call
     */
    config_1_register_map _config1;

    /**
     * @brief config 2 mapping.
     * configuration to be written in config 2 register on configure call
     */
    config_2_register_map _config2;

    /**
     * @brief power ctl mapping.
     * configuration to be written in power ctl register on configure call
     */
    power_ctl_register_map _power;
};