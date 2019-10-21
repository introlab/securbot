/**
 * @file BQ24725A.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Driver for the BQ24725A chip
 * @version 0.1
 * @date 2019-10-09
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include "hal/i2c.hpp"

/**
 * @brief Interface class for the BQ24725A chip.
 * Allow configuration and usage of a single BQ24725A chip
 */
class BQ24725A
{
public:
    /**
     * @brief charge option register mapping.
     * This register controls the chip battery charging options
     */
    union charge_option_register_map
    {
        /**
         * @brief Charge option register fields.
         * Charge option register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief charge inhibit.
             * 0 enable charge, 1 inhibit charge
             */
            uint8_t CHARGE_INHIBIT:1;
            /**
             * @brief ACOC threshold adjust.
             * 0 disabled, 1 3.33x of input current regulation limit
             */
            uint8_t ACOC:1;
            uint8_t :2;
            /**
             * @brief AC adapter indication (read only).
             * 0 AC is not present, 1 AC is present.
             */
            uint8_t AC:1;
            /**
             * @brief IOUT selection.
             * 0 IOUT is 20x adapter current amp output, 1 IOUT is 20x charge current amp output
             */
            uint8_t IOUT:1;
            /**
             * @brief Learn cycle.
             * 0 disable learn cycle, 1 enable lear cycle
             */
            uint8_t LEARN:1;
            /**
             * @brief IFAULT_LOW comparator threshold adjust.
             * 0 135mV, 1 230 mV
             */
            uint8_t IFAULT_LOW:1;
            /**
             * @brief IFAULT_HI comparator threshold adjust.
             * 0 disabled, 1 750 mV
             */
            uint8_t IFAULT_HI:1;
            /**
             * @brief EMI switching frequency enable.
             * 0 disable, 1 enable adjust PWM switching frequency
             */
            uint8_t EMI_EN:1;
            /**
             * @brief EMI switching frequency adjust.
             * 0 reduce 1 increase PWM switching frquency by 18%
             */
            uint8_t EMI_FREQ:1;
            /**
             * @brief BAT depletion comparator threshold adjust.
             * 59.19%, 62.65%, 66.55%, 70.97% of voltage regulation limit
             */
            uint8_t BAT:2;
            /**
             * @brief watchdog timer adjust.
             * 0 disable, 44s, 88s, 175s
             */
            uint8_t WATCHDOG:2;
            /**
             * @brief ACOK deglitch time adjust.
             * 0 150ms, 1 1.3s rising edge deglitch time
             */
            uint8_t ACOK:1;
        } fields;
        /**
         * @brief Charge option register value as bytes.
         * Charge option register value as byte for chip communication
         */
        uint8_t bytes[2];
    };

    /**
     * @brief Charge current register mapping.
     * This register controls the battery charge current
     */
    union charge_current_register_map
    {
        /**
         * @brief Battery charge current register fields.
         * Battery charge current bitfields for direct value access
         */
        struct
        {
            uint8_t :6;
            /**
             * @brief Charge current.
             * Charge current DACICHG * 64mA
             */
            uint8_t DACICHG:7;
            uint8_t :3;
        } fields;
        /**
         * @brief Battery charge current register value as bytes.
         * Battery charge current register as bytes for chip communication
         */
        uint8_t bytes[2];
    };

    /**
     * @brief Charge voltage register mapping.
     * This register controls the battery charge voltage
     */
    union charge_voltage_register_map
    {
        /**
         * @brief Charge voltage register fields.
         * Charge voltage register bitfields for direct value access
         */
        struct
        {
            uint8_t :4;
            /**
             * @brief Charge voltage.
             * Charge voltage DACV * 16 mV
             */
            uint16_t DACV:11;
            uint8_t :1;
        } fields;
        /**
         * @brief Charge voltage register value as bytes.
         * Charge voltage register value as bytes for chip communication
         */
        uint8_t bytes[2];
    };

    /**
     * @brief Input current register mapping.
     * This register controls the charger input current
     */
    union input_current_register_map
    {
        /**
         * @brief Input current register fields.
         * Input current register value as fields for direct value access
         */
        struct
        {
            uint8_t :7;
            /**
             * @brief Charger input current.
             * Charger input current DACIIN * 128mA 
             */
            uint8_t DACIIN:6;
            uint8_t :3;
        } fields;
        /**
         * @brief Input current register bytes.
         * Input current register value as bytes for chip communication
         */
        uint8_t bytes[2];
    };

    /**
     * @brief Construct a new BQ24725A object.
     * Construct a BQ24725A object and retrieve the appropriate i2c bus instance
     * @param i2c_bus the i2c bus number the chip is connected to
     */
    BQ24725A(i2c_port_t i2c_bus);

    /**
     * @brief Configure the chip.
     * Write the configuration to the BQ24725A chip
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t configure();

    /**
     * @brief Enable or disable charging.
     * Set the charge inhibit bit to inhibit or not battery charging
     * @param inhibit inhibit bit (1 is inihibit)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setChargeInhibit(uint8_t inhibit);

    /**
     * @brief Set the charge voltage.
     * Set the battery charge voltage
     * @param voltage charge voltage (n * 16 mV)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setChargeVoltage(uint16_t voltage);

    /**
     * @brief Set the Charge Current.
     * Set the battery charge current
     * @param current charge current (n * 64 mA)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setChargeCurrent(uint8_t current);

    /**
     * @brief Set the charger input current.
     *  Set the charger input current
     * @param current input current (n * 128 mA)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setInputCurrent(uint8_t current);

    /**
     * @brief Sets IOUT configuration bit.
     * Select if IOUT is 20x adapter current (0) or 20x charge current (1)
     * @param iout IOUT config bit value
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t selectIOUT(uint8_t iout);

    /**
     * @brief Get the Chip Id.
     * Read the device id.
     * @param id where to store the id
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t getChipId(uint16_t &id);

private:
    /**
     * @brief i2c bus instance.
     * i2c bus instance to communicate with the chip
     */
    I2C* _i2c;

    /**
     * @brief chip charge options.
     * chip charge option used to configure
     */
    charge_option_register_map _chg_opt;
};