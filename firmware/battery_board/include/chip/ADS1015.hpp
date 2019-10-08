/**
 * @file ADS1015.hpp
 * @author your Cédric Godin (cedric.godin@me.com)
 * @brief Driver for the ADS1015 ADC
 * @version 0.1
 * @date 2019-10-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "hal/i2c.hpp"

/**
 * @brief Interface class for the ADS1015 chip
 * Allow configuration and usage of a single ADS1015
 */
class ADS1015
{
public:
    /**
     * @brief address pointer register mapping
     * maps address pointer register bitfields to register bits
     */
    union address_pointer_register_map
    {
        /**
         * @brief address pointer register fields
         * address pointer register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief register address
             * which register will be write or read on the next i2c transaction
             */
            uint8_t P:2;
            /**
             * @brief reserved bits
             * Always reads back 0h
             */
            uint8_t RESERVED:6;
        } fields;
        /**
         * @brief register value as byte
         * access address pointer register value as byte
         */
        uint8_t bytes[1];
    };

    /**
     * @brief Conversion register mapping
     * maps conversion register bitfields to register bits
     */
    union conversion_register_map
    {
        /**
         * @brief conversion register fields
         * conversion register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief reserved bits
             * Always reads back 0h
             */
            uint8_t RESERVED:4;
            /**
             * @brief conversion data
             * 12-bit conversion result
             */
            int16_t D:12;
        } fields;
        /**
         * @brief register value as byte
         * access conversion register value as byte
         */
        uint8_t bytes[2];
    };

    /**
     * @brief configuration register mapping
     * maps configuration register bitfields to register bits
     */
    union config_register_map
    {
        /**
         * @brief configuration register fields
         * configuration register bitfields for direct value access
         */
        struct
        {
            /**
             * @brief ALERT/RDY trigger number
             * Number of event to trigger ALERT/RDY signal
             */
            uint8_t COMP_QUE: 2;
            /**
             * @brief Latch comparator
             * Latching or non-latching comparator
             */
            uint8_t COMP_LAT:1;
            /**
             * @brief Comparaotr polarity
             * Active low or Active high
             */
            uint8_t COMP_POL:1;
            /**
             * @brief Comparator mode
             * traditional or window
             */
            uint8_t COMP_MODE:1;
            /**
             * @brief Data rate for continuous conversion mode
             * How many sample per second to take while in continuous mode
             */
            uint8_t DR:3;
            /**
             * @brief Operation mode
             * Wheter to operate in single or contiuous reading mode
             */
            uint8_t MODE:1;
            /**
             * @brief Full scale range adjust
             * Change what is the full scale reading of the ADC
             */
            uint8_t PGA:3;
            /**
             * @brief MUX channel
             * what channel to read
             */
            uint8_t MUX_channel:2;
            /**
             * @brief MUX mode
             * single or differential reading
             */
            uint8_t MUX_diff:1;
            /**
             * @brief conversion control
             * write 1 to start a conversion
             * read 0 while conversion is in progress
             */
            uint8_t OS:1;
        } fields;
        /**
         * @brief configuration register bytes
         * configuration register byte to communicate with the chip
         */
        uint8_t bytes[2];
    };

    /**
     * @brief Construct a new ADS1015 object
     * Construct an object interfacing with the chip at the specified address
     * @param i2c_address target chip i2c address
     */
    ADS1015(uint8_t i2c_address);

    /**
     * @brief Write current config to configuration register
     * Write current config to the chip configuration register
     * @return esp_err_t did operation succeed. Check against ESP_OK
     */
    esp_err_t configure();

    /**
     * @brief Start reading the specified channel
     * Start a reading on the ADC by writing the proper channel in mux register
     * @param channel_number Channel to read (0-3)
     * @return esp_err_t did operation succeed. Check against ESP_OK
     */
    esp_err_t startReading(uint8_t channel_number);

    /**
     * @brief Check is ADC is making a measurement
     * Return config register OS bit as bool to check if a reading is in progress
     * @param value set to true if ADC is measuring voltage. Else set to false
     * @return esp_err_t did operation succeed. Check against ESP_OK
     */
    esp_err_t isReading(bool &value);

    /**
     * @brief Get the latest reading
     * Reads the value in the conversion register and parse it according to configured scale
     * @param value voltage (volts)
     * @return esp_err_t did operation succeed. Check against ESP_OK 
     */
    esp_err_t getValue(double &value);

private:
    /**
     * @brief utility to write ADS1015 register
     * Utility function to write to ADS1015 registers. It manages the byte order so that value[0] is LSB and value[1] is MSB.
     * Can directly write register map bytes field to the chip
     * @param address address_pointer_register_map configured to point to the desired register
     * @param value two byte array containing the value to write
     * @return esp_err_t did operation succeed. Check against ESP_OK 
     */
    esp_err_t writeRegister(address_pointer_register_map address, uint8_t value[2]);

    /**
     * @brief utility to read ADS1015 registers
     * Utility function to read from ADS1015 registers. It manages the byte order so that value[0] is LSB and value[1] is MSB.
     * Can directly read register map bytes field from the chip
     * @param address address_pointer_register_map configured to point to the desired register
     * @param value two byte array where to store the red value
     * @return esp_err_t did operation succeed. Check against ESP_OK 
     */
    esp_err_t readRegister(address_pointer_register_map address, uint8_t value[2]);

    /**
     * @brief i2c address of the chip
     * i2c address to which the ADS1015 chip matching this instance respond
     */
    uint8_t _i2c_address;

    /**
     * @brief i2c driver instance
     * i2c driver instance trough which communicate
     */
    I2C* _i2c;

    /**
     * @brief Chip current config
     * config_register_map to store current config between configure() call
     */
    config_register_map _config;
};