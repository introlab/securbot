/**
 * @file analogInput.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Driver to read analog inputs
 * @version 0.1
 * @date 2019-10-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "chip/ADS1015.hpp"

/**
 * @brief Driver for all analog input.
 * Driver that allows reading from analog input
 * A single driver reads from all input.
 */
class AnalogInput
{
public:
    /**
     * @brief get the AnalogInput instance.
     * Get the instance of AnalogInput. Create it if it doesnt exist
     * @return AnalogInput* pointer to the AnalogInput object
     */
    static AnalogInput* instance();

    /**
     * @brief Initialize the analogInput driver.
     * Configure the ADCs. Must be called before any reading is performed.
     * @return esp_err_t driver initialization result. Check against ESP_OK
     */
    esp_err_t begin();

    /**
     * @brief Read a channel.
     * Reads the voltage between the specified channel and ground
     * Thread safe
     * @param channel_number The channel number (0-7)
     * @param value channel voltage (V)
     * @return esp_err_t operation success. Check agaist ESP_OK
     */
    esp_err_t read(uint8_t channel_number, float &value);

private:
    /**
     * @brief Construct a new Analog Input object.
     * Construct a new Analog Input object
     * Is called automatically when an instance is requested
     */
    AnalogInput();

    /**
     * @brief Pointer to the AnalogInput instance.
     * Point to the unique AnalogInput instance
     * There is one object shared amongst all instances
     */
    static AnalogInput* _instance;

    /**
     * @brief ADCs instance.
     * Array of ADCs instance from which are used to read analog values
     */
    ADS1015* _adcs[2];

    /**
     * @brief Instance mutex.
     * Mutex to make read calls thread-safe
     * There is on per ADC
     */
    SemaphoreHandle_t _mutex[2];
};