/**
 * @file frontend.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Hardware abstraction for the battery board analog front end
 * @version 0.1
 * @date 2019-10-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <cmath>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "defines.hpp"
#include "hal/analogInput.hpp"
#include "chip/BQ76925.hpp"

/**
 * @brief Analog front end driver.
 * Provide all the function of the analog front end to monitor and balance cells
 */
class Frontend
{
public:
    /**
     * @brief Front end instance getter.
     * Gets a pointer to the front end instance. Create it if needed.
     * @return Frontend* pointer to the front end instance.
     */
    static Frontend* instance();

    /**
     * @brief Front end initialization function.
     * Prepare the hardware for proper operation of the analog front end.
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t begin();

    /**
     * @brief Get the Battery Current.
     * Reads the battery current trough the analog front end.
     * Thread safe, doesnt require AC power
     * @param current the battery current
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t getBatteryCurrent(float current);

    /**
     * @brief Set the Current Polarity.
     * Configure the front end to monitor either charge or discharge.
     * Thread safe
     * @param charging 1 is charge, 0 discharge
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setCurrentPolarity(uint8_t charging);

    /**
     * @brief Get the Overcurrent Alert.
     * Return the state of the front end over current alert pin.
     * Thread safe
     * @return int 1 is alert, 0 is ok
     */
    int getOvercurrentAlert();

    /**
     * @brief Get the Cells Voltage.
     * Reads all cells voltage in sequence.
     * Thread safe
     * @param voltage cell voltage array
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t getCellsVoltage(float voltage[4]);

    /**
     * @brief Get the Board Temperature.
     * Reads the board thermistor in sequence.
     * Thread safe
     * @param temperature board temperatures array
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t getBoardTemperature(float temperature[2]);

    /**
     * @brief Select a cell to bypass.
     * Bypasse the specified cell trough the bypass resistors.
     * @param cell cell number (1-6). 0 is no bypass
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setBalance(uint8_t cell);

private:
    /**
     * @brief Construct a new Frontend object.
     * Retrieve appropriate instance of analog input and create the mutex.
     * called automatically by the instance getter
     */
    Frontend();

    /**
     * @brief The unique front end instance.
     * Pointer to the unique front end instance.
     */
    static Frontend* _instance;

    /**
     * @brief BQ76925 driver.
     * Driver to communicate with the analog front end chip.
     */
    BQ76925 _bq76 {BQ24725A_I2C_NUM};

    /**
     * @brief Analog input driver instance.
     * an instance of the analog driver to read voltages and currents.
     */
    AnalogInput* _analog;

    /**
     * @brief Front end mutex.
     * mutex to protect the front end instance against concurrent calls
     */
    SemaphoreHandle_t _mutex;
};