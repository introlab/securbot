/**
 * @file charger.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Hardware abstraction for the battery charger hardware
 * @version 0.1
 * @date 2019-10-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "defines.hpp"
#include "hal/analogInput.hpp"
#include "chip/BQ24725A.hpp"

/**
 * @brief Battery charger abstraction class.
 * Provide full control over battery charging
 */
class Charger
{
public:
    /**
     * @brief Get a charger instance.
     * Return a pointer to the shared charger instance. Create one if it doesnt exist
     * @return Charger* pointer to the charge object
     */
    static Charger* instance();

    /**
     * @brief Initialize the charger.
     * Must be called before anything else. Prepare hardware for driver operation
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t begin();

    /**
     * @brief is the AC adapter connected.
     * Check if the AC adapter is connected trough the BQ24725A chip ACOK
     * thread safe
     * @return int 1 adapter present, 0 not present
     */
    int isAdapterPresent();

    /**
     * @brief perform required initialization when AC power is applied.
     * Write configuration to the BQ24725A chip after it has been reset by the lost of AC power
     * thread safe
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t postAdapterInit();

    /**
     * @brief Set the battery charge current.
     * Write the desired charge current to the BQ24725A chip
     * thread safe
     * @param current charge current (n * 64 mA)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setChargeCurrent(uint8_t current);

    /**
     * @brief Set the battery charge voltage.
     * Write the desired charge voltage to the BQ24725A chip
     * thread safe
     * @param voltage charge voltage (n * 16 mV)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setChargeVoltage(uint16_t voltage);

    /**
     * @brief Set the adapter current.
     * Write the desired adapter current to the BQ24725A chip
     * thread safe
     * @param current input current (n * 128 mA)
     * @return esp_err_t operation success
     */
    esp_err_t setAdapterCurrent(uint8_t current);

    /**
     * @brief Prevents charging.
     * Prevents charging even if a charge current is set and AC is connected
     * thread safe
     * @param inhibited 1 prevent charge, 0 allow
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t inhibitCharge(uint8_t inhibited);

    /**
     * @brief Get the battery current.
     * Get the battery current measured at the output of the BQ24725A
     * Thread safe
     * @param current battery current (A)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t getBatteryCurrent(float &current);

    /**
     * @brief Get the AC adapter current.
     * Get the adapter current measured at the output of the BQ24725A
     * thread safe
     * @param current adapter current (A)
     * @return esp_err_t 
     */
    esp_err_t getAdapterCurrent(float &current);

    /**
     * @brief Get the load current to the robot.
     * Get the current flowing to the robot trough the difference between adapter and battery current
     * @param current robot current (A)
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t getRobotCurrent(float &current);

private:
    /**
     * @brief Construct a new Charger object.
     * Automatically called trough the get instance static method
     */
    Charger();

    /**
     * @brief The shared charger instance.
     * Points to the shared charger instance
     */
    static Charger* _instance;

    /**
     * @brief BQ24725A battery charger chip.
     * BQ24725A chip instance to perform most battery charging operations
     */
    BQ24725A _bq24 {BQ24725A_I2C_NUM};

    /**
     * @brief Analog input driver.
     * Driver to read the IOUT pin of the battery charge to perform current measure
     */
    AnalogInput* _analog;

    /**
     * @brief Shared instance mutex.
     * Lock the charger so a single operation is performed at the same time
     */
    SemaphoreHandle_t _mutex;
};