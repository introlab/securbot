/**
 * @file switches.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Hardware abstraction for the board power switches
 * @version 0.1
 * @date 2019-10-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <driver/gpio.h>

#include "defines.hpp"

/**
 * @brief Hardware abstraction for the board power switches.
 * Controls the battery board GPIO controlled power switches.
 */
class Switches
{
public:
    /**
     * @brief Gets an instance of the driver.
     * Return a driver instance, create one if it's the first call
     * @return Switches* switch driver instance
     */
    static Switches* instance();

    /**
     * @brief Configure the used IOs.
     * Configure the GPIOs connected to the switches.
     * Must be called before using the driver
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t begin();

    /**
     * @brief Apply power to the BQ24725A chip.
     * Thread safe. Apply power to the BQ24725A chip.
     * @param enabled wether power is applied to the chip
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setBQ24725APower(uint32_t enabled);

    /**
     * @brief Apply power to the robot.
     * Thread safe. Apply power to the robot
     * @param enabled wether power is applied to the robot
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setRobotPower(uint32_t enabled);

    /**
     * @brief Apply power to the fan.
     * Thread safe. Apply power to the fan
     * @param enabled wether power is applied to the fan
     * @return esp_err_t operation success, check against ESP_OK
     */
    esp_err_t setFanPower(uint32_t enabled);

private:
    /**
     * @brief Construct a new Switches object.
     * Construct a new switched object. Do nothing for now.
     */
    Switches();

    /**
     * @brief The shared switches instance.
     * Pointer to the switched object retrieved when an instance is requested.
     */
    static Switches* _instance;
};