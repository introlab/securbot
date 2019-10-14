/**
 * @file switches.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Hardware abstraction for the board power switches
 * @version 0.1
 * @date 2019-10-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "hal/switches.hpp"

Switches* Switches::_instance = NULL;

Switches* Switches::instance()
{
    if (_instance == NULL)
    {
        _instance = new Switches();
    }
    return _instance;
}

esp_err_t Switches::begin()
{
    esp_err_t ret;

    // BQ24725A
    gpio_pad_select_gpio(BQ24725A_EN_GPIO);
    ret = gpio_set_direction((gpio_num_t)BQ24725A_EN_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Relay to the robot
    gpio_pad_select_gpio(CMD_RELAY_GPIO);
    ret = gpio_set_direction((gpio_num_t)CMD_RELAY_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Fan control
    gpio_pad_select_gpio(CMD_FAN_GPIO);
    return gpio_set_direction((gpio_num_t)CMD_FAN_GPIO, GPIO_MODE_OUTPUT);
}

esp_err_t Switches::setBQ24725APower(uint32_t enabled)
{
    return gpio_set_level((gpio_num_t)BQ24725A_EN_GPIO, enabled);
}

esp_err_t Switches::setRobotPower(uint32_t enabled)
{
    return gpio_set_level((gpio_num_t)CMD_RELAY_GPIO, enabled);
}

esp_err_t Switches::setFanPower(uint32_t enabled)
{
    return gpio_set_level((gpio_num_t)CMD_FAN_GPIO, enabled);
}

Switches::Switches()
{
    // Empty
}