/**
 * @file control.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Control task declaration
 * @version 0.1
 * @date 2019-10-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "state.hpp"
#include "hal/switches.hpp"
#include "hal/charger.hpp"
#include "hal/frontend.hpp"

/**
 * @brief control namespace.
 * Namespace for all control task public declarations
 */
namespace control
{
    /**
     * @brief Control task function.
     * Task function that control the board according to current state
     * @param pvParameters Parameters passed to the function (not used)
     */
    void controlTask_fn( void* pvParameters);
}