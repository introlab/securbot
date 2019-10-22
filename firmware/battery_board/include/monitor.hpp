/**
 * @file monitor.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Battery Monitoring Task declaration
 * @version 0.1
 * @date 2019-10-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

#include "defines.hpp"
#include "state.hpp"
#include "hal/frontend.hpp"
#include "hal/charger.hpp"

/**
 * @brief Battery board monitor namespace.
 * Hold all definitions and global variables relative to the battery board monitor
 */
namespace monitor
{
    /**
     * @brief Monitoring task function.
     * Task function that monitor the battery
     * @param pvParameters parameters to pass to the function (not used)
     */
    void monitorTask_fn( void* pvParameters );

    /**
     * @brief Subscribe to the update monitoring notifications.
     * Add a task handle to the vector of task that need to be notified
     * @param task handle to the task that must be notified
     */
    void smashThatSubscribeButton( TaskHandle_t task );
}