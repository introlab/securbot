/**
 * @file state.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Battery Board State Definition
 * @version 0.1
 * @date 2019-10-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "state.hpp"

state::BoardStatus state::current;

/**
 * @brief state private namespace.
 * Private global definitions used by the state file
 */
namespace
{
    /**
     * @brief board state mutex.
     * mutex to protect acces to the board state
     */
    SemaphoreHandle_t _mutex;
}

void state::create()
{
    current.batteryOk = false;

    current.isCharging = false;
    current.isChargerBooted = false;
    current.isAdapterConnected = false;

    current.batteryVoltage = 0.0;

    for (uint8_t iCell = 0; iCell < 4; iCell++)
    {
        current.cellVoltages[iCell] = 0.0;
    }

    for (uint8_t iTemp = 0; iTemp < 2; iTemp++)
    {
        current.boardTemperatures[iTemp] = 0.0;
    }

    current.bmsBatteryCurrent = 0.0;

    current.chargerAdapterCurrent = 0.0;
    current.chargerBatteryCurrent = 0.0;
    current.chargerRobotCurrent = 0.0;

    _mutex = xSemaphoreCreateMutex();
}

void state::lock()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
}

void state::unlock()
{
    xSemaphoreGive(_mutex);
}