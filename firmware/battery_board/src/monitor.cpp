/**
 * @file monitor.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Battery Monitoring Task definition
 * @version 0.1
 * @date 2019-10-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "monitor.hpp"

/**
 * @brief monitor private namespace.
 * Contains private global definitions used by the monitor
 */
namespace
{
    /**
     * @brief monitor logging tag.
     * Tag used when logging from the monitor file
     */
    const char* TAG = "Monitor";

    /**
     * @brief Battery charger instance.
     * Pointer to the charger instance
     */
    Charger* _charger;

    /**
     * @brief Analog front end instance.
     * Pointer to the analog frontend instance
     */
    Frontend* _frontend;

    /**
     * @brief notification subscribers.
     * Vector containing task handle to be notified on each run
     */
    std::vector<TaskHandle_t> _subscribers;

    /**
     * @brief validate if the current state is safe.
     * Check every state parameter against safety margins
     * @return true battery is ok
     * @return false battery is in an unknown or dangerous state
     */
    bool checkState()
    {
        if (state::current.batteryOk)    // battery is never ok if a reading failed
        {
            return false;
        }

        return true;
    }
}

void monitor::monitorTask_fn( void* pvParameters )
{
    state::create();    // Initialize current state

    // Get drivers instances
    _charger = Charger::instance();
    _frontend = Frontend::instance();

    esp_err_t ret;
    TickType_t lastRead = xTaskGetTickCount();

    while (1)
    {
        // Run the monitor loop at the correct rate
        vTaskDelayUntil(&lastRead, MONITOR_PERIOD_MS / portTICK_PERIOD_MS);

        // Lock the state before the update
        state::lock();

        // Define battery ok to be true
        // it will be used to check for monitoring errors
        state::current.batteryOk = true;

        // Check if the power adapter is connected
        state::current.isAdapterConnected = _charger->isAdapterPresent() == 1;

        if (!state::current.isAdapterConnected) // No power adapter connected
        {
            // the charger must be off and the battery not charging
            state::current.isChargerBooted = false;
            state::current.isCharging = false;
            state::current.isBalancing = false;
            state::current.isCharged = false;
        }

        // Read voltage, current and temperature from the analog frontend
        ret = _frontend->getBatteryVoltage(state::current.batteryVoltage);
        if (ret != ESP_OK)
        {
            state::current.batteryOk = false;
            ESP_LOGE(TAG, "%s reading battery voltage", esp_err_to_name(ret));
        }
        ret = _frontend->getCellsVoltage(state::current.cellVoltages);
        if (ret != ESP_OK)
        {
            state::current.batteryOk = false;
            ESP_LOGE(TAG, "%s reading cell voltage", esp_err_to_name(ret));
        }
        ret = _frontend->getBoardTemperature(state::current.boardTemperatures);
        if (ret != ESP_OK)
        {
            state::current.batteryOk = false;
            ESP_LOGE(TAG, "%s reading board temperature", esp_err_to_name(ret));
        }
        ret = _frontend->getBatteryCurrent(state::current.bmsBatteryCurrent);
        if (ret != ESP_OK)
        {
            state::current.batteryOk = false;
            ESP_LOGE(TAG, "%s reading frontend battery current", esp_err_to_name(ret));
        }

        if (state::current.isChargerBooted) // the charger is configured by the control task
        {
            ret = _charger->getBatteryCurrent(state::current.chargerBatteryCurrent);
            if (ret != ESP_OK)
            {
                state::current.isChargerBooted = false;
                state::current.chargerBatteryCurrent = 0.0;
                ESP_LOGW(TAG, "%s reading charger battery current", esp_err_to_name(ret));
            }
            ret = _charger->getAdapterCurrent(state::current.chargerAdapterCurrent);
            if (ret != ESP_OK)
            {
                state::current.isChargerBooted = false;
                state::current.chargerAdapterCurrent = 0.0;
                ESP_LOGW(TAG, "%s reading charger adapter current", esp_err_to_name(ret));
            }
            ret = _charger->getRobotCurrent(state::current.chargerRobotCurrent);
            if (ret != ESP_OK)
            {
                state::current.isChargerBooted = false;
                state::current.chargerRobotCurrent = 0.0;
                ESP_LOGW(TAG, "%s reading charger robot current", esp_err_to_name(ret));
            }
        }
        else    // The charger is not configured
        {
            state::current.chargerBatteryCurrent = 0.0;
            state::current.chargerAdapterCurrent = 0.0;
            state::current.chargerRobotCurrent = 0.0;
        }

        // check if the current state is ok
        state::current.batteryOk = checkState();

        // Notify subscribers that we have an update
        for (auto i = _subscribers.begin(); i < _subscribers.end(); i++)
        {
            xTaskNotify(*i, 0, eSetValueWithOverwrite);
        }

        // Unlock the state when update is done
        state::unlock();
    }
}

void monitor::smashThatSubscribeButton( TaskHandle_t task )
{
    state::lock();
    _subscribers.push_back(task);
    state::unlock();
}