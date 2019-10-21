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
     * @brief updating state.
     * Temporary state used to store updated values
     */
    state::BoardStatus _upState;

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
        if (!_upState.batteryOk)    // battery is never ok if a reading failed
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

        // Create a state working copy while we update
        state::lock();
        _upState = state::current;
        state::unlock();

        // Define battery ok to be true
        // it will be used to check for monitoring errors
        _upState.batteryOk = true;

        // Check if the power adapter is connected
        _upState.isAdapterConnected = _charger->isAdapterPresent() == 1;

        if (!_upState.isAdapterConnected) // No power adapter connected
        {
            // the charger must be off and the battery not charging
            _upState.isChargerBooted = false;
            _upState.isCharging = false;
        }

        // Read voltage, current and temperature from the analog frontend
        ret = _frontend->getBatteryVoltage(_upState.batteryVoltage);
        if (ret != ESP_OK)
        {
            _upState.batteryOk = false;
            ESP_LOGE(TAG, "%s reading battery voltage", esp_err_to_name(ret));
        }
        ret = _frontend->getCellsVoltage(_upState.cellVoltages);
        if (ret != ESP_OK)
        {
            _upState.batteryOk = false;
            ESP_LOGE(TAG, "%s reading cell voltage", esp_err_to_name(ret));
        }
        ret = _frontend->getBoardTemperature(_upState.boardTemperatures);
        if (ret != ESP_OK)
        {
            _upState.batteryOk = false;
            ESP_LOGE(TAG, "%s reading board temperature", esp_err_to_name(ret));
        }
        ret = _frontend->getBatteryCurrent(_upState.bmsBatteryCurrent);
        if (ret != ESP_OK)
        {
            _upState.batteryOk = false;
            ESP_LOGE(TAG, "%s reading frontend battery current", esp_err_to_name(ret));
        }

        if (_upState.isChargerBooted) // the charger is configured by the control task
        {
            ret = _charger->getBatteryCurrent(_upState.chargerBatteryCurrent);
            if (ret != ESP_OK)
            {
                _upState.batteryOk = false;
                _upState.isChargerBooted = false;
                ESP_LOGE(TAG, "%s reading charger battery current", esp_err_to_name(ret));
            }
            ret = _charger->getAdapterCurrent(_upState.chargerAdapterCurrent);
            if (ret != ESP_OK)
            {
                _upState.batteryOk = false;
                _upState.isChargerBooted = false;
                ESP_LOGE(TAG, "%s reading charger adapter current", esp_err_to_name(ret));
            }
            ret = _charger->getRobotCurrent(_upState.chargerRobotCurrent);
            if (ret != ESP_OK)
            {
                _upState.batteryOk = false;
                _upState.isChargerBooted = false;
                ESP_LOGE(TAG, "%s reading charger robot current", esp_err_to_name(ret));
            }
        }
        else    // The charger is not configured
        {
            _upState.chargerBatteryCurrent = 0.0;
            _upState.chargerAdapterCurrent = 0.0;
            _upState.chargerRobotCurrent = 0.0;
        }

        // check if the current state is ok
        _upState.batteryOk = checkState();

        // Lock the state before updating it
        state::lock();
        state::current = _upState;
        state::unlock();

        // Notify subscribers that we have an update
        for (auto i = _subscribers.begin(); i < _subscribers.end(); i++)
        {
            xTaskNotify(*i, 0, eSetValueWithOverwrite);
        }
    }
}

void monitor::smashThatSubscribeButton( TaskHandle_t task )
{
    state::lock();
    _subscribers.push_back(task);
    state::unlock();
}