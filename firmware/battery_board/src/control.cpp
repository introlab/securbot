/**
 * @file control.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Control task definition
 * @version 0.1
 * @date 2019-10-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "control.hpp"

/**
 * @brief Control task private namespace.
 * Anonymous namespace to hive the control task private global definitions
 */
namespace
{
    /**
     * @brief control logging tag.
     * Tag used when logging from the control task
     */
    const char* TAG = "Control";

    /**
     * @brief board power switeches instance.
     * GPIO controlled switches to turn on power to board sections
     */
    Switches* _switches;

    /**
     * @brief battery charger instance.
     * Battery charger to set charge current and voltage
     */
    Charger* _charger;

    /**
     * @brief Analog frontend instance.
     * Analog frontend to balance cells
     */
    Frontend* _frontend;
}

void control::controlTask_fn( void* pvParameters)
{
    esp_err_t ret;
    float vHigh = VMAX;
    float vLow = 0.0;
    uint8_t iHigh = 0;
    uint8_t iLow = 0;

    // Get handle to unique instances
    _switches = Switches::instance();
    _charger = Charger::instance();
    _frontend = Frontend::instance();

    while (1)
    {
        // Wait for state update before running again
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // lock state to prevent changes mid control
        state::lock();

        // power the robot if battery is ok
        uint32_t pwr = state::current.batteryOk;
        _switches->setRobotPower(pwr);
        _switches->setBQ24725APower(pwr);

        // on AC power and good battery
        if (state::current.isAdapterConnected && state::current.batteryOk)
        {
            // boot charger if required (first run since AC went live)
            if (!state::current.isChargerBooted) 
            {
                ESP_LOGD(TAG, "Booting BQ24725A");
                uint8_t attempts = 3;
                do  // in case power just went live do a few attempts
                {
                    ret = _charger->postAdapterInit();
                    attempts--;
                    if (ret != ESP_OK)
                    {
                        ESP_LOGW(TAG, "%s configuring BQ24725A", esp_err_to_name(ret));
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                } while (ret != ESP_OK && attempts > 0);
                
                if (ret == ESP_OK)  // boot is successful
                {
                    ESP_LOGI(TAG, "Charger booted");
                    state::current.isChargerBooted = true;
                }
            }

            // charger is ready. Not an else because it could be booted in the first if
            if (state::current.isChargerBooted)
            {
                // find highest and lowest cell
                state::findHighestCell(state::current.cellVoltages, vHigh, iHigh);
                state::findLowestCell(state::current.cellVoltages, vLow, iLow);

                if ( vHigh >= VMAX || state::current.isCharged)   // battery is charged
                {
                    state::current.isCharged = true;
                    state::current.isCharging = false;
                    state::current.isBalancing = false;

                    // current and voltage are all zero
                    _charger->inhibitCharge(1);
                    _charger->setAdapterCurrent(0);
                    _charger->setChargeVoltage(0);
                    _charger->setChargeCurrent(0);

                    // No bypassed cell
                    _frontend->setBalance(0);
                }
                else
                {
                    // balance when lowest cell is at catch up limit
                    if ( 3*vHigh-2*vLow >= VMAX || state::current.isBalancing)
                    {
                        state::current.isCharged = false;
                        state::current.isCharging = true;
                        state::current.isBalancing = true;

                        // full charge current
                        _charger->inhibitCharge(0);
                        _charger->setAdapterCurrent(ADAPTER_CURRENT);
                        _charger->setChargeVoltage(CHARGE_VOLTAGE);
                        _charger->setChargeCurrent(BALANCE_CURRENT);

                        // Bypass highest cell
                        _frontend->setBalance(iHigh + 1);
                    }
                    // charge at high speed
                    else
                    {
                        state::current.isCharged = false;
                        state::current.isCharging = true;
                        state::current.isBalancing = false;

                        _charger->inhibitCharge(0);
                        _charger->setAdapterCurrent(ADAPTER_CURRENT);
                        _charger->setChargeVoltage(CHARGE_VOLTAGE);
                        _charger->setChargeCurrent(CHARGE_CURRENT);

                        // No bypassed cell
                        _frontend->setBalance(0);
                    }
                    
                }
            }
        }

        // unlock the state when done
        state::unlock();
    }
}