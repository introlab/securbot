/**
 * @file state.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Battery Board State Declaration
 * @version 0.1
 * @date 2019-10-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "defines.hpp"

/**
 * @brief Battery board state namespace.
 * Hold all definitions and global variables relative to the battery board state
 */
namespace state
{
    /**
     * @brief Board Status structure.
     * Contains the battery board status, namely all current and voltages
     */
    typedef struct
    {
        /**
         * @brief Battery is OK.
         * Indicate the monitoring task successfully measure and validate all board parameters
         */
        bool batteryOk;

        /**
         * @brief charging.
         * Indicate the battery is charging.
         */
        bool isCharging;
        /**
         * @brief balancing.
         * Indicate charge is coming to an end and the cells are balancing
         */
        bool isBalancing;
        /**
         * @brief charged.
         * Indicate charge is complete
         */
        bool isCharged;
        /**
         * @brief charger booted.
         * Indicate that the battery charger ic is powered on and configured
         */
        bool isChargerBooted;
        /**
         * @brief adapter connected.
         * Indicate that the AC adapter is connected
         */
        bool isAdapterConnected;

        /**
         * @brief battery voltage.
         * Indicate the current battery voltage red by the analog frontend
         */
        float batteryVoltage;
        /**
         * @brief cell voltage.
         * Indicate the current cell voltage red by the analog frontend
         */
        float cellVoltages[4];
        /**
         * @brief board temperature.
         * Current temperature reading of the two thermistors on the board
         */
        float boardTemperatures[2];

        /**
         * @brief bms battery current.
         * Indicate the current battery current red by the analog frontend
         */
        float bmsBatteryCurrent;

        /**
         * @brief charger battery current.
         * Indicate the current batttery current red by the battery charger
         * Reads 0 if charger is not booted
         */
        float chargerBatteryCurrent;
        /**
         * @brief charger adapter battery current.
         * Indicate the current adapter current red by the battery charger
         * Reads 0 if charger is not booted
         */
        float chargerAdapterCurrent;
        /**
         * @brief charger robot current.
         * Indicate the current robot current red by the battery charger
         * Reads 0 if the charger is not booted
         */
        float chargerRobotCurrent;
    } BoardStatus;

    /**
     * @brief Current board status.
     * Global board status. Use utiliy function to protect access
     */
    extern BoardStatus current;

    /**
     * @brief Create the current status.
     * Initialize the current board status structure
     */
    void create();

    /**
     * @brief Lock the current status.
     * Acquire the board status mutex
     */
    void lock();

    /**
     * @brief Unlock the current status.
     * Release the board status mutex
     */
    void unlock();

    /**
     * @brief Finds the highest voltage among cells.
     * Utility function to find the cell with the highest voltage among the array
     * @param v cell voltage array
     * @param high highest voltage
     * @param index highest cell index
     */
    void findHighestCell(float v[4], float &high, uint8_t &index);

    /**
     * @brief Finds the lowest voltage among cells.
     * Utility function to find the cell with the lowest voltage among the array
     * @param v cell voltage array
     * @param low lowest voltage
     * @param index lowest cell index
     */
    void findLowestCell(float v[4], float &low, uint8_t &index);
}