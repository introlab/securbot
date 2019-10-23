/**
 * @file charger.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Hardware abstraction for the battery charger hardware
 * @version 0.1
 * @date 2019-10-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "hal/charger.hpp"

/**
 * @brief IOUT reads adapter current.
 * IOUT bit value to read the adapter current
 */
#define IOUT_ADAPTER 0

/**
 * @brief IOUT reads charge current.
 * IOUT bit value to read the battery charge current
 */
#define IOUT_CHARGE 1

/**
 * @brief Charger anonymous namespace.
 * Namespace to hide charge global variables
 */
namespace
{
    /**
     * @brief Charger logging tag.
     * Tag to use when logging from the charger file
     */
    const char* TAG = "Charger";
}

Charger* Charger::_instance = NULL;

Charger* Charger::instance()
{
    if (_instance == NULL)  // create the instance on the first call
    {
        _instance = new Charger();
    }
    return _instance;
}

esp_err_t Charger::begin()
{
    // configure ACOK pin as input
    gpio_pad_select_gpio(BQ24725A_ACOK_GPIO);
    return gpio_set_direction((gpio_num_t)BQ24725A_ACOK_GPIO, GPIO_MODE_INPUT);
}

int Charger::isAdapterPresent()
{
    int value;
    
    xSemaphoreTake(_mutex, portMAX_DELAY);
    value = gpio_get_level((gpio_num_t)BQ24725A_ACOK_GPIO);
    xSemaphoreGive(_mutex);

    return value;
}

esp_err_t Charger::postAdapterInit()
{
    esp_err_t ret;
    uint16_t id;

    xSemaphoreTake(_mutex, portMAX_DELAY);

    // Read the chip id
    ret = _bq24.getChipId(id);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return ret;
    }
    ESP_LOGI(TAG, "BQ24725A chip id 0x%04x", id);

    // configure the charger
    ret = _bq24.configure();

    xSemaphoreGive(_mutex);
    return ret;
}

esp_err_t Charger::setChargeCurrent(uint8_t current)
{
    esp_err_t ret;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    ret = _bq24.setChargeCurrent(current);
    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t Charger::setChargeVoltage(uint16_t voltage)
{
    esp_err_t ret;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    ret = _bq24.setChargeVoltage(voltage);
    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t Charger::setAdapterCurrent(uint8_t current)
{
    esp_err_t ret;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    ret = _bq24.setInputCurrent(current);
    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t Charger::inhibitCharge(uint8_t inhibited)
{
    esp_err_t ret;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    ret = _bq24.setChargeInhibit(inhibited);
    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t Charger::getBatteryCurrent(float &current)
{
    esp_err_t ret;

    xSemaphoreTake(_mutex, portMAX_DELAY);

    ret = _bq24.selectIOUT(IOUT_CHARGE);    // set IOUT to battery
    if (ret != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return ret;
    }
    ret = _analog->read(BQ24725A_IOUT_CHANNEL, current);    // read IOUT
    current = current / 20.0;   // IOUT is 20x battery current

    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t Charger::getAdapterCurrent(float &current)
{
    esp_err_t ret;

    xSemaphoreTake(_mutex, portMAX_DELAY);

    ret = _bq24.selectIOUT(IOUT_ADAPTER);   // set IOUT to adapter
    if (ret != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return ret;
    }
    ret = _analog->read(BQ24725A_IOUT_CHANNEL, current);    // read IOUT
    current = current / 20.0;   // IOUT is 20x adapter current

    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t Charger::getRobotCurrent(float &current)
{
    esp_err_t ret;
    float input, battery;

    ret = getAdapterCurrent(input); // get input
    if (ret != ESP_OK)
    {
        return ret;
    } 

    ret = getBatteryCurrent(battery);   // get battery
    if (ret != ESP_OK)
    {
        return ret;
    }

    current = input - battery;  // robot is difference between input and battery
    return ESP_OK;
}

Charger::Charger()
{
    _mutex = xSemaphoreCreateMutex();
    _analog = AnalogInput::instance();
}