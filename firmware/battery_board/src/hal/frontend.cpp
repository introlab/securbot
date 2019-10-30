/**
 * @file frontend.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Hardware abstraction for the battery board analog frontend
 * @version 0.1
 * @date 2019-10-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "hal/frontend.hpp"

/**
 * @brief frontend anonymous namespace.
 * Namespace to hide global variables related to the analog frontend
 */
namespace
{
    /**
     * @brief logging tag.
     * Tag to use when logging from the analog frontend
     */
    const char* TAG = "Frontend";
}

Frontend* Frontend::_instance = NULL;

Frontend* Frontend::instance()
{
    if (_instance == NULL)
    {
        _instance = new Frontend();
    }
    return _instance;
}

Frontend::Frontend()
{
    _mutex = xSemaphoreCreateMutex();
    _analog = AnalogInput::instance();
}

esp_err_t Frontend::begin()
{
    esp_err_t ret;
    uint8_t id;
    
    // set alert pin as input
    gpio_pad_select_gpio(BQ76925PWR_ALERT_GPIO);
    gpio_set_direction((gpio_num_t)BQ76925PWR_ALERT_GPIO, GPIO_MODE_INPUT);

    xSemaphoreTake(_mutex, portMAX_DELAY);

    // Read the chip id
    ret = _bq76.readChipId(id);
    ESP_LOGI(TAG, "BQ76925 chip id is 0x%02x", id);
    if (ret != ESP_OK)
    {
        
        xSemaphoreGive(_mutex);
        return ret;
    }

    ret = _bq76.configure();          // configure front end chip
    if (ret != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return ret;
    }

    ret = _bq76.setMonitorMode(1, 0); // default to monitoring discharge

    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t Frontend::getBatteryCurrent(float &current)
{
    esp_err_t ret;
    float viout;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    ret = _analog->read(VIOUT_BMS_CHANNEL, viout);
    xSemaphoreGive(_mutex);

    current = (viout-2.03604) / (0.017 * 4.0);

    return ret;
}

esp_err_t Frontend::getBatteryVoltage(float &voltage)
{
    return readCell(5, voltage);
}

esp_err_t Frontend::setCurrentPolarity(uint8_t charging)
{
    esp_err_t ret;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    ret = _bq76.setMonitorMode(1, charging);
    xSemaphoreGive(_mutex);

    return ret;
}

int Frontend::getOvercurrentAlert()
{
    return gpio_get_level((gpio_num_t)BQ76925PWR_ALERT_GPIO);
}

esp_err_t Frontend::getCellsVoltage(float voltage[4])
{
    esp_err_t ret;
    uint8_t cells[4] = {0, 1, 2, 5};

    xSemaphoreTake(_mutex, portMAX_DELAY);

    for (uint8_t i = 0; i < 4; i++)
    {
        ret = readCell(cells[i], voltage[i]);
        if (ret != ESP_OK)
        {
            xSemaphoreGive(_mutex);
            return ret;
        }
        ESP_LOGI(TAG, "Cell %d is %4.2fV at VC%d", i+1, voltage[i], cells[i]+1);
    }

    xSemaphoreGive(_mutex);
    return ESP_OK;
}

esp_err_t Frontend::getBoardTemperature(float temperature[2])
{
    esp_err_t ret;
    float voltage[2];

    xSemaphoreTake(_mutex, portMAX_DELAY);

    ret = _analog->read(THERMISTOR_R10_CHANNEL, voltage[0]);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return ret;
    }

    ret = _analog->read(THERMISTOR_R11_CHANNEL, voltage[1]);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return ret;
    }

    xSemaphoreGive(_mutex);
    
    for (uint8_t i = 0; i < 2; i++) // convert voltage to temperature
    {
        temperature[i] = -(voltage[i] - 2.10) * 41.4 + 22;
        ESP_LOGI(TAG, "Temp %d is %05.2fC", i+1, temperature[i]);
    }

    return ESP_OK;
}

esp_err_t Frontend::setBalance(uint8_t cell)
{
    esp_err_t ret;

    BQ76925::bal_ctl_register_map balance;

    // Convert cell number to proper bit in balance control register
    if (cell == 0)
    {
        balance.bytes[0] = 0;
    }
    else
    {
        balance.bytes[0] = 0b1 << (cell-1);
    }

    xSemaphoreTake(_mutex, portMAX_DELAY);
    ret = _bq76.balanceCell(balance);
    xSemaphoreGive(_mutex);

    return ret;
}

esp_err_t Frontend::readCell(uint8_t num, float &voltage)
{
    esp_err_t ret;
    float vcout;

    // Select the cell on VCOUT
    ret = _bq76.selectCell(num);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return ret;
    }

    // Read VCOUT
    ret = _analog->read(VCOUT_BMS_CHANNEL, vcout);

    // VCOUT is VREF (0.6) * VCell
    voltage = vcout / 0.6;

    return ret;
}