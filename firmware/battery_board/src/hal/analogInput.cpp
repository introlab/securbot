/**
 * @file analogInput.cpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Driver to read analog inputs
 * @version 0.1
 * @date 2019-10-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "hal/analogInput.hpp"

/**
 * @brief Analog input anonymous namespace.
 * Namespace to hide analog input global variables
 */
namespace
{
    /**
     * @brief Analog input log tag.
     * Tag to use when logging from the analog input file
     */
    const char* TAG = "Analog Input";
}

AnalogInput* AnalogInput::_instance = NULL;

AnalogInput* AnalogInput::instance()
{
    // Create an instance if it is the first call
    if (_instance == NULL)
    {
        _instance = new AnalogInput();
    }
    return _instance;
}

esp_err_t AnalogInput::begin()
{
    esp_err_t ret;
    ADS1015::config_register_map config;

    // Print ADC 0 default configuration
    ret = _adcs[0]->getConfig(config);
    if (ret!=ESP_OK)
    {
        return ret;
    }
    ESP_LOGI(TAG, "ADC 0 default config 0x%02x%02x", config.bytes[1], config.bytes[0]);

    // Print ADC 1 default configuration
    ret = _adcs[1]->getConfig(config);
    if (ret!=ESP_OK)
    {
        return ret;
    }
    ESP_LOGI(TAG, "ADC 1 default config 0x%02x%02x", config.bytes[1], config.bytes[0]);

    // Configure ADC 0
    ret = _adcs[0]->configure();
    if (ret!=ESP_OK)
    {
        return ret;
    }

    // Configure ADC 1
    return _adcs[1]->configure();
}

esp_err_t AnalogInput::read(uint8_t channel_number, float &value)
{
    esp_err_t ret;

    // Which adc doest the channel belong to
    uint8_t adc_number = channel_number / 4;

    // Adjust the channel number
    channel_number = channel_number - 4*adc_number;

    // One ADC can only read one input at the time so it is locked
    xSemaphoreTake(_mutex[adc_number], portMAX_DELAY);

    // Enable ADC for the desired channel
    ret = _adcs[adc_number]->startReading(channel_number);
    if (ret != ESP_OK)  // start reading fail. We unlock and return.
    {
        xSemaphoreGive(_mutex[adc_number]);
        return ret;
    }
    
    bool reading = true;
    do  // Poll ADC to check for reading completion
    {
        vTaskDelay(100 / portTICK_RATE_MS);
        ret = _adcs[adc_number]->isReading(reading);
        if (ret != ESP_OK)  // cannot check status. give up.
        {
            xSemaphoreGive(_mutex[adc_number]);
            return ret;
        }

    } while (reading);
    
    // Retrieve value from ADC
    ret = _adcs[adc_number]->getValue(value);

    // unlock the ADC
    xSemaphoreGive(_mutex[adc_number]);

    return ret;
}

AnalogInput::AnalogInput()
{
    // Get ADC instances
    _adcs[0] = new ADS1015(ADS1015_I2C_NUM, ADC0_I2C_ADDRESS);
    _adcs[1] = new ADS1015(ADS1015_I2C_NUM, ADC1_I2C_ADDRESS);

    // Create ADC mutexes
    _mutex[0] = xSemaphoreCreateMutex();
    _mutex[1] = xSemaphoreCreateMutex();
}