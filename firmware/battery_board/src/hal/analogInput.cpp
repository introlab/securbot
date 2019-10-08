/**
 * @file analogInput.hpp
 * @author Cedric Godin (cedric.godin@me.com)
 * @brief Driver to read analog inputs
 * @version 0.1
 * @date 2019-10-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "hal/analogInput.hpp"

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
    _adcs[0]->configure();
    _adcs[1]->configure();

    return ESP_OK;  // no actual error management for now
}

esp_err_t AnalogInput::read(uint8_t channel_number, double &value)
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