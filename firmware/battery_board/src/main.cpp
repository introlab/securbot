/**
 * @file main.cpp
 * @author Cédric Godin (cedric.godin@me.com)
 * @brief Battery Board firmware main file
 * @version 0.1
 * @date 2019-10-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#include "defines.hpp"

#include "hal/i2c.hpp"
#include "hal/analogInput.hpp"
#include "hal/switches.hpp"
#include "hal/charger.hpp"
#include "hal/frontend.hpp"

#include "monitor.hpp"
#include "control.hpp"

/**
 * @brief main program private namespace.
 * Private global definitions used by the main file
 */
namespace
{
    /**
     * @brief main login tag.
     * Tag to use when logging from the main task
     */
    const char* TAG = "Main";
}

/**
 * @brief Blink LED task function
 * Blink the onboard LED to show CPU is alive and correctly initialized
 * @param pvParameters not used
 */
void blink_task_fn( void *pvParameters )
{
    while (1)
    {
        // turn off
        gpio_set_level((gpio_num_t)ONBOARD_LED_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // turn on
        gpio_set_level((gpio_num_t)ONBOARD_LED_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Initialize hardware
 * Call begin on each peripheral. Must be called before any task is started
 */
void hardware_init(void)
{
    esp_err_t ret;

    // Initialize LED GPIO
    gpio_pad_select_gpio(ONBOARD_LED_PIN);
    ret = gpio_set_direction((gpio_num_t)ONBOARD_LED_PIN, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s setting onboard led pin", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Onboard LED initialized");
    }
    
    // Initialize ADCs i2c driver
    ret = I2C::instance(ADS1015_I2C_NUM)->begin();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s initializing ADC i2c bus", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "ADC i2c bus initialized");
    }
    

    // Initialize BQ24725A chip i2c driver
    ret = I2C::instance(BQ24725A_I2C_NUM)->begin();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s initializing BQ24725A i2c bus", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "BQ24725A i2c bus initialized");
    }

    // Initialize analog inputs driver
    ret = AnalogInput::instance()->begin();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s initializing analog inputs", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Analog inputs initialized");
    }

    // Initialize power switches
    ret = Switches::instance()->begin();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s initializing power switches", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Power switches initialized");
    }

    // Initialize front end
    ret = Frontend::instance()->begin();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s initializing analog front end", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Analog front end initialized");
    }

    // Initialize charger
    ret = Charger::instance()->begin();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s initializing battery charger", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Battery charger initialized");
    }
}

/**
 * @brief Battery board firmware main function
 * Initialize hardware then start each task to get the board up and running
 */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Hello World!");

    // Prepare hardware
    ESP_LOGI(TAG, "Initializing hardware");
    hardware_init();
    ESP_LOGI(TAG, "Hardware initialization complete");
    ESP_LOGI(TAG, "Starting tasks");

    // Starts monitor
    xTaskCreate(monitor::monitorTask_fn, "Monitor", 2048, NULL, 1, NULL);
    ESP_LOGI(TAG, "Monitoring task started");

    // Starts serial
    //ESP_LOGI(TAG, "Serial task started");
    
    // Starts control
    TaskHandle_t controlHandle;
    xTaskCreate(control::controlTask_fn, "Control", 2048, NULL, 1, &controlHandle);
    monitor::smashThatSubscribeButton(controlHandle);
    ESP_LOGI(TAG, "Control task started");
    
    // Start blinky
    xTaskCreate(blink_task_fn, "Blinky", 512, NULL, 0, NULL);
    ESP_LOGI(TAG, "Blink task started");

    ESP_LOGI(TAG, "Boot complete");
}