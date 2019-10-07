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

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#include "defines.hpp"

#include "hal/i2c.hpp"
#include "hal/analogInput.hpp"

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
    // Confure LED GPIO
    gpio_pad_select_gpio(ONBOARD_LED_PIN);
    gpio_set_direction((gpio_num_t)ONBOARD_LED_PIN, GPIO_MODE_OUTPUT);
    
    // Initialize ADCs i2c driver
    I2C* i2c = I2C::instance(ADS1015_I2C_NUM);
    i2c->begin();

    // Initialize analog inputs driver
    AnalogInput* analogIn = AnalogInput::instance();
    analogIn->begin();
}

/**
 * @brief Battery board firmware main function
 * Initialize hardware then start each task to get the board up and running
 */
extern "C" void app_main(void)
{
    // Prepare hardware
    hardware_init();
    printf("Hello World!\n");
    
    // Start blinky
    xTaskCreate(blink_task_fn, "Blinky", 512, NULL, 0, NULL);
}