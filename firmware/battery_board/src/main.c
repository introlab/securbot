#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#include "defines.h"

void blink_task_fn( void *pvParameters )
{
    while (1)
    {
        gpio_set_level(ONBOARD_LED_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(ONBOARD_LED_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void hardware_init(void)
{
    gpio_pad_select_gpio(ONBOARD_LED_PIN);
    gpio_set_direction((gpio_num_t)ONBOARD_LED_PIN, GPIO_MODE_OUTPUT);
}

void app_main(void)
{
    hardware_init();
    printf("Hello World!\n");
    
    xTaskCreate(blink_task_fn, "Blinky", 512, NULL, 0, NULL);
}