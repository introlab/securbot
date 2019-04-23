#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "command.h"


#define BLINK_PIN 13
#define BUF_SIZE 256


typedef union {
    float value;
    uint8_t bytes[sizeof(float)];
} test_packet_t;


void esp32_init();

void task_blink();
void task_handle_uart_events();

TaskHandle_t blink_task_handle;
QueueHandle_t uart0_queue;

uart_event_t uart_event;
uint8_t recv_buf[BUF_SIZE];
test_packet_t test_packet;
command_response_t response;


void app_main()
{
    esp32_init();
    xTaskCreate(task_blink, "Blink", 1024, NULL, 10, &blink_task_handle);
    xTaskCreate(task_handle_uart_events, "UART Events", 2048, NULL, 10, NULL);
}


void esp32_init()
{
    printf("Hello world!\n");

    // Init GPIO for blinking LED
    gpio_pad_select_gpio(BLINK_PIN);
    gpio_set_direction(BLINK_PIN, GPIO_MODE_OUTPUT);

    // Install UART0 driver with event queue
    uart_driver_install(UART_NUM_0, 2*BUF_SIZE, 2*BUF_SIZE, 10, &uart0_queue, 0);
}


void task_blink()
{
    gpio_set_level(BLINK_PIN, 0);

    while(1)
    {
        if(ulTaskNotifyTake(pdFALSE, portMAX_DELAY))
        {
            gpio_set_level(BLINK_PIN, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);

            gpio_set_level(BLINK_PIN, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}


void task_handle_uart_events()
{
    while(1){
        if(xQueueReceive(uart0_queue, &uart_event, portMAX_DELAY))
        {
            bzero(recv_buf, BUF_SIZE);
            switch (uart_event.type)
            {
                case UART_DATA:
                    xTaskNotifyGive(blink_task_handle);
                    uart_read_bytes(UART_NUM_0, recv_buf, uart_event.size, portMAX_DELAY);
                    command_performCommand(recv_buf, uart_event.size, &response);
                    uart_write_bytes(UART_NUM_0, (char*)response.data, response.length);
                    printf("\nProcessed event\n");
                    break;
            
                default:
                    printf("Unhandled UART event type: %d\n", uart_event.type);
                    break;
            }
        }
    }
}