// Example for RP2040 development board

#include <stdio.h>
#include "pico/stdio_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "periph/gpio_rp2040.hpp"

static void heartbeat_task(void *pvParameters)
{
    periph::gpio *led = (periph::gpio *)pvParameters;
    while(1)
    {
        led->toggle();
        vTaskDelay(500);
    }
}

int main(int argc, char *argv[])
{
    // Green LED
    periph::gpio_rp2040 led(25, periph::gpio::mode::digital_output);
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &led, 1, nullptr);
    vTaskStartScheduler();
    
    return 0;
}
