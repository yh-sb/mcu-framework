// Example for STM32VLDISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f1.hpp"
#include "periph/wdt.hpp"

static void heartbeat_task(void *pvParameters)
{
    periph::gpio *green_led = (periph::gpio *)pvParameters;
    while(1)
    {
        periph::wdt::reload();
        
        green_led->toggle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(int argc, char *argv[])
{
    periph::systick::init();
    
    periph::wdt::timeout(std::chrono::milliseconds(1000));
    periph::wdt::start();
    
    // Green LED
    periph::gpio_stm32f1 green_led(periph::gpio_stm32f1::port::c, 9, periph::gpio::mode::digital_output);
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    vTaskStartScheduler();
}
