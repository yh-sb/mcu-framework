// Example for STM32F072DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f0.hpp"
#include "periph/timer_stm32f0.hpp"

static void heartbeat_task(void *pvParameters)
{
    periph::gpio *green_led = (periph::gpio *)pvParameters;
    while(1)
    {
        green_led->toggle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(int argc, char *argv[])
{
    periph::systick::init();
    
    // Green LED
    periph::gpio_stm32f0 green_led(periph::gpio_stm32f0::port::c, 9, periph::gpio::mode::digital_output);
    
    // Blue LED
    periph::gpio_stm32f0 blue_led(periph::gpio_stm32f0::port::c, 7, periph::gpio::mode::digital_output);
    
    periph::timer_stm32f0 timer6(6);
    timer6.timeout(std::chrono::microseconds(400));
    timer6.set_callback([&blue_led](){ blue_led.toggle(); });
    timer6.start(true);
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    vTaskStartScheduler();
}
