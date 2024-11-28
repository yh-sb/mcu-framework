// Example for STM32F072DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f0.hpp"
#include "periph/exti_stm32f0.hpp"

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
    
    // External interrupt gpio (Button)
    periph::gpio_stm32f0 button1_gpio(periph::gpio_stm32f0::port::a, 0, periph::gpio::mode::digital_input, 0);
    
    periph::exti_stm32f0 exti0(button1_gpio, periph::exti::trigger::both);
    exti0.set_callback([&blue_led]() { blue_led.toggle(); });
    exti0.enable();
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    vTaskStartScheduler();
}
