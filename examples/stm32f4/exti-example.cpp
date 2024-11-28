// Example for STM32F4DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/exti_stm32f4.hpp"

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
    periph::gpio_stm32f4 green_led(periph::gpio_stm32f4::port::d, 12, periph::gpio::mode::digital_output);
    
    // Blue LED
    periph::gpio_stm32f4 blue_led(periph::gpio_stm32f4::port::d, 15, periph::gpio::mode::digital_output);
    
    // External interrupt gpio (Button)
    periph::gpio_stm32f4 button1_gpio(periph::gpio_stm32f4::port::a, 0, periph::gpio::mode::digital_input, 0);
    
    periph::exti_stm32f4 exti1(button1_gpio, periph::exti::trigger::both);
    exti1.set_callback([&blue_led]()
    {
        blue_led.toggle();
    });
    exti1.enable();
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    vTaskStartScheduler();
}
