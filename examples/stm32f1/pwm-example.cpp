// Example for STM32VLDISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f1.hpp"
#include "periph/timer_stm32f1.hpp"
#include "periph/pwm_stm32f1.hpp"

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
    periph::gpio_stm32f1 green_led(periph::gpio_stm32f1::port::c, 9, periph::gpio::mode::digital_output);
    
    // Blue LED
    periph::gpio_stm32f1 blue_led(periph::gpio_stm32f1::port::c, 8, periph::gpio::mode::alternate_function);
    
    periph::pwm_stm32f1 pwm3_ch3(3, 3, blue_led);
    pwm3_ch3.frequency(100000);
    pwm3_ch3.duty_cycle(20);
    pwm3_ch3.start();
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    vTaskStartScheduler();
}
