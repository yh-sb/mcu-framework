// Example for STM32F4DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/timer_stm32f4.hpp"
#include "periph/dac_stm32f4.hpp"

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
    
    // DAC1
    periph::gpio_stm32f4 dac1_gpio(periph::gpio_stm32f4::port::a, 4, periph::gpio::mode::analog);
    periph::dac_stm32f4 dac1(1, periph::dac_stm32f4::align::right_12, dac1_gpio);
    
    // Timer for DAC to generate sine wave
    periph::timer_stm32f4 timer6(6);
    timer6.set_callback([&dac1]() {
        static const uint16_t sin[] =
        {
            2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056,
            4095, 4056, 3939, 3750, 3495, 3185, 2831, 2447,
            2047, 1647, 1263,  909,  599,  344,  155,   38,
               0,   38,  155,  344,  599,  909, 1263, 1647
        };
        static uint8_t cnt = 0;
        
        dac1.set(sin[cnt++]);
        if(cnt == sizeof(sin) / sizeof(sin[0]))
        {
            cnt = 0;
        }
    });
    timer6.timeout(std::chrono::microseconds(32)); // Sine wave frequency is 1 kHz (sin[32])
    timer6.start(true);
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    vTaskStartScheduler();
}
