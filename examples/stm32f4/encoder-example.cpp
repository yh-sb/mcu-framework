// Example for STM32F4DISCOVERY development board
// Rotary incremental encoder connected to pins PA7 and PA8

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "drivers/encoder.hpp"

static void encoder_task(void *pvParameters)
{
    drv::encoder *encoder = (drv::encoder *)pvParameters;
    
    while(1)
    {
        encoder->poll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int main(int argc, char *argv[])
{
    periph::systick::init();
    
    // Green LED
    periph::gpio_stm32f4 green_led(periph::gpio_stm32f4::port::d, 12, periph::gpio::mode::digital_output);
    
    // Rotary incremental encoder
    periph::gpio_stm32f4 encoder_pin_a(periph::gpio_stm32f4::port::a, 7, periph::gpio::mode::digital_input, 1);
    periph::gpio_stm32f4 encoder_pin_b(periph::gpio_stm32f4::port::a, 8, periph::gpio::mode::digital_input, 1);
    drv::encoder encoder(encoder_pin_a, encoder_pin_b);
    encoder.set_callback([&green_led](int8_t diff)
    {
        green_led.toggle();
    });
    
    xTaskCreate(encoder_task, "encoder", configMINIMAL_STACK_SIZE, &encoder, 1, nullptr);
    
    vTaskStartScheduler();
}
