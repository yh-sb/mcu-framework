// Example for STM32VLDISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f1.hpp"
#include "periph/adc_stm32f1.hpp"
#include "periph/dma_stm32f1.hpp"

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
    
    // ADC1
    periph::gpio_stm32f1 adc1_gpio(periph::gpio_stm32f1::port::a, 0, periph::gpio::mode::analog);
    periph::dma_stm32f1 adc1_dma(1, 1, periph::dma_stm32f1::direction::periph_to_memory, 16);
    periph::adc_cyclic_stm32f1 adc1(1, {0}, periph::adc_cyclic_stm32f1::resolution::_12_bit, 3, adc1_dma, 3200, 16);
    adc1.set_callback(0, [](double voltage) {
        // Handle adc value here
    });
    adc1.start();
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    vTaskStartScheduler();
}
