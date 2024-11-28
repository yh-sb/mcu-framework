// Example for STM32F072DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f0.hpp"
#include "periph/dma_stm32f0.hpp"
#include "periph/spi_stm32f0.hpp"
#include "drivers/gpio_pin_debouncer.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    periph::spi &spi;
    periph::gpio &spi_cs;
    periph::gpio &led;
};

static void button_1_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    drv::gpio_pin_debouncer &button_1 = task_params->button;
    periph::spi &spi1 = task_params->spi;
    periph::gpio &spi_cs = task_params->spi_cs;
    periph::gpio &green_led = task_params->led;
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                green_led.toggle();
                
                uint8_t buff[] = {1, 2, 3, 4, 5};
                auto res = spi1.write(buff, sizeof(buff), &spi_cs);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int main(int argc, char *argv[])
{
    periph::systick::init();
    
    // Green LED
    periph::gpio_stm32f0 green_led(periph::gpio_stm32f0::port::c, 9, periph::gpio::mode::digital_output, 1);
    
    // Button 1
    periph::gpio_stm32f0 button_1_gpio(periph::gpio_stm32f0::port::a, 0, periph::gpio::mode::digital_input);
    drv::gpio_pin_debouncer button_1(button_1_gpio, std::chrono::milliseconds(50), 1);
    
    // SPI1 interface
    periph::gpio_stm32f0 spi1_mosi(periph::gpio_stm32f0::port::b, 5, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f0 spi1_miso(periph::gpio_stm32f0::port::b, 4, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f0 spi1_clk(periph::gpio_stm32f0::port::b, 3, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f0 spi1_cs(periph::gpio_stm32f0::port::b, 8, periph::gpio::mode::digital_output, 1);
    periph::dma_stm32f0 spi1_write_dma(1, 3, periph::dma_stm32f0::direction::memory_to_periph, 8);
    periph::dma_stm32f0 spi1_read_dma(1, 2, periph::dma_stm32f0::direction::periph_to_memory, 8);
    periph::spi_stm32f0 spi1(1, 1000000, periph::spi::cpol::low, periph::spi::cpha::leading,
        periph::spi::bit_order::msb, spi1_write_dma, spi1_read_dma, spi1_mosi, spi1_miso, spi1_clk);
    
    task_params_t task_params = {button_1, spi1, spi1_cs, green_led};
    xTaskCreate(button_1_task, "button_1", configMINIMAL_STACK_SIZE, &task_params, 1, nullptr);
    
    vTaskStartScheduler();
}
