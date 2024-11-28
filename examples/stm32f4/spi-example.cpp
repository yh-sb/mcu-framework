// Example for STM32F4DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/dma_stm32f4.hpp"
#include "periph/spi_stm32f4.hpp"
#include "drivers/gpio_pin_debouncer.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    periph::spi &spi;
    periph::gpio &spi_cs;
};

static void heartbeat_task(void *pvParameters)
{
    periph::gpio *green_led = (periph::gpio *)pvParameters;
    while(1)
    {
        green_led->toggle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void button_1_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    drv::gpio_pin_debouncer &button_1 = task_params->button;
    periph::spi &spi1 = task_params->spi;
    periph::gpio &spi1_cs = task_params->spi_cs;
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                uint8_t buff[] = {0xF0, 0xF1, 0xF2, 0xF3, 0xF4};
                auto res = spi1.write(buff, sizeof(buff), &spi1_cs);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int main(int argc, char *argv[])
{
    periph::systick::init();
    
    // Green LED
    periph::gpio_stm32f4 green_led(periph::gpio_stm32f4::port::d, 12, periph::gpio::mode::digital_output);
    
    // Button 1
    periph::gpio_stm32f4 button_1_gpio(periph::gpio_stm32f4::port::a, 0, periph::gpio::mode::digital_input);
    drv::gpio_pin_debouncer button_1(button_1_gpio, std::chrono::milliseconds(50), 1);
    
    // SPI1 interface
    periph::gpio_stm32f4 spi1_mosi(periph::gpio_stm32f4::port::a, 7, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f4 spi1_miso(periph::gpio_stm32f4::port::a, 6, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f4 spi1_clk(periph::gpio_stm32f4::port::a, 5, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f4 spi1_cs(periph::gpio_stm32f4::port::e, 3, periph::gpio::mode::digital_output, 1);
    periph::dma_stm32f4 spi1_write_dma(2, 3, 3, periph::dma_stm32f4::direction::memory_to_periph, 8);
    periph::dma_stm32f4 spi1_read_dma(2, 0, 3, periph::dma_stm32f4::direction::periph_to_memory, 8);
    periph::spi_stm32f4 spi1(1, 10000000, periph::spi::cpol::low, periph::spi::cpha::leading,
        periph::spi::bit_order::msb, spi1_write_dma, spi1_read_dma, spi1_mosi, spi1_miso, spi1_clk);
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    task_params_t task_params = {button_1, spi1, spi1_cs};
    xTaskCreate(button_1_task, "button_1", configMINIMAL_STACK_SIZE, &task_params, 2, nullptr);
    
    vTaskStartScheduler();
}
