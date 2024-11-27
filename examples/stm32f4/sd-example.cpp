// Example for STM32F4DISCOVERY development board
// SD card connected to SPI1 interface (PA7-MOSI, PA6-MISO, PA5-CLK, PA4-CS, PA3-CD)

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/dma_stm32f4.hpp"
#include "periph/spi_stm32f4.hpp"
#include "drivers/gpio_pin_debouncer.hpp"
#include "drivers/sd_spi.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    drv::sd &sd;
    periph::gpio &led;
};

static void button_1_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    drv::gpio_pin_debouncer &button_1 = task_params->button;
    drv::sd &sd = task_params->sd;
    periph::gpio &green_led = task_params->led;
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                green_led.toggle();
                
                auto res = sd.init();
                if(res != drv::sd::res::ok)
                {
                    drv::sd_regs::csd_t csd;
                    res = sd.read_csd(&csd);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int main(int argc, char *argv[])
{
    periph::systick::init();
    
    // Green LED
    periph::gpio_stm32f4 green_led(periph::gpio_stm32f4::port::d, 12, periph::gpio::mode::digital_output, 1);
    
    // Button 1
    periph::gpio_stm32f4 button_1(periph::gpio_stm32f4::port::a, 0, periph::gpio::mode::digital_input);
    drv::gpio_pin_debouncer button1_di(button_1, std::chrono::milliseconds(50), 1);
    
    // SD card over SPI interface
    periph::gpio_stm32f4 spi1_mosi(periph::gpio_stm32f4::port::a, 7, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 spi1_miso(periph::gpio_stm32f4::port::a, 6, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 spi1_clk(periph::gpio_stm32f4::port::a, 5, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 spi1_cs(periph::gpio_stm32f4::port::a, 4, periph::gpio::mode::digital_output, 1);
    periph::gpio_stm32f4 spi1_cd(periph::gpio_stm32f4::port::a, 3, periph::gpio::mode::digital_input, 1);
    periph::dma_stm32f4 spi1_read_dma(2, 0, 3, periph::dma_stm32f4::direction::periph_to_memory, 8);
    periph::dma_stm32f4 spi1_write_dma(2, 3, 3, periph::dma_stm32f4::direction::memory_to_periph, 8);
    periph::spi_stm32f4 spi1(1, 1000000, periph::spi::cpol::low, periph::spi::cpha::leading,
        periph::spi::bit_order::msb, spi1_write_dma, spi1_read_dma, spi1_mosi, spi1_miso, spi1_clk);
    drv::sd_spi sd_spi_1(spi1, spi1_cs, &spi1_cd);
    
    task_params_t task_params = {button1_di, sd_spi_1, green_led};
    xTaskCreate(button_1_task, "button_1_task", configMINIMAL_STACK_SIZE, &task_params, 1, nullptr);
    
    vTaskStartScheduler();
    
    return 0;
}
