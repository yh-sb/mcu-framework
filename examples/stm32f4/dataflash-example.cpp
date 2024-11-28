// Example for STM32F4DISCOVERY development board
// SD card connected to SPI1: PA7-MOSI, PA6-MISO, PA5-CLK, PA4-CS

#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/dma_stm32f4.hpp"
#include "periph/spi_stm32f4.hpp"
#include "drivers/gpio_pin_debouncer.hpp"
#include "drivers/dataflash.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    drv::dataflash &dataflash;
    periph::gpio &led;
};

static void button_1_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    drv::gpio_pin_debouncer &button_1 = task_params->button;
    drv::dataflash &at45db = task_params->dataflash;
    periph::gpio &green_led = task_params->led;
    
    auto res = at45db.init();
    drv::dataflash::info_t info = at45db.info();
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                green_led.toggle();
                
                uint8_t buff[info.page_size] = {0x01, 0x02, 0x03, 0x04, 0x05};
                res = at45db.write(buff, 1);
                memset(buff, 0, info.page_size);
                res = at45db.read(buff, 1);
                res = at45db.erase(1);
                res = at45db.read(buff, 1);
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
    periph::gpio_stm32f4 button1_gpio(periph::gpio_stm32f4::port::a, 0, periph::gpio::mode::digital_input);
    drv::gpio_pin_debouncer button_1(button_1_gpio, std::chrono::milliseconds(50), 1);
    
    // AT45DB dataflash chip
    periph::gpio_stm32f4 spi1_mosi(periph::gpio_stm32f4::port::a, 7, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 spi1_miso(periph::gpio_stm32f4::port::a, 6, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 spi1_clk(periph::gpio_stm32f4::port::a, 5, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 spi1_cs(periph::gpio_stm32f4::port::a, 4, periph::gpio::mode::digital_output, 1);
    periph::dma_stm32f4 spi1_read_dma(2, 0, 3, periph::dma_stm32f4::direction::periph_to_memory, 8);
    periph::dma_stm32f4 spi1_write_dma(2, 3, 3, periph::dma_stm32f4::direction::memory_to_periph, 8);
    periph::spi_stm32f4 spi1(1, 1000000, periph::spi::cpol::low, periph::spi::cpha::leading,
        periph::spi::bit_order::msb, spi1_write_dma, spi1_read_dma, spi1_mosi, spi1_miso, spi1_clk);
    drv::dataflash at45db(spi1, spi1_cs);
    
    task_params_t task_params = {button_1, at45db, green_led};
    xTaskCreate(button_1_task, "button_1", configMINIMAL_STACK_SIZE, &task_params, 1, nullptr);
    
    vTaskStartScheduler();
}
