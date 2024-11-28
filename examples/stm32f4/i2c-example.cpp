// Example for STM32F4DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/dma_stm32f4.hpp"
#include "periph/i2c_stm32f4.hpp"
#include "drivers/gpio_pin_debouncer.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    periph::i2c &i2c;
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
    periph::i2c &i2c1 = task_params->i2c;
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                uint8_t write_buff[] = {0xF0, 0xF1, 0xF2, 0xF3, 0xF4};
                uint8_t read_buff[5] {};
                
                // LIS302
                auto res = i2c1.write_read(11, write_buff, sizeof(write_buff), read_buff, sizeof(read_buff));
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
    
    // I2C1 interface
    periph::gpio_stm32f4 i2c1_sda(periph::gpio_stm32f4::port::b, 9, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 i2c1_scl(periph::gpio_stm32f4::port::b, 6, periph::gpio::mode::alternate_function);
    periph::dma_stm32f4 i2c1_write_dma(1, 6, 1, periph::dma_stm32f4::direction::memory_to_periph, 8);
    periph::dma_stm32f4 i2c1_read_dma(1, 0, 3, periph::dma_stm32f4::direction::periph_to_memory, 8);
    periph::i2c_stm32f4 i2c1(1, 100000, i2c1_write_dma, i2c1_read_dma, i2c1_sda, i2c1_scl);
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    task_params_t task_params = {button_1, i2c1};
    xTaskCreate(button_1_task, "button_1", configMINIMAL_STACK_SIZE, &task_params, 2, nullptr);
    
    vTaskStartScheduler();
}
