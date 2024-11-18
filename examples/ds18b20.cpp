// Example for STM32F4DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/dma_stm32f4.hpp"
#include "periph/uart_stm32f4.hpp"
#include "drivers/gpio_pin_debouncer.hpp"
#include "drivers/ds18b20.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    drv::ds18b20 &ds18b20;
    periph::gpio &led;
};

static void button_1_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    drv::gpio_pin_debouncer &button_1 = task_params->button;
    drv::ds18b20 &ds18b20 = task_params->ds18b20;
    periph::gpio &green_led = task_params->led;
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                green_led.toggle();
                
                float temperature;
                auto res = ds18b20.get_temperature(0, temperature);
            }
        }
        vTaskDelay(1);
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
    
    // DS18B20 sensor
    periph::gpio_stm32f4 uart_tx(periph::gpio_stm32f4::port::d, 8, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 uart_rx(periph::gpio_stm32f4::port::b, 11, periph::gpio::mode::alternate_function);
    periph::dma_stm32f4 uart3_tx_dma(1, 3, 4, periph::dma_stm32f4::direction::memory_to_periph, 8);
    periph::dma_stm32f4 uart3_rx_dma(1, 1, 4, periph::dma_stm32f4::direction::periph_to_memory, 8);
    periph::uart_stm32f4 uart3(3, 115200, periph::uart_stm32f4::stopbits::stopbits_1,
        periph::uart_stm32f4::parity::none, uart3_tx_dma, uart3_rx_dma, uart_tx, uart_rx);
    drv::onewire onewire(uart3);
    drv::ds18b20 ds18b20(onewire);
    
    task_params_t task_params = {button1_di, ds18b20, green_led};
    xTaskCreate(button_1_task, "button_1_task", configMINIMAL_STACK_SIZE, &task_params, 1, nullptr);
    
    vTaskStartScheduler();
    
    return 0;
}
