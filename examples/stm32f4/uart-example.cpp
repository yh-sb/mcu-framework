// Example for STM32F4DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/dma_stm32f4.hpp"
#include "periph/uart_stm32f4.hpp"
#include "drivers/gpio_pin_debouncer.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    periph::uart &uart;
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
    periph::uart &uart3 = task_params->uart;
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                char tx_buff[] = "test";
                char rx_buff[4] {};
                uint16_t rx_size = sizeof(rx_buff);
                
                auto res = uart3.write_read(tx_buff, sizeof(tx_buff) - 1, rx_buff, &rx_size);
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
    
    // UART3 interface
    periph::gpio_stm32f4 uart3_tx(periph::gpio_stm32f4::port::d, 8, periph::gpio::mode::alternate_function);
    periph::gpio_stm32f4 uart3_rx(periph::gpio_stm32f4::port::b, 11, periph::gpio::mode::alternate_function);
    periph::dma_stm32f4 uart3_tx_dma(1, 3, 4, periph::dma_stm32f4::direction::memory_to_periph, 8);
    periph::dma_stm32f4 uart3_rx_dma(1, 1, 4, periph::dma_stm32f4::direction::periph_to_memory, 8);
    periph::uart_stm32f4 uart3(3, 115200, periph::uart_stm32f4::stopbits::stopbits_1, periph::uart_stm32f4::parity::none,
        uart3_tx_dma, uart3_rx_dma, uart3_tx, uart3_rx);
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    task_params_t task_params = {button_1, uart3};
    xTaskCreate(button_1_task, "button_1", configMINIMAL_STACK_SIZE, &task_params, 2, nullptr);
    
    vTaskStartScheduler();
}
