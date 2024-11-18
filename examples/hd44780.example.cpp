// Example for STM32F4DISCOVERY development board
// HD44780 display connected to the GPIOA pins 3, 4, 5, 6, 7, 8, 10

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/timer_stm32f4.hpp"
#include "drivers/gpio_pin_debouncer.hpp"
#include "drivers/hd44780.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    drv::hd44780 &hd44780;
    periph::gpio &led;
};

static void button_1_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    drv::gpio_pin_debouncer &button_1 = task_params->button;
    drv::hd44780 &hd44780 = task_params->hd44780;
    periph::gpio &green_led = task_params->led;
    
    hd44780.init();
    hd44780.print(0, "Test");
    
    uint8_t cgram[8][8] =
    {
        {
            0b00011000,
            0b00001110,
            0b00000110,
            0b00000111,
            0b00000111,
            0b00000110,
            0b00001100,
            0b00011000
        }
    };
    hd44780.write_cgram(cgram);
    
    hd44780.print(64, char(0)); // Goto the line 2 and print custom symbol
    hd44780.print(20, "Line 3");
    hd44780.print(84, "Line 4");
    
    while(1)
    {
        bool new_state;
        uint8_t cnt = 0;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                green_led.toggle();
                
                cnt++;
                hd44780.print(0, "Test %d", cnt);
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
    
    // HD44780 display
    periph::gpio_stm32f4 rs(periph::gpio_stm32f4::port::a, 5, periph::gpio::mode::digital_output);
    periph::gpio_stm32f4 rw(periph::gpio_stm32f4::port::a, 4, periph::gpio::mode::digital_output);
    periph::gpio_stm32f4 e(periph::gpio_stm32f4::port::a, 3, periph::gpio::mode::digital_output);
    periph::gpio_stm32f4 db4(periph::gpio_stm32f4::port::a, 6, periph::gpio::mode::digital_output);
    periph::gpio_stm32f4 db5(periph::gpio_stm32f4::port::a, 7, periph::gpio::mode::digital_output);
    periph::gpio_stm32f4 db6(periph::gpio_stm32f4::port::a, 8, periph::gpio::mode::digital_output);
    periph::gpio_stm32f4 db7(periph::gpio_stm32f4::port::a, 10, periph::gpio::mode::digital_output);
    periph::timer_stm32f4 hd44780_timer(6);
    drv::hd44780 hd44780(rs, rw, e, db4, db5, db6, db7, hd44780_timer);
    
    task_params_t task_params = {button1_di, hd44780, green_led};
    xTaskCreate(button_1_task, "button_1_task", configMINIMAL_STACK_SIZE, &task_params, 1, nullptr);
    
    vTaskStartScheduler();
    
    return 0;
}
