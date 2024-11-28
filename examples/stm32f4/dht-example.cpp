// Example for STM32F4DISCOVERY development board
// PA7 and PA10 pins connected together and go to DHT11 sensor

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "periph/timer_stm32f4.hpp"
#include "periph/exti_stm32f4.hpp"
#include "drivers/gpio_pin_debouncer.hpp"
#include "drivers/dht.hpp"

struct task_params_t
{
    drv::gpio_pin_debouncer &button;
    drv::dht &dht;
    periph::gpio &led;
};

static void button_1_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    drv::gpio_pin_debouncer &button_1 = task_params->button;
    drv::dht &dht = task_params->dht;
    periph::gpio &green_led = task_params->led;
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                green_led.toggle();
                
                drv::dht::value_t val;
                auto res = dht.read(val);
                if(res == drv::dht::res::ok)
                {
                    /* printf("rh %02d,%d %%", val.rh_x10 / 10, val.rh_x10 % 10);
                    char sign = (val.t_x10 >= 0) ? '+' : '-';
                    printf("t %c%02d,%d C", sign, val.t_x10 / 10, val.t_x10 % 10); */
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
    periph::gpio_stm32f4 button_1_gpio(periph::gpio_stm32f4::port::a, 0, periph::gpio::mode::digital_input);
    drv::gpio_pin_debouncer button_1(button_1_gpio, std::chrono::milliseconds(50), 1);
    
    // DHT11 sensor
    periph::gpio_stm32f4 singlewire_exti_gpio(periph::gpio_stm32f4::port::a, 10,periph::gpio::mode::digital_input, 1);
    periph::gpio_stm32f4 singlewire_gpio(periph::gpio_stm32f4::port::a, 7, periph::gpio::mode::open_drain, 1);
    periph::exti_stm32f4 exti7(singlewire_exti_gpio);
    periph::timer_stm32f4 tim7(7);
    drv::singlewire singlewire(singlewire_gpio, tim7, exti7);
    drv::dht dht11(singlewire, drv::dht::device::dht11);
    
    task_params_t task_params = {button_1, dht11, green_led};
    xTaskCreate(button_1_task, "button_1", configMINIMAL_STACK_SIZE, &task_params, 1, nullptr);
    
    vTaskStartScheduler();
}
