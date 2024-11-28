// Example for STM32F4DISCOVERY development board

#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/rtc.hpp"
#include "periph/gpio_stm32f4.hpp"

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
    periph::gpio_stm32f4 green_led(periph::gpio_stm32f4::port::d, 12, periph::gpio::mode::digital_output);
    
    // Blue LED
    periph::gpio_stm32f4 blue_led(periph::gpio_stm32f4::port::d, 15, periph::gpio::mode::digital_output);
    
    auto res = periph::rtc::init(periph::rtc::clk_source::internal);
    std::tm tm {};
    tm.tm_year = 119;
    tm.tm_mday = 1;
    periph::rtc::set(tm);
    
    periph::rtc::set_alarm_callback([&blue_led](const std::tm &tm){ blue_led.toggle(); });
    periph::rtc::set_alarm({.tm_sec = 1});
    
    xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, &green_led, 1, nullptr);
    
    vTaskStartScheduler();
}
