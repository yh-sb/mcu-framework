#include "periph/systick.hpp"
#include "rcc.hpp"
#include "stm32f0xx.h"
#include "core_cm0.h"
#include "FreeRTOS.h"
#include "task.h"

using namespace periph;

void systick::init()
{
    uint32_t systick_freq = rcc::frequency(rcc::clk_source::ahb);
    SysTick_Config(systick_freq / configTICK_RATE_HZ);
}

std::chrono::microseconds systick::get()
{
    return std::chrono::milliseconds(xTaskGetTickCount());
}

std::chrono::microseconds systick::get_past(std::chrono::microseconds timestamp)
{
    return std::chrono::milliseconds(xTaskGetTickCount()) - timestamp;
}

/* Implemented in FreeRTOS/port.c
extern "C" void SysTick_Handler(void)
{
    systick_cnt++;
}
*/
