#include "common/macros.h"

#include "hal/STM32F0/systick.hpp"
#include "hal/STM32F0/rcc.hpp"

#include "hal/STM32F0/CMSIS/device-support/include/stm32f0xx.h"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/task.h"

using namespace hal;

void hal::systick_init(void)
{
	uint32_t systick_freq = rcc_get_freq(RCC_SRC_AHB);
	SysTick_Config(systick_freq / 1000);
}

uint32_t hal::systick_get_ms(void)
{
	return xTaskGetTickCount();
}

uint32_t hal::systick_get_past(uint32_t start)
{
	return xTaskGetTickCount() - start;
}

/* Implemented in third_party/FreeRTOS/port.c
extern "C" void SysTick_Handler(void)
{
	systick_cnt++;
}
*/
