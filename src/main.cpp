#include "gpio.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/task.h"

using namespace hal;

static void main_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		vTaskDelay(500);
	}
}

int main(void)
{
	// Example for STM32F4DISCOVERY development board
	static gpio green_led(3, 12, GPIO_MODE_DO, 0);
	
	xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1, &green_led,
		tskIDLE_PRIORITY + 1, NULL);
	
	vTaskStartScheduler();
}
