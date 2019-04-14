#include "FreeRTOS.h"
#include "task.h"
#include "gpio/gpio.hpp"

static void main_task(void *pvParameters)
{
	hal::gpio *led = static_cast<hal::gpio *>(pvParameters);
	
	while(1)
	{
		led->toggle();
		vTaskDelay(500);
	}
}

extern "C" void app_main(void)
{
	static hal::gpio blue_led(0, 2, hal::gpio::MODE_DO, 0);
	
	xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1, &blue_led,
		tskIDLE_PRIORITY + 1, NULL);
}
