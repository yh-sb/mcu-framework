#include "common/assert.h"
#include "gpio/gpio.hpp"
#include "rtc/rtc.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void main_task(void *pvParameters)
{
	gpio *green_led = (gpio *)pvParameters;
	while(1)
	{
		green_led->toggle();
		
		struct tm time = rtc::get();
		
		vTaskDelay(500);
	}
}

int main(void)
{
	static gpio green_led(3, 12, gpio::MODE_DO, 0);
	
	rtc::init(rtc::CLK_LSI);
	struct tm time = {};
	time.tm_year = 119;
	time.tm_mday = 1;
	rtc::set(time);
	
	ASSERT(xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE * 1, &green_led,
		tskIDLE_PRIORITY + 1, NULL) == pdPASS);
	
	vTaskStartScheduler();
}
