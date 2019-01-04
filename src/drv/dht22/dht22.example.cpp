#include "gpio.hpp"
#include "tim.hpp"
#include "exti.hpp"
#include "drv/di/di.hpp"
#include "drv/singlewire/singlewire.hpp"
#include "drv/dht22/dht22.hpp"

#include "FreeRTOS.h"
#include "task.h"

using namespace hal;

static void b1_cb(drv::di *di, bool state, void *ctx);

static void di_task(void *pvParameters)
{
	drv::di *b1_di = (drv::di *)pvParameters;
	while(1)
	{
		b1_di->poll();
		vTaskDelay(1);
	}
}

int main(void)
{
	// Example for STM32F4DISCOVERY development board
	static gpio b1(0, 0, gpio::mode::DI, 0);
	static gpio dht22_gpio(0, 7, gpio::mode::OD, 1);
	static gpio dht22_exti_gpio(0, 10, gpio::mode::DI, 1);
	
	static tim dht22_tim(TIM_7);
	
	static exti dht22_exti(dht22_exti_gpio);
	
	static drv::singlewire dht22_singlewire(dht22_gpio, dht22_tim, dht22_exti);
	static drv::dht22 _dht22(dht22_singlewire);
	
	static drv::di b1_di(b1, 100, 1);
	b1_di.cb(b1_cb, &_dht22);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE * 1, &b1_di,
		tskIDLE_PRIORITY + 1, NULL);
	
	vTaskStartScheduler();
}

static void b1_cb(drv::di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	drv::dht22 *_dht22 = (drv::dht22 *)ctx;
	
	uint16_t rh;
	int16_t t;
	int8_t res = _dht22->get(&rh, &t);
}
