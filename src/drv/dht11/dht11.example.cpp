#include "gpio.hpp"
#include "tim.hpp"
#include "exti.hpp"
#include "drv/di/di.hpp"
#include "drv/singlewire/singlewire.hpp"
#include "drv/dht11/dht11.hpp"

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
	static gpio dht11_gpio(0, 7, gpio::mode::OD, 1);
	static gpio dht11_exti_gpio(0, 10, gpio::mode::DI, 1);
	
	static tim dht11_tim(TIM_7);
	
	static exti dht11_exti(dht11_exti_gpio);
	
	static drv::singlewire dht11_singlewire(dht11_gpio, dht11_tim, dht11_exti);
	static drv::dht11 _dht11(dht11_singlewire);
	
	static drv::di b1_di(b1, 100, 1);
	b1_di.cb(b1_cb, &_dht11);
	
	xTaskCreate(di_task, "di", configMINIMAL_STACK_SIZE * 1, &b1_di,
		tskIDLE_PRIORITY + 1, NULL);
	
	vTaskStartScheduler();
}

static void b1_cb(drv::di *di, bool state, void *ctx)
{
	if(!state)
		return;
	
	drv::dht11 *_dht11 = (drv::dht11 *)ctx;
	
	uint8_t rh, t;
	int8_t res = _dht11->get(&rh, &t);
}
