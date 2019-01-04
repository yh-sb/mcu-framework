#include <stddef.h>

#include "common/assert.h"
#include "singlewire.hpp"

#include "exti.hpp"

using namespace hal;
using namespace drv;

enum
{
	REQ,
	WAIT_RESP_START,
	WAIT_RESP_END,
	WAIT_BIT_START_LOW,
	WAIT_BIT_START_HI,
	WAIT_BIT_CHECK
};

const static uint16_t timeout[WAIT_BIT_CHECK + 1] =
{
	18000, // REQ
	40,    // WAIT_RESP_START
	90,    // WAIT_RESP_END
	90,    // WAIT_BIT_START_LOW
	54,    // WAIT_BIT_START_HI
	35     // WAIT_BIT_CHECK
};

singlewire::singlewire(hal::gpio &gpio, hal::tim &tim, hal::exti &exti):
	_gpio(gpio),
	_tim(tim),
	_exti(exti)
{
	ASSERT(_gpio.mode() == gpio::mode::OD);
	
	lock = xSemaphoreCreateBinary();
	ASSERT(lock);
	
	_tim.cb(tim_cb, this);
	_exti.cb(exti_cb, this);
}

singlewire::~singlewire()
{
	vSemaphoreDelete(lock);
}

int8_t singlewire::read(uint8_t *buff, uint16_t size)
{
	fsm_start(buff, size);
	
	// Semaphore will be given from fsm when data reception will be finished
	xSemaphoreTake(lock, portMAX_DELAY);
	
	return res;
}

void singlewire::fsm_start(uint8_t *buff, uint16_t size)
{
	fsm.buff = buff;
	fsm.size = size;
	
	fsm.state = REQ;
	fsm.byte = 0;
	fsm.bit = 7;
	
	_gpio.set(0);
	_tim.us(timeout[REQ]);
	_tim.start();
}

void singlewire::fsm_run(bool is_tim_expired)
{
	switch(fsm.state)
	{
		case REQ:
			fsm.state = WAIT_RESP_START;
			_gpio.set(1);
			_exti.trigger(exti::edge::FALLING);
			_exti.on();
			_tim.us(timeout[WAIT_RESP_START]);
			_tim.start();
			break;
		
		case WAIT_RESP_START:
			if(is_tim_expired)
			{
				_exti.off();
				res = NODEV;
				goto Exit;
			}
			_tim.stop();
			
			fsm.state = WAIT_RESP_END;
			_exti.trigger(exti::edge::RISING);
			_tim.us(timeout[WAIT_RESP_END]);
			_tim.start();
			break;
		
		case WAIT_RESP_END:
			if(is_tim_expired)
			{
				_exti.off();
				res = DEVERR;
				goto Exit;
			}
			_tim.stop();
			
			fsm.state = WAIT_BIT_START_LOW;
			_exti.trigger(exti::edge::FALLING);
			_tim.us(timeout[WAIT_BIT_START_LOW]);
			_tim.start();
			break;
		
		case WAIT_BIT_START_LOW:
			if(is_tim_expired)
			{
				_exti.off();
				res = READERR;
				goto Exit;
			}
			_tim.stop();
			
			fsm.state = WAIT_BIT_START_HI;
			_exti.trigger(exti::edge::RISING);
			_tim.us(timeout[WAIT_BIT_START_HI]);
			_tim.start();
			break;
		
		case WAIT_BIT_START_HI:
			if(is_tim_expired)
			{
				_exti.off();
				res = READERR;
				goto Exit;
			}
			_tim.stop();
			
			fsm.state = WAIT_BIT_CHECK;
			_exti.trigger(exti::edge::FALLING);
			_tim.us(timeout[WAIT_BIT_CHECK]);
			_tim.start();
			break;
		
		case WAIT_BIT_CHECK:
			if(is_tim_expired)
			{
				fsm.buff[fsm.byte] |= 1 << fsm.bit;
				
				fsm.state =  WAIT_BIT_START_LOW;
				_exti.trigger(exti::edge::FALLING);
				_tim.us(timeout[WAIT_BIT_CHECK]);
				_tim.start();
			}
			else
			{
				_tim.stop();
				fsm.buff[fsm.byte] &= ~(1 << fsm.bit);
				
				fsm.state = WAIT_BIT_START_HI;
				_exti.trigger(exti::edge::RISING);
				_tim.us(timeout[WAIT_BIT_CHECK] + timeout[WAIT_BIT_START_HI]);
				_tim.start();
			}
			
			if(fsm.bit > 0)
				fsm.bit--;
			else
			{
				// Start reading next byte
				fsm.byte++;
				fsm.bit = 7;
				
				if(fsm.byte == fsm.size)
				{
					_tim.stop();
					_exti.off();
					res = OK;
					goto Exit;
				}
			}
			break;
	}
	return;
	
Exit:
	BaseType_t hi_task_woken = 0;
	xSemaphoreGiveFromISR(lock, &hi_task_woken);
	portYIELD_FROM_ISR(hi_task_woken);
}

void singlewire::tim_cb(hal::tim *tim, void *ctx)
{
	singlewire *obj = (singlewire *)ctx;
	
	obj->fsm_run(true);
}

void singlewire::exti_cb(hal::exti *exti, void *ctx)
{
	singlewire *obj = (singlewire *)ctx;
	
	obj->fsm_run(false);
}
