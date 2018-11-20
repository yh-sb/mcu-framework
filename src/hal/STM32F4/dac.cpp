#include <stddef.h>

#include "common/assert.h"

#include "hal/STM32F4/dac.hpp"
#include "hal/STM32F4/gpio.hpp"

#include "hal/STM32F4/CMSIS/device-support/include/stm32f4xx.h"

/* Only MCU listed below have the DAC periphery*/
#if !defined(STM32F405xx) && !defined(STM32F407xx) && !defined(STM32F410Cx) && \
	!defined(STM32F410Rx) && !defined(STM32F410Tx) && !defined(STM32F413xx) && \
	!defined(STM32F415xx) && !defined(STM32F417xx) && !defined(STM32F423xx) && \
	!defined(STM32F427xx) && !defined(STM32F429xx) && !defined(STM32F437xx) && \
	!defined(STM32F439xx) && !defined(STM32F446xx) && !defined(STM32F469xx) && \
	!defined(STM32F479xx)
	#error "Your MCU doesn't support DAC periphery"
#endif

using namespace hal;

#define V_REF (float)3.3

dac::dac(periph periph, align align, gpio &gpio):
	_periph(periph),
	_align(align),
	_gpio(gpio)
{
	ASSERT(_gpio.mode() == gpio::mode::AN);
	
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	uint32_t tmp_cr = DAC_CR_EN1 | DAC_CR_TSEL1;
	tmp_cr &= ~(DAC_CR_EN1 | DAC_CR_TSEL1 | DAC_CR_WAVE1 | DAC_CR_DMAEN1 |
		DAC_CR_BOFF1);
	
	DAC->CR = tmp_cr << ((_periph == periph::DAC_1) ? 0 : 16);
}

dac::~dac()
{
	
}

void dac::set(uint16_t val) const
{
	ASSERT(val < 4096);
	ASSERT(val < 256 || _align != align::R_8);
	
	if(_periph == periph::DAC_1)
	{
		if(_align == align::R_8)
			DAC->DHR8R1 = val;
		else if(_align == align::R_12)
			DAC->DHR12R1 = val;
		else
			DAC->DHR12L1 = val;
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
	}
	else
	{
		if(_align == align::R_8)
			DAC->DHR8R2 = val;
		else if(_align == align::R_12)
			DAC->DHR12R2 = val;
		else
			DAC->DHR12L2 = val;
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;
	}
}

void dac::set(float voltage) const
{
	ASSERT(voltage <= V_REF);
	
	uint16_t code = 0;
	
	if(_align == align::R_8)
		code = (uint16_t)((voltage / V_REF) * 255);
	else
		code = (uint16_t)((voltage / V_REF) * 4095);
	
	set(code);
}

uint16_t dac::get() const
{
	return (_periph == periph::DAC_1) ? (uint16_t)DAC->DOR1 : (uint16_t)DAC->DOR2;
}
