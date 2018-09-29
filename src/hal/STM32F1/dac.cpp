#include <stddef.h>

#include "common/macros.h"

#include "dac.hpp"
#include "gpio.hpp"

#include "CMSIS/device-support/include/stm32f1xx.h"

/* Only MCU listed below have the DAC periphery*/
#if !defined(STM32F100xB) && !defined(STM32F100xE) && !defined(STM32F101xE) && \
	!defined(STM32F101xG) && !defined(STM32F103xE) && !defined(STM32F103xG) && \
	!defined(STM32F105xC) && !defined(STM32F107xC)
	#error "Your MCU doesn't support DAC periphery"
#endif

using namespace hal;

#define V_REF (float)3.3

dac::dac(dac_t dac, dac_align_t align, gpio &gpio):
	_dac(dac),
	_align(align),
	_gpio(gpio)
{
	ASSERT(_dac < DAC_END);
	ASSERT(_gpio.mode() == GPIO_MODE_AN);
	
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	uint32_t tmp_cr = DAC_CR_EN1 | DAC_CR_TSEL1;
	tmp_cr &= ~(DAC_CR_EN1 | DAC_CR_TSEL1 | DAC_CR_WAVE1 | DAC_CR_DMAEN1 |
		DAC_CR_BOFF1);
	
	DAC->CR = tmp_cr << ((_dac == DAC_1) ? 0 : 16);
}

dac::~dac()
{
	
}

void dac::set(uint16_t val) const
{
	ASSERT(val < 4096);
	ASSERT(val < 256 || _align != DAC_ALIGN_8_R);
	
	if(_dac == DAC_1)
	{
		if(_align == DAC_ALIGN_8_R)
			DAC->DHR8R1 = val;
		else if(_align == DAC_ALIGN_12_R)
			DAC->DHR12R1 = val;
		else
			DAC->DHR12L1 = val;
		DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
	}
	else
	{
		if(_align == DAC_ALIGN_8_R)
			DAC->DHR8R2 = val;
		else if(_align == DAC_ALIGN_12_R)
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
	
	if(_align == DAC_ALIGN_8_R)
		code = (uint16_t)((voltage / V_REF) * 255);
	else
		code = (uint16_t)((voltage / V_REF) * 4095);
	
	set(code);
}

uint16_t dac::get() const
{
	return (_dac == DAC_1) ? (uint16_t)DAC->DOR1 : (uint16_t)DAC->DOR2;
}
