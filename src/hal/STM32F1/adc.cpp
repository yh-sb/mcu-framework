#include <string.h>
#include <stdlib.h>

#include "common/assert.h"

#include "adc.hpp"
#include "rcc.hpp"

#include "CMSIS/device-support/include/stm32f1xx.h"

using namespace hal;

#define MAX_TIM_RESOL 0xFFFF
#define V_REF (float)3.3

static TIM_TypeDef *const tim_list[adc::ADC_TIM_END] =
{
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F103x6) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM1,
#else
	NULL,
#endif
	TIM2, TIM3,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	TIM4,
#else
	NULL,
#endif
};

static uint32_t const rcc_list[adc::ADC_TIM_END] =
{
	RCC_APB2ENR_TIM1EN, RCC_APB1ENR_TIM2EN, RCC_APB1ENR_TIM3EN,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
	defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
	defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
	defined(STM32F105xC) || defined(STM32F107xC)
	RCC_APB1ENR_TIM4EN,
#else
	0,
#endif
};

static volatile uint32_t *const rcc_bus_list[adc::ADC_TIM_END] =
{
	&RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR
};

static rcc_src_t const rcc_src_list[adc::ADC_TIM_END] =
{
	RCC_SRC_APB2, RCC_SRC_APB1, RCC_SRC_APB1, RCC_SRC_APB1
};

static void tim_init(adc::adc_tim_t tim, uint32_t freq);
static void calc_clk(adc::adc_tim_t tim, uint32_t freq, uint16_t &presc,
	uint16_t &reload);

adc::adc(adc_t adc, adc_ch_t ch_list[], size_t ch_list_size, adc_tim_t tim,
	hal::dma &dma, adc_resol_t resol, uint32_t freq, uint8_t num_of_samples):
	_adc(adc),
	_ch_list((adc_ch_t *)malloc(sizeof(adc_ch_t) * ch_list_size)),
	_ch_list_size(ch_list_size),
	_tim(tim),
	_dma(dma),
	_resol(resol),
	_freq(freq),
	_dma_buff((uint16_t *)malloc(sizeof(uint16_t) * ch_list_size * num_of_samples)),
	_num_of_samples(num_of_samples)
{
	ASSERT(_adc < ADC_END);
	ASSERT(_ch_list);
	ASSERT(_ch_list_size > 0);
	ASSERT(_tim < ADC_TIM_END);
	ASSERT(_resol < ADC_RESOL_END);
	
	// STM32F100x supports only 12 bit resolution
	ASSERT(_resol == ADC_RESOL_12BIT);
	
	ASSERT(_freq > 0);
	ASSERT(_dma.dir() == dma::DIR_PERIPH_TO_MEM);
	ASSERT(_dma.inc_size() == dma::INC_SIZE_16);
	ASSERT(_dma_buff);
	ASSERT(_num_of_samples > 0);
	
	memcpy(_ch_list, ch_list, sizeof(adc_ch_t) * ch_list_size);
	memset(_clbks, 0, sizeof(_clbks));
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
	
	RCC->CFGR |= RCC_CFGR_ADCPRE_1;
	
	// Calibrate ADC
	//ADC1->CR2 |= ADC_CR2_CAL;
	//while(ADC1->CR2 & ADC_CR2_CAL);
	
	// Align results to the right
	ADC1->CR2 &= ~ADC_CR2_ALIGN;
	
	// Setup external trigger
	ADC1->CR2 &= ~ADC_CR2_EXTSEL;
	switch(_tim)
	{
		// CC1, CC2, CC3 events
		case ADC_TIM_1: break;
		// CC2 event
		case ADC_TIM_2: ADC1->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0; break;
		// TRGO event
		case ADC_TIM_3: ADC1->CR2 |= ADC_CR2_EXTSEL_2; break;
		// CC4 event
		case ADC_TIM_4: ADC1->CR2 |= ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_0; break;
		default: ASSERT(0);
	}
	
	// Enable conversation by external trigger
	ADC1->CR2 |= ADC_CR2_EXTTRIG;
	
	for(uint8_t i = 0; i < _ch_list_size; i++)
		init_regular_chnls(i, _ch_list[i]);
	
	// Enable DMA support and enable ADC
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_ADON;
	
	_dma.src((uint16_t *)&ADC1->DR);
	_dma.dst(_dma_buff);
	_dma.size(_ch_list_size * _num_of_samples);
	_dma.start_cyclic(on_dma, this);
	
	tim_init(_tim, _freq);
}

adc::~adc()
{
	enable(false);
	
	if(_dma_buff)
	{
		free(_dma_buff);
		_dma_buff = NULL;
	}
}

void adc::resol(adc_resol_t resol)
{
	// STM32F100x supports only 12 bit resolution
	ASSERT(resol == ADC_RESOL_12BIT);
}

void adc::freq(uint32_t freq)
{
	
}

void adc::add_clbk(adc_ch_t ch, adc_cb_t clbk, void *ctx)
{
	ASSERT(ch < ADC_CH_END);
	
	// Check that requested channel was initialized in constructor
	uint8_t ch_idx = 0;
	for(; ch_idx < _ch_list_size; ch_idx++)
	{
		if(ch == _ch_list[ch_idx])
			break;
	}
	ASSERT(ch_idx < _ch_list_size); // Requested channel wasn't found
	
	_clbks[ch] = {.clbk = clbk, .ctx = ctx};
}

void adc::enable(bool enable)
{
	if(enable)
	{
		tim_list[_tim]->CNT = 0;
		tim_list[_tim]->CR1 |= TIM_CR1_CEN;
	}
	else
	{
		tim_list[_tim]->CR1 &= ~TIM_CR1_CEN;
		tim_list[_tim]->CNT = 0;
	}
}

void adc::init_regular_chnls(uint8_t index, adc_ch_t ch)
{
	// 0..5
	if(index <= 5)
	{
		ADC1->SQR3 &= ~(ADC_SQR3_SQ1 << (index * 5));
		ADC1->SQR3 |= ch << (index * 5);
	}
	// 6..11
	else if(index <= 11)
	{
		index -= 6;
		ADC1->SQR2 &= ~(ADC_SQR2_SQ7 << (index * 5));
		ADC1->SQR2 |= ch << (index * 5);
	}
	// 12..15
	else
	{
		index -= 12;
		ADC1->SQR1 &= ~(ADC_SQR1_SQ13 << (index * 5));
		ADC1->SQR1 |= ch << (index * 5);
	}
}

void adc::on_dma(dma *dma, dma::event_t event, void *ctx)
{
	if(event != dma::EVENT_CMPLT)
		return;
	
	adc *obj = static_cast<adc *>(ctx);
	
	// Go through all initialized ADC channels
	for(uint8_t i = 0; i < obj->_ch_list_size; i++)
	{
		if(!obj->_clbks[obj->_ch_list[i]].clbk)
			continue;
		
		// Calculate voltage value for specific ADC channel
		float voltage = 0;
		for(uint16_t j = i; j < (obj->_ch_list_size * obj->_num_of_samples);
			j += obj->_ch_list_size)
		{
			voltage += obj->_dma_buff[j];
		}
		voltage /= obj->_num_of_samples;
		voltage = (voltage / 4095) * V_REF;
		
		obj->_clbks[obj->_ch_list[i]].clbk(obj, obj->_ch_list[i], voltage,
			obj->_clbks[obj->_ch_list[i]].ctx);
	}
}

static void tim_init(adc::adc_tim_t tim, uint32_t freq)
{
	uint16_t presc = 0;
	uint16_t reload = 0;
	calc_clk(tim, freq, presc, reload);
	
	*rcc_bus_list[tim] |= rcc_list[tim];
	
	tim_list[tim]->PSC = presc;
	tim_list[tim]->ARR = reload;
	
	// Enable generation of TRGO event
	tim_list[tim]->CR2 &= ~TIM_CR2_MMS;
	tim_list[tim]->CR2 |= TIM_CR2_MMS_1;
}

static void calc_clk(adc::adc_tim_t tim, uint32_t freq, uint16_t &presc,
	uint16_t &reload)
{
	uint32_t clk_freq = rcc_get_freq(rcc_src_list[tim]);
	/* If APBx prescaller no equal to 1, TIMx prescaller multiplies by 2 */
	if(clk_freq != rcc_get_freq(RCC_SRC_AHB))
		clk_freq *= 2;
	
	uint32_t tmp_presc = 0;
	uint32_t tmp_reload = clk_freq / freq;
	if(tmp_reload <= MAX_TIM_RESOL)
		tmp_presc = 1;
	else
	{
		tmp_presc = ((tmp_reload + (MAX_TIM_RESOL / 2)) / MAX_TIM_RESOL) + 1;
		tmp_reload /= tmp_presc;
	}
	
	ASSERT(tmp_presc <= MAX_TIM_RESOL);
	ASSERT(tmp_reload <= MAX_TIM_RESOL);
	
	presc = (uint16_t)(tmp_presc - 1);
	reload = (uint16_t)(tmp_reload - 1);
}
