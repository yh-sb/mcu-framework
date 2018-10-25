#include <stdint.h>

#include "common/assert.h"

#include "hal/STM32F4/rcc.hpp"

//#include "hal/STM32F4/CMSIS/core-support/core_cm4.h"
#include "hal/STM32F4/CMSIS/device-support/include/stm32f4xx.h"
#include "hal/STM32F4/CMSIS/device-support/include/system_stm32f4xx.h"

extern uint32_t SystemCoreClock;

using namespace hal;

/* Converting CFGR[7:4] HPRE value to prescaller */
static uint32_t const ahb_presc_list[16] =
{
	0, 0, 0, 0, 0, 0, 0, 0, /* 0-7 AHB prescaller 1 */
	2,                      /* 8   AHB prescaller 2 */
	4,                      /* 9   AHB prescaller 4 */
	8,                      /* 10  AHB prescaller 8 */
	16,                     /* 11  AHB prescaller 16 */
	64,                     /* 12  AHB prescaller 64 */
	128,                    /* 13  AHB prescaller 128 */
	256,                    /* 14  AHB prescaller 256 */
	512                     /* 15  AHB prescaller 512 */
};

/* Converting CFGR[12:10] PPRE1 value to prescaller */
static uint32_t const apb1_presc_list[8] =
{
	0, 0, 0, 0, /* APB1 prescaller 1 */
	2,          /* APB1 prescaller 2 */
	4,          /* APB1 prescaller 4 */
	8,          /* APB1 prescaller 8 */
	16          /* APB1 prescaller 16 */
};

/* Converting CFGR[15:13] PPRE2 value to prescaller */
static uint32_t const apb2_presc_list[8] =
{
	0, 0, 0, 0, /* APB2 prescaller 1 */
	2,          /* APB2 prescaller 2 */
	4,          /* APB2 prescaller 4 */
	8,          /* APB2 prescaller 8 */
	16          /* APB2 prescaller 16 */
};

static uint32_t rst_reg = 0;

bool hal::rcc_init(void)
{
	rst_reg = RCC->CSR;
	/* Clear the reset reason */
	RCC->CSR |= RCC_CSR_RMVF;
	
	/* Specific RCC initialization goes here: */
	
	return true;
}

uint32_t hal::rcc_get_freq(rcc_src_t src)
{
	uint32_t res = 0;
	uint32_t tmp = 0;
	uint32_t ahb_presc = 0;
	uint32_t apb1_presc = 0;
	uint32_t apb2_presc = 0;
	
	tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
	ahb_presc = ahb_presc_list[tmp];
	
	tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
	apb1_presc = apb1_presc_list[tmp];
	
	tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
	apb2_presc = apb2_presc_list[tmp];
	
	SystemCoreClockUpdate();
	res = SystemCoreClock;
	switch(src)
	{
		case RCC_SRC_SYSCLK:
			break;
		
		case RCC_SRC_AHB:
			if(ahb_presc > 0)
				res /= ahb_presc;
			break;
		
		case RCC_SRC_APB1:
			if(ahb_presc > 0)
				res /= ahb_presc;
			if(apb1_presc > 0)
				res /= apb1_presc;
			break;
		
		case RCC_SRC_APB2:
			if(ahb_presc > 0)
				res /= ahb_presc;
			if(apb2_presc > 0)
				res /= apb2_presc;
			break;
	}
	
	return res;
}

void hal::rcc_reset(void)
{
	NVIC_SystemReset();
}

rcc_rst_reason_t hal::rcc_get_rst_reason(void)
{
	if(!rst_reg)
		rcc_init();
	
	if(rst_reg & RCC_CSR_BORRSTF)
		return RCC_RST_REASON_LOW_POWER;
	else if(rst_reg & RCC_CSR_PADRSTF)
		return RCC_RST_REASON_EXTERNAL;
	else if(rst_reg & RCC_CSR_PORRSTF)
		return RCC_RST_REASON_LOW_POWER;
	else if(rst_reg & RCC_CSR_SFTRSTF)
		return RCC_RST_REASON_INTERNAL;
	else if(rst_reg & RCC_CSR_WDGRSTF)
		return RCC_RST_REASON_WDT;
	else if(rst_reg & RCC_CSR_WWDGRSTF)
		return RCC_RST_REASON_WDT;
	else if (rst_reg & RCC_CSR_LPWRRSTF)
		return RCC_RST_REASON_LOW_POWER;
	return RCC_RST_REASON_UNKNOWN;
}
