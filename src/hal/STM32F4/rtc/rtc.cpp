#include <stddef.h>

#include "common/assert.h"
#include "rtc.hpp"
#include "systick/systick.hpp"
#include "CMSIS/Device/STM32F4xx/Include/stm32f4xx.h"

using namespace hal;

#define INIT_TIMEOUT 10 /* ms */

static int8_t first_setup(void);
static void config_lsi(void);
static int8_t config_lse(void);
static int8_t enter_init(void);

int8_t rtc::init(clk_t clk)
{
	/* Backup domain software force reset */
	//RCC->BDCR |= RCC_BDCR_BDRST;
	//RCC->BDCR &= ~RCC_BDCR_BDRST;
	
	/* PWR periphery clock enable */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	/* Disable backup domain write protection */
	PWR->CR |= PWR_CR_DBP;
	
	if(clk == CLK_LSI)
		config_lsi();
	else
	{
		if(config_lse())
			return -1;
	}
	
	/* Enable RTC clk */
	RCC->BDCR |= RCC_BDCR_RTCEN;
	
	/* If rtc isn't running yet (if year field is equal to 0) */
	if(!(RTC->ISR & RTC_ISR_INITS) && !first_setup())
		return -1;
	
	return 0;
}

void rtc::get(time_t *time)
{
	ASSERT(time);
	
	uint32_t tmp_time = RTC->TR;
	uint32_t tmp_date = RTC->DR;
	
	time->year = ((tmp_date >> 20) & 0x0F) * 10;
	time->year += (tmp_date >> 16) & 0x0F;
	
	time->wday = (tmp_date >> 13) & 0x07;
	
	time->month = ((tmp_date >> 12) & 0x01) * 10;
	time->month += (tmp_date >> 8) & 0x0F;
	
	time->day = ((tmp_date >> 4) & 0x03) * 10;
	time->day += (tmp_date >> 0) & 0x0F;
	
	time->h = ((tmp_time >> 20) & 0x03) * 10;
	time->h += (tmp_time >> 16) & 0x0F;
	
	time->m = ((tmp_time >> 12) & 0x07) * 10;
	time->m += (tmp_time >> 8) & 0x0F;
	
	time->s = ((tmp_time >> 4) & 0x07) * 10;
	time->s += (tmp_time >> 0) & 0x0F;
}

int8_t rtc::set(time_t *time)
{
	ASSERT(time);
	ASSERT(time->year <= 99);
	ASSERT(time->month > 0 && time->month <= 12);
	ASSERT(time->day > 0 && time->day <= 31);
	ASSERT(time->wday > 0 && time->wday <= 7);
	ASSERT(time->h <= 23);
	ASSERT(time->m <= 59);
	ASSERT(time->s <= 59);
		
	/* Disable write protection */
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	int8_t res = 0;
	uint32_t tmp_date, tmp_time;
	
	tmp_date = (time->year / 10) << 20;
	tmp_date |= (time->year % 10) << 16;
	
	tmp_date |= time->wday << 13;
	
	tmp_date |= (time->month / 10) << 12;
	tmp_date |= (time->month % 10) << 8;
	
	tmp_date |= (time->day / 10) << 4;
	tmp_date |= (time->day % 10) << 0;
	
	tmp_time = (time->h / 10) << 20;
	tmp_time |= (time->h % 10) << 16;
	
	tmp_time |= (time->m / 10) << 12;
	tmp_time |= (time->m % 10) << 8;
	
	tmp_time |= (time->s / 10) << 4;
	tmp_time |= (time->s % 10) << 0;
	
	if(enter_init())
	{
		res = -1;
		goto Exit;
	}
	RTC->TR = tmp_time;
	RTC->DR = tmp_date;
	/* Exit initialization mode */
	RTC->ISR &= ~RTC_ISR_INIT;
	
Exit:
	/* Write protection enable */
	RTC->WPR = 0xFF;
	return res;
}

void rtc::bckp_write(uint8_t addr, void *buff, size_t size)
{
	
}

void rtc::bckp_read(uint8_t addr, void *buff, size_t size)
{
	
}

static int8_t first_setup(void)
{
	/* Disable write protection */
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	
	if(enter_init())
		return -1;
	
	/* 24 hour selection */
	RTC->CR |= RTC_CR_FMT;
	
	/* Bypass the shadow registers enable */
	RTC->CR |= RTC_CR_BYPSHAD;
	
	/* Prescaler setup for 32768 Hz (1 Hz output):
	(32768 / PREDIV_ASYNC) / PREDIV_SYNC = 1
	PREDIV_ASYNC = 128, PREDIV_SYNC = 256*/
	uint16_t sync_presc = 256 - 1;
	uint16_t async_presc = 128 - 1;
	
	RTC->PRER |= sync_presc | (async_presc << 16);
	
	/* Exit from initialization mode */
	RTC->ISR &= ~RTC_ISR_INIT;
	
	/* Enable write protection */
	RTC->WPR = 0xFF;
	
	return 0;
}

static void config_lsi(void)
{
	RCC->CSR |= RCC_CSR_LSION;
	while(!(RCC->CSR & RCC_CSR_LSIRDY));
	
	/* Setup RTC clock source */
	RCC->BDCR &= ~RCC_BDCR_RTCSEL_0;
	RCC->BDCR |= RCC_BDCR_RTCSEL_1;
}

static int8_t config_lse(void)
{
	RCC->BDCR |= RCC_BDCR_LSEON;
	uint32_t last_tim = systick_get_ms();
	while(1)
	{
		if(RCC->BDCR & RCC_BDCR_LSERDY)
		{
			/* Setup RTC clock source */
			RCC->BDCR &= ~RCC_BDCR_RTCSEL;
			RCC->BDCR |= RCC_BDCR_RTCSEL_0;
			return 0;
		}
		else if(systick_get_past(last_tim) >= INIT_TIMEOUT)
			return -1;
	}
}

static int8_t enter_init(void)
{
	/* Request initialization mode */
	RTC->ISR = 0xFFFFFFFF;
	uint32_t last_tim = systick_get_ms();
	while(1)
	{
		if(RTC->ISR & RTC_ISR_INITF)
			return 0;
		
		if(systick_get_past(last_tim) >= INIT_TIMEOUT)
			return -1;
	}
}
