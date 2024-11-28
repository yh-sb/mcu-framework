#include <cstring>
#include <cassert>
#include "periph/rtc.hpp"
#include "periph/systick.hpp"
#include "stm32f4xx.h"

using namespace periph;

constexpr auto init_timeout = std::chrono::milliseconds(10);
static std::function<void(const std::tm &)> on_alarm;

static rtc::res first_setup(void);
static void config_lsi(void);
static rtc::res config_lse(void);
static rtc::res enter_init(void);

rtc::res rtc::init(clk_source clk)
{
    // Backup domain software force reset
    //RCC->BDCR |= RCC_BDCR_BDRST;
    //RCC->BDCR &= ~RCC_BDCR_BDRST;
    
    // PWR periphery clock enable
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    // Disable backup domain write protection
    PWR->CR |= PWR_CR_DBP;
    
    if(clk == clk_source::internal)
    {
        config_lsi();
    }
    else
    {
        if(config_lse() != res::ok)
        {
            return res::error;
        }
    }
    
    // Enable RTC clk
    RCC->BDCR |= RCC_BDCR_RTCEN;
    
    // If rtc isn't running yet (if year field is equal to 0)
    if(!(RTC->ISR & RTC_ISR_INITS) && first_setup() == res::ok)
    {
        return res::error;
    }
    
    NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);
    NVIC_SetPriority(RTC_Alarm_IRQn, 7);
    NVIC_EnableIRQ(RTC_Alarm_IRQn);
    
    return res::ok;
}

std::tm rtc::get()
{
    uint32_t time = RTC->TR;
    uint32_t date = RTC->DR;
    
    std::tm tm = {};
    
    tm.tm_year = ((date & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos) * 10;
    tm.tm_year += (date & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos;
    /* year in std::tm starts with 0, meaning that it starts from 1900. Normalize
       years from STM32F4 RTC since its year starts from 2000
    */
    tm.tm_year += 100;
    
    tm.tm_wday = (date & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos;
    tm.tm_wday--; // Weekday starts with 0 in std::tm
    
    tm.tm_mon = ((date & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos) * 10;
    tm.tm_mon += (date & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos;
    tm.tm_mon--; // Month starts with 0 in std::tm
    
    tm.tm_mday = ((date & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos) * 10;
    tm.tm_mday += (date & RTC_DR_DU_Msk) >> RTC_DR_DU_Pos;
    
    tm.tm_hour = ((time & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos) * 10;
    tm.tm_hour += (time & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos;
    
    tm.tm_min = ((time & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos) * 10;
    tm.tm_min += (time & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos;
    
    tm.tm_sec = ((time & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos) * 10;
    tm.tm_sec += (time & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos;
    
    return tm;
}

rtc::res rtc::set(std::tm &tm)
{
    assert(is_valid(tm));
    
    // Normalize time for STM32 RTC periphery
    tm.tm_mon++;
    tm.tm_wday++;
    tm.tm_year %= 100;
    
    // Disable write protection
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    
    uint32_t tmp_date = (tm.tm_year / 10) << RTC_DR_YT_Pos;
    tmp_date |= (tm.tm_year % 10) << RTC_DR_YU_Pos;
    tmp_date |= tm.tm_wday << RTC_DR_WDU_Pos;
    tmp_date |= (tm.tm_mon / 10) << RTC_DR_MT_Pos;
    tmp_date |= (tm.tm_mon % 10) << RTC_DR_MU_Pos;
    tmp_date |= (tm.tm_mday / 10) << RTC_DR_DT_Pos;
    tmp_date |= (tm.tm_mday % 10) << RTC_DR_DU_Pos;
    
    uint32_t tmp_time = (tm.tm_hour / 10) << RTC_TR_HT_Pos;
    tmp_time |= (tm.tm_hour % 10) << RTC_TR_HU_Pos;
    tmp_time |= (tm.tm_min / 10) << RTC_TR_MNT_Pos;
    tmp_time |= (tm.tm_min % 10) << RTC_TR_MNU_Pos;
    tmp_time |= (tm.tm_sec / 10) << RTC_TR_ST_Pos;
    tmp_time |= (tm.tm_sec % 10) << RTC_TR_SU_Pos;
    
    auto res = enter_init();
    if(res != res::ok)
    {
        // Write protection enable
        RTC->WPR = 0xFF;
        return res;
    }
    
    RTC->TR = tmp_time;
    RTC->DR = tmp_date;
    // Exit initialization mode
    RTC->ISR &= ~RTC_ISR_INIT;
    
    // Write protection enable
    RTC->WPR = 0xFF;
    return res::ok;
}

void rtc::bckp_write(uint8_t addr, const void *buff, size_t size)
{
    // TODO
}

void rtc::bckp_read(uint8_t addr, void *buff, size_t size)
{
    // TODO
}

bool rtc::is_valid(const std::tm &tm)
{
    return tm.tm_sec <= 59 && tm.tm_min <= 59 && tm.tm_hour <= 23 &&
        tm.tm_mday >= 1 && tm.tm_mday <= 31 && tm.tm_mon <= 11 &&
        // tm_year should be relative to 1900 year. Suppose it is at least 2000 year now
        tm.tm_year > 100 &&
        tm.tm_wday <= 6 && tm.tm_yday <= 365;
}

void rtc::set_alarm_callback(std::function<void(const std::tm &tm)> on_alarm)
{
    ::on_alarm = on_alarm;
}

void rtc::set_alarm(const std::tm &tm)
{
    // Disable write protection
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    
    // Disable alarm A and its interrupt
    RTC->CR &= ~(RTC_CR_ALRAE_Msk | RTC_CR_ALRAIE_Msk);
    
    std::tm tm_all_0 = {};
    if(!memcmp(&tm, &tm_all_0, sizeof(tm)))
    {
        // Enable write protection
        RTC->WPR = 0xFF;
        EXTI->IMR &= ~EXTI_IMR_MR17;
        EXTI->RTSR &= ~EXTI_IMR_MR17;
        return;
    }
    
    // Check that the RTC->ALRMAR register can be accessed
    while(!(RTC->ISR & RTC_ISR_ALRAWF_Msk));
    
    // Alarm every second
    RTC->ALRMAR |= RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK1;
    
    // Enable alarm A and its interrupt
    RTC->CR |= RTC_CR_ALRAE_Msk | RTC_CR_ALRAIE_Msk;
    
    // Enable write protection
    RTC->WPR = 0xFF;
    
    EXTI->IMR |= EXTI_IMR_MR17;
    EXTI->RTSR |= EXTI_IMR_MR17;
}

static rtc::res first_setup(void)
{
    // Disable write protection
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    
    if(enter_init() != rtc::res::ok)
    {
        return rtc::res::error;
    }
    
    // 24 hour selection
    RTC->CR |= RTC_CR_FMT;
    
    // Bypass the shadow registers enable
    RTC->CR |= RTC_CR_BYPSHAD;
    
    /* Prescaler setup for 32768 Hz (1 Hz output):
       (32768 / PREDIV_ASYNC) / PREDIV_SYNC = 1
       PREDIV_ASYNC = 128, PREDIV_SYNC = 256
    */
    uint16_t sync_presc = 256 - 1;
    uint16_t async_presc = 128 - 1;
    
    RTC->PRER |= sync_presc | (async_presc << 16);
    
    // Exit from initialization mode
    RTC->ISR &= ~RTC_ISR_INIT;
    
    // Enable write protection
    RTC->WPR = 0xFF;
    
    return rtc::res::ok;
}

static void config_lsi(void)
{
    RCC->CSR |= RCC_CSR_LSION;
    while(!(RCC->CSR & RCC_CSR_LSIRDY))
    {
    };
    
    // Setup RTC clock source
    RCC->BDCR &= ~RCC_BDCR_RTCSEL_0;
    RCC->BDCR |= RCC_BDCR_RTCSEL_1;
}

static rtc::res config_lse(void)
{
    RCC->BDCR |= RCC_BDCR_LSEON;
    auto last_time = systick::get();
    while(1)
    {
        if(RCC->BDCR & RCC_BDCR_LSERDY)
        {
            // Setup RTC clock source
            RCC->BDCR &= ~RCC_BDCR_RTCSEL;
            RCC->BDCR |= RCC_BDCR_RTCSEL_0;
            return rtc::res::ok;
        }
        else if(systick::get_past(last_time) >= init_timeout)
        {
            return rtc::res::error;
        }
    }
}

static rtc::res enter_init(void)
{
    // Request initialization mode
    RTC->ISR = 0xFFFFFFFF;
    auto last_tim = systick::get();
    while(1)
    {
        if(RTC->ISR & RTC_ISR_INITF)
        {
            return rtc::res::ok;
        }
        
        if(systick::get_past(last_tim) >= init_timeout)
        {
            return rtc::res::error;
        }
    }
}

extern "C" void rtc_irq_hndlr()
{
    if(on_alarm)
    {
        on_alarm(rtc::get());
    }
}

extern "C" void RTC_Alarm_IRQHandler(void)
{
    rtc_irq_hndlr();
}
