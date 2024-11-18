#include <limits>
#include <cassert>
#include "periph/timer_stm32f1.hpp"
#include "timer_hw_mapping.hpp"
#include "rcc.hpp"
#include "stm32f1xx.h"
#include "core_cm3.h"

using namespace periph;

static timer_stm32f1 *obj_list[timer_stm32f1::timers];

timer_stm32f1::timer_stm32f1(uint8_t timer)
{
    assert(timer >= 1 && timer < timers && timer_hw_mapping::timer[timer - 1]);
    tim = timer - 1; // We count timers from 0 in timer_hw_mapping::
    
    *timer_hw_mapping::rcc_en_reg[tim] |= timer_hw_mapping::rcc_en[tim];
    
    obj_list[tim] = this;
    
    /* Allow that only counter overflow/underflow generates irq
    (avoid irq generation when set UG bit) */
    timer_hw_mapping::timer[tim]->CR1 |= TIM_CR1_URS;
    
    // Enable interrupt
    timer_hw_mapping::timer[tim]->DIER |= TIM_DIER_UIE;
    
    NVIC_SetPriority(timer_hw_mapping::irqn[tim], 1);
    NVIC_EnableIRQ(timer_hw_mapping::irqn[tim]);
}

timer_stm32f1::~timer_stm32f1()
{
    NVIC_DisableIRQ(timer_hw_mapping::irqn[tim]);
    timer_hw_mapping::timer[tim]->CR1 &= ~TIM_CR1_CEN;
    *timer_hw_mapping::rcc_en_reg[tim] &= ~timer_hw_mapping::rcc_en[tim];
    obj_list[tim] = nullptr;
}

void timer_stm32f1::set_callback(std::function<void()> on_timeout)
{
    this->on_timeout = std::move(on_timeout);
}

void timer_stm32f1::timeout(std::chrono::microseconds timeout)
{
    assert(timeout.count() > 0);
    
    _timeout = timeout;
    uint16_t psc, arr;
    calc_clk(tim, _timeout.count(), psc, arr);
    
    TIM_TypeDef *tim_reg = timer_hw_mapping::timer[tim];
    tim_reg->PSC = psc;
    tim_reg->ARR = arr;
    
    // Update ARR, PSC and clear CNT register
    tim_reg->EGR = TIM_EGR_UG;
}

void timer_stm32f1::start(bool is_cyclic)
{
    assert(_timeout.count() > 0);
    assert(on_timeout != nullptr);
    
    TIM_TypeDef *tim_reg = timer_hw_mapping::timer[tim];
    // This action allowed only when TIM is disabled
    assert(!(tim_reg->CR1 & TIM_CR1_CEN));
    
    if(is_cyclic)
    {
        tim_reg->CR1 &= ~TIM_CR1_OPM;
    }
    else
    {
        tim_reg->CR1 |= TIM_CR1_OPM;
    }
    
    tim_reg->CNT = 0;
    tim_reg->CR1 |= TIM_CR1_CEN;
}

void timer_stm32f1::stop()
{
    TIM_TypeDef *tim_reg = timer_hw_mapping::timer[tim];
    
    tim_reg->CR1 &= ~TIM_CR1_CEN;
    tim_reg->CNT = 0;
    tim_reg->SR &= ~TIM_SR_UIF;
}

bool timer_stm32f1::is_expired() const
{
    return !(timer_hw_mapping::timer[tim]->CR1 & TIM_CR1_CEN);
}

void timer_stm32f1::calc_clk(uint8_t tim, uint32_t usec, uint16_t &psc, uint16_t &arr)
{
    uint32_t clk_freq = rcc::frequency(timer_hw_mapping::rcc_src[tim]);
    // If APBx prescaller no equal to 1, TIMx prescaller multiplies by 2
    if(clk_freq != rcc::frequency(rcc::clk_source::ahb))
    {
        clk_freq *= 2;
    }
    
    uint32_t tmp_arr = usec * (clk_freq / 1000000);
    constexpr auto tim_max_resol = std::numeric_limits<uint16_t>::max();
    uint32_t tmp_psc = 1;
    
    if(tmp_arr > tim_max_resol)
    {
        // tmp_arr is too big for ARR register (16 bit), increase the prescaler
        tmp_psc = ((tmp_arr + (tim_max_resol / 2)) / tim_max_resol) + 1;
        tmp_arr /= tmp_psc;
    }
    
    assert(tmp_psc <= tim_max_resol);
    assert(tmp_arr <= tim_max_resol);
    
    psc = tmp_psc - 1;
    arr = tmp_arr - 1;
}

extern "C" void tim_irq_hndlr(periph::timer_stm32f1 *obj)
{
    TIM_TypeDef *tim_reg = timer_hw_mapping::timer[obj->tim];
    uint32_t sr = tim_reg->SR;
    
    if((tim_reg->DIER & TIM_DIER_UIE) && (sr & TIM_SR_UIF))
    {
        tim_reg->SR &= ~TIM_SR_UIF;
    }
    else if((tim_reg->DIER & TIM_DIER_CC1IE) && (sr & TIM_SR_CC1IF))
    {
        tim_reg->SR &= ~TIM_SR_CC1IF;
    }
    else if((tim_reg->DIER & TIM_DIER_CC2IE) && (sr & TIM_SR_CC2IF))
    {
        tim_reg->SR &= ~TIM_SR_CC2IF;
    }
    else if((tim_reg->DIER & TIM_DIER_CC3IE) && (sr & TIM_SR_CC3IF))
    {
        tim_reg->SR &= ~TIM_SR_CC3IF;
    }
    else if((tim_reg->DIER & TIM_DIER_CC4IE) && (sr & TIM_SR_CC4IF))
    {
        tim_reg->SR &= ~TIM_SR_CC4IF;
    }
    else
    {
        return;
    }
    
    if(obj->on_timeout)
    {
        obj->on_timeout();
    }
}

#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F103x6) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void TIM1_CC_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[0]);
}
#endif

extern "C" void TIM2_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[1]);
}

extern "C" void TIM3_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[2]);
}

#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void TIM4_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[3]);
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
extern "C" void TIM5_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[4]);
}
#endif

#if defined(STM32F100xB) || defined(STM32F100xE)
extern "C" void TIM6_DAC_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[5]);
}

extern "C" void TIM7_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[6]);
}
#elif defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || \
    defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void TIM6_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[5]);
}

extern "C" void TIM7_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[6]);
}
#endif

#if defined(STM32F103xE) || defined(STM32F103xG)
extern "C" void TIM8_CC_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[7]);
}
#endif

#if defined(STM32F101xG)
extern "C" void TIM9_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[8]);
}

extern "C" void TIM10_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[9]);
}

extern "C" void TIM11_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[10]);
}
#elif defined(STM32F103xG)
extern "C" void TIM1_BRK_TIM9_IRQHandler(void)
{
    if((TIM1->DIER & TIM_DIER_BIE) && (TIM1->SR & TIM_SR_BIF))
    {
        tim_irq_hndlr(obj_list[0]);
    }
    else
    {
        tim_irq_hndlr(obj_list[8]);
    }
}

extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
    if((TIM1->DIER & TIM_DIER_UIE) && (TIM1->SR & TIM_SR_UIF))
    {
        tim_irq_hndlr(obj_list[0]);
    }
    else
    {
        tim_irq_hndlr(obj_list[9]);
    }
}

extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    uint32_t sr = TIM1->SR;
    
    if(((TIM1->DIER & TIM_DIER_COMIE) && (sr & TIM_SR_COMIF)) ||
        ((TIM1->DIER & TIM_DIER_TIE) && (sr & TIM_SR_TIF)))
    {
        tim_irq_hndlr(obj_list[0]);
    }
    else
    {
        tim_irq_hndlr(obj_list[10]);
    }
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xG)
extern "C" void TIM12_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[11]);
}

extern "C" void TIM13_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[12]);
}

extern "C" void TIM14_IRQHandler(void)
{
    tim_irq_hndlr(obj_list[13]);
}
#elif defined(STM32F103xG)
extern "C" void TIM8_BRK_TIM12_IRQHandler(void)
{
    if((TIM8->DIER & TIM_DIER_BIE) && (TIM8->SR & TIM_SR_BIF))
    {
        tim_irq_hndlr(obj_list[7]);
    }
    else
    {
        tim_irq_hndlr(obj_list[11]);
    }
}

extern "C" void TIM8_UP_TIM13_IRQHandler(void)
{
    if((TIM8->DIER & TIM_DIER_UIE) && (TIM8->SR & TIM_SR_UIF))
    {
        tim_irq_hndlr(obj_list[7]);
    }
    else
    {
        tim_irq_hndlr(obj_list[12]);
    }
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    uint32_t sr = TIM8->SR;
    
    if(((TIM8->DIER & TIM_DIER_COMIE) && (sr & TIM_SR_COMIF)) ||
        ((TIM8->DIER & TIM_DIER_TIE) && (sr & TIM_SR_TIF)))
    {
        tim_irq_hndlr(obj_list[7]);
    }
    else
    {
        tim_irq_hndlr(obj_list[13]);
    }
}
#endif

#if defined(STM32F100xB) || defined(STM32F100xE)
extern "C" void TIM1_BRK_TIM15_IRQHandler(void)
{
    if((TIM1->DIER & TIM_DIER_BIE) && (TIM1->SR & TIM_SR_BIF))
    {
        tim_irq_hndlr(obj_list[0]);
    }
    else
    {
        tim_irq_hndlr(obj_list[14]);
    }
}

extern "C" void TIM1_UP_TIM16_IRQHandler(void)
{
    if((TIM1->DIER & TIM_DIER_UIE) && (TIM1->SR & TIM_SR_UIF))
    {
        tim_irq_hndlr(obj_list[0]);
    }
    else
    {
        tim_irq_hndlr(obj_list[15]);
    }
}

extern "C" void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    uint32_t sr = TIM1->SR;
    
    if(((TIM1->DIER & TIM_DIER_COMIE) && (sr & TIM_SR_COMIF)) ||
        ((TIM1->DIER & TIM_DIER_TIE) && (sr & TIM_SR_TIF)))
    {
        tim_irq_hndlr(obj_list[0]);
    }
    else
    {
        tim_irq_hndlr(obj_list[16]);
    }
}
#endif
