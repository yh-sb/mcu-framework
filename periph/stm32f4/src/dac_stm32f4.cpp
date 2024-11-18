#include <cassert>
#include "periph/dac_stm32f4.hpp"
#include "periph/gpio_stm32f4.hpp"
#include "stm32f4xx.h"

#if !defined(STM32F405xx) && !defined(STM32F407xx) && !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && !defined(STM32F410Tx) && !defined(STM32F413xx) && \
    !defined(STM32F415xx) && !defined(STM32F417xx) && !defined(STM32F423xx) && \
    !defined(STM32F427xx) && !defined(STM32F429xx) && !defined(STM32F437xx) && \
    !defined(STM32F439xx) && !defined(STM32F446xx) && !defined(STM32F469xx) && \
    !defined(STM32F479xx)
    #error "MCU doesn't have DAC"
#endif

using namespace periph;

dac_stm32f4::dac_stm32f4(uint8_t dac, enum align align, periph::gpio &gpio):
    dac(dac),
    align(align),
    gpio(gpio)
{
    assert(dac >= 1 && dac <= 2);
    assert(align <= align::left_12);
    assert(gpio.mode() == gpio::mode::analog);
    
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    
    uint32_t cr = DAC_CR_EN1 | DAC_CR_TSEL1;
    DAC->CR = cr << (dac == 1 ? 0 : DAC_CR_EN2_Pos);
}

dac_stm32f4::~dac_stm32f4()
{
    DAC->CR &= ~(DAC_CR_EN1 << (dac == 1 ? 0 : DAC_CR_EN2_Pos));
    RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
}

void dac_stm32f4::set(uint16_t code)
{
    assert(code < 4096);
    assert(code < 256 || align != align::right_8);
    
    if(dac == 1)
    {
        if(align == align::right_8)
        {
            DAC->DHR8R1 = code;
        }
        else if(align == align::right_12)
        {
            DAC->DHR12R1 = code;
        }
        else
        {
            DAC->DHR12L1 = code;
        }
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
    }
    else
    {
        if(align == align::right_8)
        {
            DAC->DHR8R2 = code;
        }
        else if(align == align::right_12)
        {
            DAC->DHR12R2 = code;
        }
        else
        {
            DAC->DHR12L2 = code;
        }
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;
    }
}

void dac_stm32f4::set(float voltage)
{
    assert(voltage <= v_ref);
    
    uint16_t code = 0;
    
    if(align == align::right_8)
    {
        code = (voltage / v_ref) * 255;
    }
    else
    {
        code = (voltage / v_ref) * 4095;
    }
    
    set(code);
}

uint16_t dac_stm32f4::get() const
{
    return dac == 1 ? DAC->DOR1 : DAC->DOR2;
}
