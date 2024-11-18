#include <cassert>
#include "periph/dac_stm32f1.hpp"
#include "periph/gpio_stm32f1.hpp"
#include "stm32f1xx.h"

#if !defined(STM32F100xB) && !defined(STM32F100xE) && !defined(STM32F101xE) && \
    !defined(STM32F101xG) && !defined(STM32F103xE) && !defined(STM32F103xG) && \
    !defined(STM32F105xC) && !defined(STM32F107xC)
    #error "MCU doesn't have DAC"
#endif

using namespace periph;

dac_stm32f1::dac_stm32f1(uint8_t dac, enum align align, periph::gpio &gpio):
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

dac_stm32f1::~dac_stm32f1()
{
    DAC->CR &= ~(DAC_CR_EN1 << (dac == 1 ? 0 : DAC_CR_EN2_Pos));
    RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
}

void dac_stm32f1::set(uint16_t code)
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

void dac_stm32f1::set(float voltage)
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

uint16_t dac_stm32f1::get() const
{
    return dac == 1 ? DAC->DOR1 : DAC->DOR2;
}
