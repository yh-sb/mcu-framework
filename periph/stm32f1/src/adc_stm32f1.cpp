#include <cassert>
#include <limits>
#include "periph/adc_stm32f1.hpp"
#include "rcc.hpp"
#include "stm32f1xx.h"

using namespace periph;

constexpr ADC_TypeDef *const adcs[2] =
{
    ADC1,
#if defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || \
    defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
    ADC2
#else
    nullptr
#endif
};

constexpr TIM_TypeDef *const timers[4] = // ADC timers
{
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F103x6) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    TIM1,
#else
    nullptr,
#endif
    TIM2, TIM3,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    TIM4
#else
    nullptr
#endif
};

constexpr uint32_t rcc_en[4] = // TODO
{
    RCC_APB2ENR_TIM1EN, RCC_APB1ENR_TIM2EN, RCC_APB1ENR_TIM3EN,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    RCC_APB1ENR_TIM4EN
#else
    0
#endif
};

constexpr volatile uint32_t *const rcc_en_reg[4] = // TODO
{
    &RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR
};

constexpr rcc::clk_source rcc_src[4] = // TODO
{
    rcc::clk_source::apb2, rcc::clk_source::apb1, rcc::clk_source::apb1, rcc::clk_source::apb1
};

adc_onetime_stm32f1::adc_onetime_stm32f1(uint8_t adc, uint8_t channel, enum resolution resolution):
    channel(channel),
    resol(resolution)
{
    assert(adc >= 1 && adc <= 2 && adcs[adc - 1]);
    assert(channel >= 0 && channel <= 17); // 0-15 ADC, 16 - temp, 17 - vref
    
    this->adc = adc - 1;
    
    if(adc == 0)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
    }
#if defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || \
    defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
    else
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_ADC2RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC2RST;
    }
#endif
    
    ADC_TypeDef *adc_reg = adcs[this->adc];
    
    adc_reg->CR2 &= ~ADC_CR2_ALIGN;
    
    RCC->CFGR |= RCC_CFGR_ADCPRE_1;
}

adc_onetime_stm32f1::~adc_onetime_stm32f1()
{
    if(adc == 0)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    }
#if defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || \
    defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
    else
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
    }
#endif
    
    adcs[adc]->CR2 &= ~ADC_CR2_ADON;
}

double adc_onetime_stm32f1::read()
{
    ADC_TypeDef *adc_reg = adcs[this->adc];
    
    // Enable ADC with software trigger
    adc_reg->CR2 |= ADC_CR2_ADON | ADC_CR2_SWSTART;
    
    while(!(adc_reg->SR & ADC_SR_EOC))
    {
    }
    
    double val = adc_reg->DR;
    
    return (val / 4095) * 3.3; // If vref is 3.3v
}

adc_cyclic_stm32f1::adc_cyclic_stm32f1(uint8_t adc, std::vector<uint8_t> channels, enum resolution resolution,
    uint8_t timer, periph::dma_stm32f1 &dma, uint32_t frequency, uint8_t number_of_samples):
    resol(resolution),
    channels(channels),
    dma(dma),
    freq(frequency),
    number_of_samples(number_of_samples)
{
    assert(adc >= 1 && adc <= 2 && adcs[adc - 1]);
    assert(channels.size() > 0 && channels.size() < ch_max_num);
    for(auto channel : this->channels)
    {
        assert(channel >= 0 && channel <= ch_max_num); // 0-15 ADC, 16 - temp, 17 - vref
    }
    assert(timer >= 1 && timer <= 4);
#if defined(STM32F100xB) || defined(STM32F100xE)
    // STM32F100x supports only 12 bit resolution
    assert(resol == resolution::_12_bit);
#endif
    assert(dma.direction() == dma_stm32f1::direction::periph_to_memory);
    assert(dma.increment_size() == 16);
    assert(number_of_samples > 0);
    
    this->adc = adc - 1;
    this->timer = timer - 1;
    
    dma_buff = new uint16_t[channels.size() * number_of_samples];
    
    if(this->adc == 0)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
    }
#if defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || \
    defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
    else
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_ADC2RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC2RST;
    }
#endif
    
    RCC->CFGR |= RCC_CFGR_ADCPRE_1;
    
    ADC_TypeDef *adc_reg = adcs[this->adc];
    
    // Calibrate ADC
    /*adc_reg->CR2 |= ADC_CR2_CAL;
    while(adc_reg->CR2 & ADC_CR2_CAL)
    {
    }*/
    
    // Align results to the right
    adc_reg->CR2 &= ~ADC_CR2_ALIGN;
    
    // Setup external trigger
    adc_reg->CR2 &= ~ADC_CR2_EXTSEL;
    
    switch(timer)
    {
        // CC1, CC2, CC3 events
        case 0: break;
        
        // CC2 event
        case 1: adc_reg->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0; break;
        
        // TRGO event
        case 2: adc_reg->CR2 |= ADC_CR2_EXTSEL_2; break;
        
        // CC4 event
        case 3: adc_reg->CR2 |= ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_0; break;
        default: assert(0);
    }
    
    // Enable conversation by external trigger
    adc_reg->CR2 |= ADC_CR2_EXTTRIG;
    
    for(uint8_t i = 0; i < channels.size(); i++)
    {
        init_regular_chnls(i, channels[i]);
    }
    
    // Enable DMA support and enable ADC
    adc_reg->CR2 |= ADC_CR2_DMA | ADC_CR2_ADON;
    
    dma.source((uint16_t *)&adc_reg->DR);
    dma.destination(dma_buff);
    dma.size(channels.size() * number_of_samples);
    dma.set_callback([this](dma_stm32f1::event event) { on_dma(event); });
    dma.start(true);
    
    timer_init(timer, frequency);
}

adc_cyclic_stm32f1::~adc_cyclic_stm32f1()
{
    timers[timer]->CR1 &= ~TIM_CR1_CEN;
    timers[timer]->CNT = 0;
    
    if(adc == 0)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    }
#if defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || \
    defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
    else
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
    }
#endif
    
    adcs[adc]->CR2 &= ~ADC_CR2_ADON;
    
    delete[] dma_buff;
}

void adc_cyclic_stm32f1::frequency(uint32_t frequency)
{
    freq = frequency;
    timer_init(timer, freq);
}

void adc_cyclic_stm32f1::set_callback(uint8_t channel, std::function<void(double voltage)> on_value)
{
    assert(channel >= 0 && channel <= ch_max_num); // 0-15 ADC, 16 - temp, 17 - vref
    
    for(auto ch : channels)
    {
        if(ch == channel)
        {
            callbacks[channel] = on_value;
            break;
        }
    }
}

void adc_cyclic_stm32f1::start()
{
    timers[timer]->CNT = 0;
    timers[timer]->CR1 |= TIM_CR1_CEN;
}

void adc_cyclic_stm32f1::stop()
{
    timers[timer]->CR1 &= ~TIM_CR1_CEN;
    timers[timer]->CNT = 0;
}

void adc_cyclic_stm32f1::timer_init(uint8_t timer, uint32_t frequency)
{
    uint16_t presc = 0;
    uint16_t reload = 0;
    calc_clk(timer, frequency, presc, reload);
    
    *rcc_en_reg[timer] |= rcc_en[timer];
    
    TIM_TypeDef *tim = timers[timer];
    
    tim->PSC = presc;
    tim->ARR = reload;
    
    // Enable generation of TRGO event
    tim->CR2 &= ~TIM_CR2_MMS;
    tim->CR2 |= TIM_CR2_MMS_1;
}

void adc_cyclic_stm32f1::calc_clk(uint8_t timer, uint32_t freq, uint16_t &presc, uint16_t &reload)
{
    uint32_t clk_freq = rcc::frequency(rcc_src[timer]);
    // If APBx prescaller no equal to 1, TIMx prescaller multiplies by 2
    if(clk_freq != rcc::frequency(rcc::clk_source::ahb))
    {
        clk_freq *= 2;
    }
    
    /* Increase timer clock frequency or use timer with higher clock frequency
    to pass this assert */
    assert(freq <= clk_freq);
    
    uint32_t tmp_presc = 0;
    uint32_t tmp_reload = clk_freq / freq;
    constexpr auto tim_max_resol = std::numeric_limits<uint16_t>::max();
    
    if(tmp_reload <= tim_max_resol)
    {
        tmp_presc = 1;
    }
    else
    {
        tmp_presc = ((tmp_reload + (tim_max_resol / 2)) / tim_max_resol) + 1;
        tmp_reload /= tmp_presc;
    }
    
    assert(tmp_presc <= tim_max_resol);
    assert(tmp_reload <= tim_max_resol);
    
    presc = tmp_presc - 1;
    reload = tmp_reload - 1;
}

void adc_cyclic_stm32f1::init_regular_chnls(uint8_t index, uint8_t channel)
{
    ADC_TypeDef *adc_reg = adcs[adc];
    
    // 0..5
    if(index <= 5)
    {
        adc_reg->SQR3 &= ~(ADC_SQR3_SQ1 << (index * 5));
        adc_reg->SQR3 |= channel << (index * 5);
    }
    // 6..11
    else if(index <= 11)
    {
        index -= 6;
        adc_reg->SQR2 &= ~(ADC_SQR2_SQ7 << (index * 5));
        adc_reg->SQR2 |= channel << (index * 5);
    }
    // 12..15
    else
    {
        index -= 12;
        adc_reg->SQR1 &= ~(ADC_SQR1_SQ13 << (index * 5));
        adc_reg->SQR1 |= channel << (index * 5);
    }
}

void adc_cyclic_stm32f1::on_dma(dma_stm32f1::event event)
{
    if(event != dma_stm32f1::event::complete)
    {
        return;
    }
    
    for(auto ch : channels)
    {
        if(!callbacks[ch])
        {
            continue;
        }
        
        // Calculate voltage value for specific ADC channel
        double voltage = 0;
        for(uint16_t i = ch; i < (channels.size() * number_of_samples); i += channels.size())
        {
            voltage += dma_buff[i];
        }
        voltage /= number_of_samples;
        voltage = (voltage / 4095) * 3.3; // If vref is 3.3v
        
        callbacks[ch](voltage);
    }
}
