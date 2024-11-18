#pragma once

#include <vector>
#include "periph/adc.hpp"
#include "dma_stm32f1.hpp"

namespace periph
{
class adc_onetime_stm32f1 : public adc_onetime
{
public:
    enum class resolution : uint8_t
    {
        _6_bit,
        _8_bit,
        _10_bit,
        _12_bit
    };
    
    // adc: 0-15 ADC, 16 - temp, 17 - vref
    adc_onetime_stm32f1(uint8_t adc, uint8_t channel, resolution resolution = resolution::_12_bit);
    ~adc_onetime_stm32f1();
    
    double read() final;
    
private:
    uint8_t adc;
    uint8_t channel;
    enum resolution resol;
};

class adc_cyclic_stm32f1 : public adc_cyclic
{
public:
    enum class resolution : uint8_t
    {
        _6_bit,
        _8_bit,
        _10_bit,
        _12_bit
    };
    
    // adc: 0-15 ADC, 16 - temp, 17 - vref
    adc_cyclic_stm32f1(uint8_t adc, std::vector<uint8_t> channels, resolution resolution,
        uint8_t timer, periph::dma_stm32f1 &dma, uint32_t frequency, uint8_t number_of_samples);
    ~adc_cyclic_stm32f1();
    
    void frequency(uint32_t frequency) final;
    uint32_t frequency() const final { return freq; };
    
    void set_callback(uint8_t channel, std::function<void(double voltage)> on_value) final;
    
    void start() final;
    void stop() final;
    
private:
    static constexpr uint8_t ch_max_num = 17; // The total number of channels in ADC
    uint8_t adc;
    std::vector<uint8_t> channels;
    std::function<void(double voltage)> callbacks[ch_max_num];
    enum resolution resol;
    uint8_t timer;
    periph::dma_stm32f1 &dma;
    uint32_t freq;
    uint8_t number_of_samples;
    uint16_t *dma_buff;
    
    static void timer_init(uint8_t timer, uint32_t frequency);
    static void calc_clk(uint8_t timer, uint32_t freq, uint16_t &presc, uint16_t &reload);
    void init_regular_chnls(uint8_t index, uint8_t channel);
    void on_dma(dma_stm32f1::event event);
};
} // namespace periph
