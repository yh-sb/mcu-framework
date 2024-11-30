#pragma once

#include "periph/dac.hpp"
#include "periph/gpio.hpp"

namespace periph
{
class dac_stm32f4 : public dac
{
public:
    enum class align : uint8_t
    {
        right_8,
        right_12,
        left_12
    };
    
    /**
     * @brief  Construct dac (digital to analog converter) object
     * 
     * @param  dac   Can be 1 or 2. Refers to the DAC1 and DAC2
     * @param  align Alignment of the data
     * @param  gpio  GPIO pin to be used as DAC output
     */
    dac_stm32f4(uint8_t dac, enum align align, gpio &gpio);
    ~dac_stm32f4();
    
    void set(uint16_t code) final;
    void set(float voltage) final;
    uint16_t get() const final;
    
    // Delete copy constructor and copy assignment operator
    dac_stm32f4(const dac_stm32f4&) = delete;
    dac_stm32f4& operator=(const dac_stm32f4&) = delete;
    
    // Delete move constructor and move assignment operator
    dac_stm32f4(dac_stm32f4&&) = delete;
    dac_stm32f4& operator=(dac_stm32f4&&) = delete;
    
private:
    uint8_t dac;
    align align;
    gpio &gpio;
    static constexpr float v_ref = 3.3;
};
} // namespace periph
