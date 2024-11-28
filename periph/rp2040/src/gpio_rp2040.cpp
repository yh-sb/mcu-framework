#include <cassert>
#include "periph/gpio_rp2040.hpp"
#include "hardware/gpio.h"

using namespace periph;

constexpr auto pins = 30;  // Total number of gpio pins

gpio_rp2040::gpio_rp2040(uint8_t pin, enum mode mode, bool state):
    _pin(pin),
    _mode(mode)
{
    assert(_pin < pins);
    
    gpio_rp2040::mode(mode, state);
}

gpio_rp2040::~gpio_rp2040()
{
    gpio_set_function(_pin, GPIO_FUNC_NULL);
}

void gpio_rp2040::set(bool state)
{
    gpio_put(_pin, state);
}

void gpio_rp2040::toggle()
{
    gpio_xor_mask(1 << _pin);
}

bool gpio_rp2040::get() const
{
    return gpio_get(_pin);
}

void gpio_rp2040::mode(enum mode mode, bool state)
{
    _mode = mode;
    
    gpio_set_dir(_pin, mode == mode::digital_output ? GPIO_OUT : GPIO_IN);
    gpio_put(_pin, state);
    gpio_set_function(_pin, GPIO_FUNC_SIO);
}
