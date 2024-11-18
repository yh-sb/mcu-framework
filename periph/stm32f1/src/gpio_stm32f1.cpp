#include <cassert>
#include "periph/gpio_stm32f1.hpp"
#include "gpio_hw_mapping.hpp"

using namespace periph;

gpio_stm32f1::gpio_stm32f1(enum port port, uint8_t pin, enum mode mode, bool state):
    _port(port),
    _pin(pin),
    _mode(mode)
{
    assert(pin < gpio_hw_mapping::pins);
    
    RCC->APB2ENR |= gpio_hw_mapping::rcc_en[static_cast<uint8_t>(port)];
    
    gpio_stm32f1::mode(mode, state);
}

gpio_stm32f1::~gpio_stm32f1()
{
    GPIO_TypeDef *gpio = gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)];
    volatile uint32_t &cr = _pin < 8 ? gpio->CRL : gpio->CRH;
    
    // No pull-up, no pull-down
    cr &= ~(GPIO_CRL_MODE0 << ((_pin % 8) * 4));
    // Analog mode
    cr &= ~(GPIO_CRL_CNF0 << ((_pin % 8) * 4));
}

void gpio_stm32f1::set(bool state)
{
    assert(_mode == mode::digital_output || _mode == mode::open_drain);
    
    gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)]->BSRR = 1 << (state ? _pin : _pin +
        GPIO_BSRR_BR0_Pos);
}

void gpio_stm32f1::toggle()
{
    assert(_mode == mode::digital_output || _mode == mode::open_drain);
    
    gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)]->ODR ^= 1 << _pin;
}

bool gpio_stm32f1::get() const
{
    assert(_mode != mode::analog && _mode != mode::alternate_function);
    
    return gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)]->IDR & (1 << _pin);
}

void gpio_stm32f1::mode(enum mode mode, bool state)
{
    _mode = mode;
    GPIO_TypeDef *gpio = gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)];
    volatile uint32_t &cr = _pin < 8 ? gpio->CRL : gpio->CRH;
    
    // No pull-up, no pull-down
    cr &= ~(GPIO_CRL_MODE0 << ((_pin % 8) * 4));
    // Set very high speed
    cr &= ~(GPIO_CRL_CNF0 << ((_pin % 8) * 4));
    
    switch(mode)
    {
        case mode::digital_output:
            // Digital output push-pull
            cr |= GPIO_CRL_MODE0 << ((_pin % 8) * 4);
            break;
        
        case mode::open_drain:
            // Digital output
            cr |= GPIO_CRL_MODE0 << ((_pin % 8) * 4);
            // Open drain
            cr |= GPIO_CRL_CNF0_0 << ((_pin % 8) * 4);
            break;
        
        case mode::digital_input:
            // Digital input
            cr |= GPIO_CRL_CNF0_1 << ((_pin % 8) * 4);
            if(state)
            {
                gpio->ODR |= GPIO_ODR_ODR0 << _pin; // Pull-up
            }
            else
            {
                gpio->ODR &= ~(GPIO_ODR_ODR0 << _pin); // Pull-down
            }
            break;
        
        case mode::analog:
            // Analog mode is already enabled
            break;
        
        case mode::alternate_function:
            // Alternate function
            cr |= GPIO_CRL_MODE0 << ((_pin % 8) * 4);
            cr |= GPIO_CRL_CNF0_1 << ((_pin % 8) * 4);
            // Modification of AFR register should be done during periph init
            break;
    }
    
    // Setup default state
    if(mode == mode::digital_output || mode == mode::open_drain)
    {
        gpio->BSRR =  1 << (state ? _pin : _pin + GPIO_BSRR_BR0_Pos);
    }
}
