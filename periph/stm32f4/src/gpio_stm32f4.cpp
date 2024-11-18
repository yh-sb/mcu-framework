#include <cassert>
#include "periph/gpio_stm32f4.hpp"
#include "gpio_hw_mapping.hpp"

using namespace periph;

gpio_stm32f4::gpio_stm32f4(enum port port, uint8_t pin, enum mode mode, bool state):
    _port(port),
    _pin(pin),
    _mode(mode)
{
    assert(pin < gpio_hw_mapping::pins);
    
    RCC->AHB1ENR |= gpio_hw_mapping::rcc_en[static_cast<uint8_t>(port)];
    
    gpio_stm32f4::mode(mode, state);
}

gpio_stm32f4::~gpio_stm32f4()
{
    GPIO_TypeDef *gpio = gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)];
    
    // No pull-up, no pull-down
    gpio->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (_pin * 2));
    // Analog mode
    gpio->MODER |= GPIO_MODER_MODE0 << (_pin * 2);
}

void gpio_stm32f4::set(bool state)
{
    assert(_mode == mode::digital_output || _mode == mode::open_drain);
    
    gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)]->BSRR =
        1 << (state ? _pin : _pin + GPIO_BSRR_BR0_Pos);
}

void gpio_stm32f4::toggle()
{
    assert(_mode == mode::digital_output || _mode == mode::open_drain);
    
    gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)]->ODR ^= 1 << _pin;
}

bool gpio_stm32f4::get() const
{
    assert(_mode != mode::analog && _mode != mode::alternate_function);
    
    return gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)]->IDR & (1 << _pin);
}

void gpio_stm32f4::mode(enum mode mode, bool state)
{
    _mode = mode;
    GPIO_TypeDef *gpio = gpio_hw_mapping::gpio[static_cast<uint8_t>(_port)];
    
    // Input
    gpio->MODER &= ~(GPIO_MODER_MODE0 << (_pin * 2));
    // No pull-up, no pull-down
    gpio->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (_pin * 2));
    // Set very high speed
    gpio->OSPEEDR |= GPIO_OSPEEDR_OSPEED0 << (_pin * 2);
    
    switch(mode)
    {
        case mode::digital_output:
            // Digital output
            gpio->MODER |= GPIO_MODER_MODE0_0 << (_pin * 2);
            // Push-pull
            gpio->OTYPER &= ~(GPIO_OTYPER_OT0 << _pin);
            break;
        
        case mode::open_drain:
            // Digital output
            gpio->MODER |= GPIO_MODER_MODE0_0 << (_pin * 2);
            // Open drain
            gpio->OTYPER |= GPIO_OTYPER_OT0 << _pin;
            break;
        
        case mode::digital_input:
            if(state)
            {
                gpio->PUPDR |= GPIO_PUPDR_PUPD0_0 << (_pin * 2); // Pull-up
            }
            else
            {
                gpio->PUPDR |= GPIO_PUPDR_PUPD0_1 << (_pin * 2); // Pull-down
            }
            break;
        
        case mode::analog:
            // Analog mode
            gpio->MODER |= GPIO_MODER_MODE0 << (_pin * 2);
            break;
        
        case mode::alternate_function:
            // Alternate function
            gpio->MODER |= GPIO_MODER_MODE0_1 << (_pin * 2);
            // Modification of AFR register should be done during periph init
            break;
    }
    
    // Setup default state
    if(mode == mode::digital_output || mode == mode::open_drain)
    {
        gpio->BSRR =  1 << (state ? _pin : _pin + GPIO_BSRR_BR0_Pos);
    }
}
