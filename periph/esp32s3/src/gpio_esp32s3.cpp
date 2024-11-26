#include <cassert>
#include "periph/gpio_esp32s3.hpp"
#include "driver/gpio.h"

using namespace periph;

gpio_esp32s3::gpio_esp32s3(uint8_t pin, enum mode mode, bool state):
    _pin(pin),
    _mode(mode)
{
    // There are no pins between GPIO21 and GPIO26 in ESP32S3
    assert(_pin <= 21 || (_pin >= 26 && _pin <= 48));
    
    gpio_esp32s3::mode(mode, state);
}

gpio_esp32s3::~gpio_esp32s3()
{
    gpio_reset_pin(static_cast<gpio_num_t>(_pin));
}

void gpio_esp32s3::set(bool state)
{
    gpio_set_level(static_cast<gpio_num_t>(_pin), state);
}

void gpio_esp32s3::toggle()
{
    gpio_set_level(static_cast<gpio_num_t>(_pin), !gpio_get_level(static_cast<gpio_num_t>(_pin)));
}

bool gpio_esp32s3::get() const
{
    return gpio_get_level(static_cast<gpio_num_t>(_pin));
}

void gpio_esp32s3::mode(enum mode mode, bool state)
{
    _mode = mode;
    
    gpio_mode_t esp_gpio_mode = GPIO_MODE_DISABLE;
    switch(mode)
    {
        case mode::digital_output: esp_gpio_mode = GPIO_MODE_OUTPUT; break;
        case mode::open_drain: esp_gpio_mode = GPIO_MODE_OUTPUT_OD; break;
        case mode::analog:
        case mode::alternate_function:
        case mode::digital_input: esp_gpio_mode = GPIO_MODE_INPUT; break;
        default: assert(0);
    }
    
    gpio_pullup_t input_pull_up = mode == periph::gpio::mode::digital_input && state ?
        GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    gpio_pulldown_t input_pull_down = mode == periph::gpio::mode::digital_input && !state ?
        GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    
    gpio_config_t config =
    {
        .pin_bit_mask = 1 << _pin,
        .mode = esp_gpio_mode,
        .pull_up_en = input_pull_up,
        .pull_down_en = input_pull_down,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t res = gpio_config(&config);
    
    if(mode == periph::gpio::mode::digital_output || mode == periph::gpio::mode::open_drain)
    {
        gpio_set_level(static_cast<gpio_num_t>(_pin), state);
    }
}
