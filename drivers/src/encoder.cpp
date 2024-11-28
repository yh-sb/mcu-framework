#include <cassert>
#include "drivers/encoder.hpp"

using namespace drv;

enum
{
    STATE_1_1 = 0b11,
    STATE_0_1 = 0b01,
    STATE_0_0 = 0b00,
    STATE_1_0 = 0b10
};

encoder::encoder(periph::gpio &a, periph::gpio &b):
    a(a),
    b(b),
    prev_state(STATE_0_0),
    prev_prev_state(STATE_0_0)
{
    assert(a.mode() == periph::gpio::mode::digital_input);
    assert(b.mode() == periph::gpio::mode::digital_input);
}

encoder::~encoder()
{
}

void encoder::set_callback(std::function<void(int8_t diff)> on_change)
{
    this->on_change = on_change;
}

void encoder::poll()
{
    uint8_t state = (a.get() << 1) | b.get();
    if(state == prev_state)
    {
        return;
    }
    
    if(prev_state == STATE_1_1 && state != prev_prev_state && on_change)
    {
        on_change((state == STATE_1_0) ? +1 : -1);
    }
    
    prev_prev_state = prev_state;
    prev_state = state;
}
