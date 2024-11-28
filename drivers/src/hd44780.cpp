#include <cassert>
#include "drivers/hd44780.hpp"
#include "printf.h"

using namespace drv;

#define DDRAM1_MIN_ADDR 0
#define DDRAM1_MAX_ADDR 39
#define DDRAM2_MIN_ADDR 64
#define DDRAM2_MAX_ADDR 103

#define STROB_DELAY 37 // us

enum cmd_t
{
    CLEAR_DISPLAY           = 1 << 0,
    RETURN_HOME             = 1 << 1,
    ENTRY_MODE_SET          = 1 << 2,
    DISPLAY_ON_OFF_CONTROL  = 1 << 3,
    CURSOR_OR_DISPLAY_SHIFT = 1 << 4,
    FUNCTION_SET            = 1 << 5,
    SET_CGRAM_ADDRESS       = 1 << 6,
    SET_DDRAM_ADDRESS       = 1 << 7
};

// Bits for ENTRY_MODE_SET command
enum entry_mode_set_bits_t
{
    I_D_BIT = 1 << 1, // Increment/Decrement DDRAM address (cursor position):
                      // 0 - decrement, 1 - increment
    S_BIT   = 1 << 0  // Shift the dispaly with each new character
};

// Bits for DISPLAY_ON_OFF_CONTROL command
enum display_on_off_control_bits_t
{
    D_BIT = 1 << 2, // On/off entire display
    C_BIT = 1 << 1, // On/off cursor
    B_BIT = 1 << 0  // On/off blinking cursor position
};

// Bits for CURSOR_OR_DISPLAY_SHIFT command
enum cursor_or_display_shift_bits_t
{
    S_C_BIT = 1 << 3, // Shift display or cursor: 0 - cursor, 1 - display
    R_L_BIT = 1 << 2  // Direction of shift: 0 - to the left, 1 - to the right
};

// Bits for FUNCTION_SET command
enum function_set_bits_t
{
    DL_BIT  = 1 << 4, // Interface data length: 0 - 4 bit, 1 - 8 bit
    N_BIT   = 1 << 3, // Number of display lines: 0 - one line, 1 - two line
    F_BIT   = 1 << 2, // Character font: 0 - 5x8, 1 - 5x10
    FT1_BIT = 1 << 1, // Font table: (FT1:FT0)
    FT0_BIT = 1 << 0, //            00 - ENGLISH_JAPANESE
                      //            01 - WESTERN EUROPEAN
                      //            10 - ENGLISH_RUSSIAN
                      //            11 - N/A
};

hd44780::hd44780(periph::gpio &rs, periph::gpio &rw, periph::gpio &e, periph::gpio &db4,
    periph::gpio &db5, periph::gpio &db6, periph::gpio &db7, periph::timer &timer):
    rs(rs),
    rw(rw),
    e(e),
    db{&db4, &db5, &db6, &db7},
    timer(timer)
{
    assert(rs.mode() == periph::gpio::mode::digital_output);
    assert(rw.mode() == periph::gpio::mode::digital_output);
    assert(e.mode() == periph::gpio::mode::digital_output);
    
    rs.set(1);
    rw.set(1);
    e.set(1);
    
    for(uint8_t i = 0; i < (sizeof(db) / sizeof(db[0])); i++)
    {
        assert(db[i]->mode() == periph::gpio::mode::digital_output);
        db[i]->set(1);
    }
    
    this->timer.set_callback([this]() {
        BaseType_t hi_task_woken = 0;
        vTaskNotifyGiveFromISR(task, &hi_task_woken);
        portYIELD_FROM_ISR(hi_task_woken);
    });
    
    assert(api_lock = xSemaphoreCreateMutex());
}

hd44780::~hd44780()
{
    timer.stop();
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
}

void hd44780::init()
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    rw.set(0);
    rs.set(0);
    
    write_4bit(FUNCTION_SET >> 4);
    delay(4100);
    write_4bit(FUNCTION_SET >> 4);
    delay(100);
    write_4bit(FUNCTION_SET >> 4);
    
    write_4bit((static_cast<uint8_t>(FUNCTION_SET) | static_cast<uint8_t>(N_BIT)) >> 4);
    write(write_type::cmd, static_cast<uint8_t>(FUNCTION_SET) | static_cast<uint8_t>(N_BIT));
    
    write(write_type::cmd, static_cast<uint8_t>(DISPLAY_ON_OFF_CONTROL) | static_cast<uint8_t>(D_BIT));
    
    write(write_type::cmd, CLEAR_DISPLAY);
    delay(6200); // OLED display requires 6,2 ms rather than 1,53 ms
    
    write(write_type::cmd, static_cast<uint8_t>(ENTRY_MODE_SET) | static_cast<uint8_t>(I_D_BIT));
    
    xSemaphoreGive(api_lock);
}

uint8_t hd44780::print(uint8_t ddram_addr, const char *format, ...)
{
    assert(format);
    assert((ddram_addr >= DDRAM1_MIN_ADDR && ddram_addr <= DDRAM1_MAX_ADDR) ||
        (ddram_addr >= DDRAM2_MIN_ADDR && ddram_addr <= DDRAM2_MAX_ADDR));
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    write(write_type::cmd, SET_DDRAM_ADDRESS | ddram_addr);
    
    va_list args;
    
    va_start(args, format);
    
    char message[DDRAM2_MAX_ADDR] = {};
    uint8_t new_ddram_addr = vsnprintf_(message, sizeof(message) - 1,
        format, args) + ddram_addr;
    
    for(uint8_t i = 0; message[i] != '\0'; i++)
    {
        write(write_type::data, message[i]);
    }
    
    va_end(args);
    
    xSemaphoreGive(api_lock);
    return new_ddram_addr;
}

uint8_t hd44780::print(uint8_t ddram_addr, char byte)
{
    assert((ddram_addr >= DDRAM1_MIN_ADDR && ddram_addr <= DDRAM1_MAX_ADDR) ||
        (ddram_addr >= DDRAM2_MIN_ADDR && ddram_addr <= DDRAM2_MAX_ADDR));
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    write(write_type::cmd, SET_DDRAM_ADDRESS | ddram_addr);
    write(write_type::data, byte);
    
    xSemaphoreGive(api_lock);
    return ddram_addr++; // TODO: Check returned value to be the really ddram_addr+1
}

uint8_t hd44780::ddram_addr()
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    // 7   bit  - busy flag
    // 0:6 bits - ddram/cgram address
    uint8_t addr = read_bf_and_ddram_addr() & 0b01111111;
    
    xSemaphoreGive(api_lock);
    return addr;
}

void hd44780::clear()
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    write(write_type::cmd, CLEAR_DISPLAY);
    delay(6200); // OLED display requires 6,2 ms unlike LCD (1,53 ms)
    
    xSemaphoreGive(api_lock);
}

void hd44780::write_cgram(uint8_t buff[8][8])
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    uint8_t old_addr = read_bf_and_ddram_addr() & 0b01111111;
    
    write(write_type::cmd, SET_CGRAM_ADDRESS);
    
    uint8_t *p = &buff[0][0];
    for(uint8_t i = 0; i < 64; i++)
    {
        write(write_type::data, p[i]);
    }
    
    write(write_type::cmd, SET_DDRAM_ADDRESS | old_addr);
    
    xSemaphoreGive(api_lock);
}

void hd44780::read_cgram(uint8_t buff[8][8])
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    uint8_t old_addr = read_bf_and_ddram_addr() & 0b01111111;
    
    write(write_type::cmd, SET_CGRAM_ADDRESS);
    
    for(uint8_t i = 0; i < (sizeof(db) / sizeof(db[0])); i++)
    {
        db[i]->mode(periph::gpio::mode::digital_input);
    }
    
    rw.set(1);
    rs.set(1);
    
    uint8_t *p = &buff[0][0];
    for(uint8_t i = 0; i < 64; i++)
    {
        p[i] = read_4bit() << 4;
        p[i] |= read_4bit();
    }
    
    for(uint8_t i = 0; i < (sizeof(db) / sizeof(db[0])); i++)
    {
        db[i]->mode(periph::gpio::mode::digital_output);
    }
    
    write(write_type::cmd, SET_DDRAM_ADDRESS | old_addr);
    
    xSemaphoreGive(api_lock);
}

uint8_t hd44780::read_bf_and_ddram_addr()
{
    for(uint8_t i = 0; i < (sizeof(db) / sizeof(db[0])); i++)
    {
        db[i]->mode(periph::gpio::mode::digital_input);
    }
    
    rw.set(1);
    rs.set(0);
    
    uint8_t byte = read_4bit() << 4;
    byte |= read_4bit();
    
    for(uint8_t i = 0; i < (sizeof(db) / sizeof(db[0])); i++)
    {
        db[i]->mode(periph::gpio::mode::digital_output);
    }
    
    return byte;
}

void hd44780::write_4bit(uint8_t half_byte)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        db[i]->set((half_byte >> i) & 1);
    }
    
    e.set(1);
    delay(STROB_DELAY);
    e.set(0);
    delay(STROB_DELAY);
}

void hd44780::write(write_type type, uint8_t byte)
{
    rw.set(0);
    rs.set(type == write_type::cmd ? 0 : 1);
    
    write_4bit(byte >> 4);
    write_4bit(byte);
}

uint8_t hd44780::read_4bit()
{
    uint8_t half_byte = 0;
    
    e.set(1);
    delay(STROB_DELAY);
    
    for(uint8_t i = 0; i < 4; i++)
    {
        if(db[i]->get())
        {
            half_byte |= 1 << i;
        }
    }
    
    e.set(0);
    delay(STROB_DELAY);
    
    return half_byte;
}

void hd44780::delay(uint32_t us)
{
    task = xTaskGetCurrentTaskHandle();
    timer.timeout(std::chrono::microseconds(us));
    timer.start();
    
    ulTaskNotifyTake(true, portMAX_DELAY);
}
