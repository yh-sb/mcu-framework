#pragma once

#include <cstdarg>
#include "periph/gpio.hpp"
#include "periph/timer.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

namespace drv
{
class hd44780
{
public:
    hd44780(periph::gpio &rs, periph::gpio &rw, periph::gpio &e, periph::gpio &db4,
        periph::gpio &db5, periph::gpio &db6, periph::gpio &db7, periph::timer &timer);
    
    ~hd44780();
    
    void init();
    
    /**
     * @brief Print formatted text
     * 
     * @param ddram_addr DDRAM address to which text will be writed.
     *                   Example for 4 line display: 0 - 1st line,
     *                   64 - 2nd line, 20 - 3rd line, 84 - 4th line
     * @param format A string that specifies the format of the output
     * @param ... Arguments used by format string
     * @return uint8_t New DDRAM address after writing
     */
    uint8_t print(uint8_t ddram_addr, const char *format, ...);
    
    /**
     * @brief Print one byte
     * 
     * @param ddram_addr DDRAM address to which text will be writed
     * @param byte ASCII symbol
     * @return uint8_t New DDRAM address after writing
     */
    uint8_t print(uint8_t ddram_addr, char byte);
    
    uint8_t ddram_addr();
    
    void write_cgram(uint8_t buff[8][8]);
    void read_cgram(uint8_t buff[8][8]);
    
    void clear();
    
    // Delete copy constructor and copy assignment operator
    hd44780(const hd44780&) = delete;
    hd44780& operator=(const hd44780&) = delete;
    
    // Delete move constructor and move assignment operator
    hd44780(hd44780&&) = delete;
    hd44780& operator=(hd44780&&) = delete;
    
private:
    periph::gpio &rs, &rw, &e;
    periph::gpio *db[4];
    periph::timer &timer;
    TaskHandle_t task;
    SemaphoreHandle_t api_lock;
    
    enum class write_type : uint8_t { cmd, data };
    
    void write_4bit(uint8_t half_byte);
    void write(write_type type, uint8_t byte);
    
    uint8_t read_4bit();
    uint8_t read_bf_and_ddram_addr();
    
    void delay(uint32_t us);
};
} // namespace drv
