#pragma once

#include "periph/i2c.hpp"
#include "dma_stm32f4.hpp"
#include "gpio_stm32f4.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace periph { class i2c_stm32f4; }
// For internal use only! (called from ISR)
extern "C" void write_hndlr(periph::i2c_stm32f4 *obj, BaseType_t *hi_task_woken);
extern "C" void read_hndlr(periph::i2c_stm32f4 *obj, BaseType_t *hi_task_woken);
extern "C" void error_hndlr(periph::i2c_stm32f4 *obj, periph::i2c::res err, BaseType_t *hi_task_woken);
extern "C" void i2c_event_irq_hndlr(periph::i2c_stm32f4 *obj);
extern "C" void i2c_error_irq_hndlr(periph::i2c_stm32f4 *obj);

namespace periph
{
class i2c_stm32f4 : public i2c
{
public:
    i2c_stm32f4(uint8_t i2c, uint32_t baudrate, dma_stm32f4 &dma_write, dma_stm32f4 &dma_read,
        gpio_stm32f4 &sda, gpio_stm32f4 &scl);
    ~i2c_stm32f4();
    
    virtual void baudrate(uint32_t baudrate) = 0;
    
    virtual uint32_t baudrate() const = 0;
    
    virtual res write_read(uint16_t address, const void *write_buff, uint16_t write_size,
        void *read_buff, uint16_t read_size) = 0;
    
private:
    uint8_t i2c;
    uint32_t baud;
    SemaphoreHandle_t api_lock;
    TaskHandle_t task;
    enum res irq_res;
    gpio_stm32f4 &sda;
    gpio_stm32f4 &scl;
    
    uint16_t addr;
    
    dma_stm32f4 &write_dma;
    const void *write_buff;
    uint16_t write_size;
    
    dma_stm32f4 &read_dma;
    void *read_buff;
    uint16_t read_size;
    
    void gpio_af_init(gpio_stm32f4 &gpio);
    static void calc_clk(uint8_t i2c, uint32_t baud, uint8_t &freq, uint8_t &trise, uint32_t &ccr);
    void on_dma_write(dma_stm32f4::event event);
    void on_dma_read(dma_stm32f4::event event);
    friend void ::write_hndlr(i2c_stm32f4 *obj, BaseType_t *hi_task_woken);
    friend void ::read_hndlr(i2c_stm32f4 *obj, BaseType_t *hi_task_woken);
    friend void ::error_hndlr(i2c_stm32f4 *obj, res err, BaseType_t *hi_task_woken);
    friend void ::i2c_event_irq_hndlr(i2c_stm32f4 *obj);
    friend void ::i2c_error_irq_hndlr(i2c_stm32f4 *obj);
};
} // namespace periph
