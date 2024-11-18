#pragma once

#include "periph/spi.hpp"
#include "dma_stm32f4.hpp"
#include "gpio_stm32f4.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace periph { class spi_stm32f4; }
// For internal use only! (called from ISR)
extern "C" void spi_irq_hndlr(periph::spi_stm32f4 *obj);

namespace periph
{
class spi_stm32f4 : public spi
{
public:
    spi_stm32f4(uint8_t spi, uint32_t baudrate, enum cpol cpol, enum cpha cpha,
        enum bit_order bit_order, dma_stm32f4 &dma_write, dma_stm32f4 &dma_read,
        gpio_stm32f4 &mosi, gpio_stm32f4 &miso, gpio_stm32f4 &clk);
    ~spi_stm32f4();
    
    void baudrate(uint32_t baudrate) final;
    
    uint32_t baudrate() const final { return baud; }
    
    void cpol(enum cpol cpol) final;
    
    enum cpol cpol() const final { return _cpol; }
    
    void cpha(enum cpha cpha) final;
    
    enum cpha cpha() const final { return _cpha; }
    
    void bit_order(enum bit_order bit_order) final;
    
    enum bit_order bit_order() const final { return _bit_order; }
    
    res write(const void *buff, uint16_t size, gpio *cs = nullptr) final;
    
    res write(uint8_t byte, gpio *cs = nullptr) final;
    
    res read(void *buff, uint16_t size, gpio *cs = nullptr) final;
    
    res write_read(const void *write_buff, void *read_buff, uint16_t size, gpio *cs = nullptr) final;
    
private:
    uint8_t spi;
    uint32_t baud;
    enum cpol _cpol;
    enum cpha _cpha;
    enum bit_order _bit_order;
    SemaphoreHandle_t api_lock;
    TaskHandle_t task;
    enum res irq_res;
    gpio_stm32f4 &mosi, &miso, &clk;
    gpio *cs;
    dma_stm32f4 &write_dma;
    const void *write_buff;
    dma_stm32f4 &read_dma;
    void *read_buff;
    
    void gpio_af_init(gpio_stm32f4 &gpio);
    static uint8_t calc_presc(uint8_t spi, uint32_t baud);
    void on_dma_write(dma_stm32f4::event event);
    void on_dma_read(dma_stm32f4::event event);
    friend void ::spi_irq_hndlr(spi_stm32f4 *obj);
};
} // namespace periph
