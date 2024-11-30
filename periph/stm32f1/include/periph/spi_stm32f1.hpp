#pragma once

#include "periph/spi.hpp"
#include "dma_stm32f1.hpp"
#include "gpio_stm32f1.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace periph { class spi_stm32f1; }
// For internal use only! (called from ISR)
extern "C" void spi_irq_hndlr(periph::spi_stm32f1 *obj);

namespace periph
{
class spi_stm32f1 : public spi
{
public:
    /**
     * @brief  Construct spi (serial peripheral interface) object
     * 
     * @param  spi       Number of SPI hardware interface. Can be 1 to 3
     * @param  baudrate  Baudrate in Hz
     * @param  cpol      Clock polarity
     * @param  cpha      Clock phase
     * @param  bit_order Bit order
     * @param  dma_write DMA instance for write operations
     * @param  dma_read  DMA instance for read operations
     * @param  mosi      GPIO pin for master output slave input
     * @param  miso      GPIO pin for master input slave output
     * @param  clk       GPIO pin for clock
     */
    spi_stm32f1(uint8_t spi, uint32_t baudrate, enum cpol cpol, enum cpha cpha,
        enum bit_order bit_order, dma_stm32f1 &dma_write, dma_stm32f1 &dma_read,
        gpio_stm32f1 &mosi, gpio_stm32f1 &miso, gpio_stm32f1 &clk);
    ~spi_stm32f1();
    
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
    
    // Delete copy constructor and copy assignment operator
    spi_stm32f1(const spi_stm32f1&) = delete;
    spi_stm32f1& operator=(const spi_stm32f1&) = delete;
    
    // Delete move constructor and move assignment operator
    spi_stm32f1(spi_stm32f1&&) = delete;
    spi_stm32f1& operator=(spi_stm32f1&&) = delete;
    
private:
    uint8_t spi;
    uint32_t baud;
    enum cpol _cpol;
    enum cpha _cpha;
    enum bit_order _bit_order;
    SemaphoreHandle_t api_lock;
    TaskHandle_t task;
    enum res irq_res;
    gpio_stm32f1 &mosi, &miso, &clk;
    gpio *cs;
    dma_stm32f1 &write_dma;
    const void *write_buff;
    dma_stm32f1 &read_dma;
    void *read_buff;
    
    static uint8_t calc_presc(uint8_t spi, uint32_t baud);
    void on_dma_write(dma_stm32f1::event event);
    void on_dma_read(dma_stm32f1::event event);
    friend void ::spi_irq_hndlr(spi_stm32f1 *obj);
};
} // namespace periph
