#pragma once

#include "periph/uart.hpp"
#include "dma_stm32f1.hpp"
#include "gpio_stm32f1.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace periph { class uart_stm32f1; }
// For internal use only! (called from ISR)
extern "C" void uart_irq_hndlr(periph::uart_stm32f1 *obj);

namespace periph
{
class uart_stm32f1 : public uart
{
public:
    enum class stopbits : uint8_t
    {
        stopbits_0_5,
        stopbits_1,
        stopbits_1_5,
        stopbits_2
    };
    
    enum class parity : uint8_t
    {
        none,
        even,
        odd
    };
    
    uart_stm32f1(uint8_t uart, uint32_t baudrate, stopbits stopbits, parity parity,
        dma_stm32f1 &dma_tx, dma_stm32f1 &dma_rx, gpio_stm32f1 &gpio_tx, gpio_stm32f1 &gpio_rx);
    ~uart_stm32f1();
    
    void baudrate(uint32_t baudrate) final;
    
    uint32_t baudrate() const final { return baud; }
    
    res write(const void *buff, uint16_t size) final;
    
    res read(void *buff, uint16_t *size, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) final;
    
    res write_read(const void *write_buff, uint16_t write_size, void *read_buff,
        uint16_t *read_size, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) final;
    
private:
    uint8_t uart;
    uint32_t baud;
    stopbits stopbits;
    parity parity;
    dma_stm32f1 &tx_dma;
    gpio_stm32f1 &tx_gpio;
    res tx_irq_res;
    dma_stm32f1 &rx_dma;
    gpio_stm32f1 &rx_gpio;
    uint16_t *rx_cnt;
    res rx_irq_res;
    SemaphoreHandle_t api_lock;
    TaskHandle_t task;
    
    void on_dma_tx(dma_stm32f1::event event);
    void on_dma_rx(dma_stm32f1::event event);
    friend void ::uart_irq_hndlr(uart_stm32f1 *obj);
};
} // namespace periph
