#pragma once

#include "periph/uart.hpp"
#include "dma_stm32f4.hpp"
#include "gpio_stm32f4.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace periph { class uart_stm32f4; }
// For internal use only! (called from ISR)
extern "C" void uart_irq_hndlr(periph::uart_stm32f4 *obj);

namespace periph
{
class uart_stm32f4 : public uart
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
    
    /**
     * @brief  Construct uart (universal asynchronous receiver-transmitter) object
     * 
     * @param  uart     Number of UART hardware interface. Can be 1 to 6
     * @param  baudrate Baudrate in bits per second
     * @param  stopbits Number of stop bits
     * @param  parity   Parity bit
     * @param  dma_tx   DMA instance for transmitting data
     * @param  dma_rx   DMA instance for receiving data
     * @param  gpio_tx  GPIO pin for transmitting data
     * @param  gpio_rx  GPIO pin for receiving data
     */
    uart_stm32f4(uint8_t uart, uint32_t baudrate, enum stopbits stopbits, enum parity parity,
        dma_stm32f4 &dma_tx, dma_stm32f4 &dma_rx, gpio_stm32f4 &gpio_tx, gpio_stm32f4 &gpio_rx);
    ~uart_stm32f4();
    
    void baudrate(uint32_t baudrate) final;
    
    uint32_t baudrate() const final { return baud; }
    
    res write(const void *buff, uint16_t size) final;
    
    res read(void *buff, uint16_t *size, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) final;
    
    res write_read(const void *write_buff, uint16_t write_size, void *read_buff,
        uint16_t *read_size, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()) final;
    
    // Delete copy constructor and copy assignment operator
    uart_stm32f4(const uart_stm32f4&) = delete;
    uart_stm32f4& operator=(const uart_stm32f4&) = delete;
    
    // Delete move constructor and move assignment operator
    uart_stm32f4(uart_stm32f4&&) = delete;
    uart_stm32f4& operator=(uart_stm32f4&&) = delete;
    
private:
    uint8_t uart;
    uint32_t baud;
    stopbits stopbits;
    parity parity;
    dma_stm32f4 &tx_dma;
    gpio_stm32f4 &tx_gpio;
    res tx_irq_res;
    dma_stm32f4 &rx_dma;
    gpio_stm32f4 &rx_gpio;
    uint16_t *rx_cnt;
    res rx_irq_res;
    SemaphoreHandle_t api_lock;
    TaskHandle_t task;
    
    void gpio_af_init(gpio_stm32f4 &gpio);
    void on_dma_tx(dma_stm32f4::event event);
    void on_dma_rx(dma_stm32f4::event event);
    friend void ::uart_irq_hndlr(uart_stm32f4 *obj);
};
} // namespace periph
