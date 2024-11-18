#include <cassert>
#include <limits>
#include "periph/uart_stm32f1.hpp"
#include "rcc.hpp"
#include "gpio_hw_mapping.hpp"
#include "stm32f1xx.h"
#include "core_cm3.h"

using namespace periph;

static constexpr auto uarts = 5; // Total number of UART periph in STM32F1

static uart_stm32f1 *obj_list[uarts];

constexpr USART_TypeDef *const uart_regs[uarts] =
{
    USART1, USART2,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    USART3,
#else
    nullptr,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    UART4, UART5
#else
    nullptr, nullptr
#endif
};

constexpr IRQn_Type irqn_num[uarts] =
{
    USART1_IRQn, USART2_IRQn,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    USART3_IRQn,
#else
    static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    UART4_IRQn, UART5_IRQn
#else
    static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[uarts] =
{
    RCC_APB2ENR_USART1EN, RCC_APB1ENR_USART2EN,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    RCC_APB1ENR_USART3EN,
#else
    0
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    RCC_APB1ENR_UART4EN, RCC_APB1ENR_UART5EN
#else
    0, 0
#endif
};

constexpr volatile uint32_t *rcc_en_reg[uarts] =
{
    &RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR
};

constexpr rcc::clk_source rcc_src[uarts] =
{
    rcc::clk_source::apb2, rcc::clk_source::apb1, rcc::clk_source::apb1,
    rcc::clk_source::apb1, rcc::clk_source::apb1
};

uart_stm32f1::uart_stm32f1(uint8_t uart, uint32_t baudrate, enum stopbits stopbits, enum parity parity,
    dma_stm32f1 &dma_tx, dma_stm32f1 &dma_rx, gpio_stm32f1 &gpio_tx, gpio_stm32f1 &gpio_rx):
    baud(baudrate),
    stopbits(stopbits),
    parity(parity),
    tx_dma(dma_tx),
    tx_gpio(gpio_tx),
    tx_irq_res(res::ok),
    rx_dma(dma_rx),
    rx_gpio(gpio_rx),
    rx_cnt(nullptr),
    rx_irq_res(res::ok)
{
    assert(uart > 0 && uart <= uarts && uart_regs[uart - 1]);
    assert(baud > 0);
    assert(tx_dma.direction() == dma_stm32f1::direction::memory_to_periph);
    assert(tx_dma.increment_size() == 8);
    assert(rx_dma.direction() == dma_stm32f1::direction::periph_to_memory);
    assert(rx_dma.increment_size() == 8);
    assert(tx_gpio.mode() == gpio::mode::alternate_function);
    assert(rx_gpio.mode() == gpio::mode::alternate_function);
    
    assert(api_lock = xSemaphoreCreateMutex());
    
    this->uart = uart - 1;
    obj_list[this->uart] = this;
    
    *rcc_en_reg[this->uart] |= rcc_en[this->uart];
    
    USART_TypeDef *uart_reg = uart_regs[this->uart];
    
    switch(stopbits)
    {
        case stopbits::stopbits_0_5: uart_reg->CR2 |= USART_CR2_STOP_0; break;
        case stopbits::stopbits_1: uart_reg->CR2 &= ~USART_CR2_STOP; break;
        case stopbits::stopbits_1_5: uart_reg->CR2 |= USART_CR2_STOP; break;
        case stopbits::stopbits_2: uart_reg->CR2 |= USART_CR2_STOP_1; break;
    }
    
    switch(parity)
    {
        case parity::none:
            uart_reg->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
            break;
        case parity::even:
            uart_reg->CR1 |= USART_CR1_PCE;
            uart_reg->CR1 &= ~USART_CR1_PS;
            break;
        case parity::odd:
            uart_reg->CR1 |= USART_CR1_PCE | USART_CR1_PS;
            break;
    }
    
    // Calculate UART prescaller
    uint32_t div = rcc::frequency(rcc_src[this->uart]) / baud;
    
    const auto brr_max = std::numeric_limits<uint16_t>::max();
    assert(div > 0 && div <= brr_max); // Baud rate is too low or too high
    uart_reg->BRR = div;
    
    tx_dma.destination((void *)&uart_reg->DR);
    rx_dma.source((void *)&uart_reg->DR);
    tx_dma.set_callback([this](dma_stm32f1::event event) { on_dma_tx(event); });
    rx_dma.set_callback([this](dma_stm32f1::event event) { on_dma_rx(event); });
    
    uart_reg->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_PEIE;
    uart_reg->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_EIE | USART_CR3_ONEBIT;
    
    NVIC_ClearPendingIRQ(irqn_num[this->uart]);
    NVIC_SetPriority(irqn_num[this->uart], 6);
    NVIC_EnableIRQ(irqn_num[this->uart]);
}

uart_stm32f1::~uart_stm32f1()
{
    NVIC_DisableIRQ(irqn_num[uart]);
    uart_regs[uart]->CR1 &= ~USART_CR1_UE;
    *rcc_en_reg[uart] &= ~rcc_en[uart];
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
    obj_list[uart] = nullptr;
}

void uart_stm32f1::baudrate(uint32_t baudrate)
{
    assert(baudrate > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    baud = baudrate;
    USART_TypeDef *uart_reg = uart_regs[uart];
    uart_reg->CR1 &= ~USART_CR1_UE;
    uint32_t div = rcc::frequency(rcc_src[uart]) / baud;
    
    const auto brr_max = std::numeric_limits<uint16_t>::max();
    assert(div > 0 && div <= brr_max); // Baud rate is too low or too high
    
    uart_reg->BRR = div;
    uart_reg->CR1 |= USART_CR1_UE;
    
    xSemaphoreGive(api_lock);
}

uart::res uart_stm32f1::write(const void *buff, uint16_t size)
{
    assert(buff);
    assert(size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    task = xTaskGetCurrentTaskHandle();
    tx_dma.source(buff);
    tx_dma.size(size);
    tx_dma.start();
    
    // Task will be unlocked later from isr
    ulTaskNotifyTake(true, portMAX_DELAY);
    
    xSemaphoreGive(api_lock);
    return tx_irq_res;
}

uart::res uart_stm32f1::read(void *buff, uint16_t *size, std::chrono::milliseconds timeout)
{
    assert(buff);
    assert(size);
    assert(*size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    rx_dma.destination(buff);
    rx_dma.size(*size);
    *size = 0;
    rx_cnt = size;
    
    task = xTaskGetCurrentTaskHandle();
    USART_TypeDef *uart_reg = uart_regs[uart];
    uart_reg->CR1 |= USART_CR1_RE;
    rx_dma.start();
    
    // Task will be unlocked later from isr
    if(!ulTaskNotifyTake(true, timeout.count()))
    {
        vPortEnterCritical();
        // Prevent common (non-DMA) UART IRQ
        uart_reg->CR1 &= ~USART_CR1_RE;
        uint32_t sr = uart_reg->SR;
        uint32_t dr = uart_reg->DR;
        NVIC_ClearPendingIRQ(irqn_num[uart]);
        // Prevent DMA IRQ
        rx_dma.stop();
        rx_irq_res = res::read_timeout;
        vPortExitCritical();
    }
    xSemaphoreGive(api_lock);
    return rx_irq_res;
}

uart::res uart_stm32f1::write_read(const void *write_buff, uint16_t write_size,
    void *read_buff, uint16_t *read_size, std::chrono::milliseconds timeout)
{
    assert(write_buff);
    assert(read_buff);
    assert(write_size > 0);
    assert(read_size);
    assert(*read_size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    // Prepare tx
    tx_dma.source(write_buff);
    tx_dma.size(write_size);
    
    // Prepare rx
    rx_dma.destination(read_buff);
    rx_dma.size(*read_size);
    *read_size = 0;
    rx_cnt = read_size;
    
    task = xTaskGetCurrentTaskHandle();
    // Start rx
    USART_TypeDef *uart_reg = uart_regs[uart];
    uart_reg->CR1 |= USART_CR1_RE;
    rx_dma.start();
    // Start tx
    tx_dma.start();
    
    // Task will be unlocked later from isr
    if(!ulTaskNotifyTake(true, timeout.count()))
    {
        vPortEnterCritical();
        // Prevent common (non-DMA) UART IRQ
        uart_reg->CR1 &= ~USART_CR1_RE;
        uint32_t sr = uart_reg->SR;
        uint32_t dr = uart_reg->DR;
        NVIC_ClearPendingIRQ(irqn_num[uart]);
        // Prevent DMA IRQ
        rx_dma.stop();
        rx_irq_res = res::read_timeout;
        vPortExitCritical();
    }
    
    xSemaphoreGive(api_lock);
    return tx_irq_res != res::ok ? tx_irq_res : rx_irq_res;
}

void uart_stm32f1::on_dma_tx(dma_stm32f1::event event)
{
    if(event == dma_stm32f1::event::half_complete)
    {
        return;
    }
    
    if(event == dma_stm32f1::event::complete)
    {
        tx_irq_res = res::ok;
    }
    else if(event == dma_stm32f1::event::error)
    {
        tx_irq_res = res::write_error;
    }
    
    if(rx_dma.is_busy())
    {
        // Wait for rx operation
        return;
    }
    
    BaseType_t hi_task_woken = 0;
    vTaskNotifyGiveFromISR(task, &hi_task_woken);
    portYIELD_FROM_ISR(hi_task_woken);
}

void uart_stm32f1::on_dma_rx(dma_stm32f1::event event)
{
    if(event == dma_stm32f1::event::half_complete)
    {
        return;
    }
    
    USART_TypeDef *uart_reg = uart_regs[uart];
    
    // Prevent common (non-DMA) UART IRQ
    uart_reg->CR1 &= ~USART_CR1_RE;
    uint32_t sr = uart_reg->SR;
    uint32_t dr = uart_reg->DR;
    NVIC_ClearPendingIRQ(irqn_num[uart]);
    
    if(event == dma_stm32f1::event::complete)
    {
        rx_irq_res = res::ok;
    }
    else if(event == dma_stm32f1::event::error)
    {
        rx_irq_res = res::write_error;
    }
    /* Rx buffer has partly filled (package has received) or Rx buffer has
    totally filled */
    if(rx_cnt)
    {
        *rx_cnt = rx_dma.transfered();
    }
    
    if(tx_dma.is_busy())
    {
        // Wait for tx operation
        return;
    }
    
    BaseType_t hi_task_woken = 0;
    vTaskNotifyGiveFromISR(task, &hi_task_woken);
    portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void uart_irq_hndlr(uart_stm32f1 *obj)
{
    USART_TypeDef *uart_reg = uart_regs[obj->uart];
    uint32_t sr = uart_reg->SR;
    uint32_t dr = uart_reg->DR;
    
    if((uart_reg->CR1 & USART_CR1_IDLEIE) && (sr & USART_SR_IDLE))
    {
        // IDLE event has happened (package has been received)
        obj->rx_irq_res = uart::res::ok;
    }
    else if((uart_reg->CR3 & USART_CR3_EIE) && (sr & (USART_SR_PE | USART_SR_FE |
        USART_SR_NE | USART_SR_ORE)))
    {
        // Error event has happened
        obj->rx_irq_res = uart::res::read_error;
    }
    else
    {
        return;
    }
    
    // Prevent DMA IRQ
    obj->rx_dma.stop();
    
    uart_reg->CR1 &= ~USART_CR1_RE;
    if(obj->rx_cnt)
    {
        *obj->rx_cnt = obj->rx_dma.transfered();
    }
    
    if(obj->tx_dma.is_busy())
    {
        // Wait for tx operation
        return;
    }
    
    BaseType_t hi_task_woken = 0;
    vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
    portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void USART1_IRQHandler(void)
{
    uart_irq_hndlr(obj_list[0]);
}

extern "C" void USART2_IRQHandler(void)
{
    uart_irq_hndlr(obj_list[1]);
}

#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void USART3_IRQHandler(void)
{
    uart_irq_hndlr(obj_list[2]);
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
extern "C" void UART4_IRQHandler(void)
{
    uart_irq_hndlr(obj_list[3]);
}

extern "C" void UART5_IRQHandler(void)
{
    uart_irq_hndlr(obj_list[4]);
}
#endif
