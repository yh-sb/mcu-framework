#include <cassert>
#include <limits>
#include "periph/uart_stm32f0.hpp"
#include "rcc.hpp"
#include "gpio_hw_mapping.hpp"
#include "stm32f0xx.h"
#include "core_cm0.h"

using namespace periph;

static constexpr auto uarts = 8; // Total number of UART interfaces

constexpr auto isr_err_flags = USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE;

static uart_stm32f0 *obj_list[uarts];

constexpr USART_TypeDef *const uart_regs[uarts] =
{
    USART1,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    USART2,
#else
    nullptr,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    USART3, USART4,
#else
    nullptr, nullptr,
#endif
#if defined(STM32F030xC) || defined(STM32F091xC) || defined(STM32F098xx)
    USART5, USART6,
#else
    nullptr, nullptr,
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
    USART7, USART8
#else
    nullptr, nullptr
#endif
};

constexpr IRQn_Type irqn_num[uarts] =
{
    USART1_IRQn,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    USART2_IRQn,
#else
    static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F030xC)
    USART3_6_IRQn, USART3_6_IRQn,
#elif defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
    defined(STM32F078xx)
    USART3_4_IRQn, USART3_4_IRQn,
#elif defined(STM32F091xC) || defined(STM32F098xx)
    USART3_8_IRQn, USART3_8_IRQn,
#else
    static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F030xC)
    USART3_6_IRQn, USART3_6_IRQn,
#elif defined(STM32F091xC) || defined(STM32F098xx)
    USART3_8_IRQn, USART3_8_IRQn,
#else
    static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
    USART3_8_IRQn, USART3_8_IRQn
#else
    static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[uarts] =
{
    RCC_APB2ENR_USART1EN,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    RCC_APB1ENR_USART2EN,
#else
    0,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    RCC_APB1ENR_USART3EN, RCC_APB1ENR_USART4EN,
#else
    0, 0,
#endif
#if defined(STM32F030xC) || defined(STM32F091xC) || defined(STM32F098xx)
    RCC_APB1ENR_USART5EN, RCC_APB2ENR_USART6EN,
#else
    0, 0,
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
    RCC_APB2ENR_USART7EN, RCC_APB2ENR_USART8EN
#else
    0, 0
#endif
};

constexpr uint32_t rcc_rst[uarts] =
{
    RCC_APB2RSTR_USART1RST,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    RCC_APB1RSTR_USART2RST,
#else
    0,
#endif
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
    RCC_APB1RSTR_USART3RST, RCC_APB1RSTR_USART4RST,
#else
    0, 0,
#endif
#if defined(STM32F030xC) || defined(STM32F091xC) || defined(STM32F098xx)
    RCC_APB1RSTR_USART5RST, RCC_APB2RSTR_USART6RST,
#else
    0, 0,
#endif
#if defined(STM32F091xC) || defined(STM32F098xx)
    RCC_APB2RSTR_USART7RST, RCC_APB2RSTR_USART8RST
#else
    0, 0
#endif
};

constexpr volatile uint32_t *rcc_en_reg[uarts] =
{
    &RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR, &RCC->APB1ENR,
    &RCC->APB2ENR, &RCC->APB2ENR, &RCC->APB2ENR
};

constexpr volatile uint32_t *rcc_rst_reg[uarts] =
{
    &RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR,
    &RCC->APB1RSTR, &RCC->APB2RSTR, &RCC->APB2RSTR, &RCC->APB2RSTR
};

constexpr rcc::clk_source rcc_src[uarts] =
{
    rcc::clk_source::apb2, rcc::clk_source::apb1, rcc::clk_source::apb1,
    rcc::clk_source::apb1, rcc::clk_source::apb1, rcc::clk_source::apb2,
    rcc::clk_source::apb2, rcc::clk_source::apb2
};

/* Get gpio alternative function index by uart interface and port number:
   af = uart2afr[_uart][gpio.port()]
*/
constexpr uint8_t uart2afr[][gpio_hw_mapping::ports] =
{
    {1, 0},
    {1, 0, 0, 0},
    {0, 4, 1, 0},
    {4, 0, 0}
};

uart_stm32f0::uart_stm32f0(uint8_t uart, uint32_t baudrate, enum stopbits stopbits, enum parity parity,
    dma_stm32f0 &dma_tx, dma_stm32f0 &dma_rx, gpio_stm32f0 &gpio_tx, gpio_stm32f0 &gpio_rx):
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
    assert(tx_dma.direction() == dma_stm32f0::direction::memory_to_periph);
    assert(tx_dma.increment_size() == 8);
    assert(rx_dma.direction() == dma_stm32f0::direction::periph_to_memory);
    assert(rx_dma.increment_size() == 8);
    assert(tx_gpio.mode() == gpio::mode::alternate_function);
    assert(rx_gpio.mode() == gpio::mode::alternate_function);
    
    assert(api_lock = xSemaphoreCreateMutex());
    
    this->uart = uart - 1;
    obj_list[this->uart] = this;
    
    *rcc_en_reg[this->uart] |= rcc_en[this->uart];
    *rcc_rst_reg[this->uart] |= rcc_rst[this->uart];
    *rcc_rst_reg[this->uart] &= ~rcc_rst[this->uart];
    
    gpio_af_init(tx_gpio);
    gpio_af_init(rx_gpio);
    
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    remap_dma(this->uart, tx_dma);
    remap_dma(this->uart, rx_dma);
    
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
    
    tx_dma.destination((void *)&uart_reg->TDR);
    rx_dma.source((void *)&uart_reg->RDR);
    tx_dma.set_callback([this](dma_stm32f0::event event) { on_dma_tx(event); });
    rx_dma.set_callback([this](dma_stm32f0::event event) { on_dma_rx(event); });
    
    uart_reg->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_PEIE;
    uart_reg->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_EIE | USART_CR3_ONEBIT;
    
    NVIC_ClearPendingIRQ(irqn_num[this->uart]);
    NVIC_SetPriority(irqn_num[this->uart], 6);
    NVIC_EnableIRQ(irqn_num[this->uart]);
}

uart_stm32f0::~uart_stm32f0()
{
    NVIC_DisableIRQ(irqn_num[uart]);
    uart_regs[uart]->CR1 &= ~USART_CR1_UE;
    *rcc_en_reg[uart] &= ~rcc_en[uart];
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
    obj_list[uart] = nullptr;
}

void uart_stm32f0::baudrate(uint32_t baudrate)
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

uart::res uart_stm32f0::write(const void *buff, uint16_t size)
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

uart::res uart_stm32f0::read(void *buff, uint16_t *size, std::chrono::milliseconds timeout)
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
        uint32_t sr = uart_reg->ISR;
        uint32_t dr = uart_reg->RDR;
        NVIC_ClearPendingIRQ(irqn_num[uart]);
        // Prevent DMA IRQ
        rx_dma.stop();
        rx_irq_res = res::read_timeout;
        vPortExitCritical();
    }
    xSemaphoreGive(api_lock);
    return rx_irq_res;
}

uart::res uart_stm32f0::write_read(const void *write_buff, uint16_t write_size,
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
        uint32_t sr = uart_reg->ISR;
        uint32_t dr = uart_reg->RDR;
        NVIC_ClearPendingIRQ(irqn_num[uart]);
        // Prevent DMA IRQ
        rx_dma.stop();
        rx_irq_res = res::read_timeout;
        vPortExitCritical();
    }
    
    xSemaphoreGive(api_lock);
    return tx_irq_res != res::ok ? tx_irq_res : rx_irq_res;
}

void uart_stm32f0::gpio_af_init(gpio_stm32f0 &gpio)
{
    GPIO_TypeDef *gpio_reg = gpio_hw_mapping::gpio[static_cast<uint8_t>(gpio.port())];
    uint8_t pin = gpio.pin();
    
    // Push-pull
    gpio_reg->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
    
    // Configure alternate function
    gpio_reg->AFR[pin / 8] &= ~(GPIO_AFRL_AFSEL0 << ((pin % 8) * 4));
    gpio_reg->AFR[pin / 8] |= uart2afr[uart][static_cast<uint8_t>(gpio.port())] << ((pin % 8) * 4);
}

void uart_stm32f0::remap_dma(uint8_t uart, dma_stm32f0 &dma)
{
#if defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx)
    switch(dma.channel())
    {
        case 1:
            if(uart == 0)
            {
                SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1TX_DMA_RMP;
            }
            else if(uart == 2)
            {
                SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART3_DMA_RMP;
            }
            break;
        
        case 2:
            if(uart == 0)
            {
                SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1RX_DMA_RMP;
            }
            else if(uart == 2)
            {
                SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART3_DMA_RMP;
            }
            break;
        
        case 3:
            if(uart == 0)
            {
                SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
            }
            else if(uart == 1)
            {
                SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART2_DMA_RMP;
            }
            break;
        
        case 4:
            if(uart == 0)
            {
                SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
            }
            else if(uart == 1)
            {
                SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART2_DMA_RMP;
            }
            break;
        
        case 5:
        case 6:
            if(uart == 1)
            {
                SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART2_DMA_RMP;
            }
            else if(uart == 2)
            {
                SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART3_DMA_RMP;
            }
            break;
        
        default: assert(0);
    }
#elif defined(STM32F091xC) || defined(STM32F098xx)
#error Not implemented. Need to change DMA1_CSELR: "DMAx channel selection registers"
#else
    switch(dma.channel())
    {
        case 1:
            if(uart == 0)
            {
                SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1TX_DMA_RMP;
            }
            break;
        
        case 2:
            if(uart == 0)
            {
                SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_USART1RX_DMA_RMP;
            }
            break;
        
        case 3:
            if(uart == 0)
            {
                SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
            }
            break;
        
        case 4:
            if(uart == 0)
            {
                SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;
            }
            break;
        
        default: assert(0);
    }
#endif
}

void uart_stm32f0::on_dma_tx(dma_stm32f0::event event)
{
    if(event == dma_stm32f0::event::half_complete)
    {
        return;
    }
    
    if(event == dma_stm32f0::event::complete)
    {
        tx_irq_res = res::ok;
    }
    else if(event == dma_stm32f0::event::error)
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

void uart_stm32f0::on_dma_rx(dma_stm32f0::event event)
{
    if(event == dma_stm32f0::event::half_complete)
    {
        return;
    }
    
    USART_TypeDef *uart_reg = uart_regs[uart];
    
    // Prevent common (non-DMA) UART IRQ
    uart_reg->CR1 &= ~USART_CR1_RE;
    uint32_t sr = uart_reg->ISR;
    uint32_t dr = uart_reg->RDR;
    NVIC_ClearPendingIRQ(irqn_num[uart]);
    
    if(event == dma_stm32f0::event::complete)
    {
        rx_irq_res = res::ok;
    }
    else if(event == dma_stm32f0::event::error)
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

extern "C" void uart_irq_hndlr(uart_stm32f0 *obj)
{
    USART_TypeDef *uart_reg = uart_regs[obj->uart];
    uint32_t sr = uart_reg->ISR;
    uint32_t dr = uart_reg->RDR;
    
    if((uart_reg->CR1 & USART_CR1_IDLEIE) && (sr & USART_ISR_IDLE))
    {
        // IDLE event has happened (package has been received)
        obj->rx_irq_res = uart::res::ok;
    }
    else if((uart_reg->CR3 & USART_CR3_EIE) && (sr & isr_err_flags))
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

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
extern "C" void USART2_IRQHandler(void)
{
    uart_irq_hndlr(obj_list[1]);
}
#endif

#if defined(STM32F030xC)
extern "C" void USART3_6_IRQHandler(void)
{
    for(uint8_t i = 2; i <= 5; i++)
    {
        USART_TypeDef *uart_reg = uart_regs[i];
        uint32_t sr = uart_reg->ISR;
        
        if((uart_reg->CR1 & USART_CR1_UE) &&
            (((uart_reg->CR1 & USART_CR1_IDLEIE) && (sr & USART_ISR_IDLE)) ||
            ((uart_reg->CR3 & USART_CR3_EIE) && (sr & isr_err_flags))))
        {
            uart_irq_hndlr(obj_list[i]);
            break;
        }
    }
}
#elif defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx)
extern "C" void USART3_4_IRQHandler(void)
{
    for(uint8_t i = 2; i <= 3; i++)
    {
        USART_TypeDef *uart_reg = uart_regs[i];
        uint32_t sr = uart_reg->ISR;
        
        if((uart_reg->CR1 & USART_CR1_UE) &&
            (((uart_reg->CR1 & USART_CR1_IDLEIE) && (sr & USART_ISR_IDLE)) ||
            ((uart_reg->CR3 & USART_CR3_EIE) && (sr & isr_err_flags))))
        {
            uart_irq_hndlr(obj_list[i]);
            break;
        }
    }
}
#elif defined(STM32F091xC) || defined(STM32F098xx)
extern "C" void USART3_8_IRQHandler(void)
{
    for(uint8_t i = 2; i <= 7; i++)
    {
        USART_TypeDef *uart_reg = uart_regs[i];
        uint32_t sr = uart_reg->ISR;
        
        if((uart_reg->CR1 & USART_CR1_UE) &&
            (((uart_reg->CR1 & USART_CR1_IDLEIE) && (sr & USART_ISR_IDLE)) ||
            ((uart_reg->CR3 & USART_CR3_EIE) && (sr & isr_err_flags))))
        {
            uart_irq_hndlr(obj_list[i]);
            break;
        }
    }
}
#endif
