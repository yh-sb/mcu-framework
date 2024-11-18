#include <cassert>
#include "periph/spi_stm32f1.hpp"
#include "rcc.hpp"
#include "gpio_hw_mapping.hpp"
#include "stm32f1xx.h"
#include "core_cm3.h"

using namespace periph;

static constexpr auto spis = 3; // Total number of SPI periph in STM32F1

static spi_stm32f1 *obj_list[spis];

constexpr SPI_TypeDef *const spi_regs[spis] =
{
    SPI1,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    SPI2,
#else
    nullptr,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    SPI3
#else
    nullptr
#endif
};

constexpr IRQn_Type irqn[spis] =
{
    SPI1_IRQn,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    SPI2_IRQn,
#else
    static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    SPI3_IRQn
#else
    static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[spis] =
{
    RCC_APB2ENR_SPI1EN,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    RCC_APB1ENR_SPI2EN,
#else
    0,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    RCC_APB1ENR_SPI3EN
#else
    0
#endif
};

constexpr uint32_t rcc_rst[spis] =
{
    RCC_APB2RSTR_SPI1RST,
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
    RCC_APB1RSTR_SPI2RST,
#else
    0,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    RCC_APB1RSTR_SPI3RST
#else
    0
#endif
};

constexpr volatile uint32_t *rcc_en_reg[spis] =
{
    &RCC->APB2ENR, &RCC->APB1ENR, &RCC->APB1ENR
};

constexpr volatile uint32_t *rcc_rst_reg[spis] =
{
    &RCC->APB2RSTR, &RCC->APB1RSTR, &RCC->APB1RSTR
};

constexpr rcc::clk_source rcc_src[spis] =
{
    rcc::clk_source::apb2, rcc::clk_source::apb1, rcc::clk_source::apb1
};

spi_stm32f1::spi_stm32f1(uint8_t spi, uint32_t baudrate, enum cpol cpol, enum cpha cpha,
    enum bit_order bit_order, dma_stm32f1 &dma_write, dma_stm32f1 &dma_read,
    gpio_stm32f1 &mosi, gpio_stm32f1 &miso, gpio_stm32f1 &clk):
    baud(baudrate),
    _cpol(cpol),
    _cpha(cpha),
    _bit_order(bit_order),
    api_lock(nullptr),
    irq_res(res::ok),
    mosi(mosi),
    miso(miso),
    clk(clk),
    cs(nullptr),
    write_dma(dma_write),
    write_buff(nullptr),
    read_dma(dma_read),
    read_buff(nullptr)
{
    assert(spi > 0 && spi <= spis && spi_regs[spi - 1]);
    assert(baud > 0);
    assert(write_dma.direction() == dma_stm32f1::direction::memory_to_periph);
    assert(write_dma.increment_size() == 8);
    assert(read_dma.direction() == dma_stm32f1::direction::periph_to_memory);
    assert(read_dma.increment_size() == 8);
    assert(mosi.mode() == gpio::mode::alternate_function);
    assert(miso.mode() == gpio::mode::alternate_function);
    assert(clk.mode() == gpio::mode::alternate_function);
    
    assert(api_lock = xSemaphoreCreateMutex());
    
    this->spi = spi - 1;
    obj_list[this->spi] = this;
    
    *rcc_en_reg[this->spi] |= rcc_en[this->spi];
    *rcc_rst_reg[this->spi] |= rcc_rst[this->spi];
    *rcc_rst_reg[this->spi] &= ~rcc_rst[this->spi];
    
    SPI_TypeDef *spi_reg = spi_regs[this->spi];
    
    // Master mode
    spi_reg->CR1 |= SPI_CR1_MSTR;
    
    if(_cpol == cpol::low)
    {
        spi_reg->CR1 &= ~SPI_CR1_CPOL;
    }
    else
    {
        spi_reg->CR1 |= SPI_CR1_CPOL;
    }
    
    if(_cpha == cpha::leading)
    {
        spi_reg->CR1 &= ~SPI_CR1_CPHA;
    }
    else
    {
        spi_reg->CR1 |= SPI_CR1_CPHA;
    }
    
    if(_bit_order == bit_order::msb)
    {
        spi_reg->CR1 &= ~SPI_CR1_LSBFIRST;
    }
    else
    {
        spi_reg->CR1 |= SPI_CR1_LSBFIRST;
    }
    
    // Disable NSS hardware management
    spi_reg->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    
    uint8_t presc = calc_presc(this->spi, baud);
    spi_reg->CR1 |= ((presc << SPI_CR1_BR_Pos) & SPI_CR1_BR);
    
    spi_reg->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    
    // TODO: overrun error has happend each time with this bit
    //spi_reg->CR2 |= SPI_CR2_ERRIE;
    
    spi_reg->CR1 |= SPI_CR1_SPE;
    
    write_dma.destination((uint8_t *)&spi_reg->DR);
    read_dma.source((uint8_t *)&spi_reg->DR);
    write_dma.set_callback([this](dma_stm32f1::event event) { on_dma_write(event); });
    read_dma.set_callback([this](dma_stm32f1::event event) { on_dma_read(event); });
    
    NVIC_ClearPendingIRQ(irqn[this->spi]);
    NVIC_SetPriority(irqn[this->spi], 4);
    NVIC_EnableIRQ(irqn[this->spi]);
}

spi_stm32f1::~spi_stm32f1()
{
    NVIC_DisableIRQ(irqn[spi]);
    spi_regs[spi]->CR1 &= ~SPI_CR1_SPE;
    *rcc_en_reg[spi] &= ~rcc_en[spi];
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
    obj_list[spi] = nullptr;
}

void spi_stm32f1::baudrate(uint32_t baudrate)
{
    assert(baudrate > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    baud = baudrate;
    uint8_t presc = calc_presc(spi, baudrate);
    
    SPI_TypeDef *spi_reg = spi_regs[spi];
    
    spi_reg->CR1 &= ~(SPI_CR1_SPE | SPI_CR1_BR);
    spi_reg->CR1 |= ((presc << SPI_CR1_BR_Pos) & SPI_CR1_BR) | SPI_CR1_SPE;
    
    xSemaphoreGive(api_lock);
}

void spi_stm32f1::cpol(enum cpol cpol)
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    _cpol = cpol;
    SPI_TypeDef *spi_reg = spi_regs[spi];
    
    spi_reg->CR1 &= ~SPI_CR1_SPE;
    
    if(_cpol == cpol::low)
    {
        spi_reg->CR1 &= ~SPI_CR1_CPOL;
    }
    else
    {
        spi_reg->CR1 |= SPI_CR1_CPOL;
    }
    
    spi_reg->CR1 |= SPI_CR1_SPE;
    
    xSemaphoreGive(api_lock);
}

void spi_stm32f1::cpha(enum cpha cpha)
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    _cpha = cpha;
    SPI_TypeDef *spi_reg = spi_regs[spi];
    
    spi_reg->CR1 &= ~SPI_CR1_SPE;
    
    if(_cpha == cpha::leading)
    {
        spi_reg->CR1 &= ~SPI_CR1_CPHA;
    }
    else
    {
        spi_reg->CR1 |= SPI_CR1_CPHA;
    }
    
    spi_reg->CR1 |= SPI_CR1_SPE;
    
    xSemaphoreGive(api_lock);
}

void spi_stm32f1::bit_order(enum bit_order bit_order)
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    _bit_order = bit_order;
    SPI_TypeDef *spi_reg = spi_regs[spi];
    
    spi_reg->CR1 &= ~SPI_CR1_SPE;
    
    if(_bit_order == bit_order::msb)
    {
        spi_reg->CR1 &= ~SPI_CR1_LSBFIRST;
    }
    else
    {
        spi_reg->CR1 |= SPI_CR1_LSBFIRST;
    }
    
    spi_reg->CR1 |= SPI_CR1_SPE;
    
    xSemaphoreGive(api_lock);
}

spi::res spi_stm32f1::write(const void *buff, uint16_t size, gpio *cs)
{
    assert(buff);
    assert(size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    this->cs = cs;
    if(this->cs)
    {
        this->cs->set(0);
    }
    
    task = xTaskGetCurrentTaskHandle();
    write_buff = buff;
    write_dma.source((uint8_t*)write_buff);
    write_dma.size(size);
    write_dma.start();
    spi_regs[this->spi]->CR2 |= SPI_CR2_TXDMAEN;
    
    // Task will be unlocked later from isr
    ulTaskNotifyTake(true, portMAX_DELAY);
    
    xSemaphoreGive(api_lock);
    return irq_res;
}

spi::res spi_stm32f1::write(uint8_t byte, gpio *cs)
{
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    this->cs = cs;
    if(this->cs)
    {
        this->cs->set(0);
    }
    
    task = xTaskGetCurrentTaskHandle();
    write_buff = &byte;
    write_dma.source((uint8_t*)write_buff);
    write_dma.size(1);
    write_dma.start();
    spi_regs[spi]->CR2 |= SPI_CR2_TXDMAEN;
    
    // Task will be unlocked later from isr
    ulTaskNotifyTake(true, portMAX_DELAY);
    
    xSemaphoreGive(api_lock);
    return irq_res;
}

spi::res spi_stm32f1::read(void *buff, uint16_t size, gpio *cs)
{
    assert(buff);
    assert(size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    this->cs = cs;
    if(this->cs)
    {
        this->cs->set(0);
    }
    
    read_buff = buff;
    read_dma.destination((uint8_t*)read_buff);
    read_dma.size(size);
    read_dma.start();
    
    task = xTaskGetCurrentTaskHandle();
    // Setup tx for reception
    write_buff = read_buff;
    write_dma.source((uint8_t*)write_buff);
    write_dma.size(size);
    write_dma.start();
    uint8_t dr = spi_regs[spi]->DR;
    spi_regs[spi]->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    
    // Task will be unlocked later from isr
    ulTaskNotifyTake(true, portMAX_DELAY);
    
    xSemaphoreGive(api_lock);
    return irq_res;
}

spi::res spi_stm32f1::write_read(const void *write_buff, void *read_buff, uint16_t size, gpio *cs)
{
    assert(write_buff);
    assert(read_buff);
    assert(size > 0);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    this->cs = cs;
    if(this->cs)
    {
        this->cs->set(0);
    }
    
    this->read_buff = read_buff;
    read_dma.destination((uint8_t*)(this->read_buff));
    read_dma.size(size);
    uint8_t dr = spi_regs[spi]->DR;
    
    task = xTaskGetCurrentTaskHandle();
    this->write_buff = write_buff;
    write_dma.source((uint8_t*)(this->write_buff));
    write_dma.size(size);
    read_dma.start();
    write_dma.start();
    
    spi_regs[spi]->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    
    // Task will be unlocked later from isr
    ulTaskNotifyTake(true, portMAX_DELAY);
    
    xSemaphoreGive(api_lock);
    return irq_res;
}

uint8_t spi_stm32f1::calc_presc(uint8_t spi, uint32_t baud)
{
    uint32_t div = rcc::frequency(rcc_src[spi]) / baud;
    
    // Baud rate is too low or too high
    assert(div > 1 && div <= 256);
    
    uint8_t presc = 0;
    // Calculate how many times div can be divided by 2
    while((div /= 2) > 1)
    {
        presc++;
    }
    
    return presc;
}

void spi_stm32f1::on_dma_write(dma_stm32f1::event event)
{
    if(event == dma_stm32f1::event::half_complete)
    {
        return;
    }
    
    BaseType_t hi_task_woken = 0;
    
    write_buff = nullptr;
    SPI_TypeDef *spi_reg = spi_regs[spi];
    
    spi_reg->CR2 &= ~SPI_CR2_TXDMAEN;
    if(event == dma_stm32f1::event::complete)
    {
        if(read_buff)
        {
            return;
        }
        
        spi_reg->CR2 |= SPI_CR2_TXEIE;
    }
    else if(event == dma_stm32f1::event::error)
    {
        if(read_buff)
        {
            spi_reg->CR2 &= ~SPI_CR2_RXDMAEN;
            read_dma.stop();
            read_buff = nullptr;
        }
        
        if(cs)
        {
            cs->set(1);
            cs = nullptr;
        }
        
        irq_res = res::error;
        vTaskNotifyGiveFromISR(task, &hi_task_woken);
        portYIELD_FROM_ISR(hi_task_woken);
    }
}

void spi_stm32f1::on_dma_read(dma_stm32f1::event event)
{
    if(event == dma_stm32f1::event::half_complete)
    {
        return;
    }
    
    SPI_TypeDef *spi_reg = spi_regs[spi];
    
    read_buff = nullptr;
    spi_reg->CR2 &= ~SPI_CR2_RXDMAEN;
    if(event == dma_stm32f1::event::complete)
    {
        irq_res = res::ok;
    }
    else if(event == dma_stm32f1::event::error)
    {
        irq_res = res::error;
    }
    
    if(cs)
    {
        cs->set(1);
        cs = nullptr;
    }
    
    BaseType_t hi_task_woken = 0;
    vTaskNotifyGiveFromISR(task, &hi_task_woken);
    portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void spi_irq_hndlr(periph::spi_stm32f1 *obj)
{
    SPI_TypeDef *spi_reg = spi_regs[obj->spi];
    uint32_t sr = spi_reg->SR;
    uint8_t dr = spi_reg->DR;
    
    if((spi_reg->CR2 & SPI_CR2_TXEIE) && (sr & SPI_SR_TXE))
    {
        spi_reg->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_TXDMAEN);
        // Wait for last byte transmission/receiving
        while(spi_reg->SR & SPI_SR_BSY)
        {
        }
        obj->irq_res = spi::res::ok;
    }
    else if((spi_reg->CR2 & SPI_CR2_ERRIE) && (sr & (SPI_SR_UDR | SPI_SR_MODF | SPI_SR_OVR)))
    {
        spi_reg->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
        if(obj->write_buff)
        {
            obj->write_dma.stop();
            obj->write_buff = nullptr;
        }
        if(obj->read_buff)
        {
            obj->read_dma.stop();
            obj->read_buff = nullptr;
        }
        obj->irq_res = spi::res::error;
    }
    
    if(obj->cs)
    {
        obj->cs->set(1);
        obj->cs = nullptr;
    }
    
    BaseType_t hi_task_woken = 0;
    vTaskNotifyGiveFromISR(obj->task, &hi_task_woken);
    portYIELD_FROM_ISR(hi_task_woken);
}

extern "C" void SPI1_IRQHandler(void)
{
    spi_irq_hndlr(obj_list[0]);
}

#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102xB) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void SPI2_IRQHandler(void)
{
    spi_irq_hndlr(obj_list[1]);
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
extern "C" void SPI3_IRQHandler(void)
{
    spi_irq_hndlr(obj_list[2]);
}
#endif
