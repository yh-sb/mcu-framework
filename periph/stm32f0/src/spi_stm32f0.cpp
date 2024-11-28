#include <cassert>
#include "periph/spi_stm32f0.hpp"
#include "rcc.hpp"
#include "gpio_hw_mapping.hpp"
#include "stm32f0xx.h"
#include "core_cm0.h"

using namespace periph;

static constexpr auto spis = 2; // Total number of SPI interfaces

static spi_stm32f0 *obj_list[spis];

constexpr SPI_TypeDef *const spi_regs[spis] =
{
    SPI1,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
    SPI2
#else
    nullptr
#endif
};

constexpr IRQn_Type irqn[spis] =
{
    SPI1_IRQn,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
    SPI2_IRQn
#else
    static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[spis] =
{
    RCC_APB2ENR_SPI1EN,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
    RCC_APB1ENR_SPI2EN
#else
    0
#endif
};

constexpr uint32_t rcc_rst[spis] =
{
    RCC_APB2RSTR_SPI1RST,
#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
    RCC_APB1RSTR_SPI2RST
#else
    0
#endif
};

constexpr volatile uint32_t *rcc_en_reg[spis] =
{
    &RCC->APB2ENR, &RCC->APB1ENR
};

constexpr volatile uint32_t *rcc_rst_reg[spis] =
{
    &RCC->APB2RSTR, &RCC->APB1RSTR
};

constexpr rcc::clk_source rcc_src[spis] =
{
    rcc::clk_source::apb2, rcc::clk_source::apb1
};

/* Get AF index by spi interface and port number:
   af = spi2afr[_spi][gpio.port()]
*/
constexpr uint8_t spi2afr[][gpio_hw_mapping::ports] =
{
    {0, 0, 0, 0, 1},
    {0, 0, 1, 1}
};

spi_stm32f0::spi_stm32f0(uint8_t spi, uint32_t baudrate, enum cpol cpol, enum cpha cpha,
    enum bit_order bit_order, dma_stm32f0 &dma_write, dma_stm32f0 &dma_read,
    gpio_stm32f0 &mosi, gpio_stm32f0 &miso, gpio_stm32f0 &clk):
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
    assert(write_dma.direction() == dma_stm32f0::direction::memory_to_periph);
    assert(write_dma.increment_size() == 8);
    assert(read_dma.direction() == dma_stm32f0::direction::periph_to_memory);
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
    
    gpio_af_init(mosi);
    gpio_af_init(miso);
    gpio_af_init(clk);
    
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    remap_dma(this->spi, dma_write);
    remap_dma(this->spi, dma_read);
    
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
    
    // Set FRXTH if data size < 16 bit
    spi_reg->CR2 |= SPI_CR2_FRXTH;
    
    spi_reg->CR1 |= SPI_CR1_SPE;
    
    write_dma.destination((uint8_t *)&spi_reg->DR);
    read_dma.source((uint8_t *)&spi_reg->DR);
    write_dma.set_callback([this](dma_stm32f0::event event) { on_dma_write(event); });
    read_dma.set_callback([this](dma_stm32f0::event event) { on_dma_read(event); });
    
    NVIC_ClearPendingIRQ(irqn[this->spi]);
    NVIC_SetPriority(irqn[this->spi], 4);
    NVIC_EnableIRQ(irqn[this->spi]);
}

spi_stm32f0::~spi_stm32f0()
{
    NVIC_DisableIRQ(irqn[spi]);
    spi_regs[spi]->CR1 &= ~SPI_CR1_SPE;
    *rcc_en_reg[spi] &= ~rcc_en[spi];
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
    obj_list[spi] = nullptr;
}

void spi_stm32f0::baudrate(uint32_t baudrate)
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

void spi_stm32f0::cpol(enum cpol cpol)
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

void spi_stm32f0::cpha(enum cpha cpha)
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

void spi_stm32f0::bit_order(enum bit_order bit_order)
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

spi::res spi_stm32f0::write(const void *buff, uint16_t size, gpio *cs)
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
    spi_regs[spi]->CR2 |= SPI_CR2_TXDMAEN;
    
    // Task will be unlocked later from isr
    ulTaskNotifyTake(true, portMAX_DELAY);
    
    xSemaphoreGive(api_lock);
    return irq_res;
}

spi::res spi_stm32f0::write(uint8_t byte, gpio *cs)
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

spi::res spi_stm32f0::read(void *buff, uint16_t size, gpio *cs)
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

spi::res spi_stm32f0::write_read(const void *write_buff, void *read_buff, uint16_t size, gpio *cs)
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

void spi_stm32f0::gpio_af_init(gpio_stm32f0 &gpio)
{
    GPIO_TypeDef *gpio_reg = gpio_hw_mapping::gpio[static_cast<uint8_t>(gpio.port())];
    uint8_t pin = gpio.pin();
    
    // Push-pull
    gpio_reg->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
    
    // Configure alternate function
    gpio_reg->AFR[pin / 8] &= ~(GPIO_AFRL_AFSEL0 << ((pin % 8) * 4));
    gpio_reg->AFR[pin / 8] |= spi2afr[spi][static_cast<uint8_t>(gpio.port())] << ((pin % 8) * 4);
}

void spi_stm32f0::remap_dma(uint8_t spi, dma_stm32f0 &dma)
{
    auto ch = dma.channel();
    
#if defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx)
    if((ch == 3 || ch == 4) && spi == 1)
    {
        SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_SPI2_DMA_RMP;
    }
    else if((ch == 5 || ch == 6) && spi == 1)
    {
        SYSCFG->CFGR1 |= SYSCFG_CFGR1_SPI2_DMA_RMP;
    }
#elif defined(STM32F091xC) || defined(STM32F098xx)
#error Not implemented. Need to change DMA1_CSELR: "DMAx channel selection registers"
#endif
}

uint8_t spi_stm32f0::calc_presc(uint8_t spi, uint32_t baud)
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

void spi_stm32f0::on_dma_write(dma_stm32f0::event event)
{
    if(event == dma_stm32f0::event::half_complete)
    {
        return;
    }
    
    BaseType_t hi_task_woken = 0;
    
    write_buff = nullptr;
    SPI_TypeDef *spi_reg = spi_regs[spi];
    
    spi_reg->CR2 &= ~SPI_CR2_TXDMAEN;
    if(event == dma_stm32f0::event::complete)
    {
        if(read_buff)
        {
            return;
        }
        
        spi_reg->CR2 |= SPI_CR2_TXEIE;
    }
    else if(event == dma_stm32f0::event::error)
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

void spi_stm32f0::on_dma_read(dma_stm32f0::event event)
{
    if(event == dma_stm32f0::event::half_complete)
    {
        return;
    }
    
    SPI_TypeDef *spi_reg = spi_regs[spi];
    
    read_buff = nullptr;
    spi_reg->CR2 &= ~SPI_CR2_RXDMAEN;
    if(event == dma_stm32f0::event::complete)
    {
        irq_res = res::ok;
    }
    else if(event == dma_stm32f0::event::error)
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

extern "C" void spi_irq_hndlr(periph::spi_stm32f0 *obj)
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
#if defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F071xB) || defined(STM32F072xB) || defined(STM32F078xx) || \
    defined(STM32F091xC) || defined(STM32F098xx)
    else if((spi_reg->CR2 & SPI_CR2_ERRIE) && (sr & (SPI_SR_UDR | SPI_SR_MODF | SPI_SR_OVR)))
#else
    else if((spi_reg->CR2 & SPI_CR2_ERRIE) && (sr & (SPI_SR_MODF | SPI_SR_OVR)))
#endif
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

#if defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) || \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx)
extern "C" void SPI2_IRQHandler(void)
{
    spi_irq_hndlr(obj_list[1]);
}
#endif
