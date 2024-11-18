
#include <algorithm>
#include <cassert>
#include "periph/i2c_stm32f4.hpp"
#include "rcc.hpp"
#include "gpio_hw_mapping.hpp"
#include "stm32f4xx.h"
#include "core_cm4.h"

using namespace periph;

constexpr auto i2cs = 3; // Total number of I2C interfaces
constexpr auto i2c_standard_max_speed = 100000; // bps
constexpr auto i2c_fast_max_speed = 400000; // bps

static i2c_stm32f4 *obj_list[i2cs];

constexpr I2C_TypeDef *const i2c_regs[i2cs] =
{
    I2C1, I2C2,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
    defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
    defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
    I2C3
#else
    nullptr
#endif
};

constexpr IRQn_Type irqn_events[i2cs] =
{
    I2C1_EV_IRQn, I2C2_EV_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
    defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
    defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
    I2C3_EV_IRQn
#else
    static_cast<IRQn_Type>(0)
#endif
};

constexpr IRQn_Type irqn_errors[i2cs] =
{
    I2C1_ER_IRQn, I2C2_ER_IRQn,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
    defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
    defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
    I2C3_ER_IRQn
#else
    static_cast<IRQn_Type>(0)
#endif
};

constexpr uint32_t rcc_en[i2cs] =
{
    RCC_APB1ENR_I2C1EN, RCC_APB1ENR_I2C2EN,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
    defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
    defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
    RCC_APB1ENR_I2C3EN
#else
    0
#endif
};

constexpr uint32_t rcc_rst[i2cs] =
{
    RCC_APB1RSTR_I2C1RST, RCC_APB1RSTR_I2C2RST,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
    defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
    defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
    RCC_APB1RSTR_I2C3RST
#else
    0
#endif
};

constexpr uint8_t gpio_af[i2cs] =
{
    0x04,
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE)
    0x09, 0x09
#else
    0x04, 0x04
#endif
};

i2c_stm32f4::i2c_stm32f4(uint8_t i2c, uint32_t baudrate, dma_stm32f4 &dma_write,
    dma_stm32f4 &dma_read, gpio_stm32f4 &sda, gpio_stm32f4 &scl):
    baud(baudrate),
    irq_res(res::ok),
    sda(sda),
    scl(scl),
    write_dma(dma_write),
    write_buff(nullptr),
    write_size(0),
    read_dma(dma_read),
    read_buff(nullptr),
    read_size(0)
{
    assert(i2c > 0 && i2c <= i2cs && i2c_regs[i2c - 1]);
    assert(baud > 0 && baud <= i2c_fast_max_speed);
    assert(dma_write.direction() == dma_stm32f4::direction::memory_to_periph);
    assert(dma_write.increment_size() == 8);
    assert(dma_read.direction() == dma_stm32f4::direction::periph_to_memory);
    assert(dma_read.increment_size() == 8);
    assert(sda.mode() == gpio::mode::alternate_function);
    assert(scl.mode() == gpio::mode::alternate_function);
    
    assert(api_lock = xSemaphoreCreateMutex());
    
    this->i2c = i2c - 1;
    obj_list[this->i2c] = this;
    
    RCC->APB1ENR |= rcc_en[this->i2c];
    RCC->APB1RSTR |= rcc_rst[this->i2c];
    RCC->APB1RSTR &= ~rcc_rst[this->i2c];
    
    gpio_af_init(sda);
    gpio_af_init(scl);
    
    I2C_TypeDef *i2c_reg = i2c_regs[this->i2c];
    
    i2c_reg->CR1 |= I2C_CR1_SWRST;
    i2c_reg->CR1 &= ~I2C_CR1_SWRST;
    
    // Setup i2c speed
    uint8_t freq, trise;
    uint32_t ccr;
    calc_clk(this->i2c, baud, freq, trise, ccr);
    i2c_reg->CR2 |= (freq & I2C_CR2_FREQ);
    i2c_reg->TRISE = trise;
    i2c_reg->CCR = ccr;
    //i2c_reg->TRISE = 11;
    //i2c_reg->CCR = 30;
    
    i2c_reg->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
    i2c_reg->CR2 |= (I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
    i2c_reg->CR1 |= I2C_CR1_PE;
    
    dma_write.destination((void *)&i2c_reg->DR);
    dma_read.source((void *)&i2c_reg->DR);
    
    NVIC_ClearPendingIRQ(irqn_events[this->i2c]);
    NVIC_ClearPendingIRQ(irqn_errors[this->i2c]);
    NVIC_SetPriority(irqn_events[this->i2c], 5);
    NVIC_SetPriority(irqn_errors[this->i2c], 5);
    NVIC_EnableIRQ(irqn_events[this->i2c]);
    NVIC_EnableIRQ(irqn_errors[this->i2c]);
}

i2c_stm32f4::~i2c_stm32f4()
{
    NVIC_DisableIRQ(irqn_events[i2c]);
    NVIC_DisableIRQ(irqn_errors[i2c]);
    i2c_regs[i2c]->CR1 &= ~I2C_CR1_PE;
    RCC->APB1ENR &= ~rcc_en[i2c];
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
    obj_list[i2c] = nullptr;
}

void i2c_stm32f4::baudrate(uint32_t baudrate)
{
    assert(baudrate > 0 && baudrate <= i2c_fast_max_speed);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    baud = baudrate;
    uint8_t freq = 0, trise = 0;
    uint32_t ccr = 0;
    calc_clk(i2c, baud, freq, trise, ccr);
    
    I2C_TypeDef *i2c_reg = i2c_regs[i2c];
    
    i2c_reg->CR1 &= ~I2C_CR1_PE;
    
    i2c_reg->CR2 &= ~I2C_CR2_FREQ;
    i2c_reg->CR2 |= (freq & I2C_CR2_FREQ);
    i2c_reg->TRISE = trise;
    i2c_reg->CCR = ccr;
    
    i2c_reg->CR1 |= I2C_CR1_PE;
    
    xSemaphoreGive(api_lock);
}

i2c::res i2c_stm32f4::write_read(uint16_t address, const void *write_buff,
    uint16_t write_size, void *read_buff, uint16_t read_size)
{
    assert(write_buff);
    assert(write_size > 0);
    assert(read_buff);
    assert(read_size > 0);
    
    // 10-bit addresses haven't supported yet
    assert(address <= 127);
    
    xSemaphoreTake(api_lock, portMAX_DELAY);
    
    addr = address;
    this->write_buff = write_buff;
    this->write_size = write_size;
    this->read_buff = read_buff;
    this->read_size = read_size;
    
    task = xTaskGetCurrentTaskHandle();
    i2c_regs[this->i2c]->CR1 |= I2C_CR1_START;
    
    // Task will be unlocked later from isr
    ulTaskNotifyTake(true, portMAX_DELAY);
    
    xSemaphoreGive(api_lock);
    return irq_res;
}

void i2c_stm32f4::gpio_af_init(gpio_stm32f4 &gpio)
{
    GPIO_TypeDef *gpio_reg = gpio_hw_mapping::gpio[static_cast<uint8_t>(gpio.port())];
    uint8_t pin = gpio.pin();
    
    // Open drain
    gpio_reg->OTYPER |= GPIO_OTYPER_OT0 << pin;
    
    // Configure alternate function
    gpio_reg->AFR[pin / 8] &= ~(GPIO_AFRL_AFSEL0 << ((pin % 8) * 4));
    gpio_reg->AFR[pin / 8] |= gpio_af[i2c] << ((pin % 8) * 4);
}

void i2c_stm32f4::calc_clk(uint8_t i2c, uint32_t baud, uint8_t &freq, uint8_t &trise, uint32_t &ccr)
{
    uint32_t apb1_freq = rcc::frequency(rcc::clk_source::apb1);
    freq = apb1_freq / 1000000;
    /* According to RM0090 page 853:
    "Bits 5:0 FREQ[5:0]: Peripheral clock frequency"
    2 Mhz is min and allowed and 50 Mhz is max allowed */
    assert(freq >= 2 && freq <= 50);
    
    /* According to RM0090 page 860:
    fPCLK1 must be at least 2 MHz to achieve Sm mode I2C frequencies. It must be
    at least 4 MHz to achieve Fm mode I2C frequencies. It must be a multiple of
    10MHz to reach the 400 kHz maximum I2C Fm mode clock */
    assert(baud <= i2c_standard_max_speed || freq >= 2);
    assert(baud < i2c_standard_max_speed || freq >= 4);
    
    if(baud <= i2c_standard_max_speed)
    {
        ccr = apb1_freq / (baud << 1);
        ccr = std::max<uint32_t>(ccr, 4);
        ccr &= ~(I2C_CCR_FS | I2C_CCR_DUTY);
        trise = freq + 1;
    }
    else
    {
        ccr = apb1_freq / (baud * 25);
        ccr = std::max<uint32_t>(ccr, 1);
        ccr |= I2C_CCR_FS | I2C_CCR_DUTY;
        trise = ((freq * 300) / 1000) + 1;
    }
}

void i2c_stm32f4::on_dma_write(dma_stm32f4::event event)
{
    if(event == dma_stm32f4::event::half_complete)
    {
        return;
    }
    
    BaseType_t hi_task_woken = 0;
    
    if(event == dma_stm32f4::event::complete)
    {
        write_hndlr(this, &hi_task_woken);
    }
    else if(event == dma_stm32f4::event::error)
    {
        error_hndlr(this, res::write_error, &hi_task_woken);
    }
}

void i2c_stm32f4::on_dma_read(dma_stm32f4::event event)
{
    if(event == dma_stm32f4::event::half_complete)
    {
        return;
    }
    
    BaseType_t hi_task_woken = 0;
    
    if(event == dma_stm32f4::event::complete)
    {
        read_hndlr(this, &hi_task_woken);
    }
    else if(event == dma_stm32f4::event::error)
    {
        error_hndlr(this, res::read_error, &hi_task_woken);
    }
}

extern "C" void write_hndlr(i2c_stm32f4 *obj, BaseType_t *hi_task_woken)
{
    *hi_task_woken = 0;
    obj->write_buff = nullptr;
    obj->write_size = 0;
    
    I2C_TypeDef *i2c_reg = i2c_regs[obj->i2c];
    
    if(obj->read_buff && obj->read_size > 0)
    {
        // Generate the second start to read the data
        i2c_reg->CR1 |= I2C_CR1_START;
        return;
    }
    // Wait for last byte transmitting
    while(!(i2c_reg->SR1 & I2C_SR1_TXE));
    
    i2c_reg->CR1 |= I2C_CR1_STOP;
    
    obj->irq_res = i2c::res::ok;
    vTaskNotifyGiveFromISR(obj->task, hi_task_woken);
    portYIELD_FROM_ISR(*hi_task_woken);
}

extern "C" void read_hndlr(i2c_stm32f4 *obj, BaseType_t *hi_task_woken)
{
    *hi_task_woken = 0;
    obj->read_buff = nullptr;
    obj->read_size = 0;
    
    i2c_regs[obj->i2c]->CR2 &= ~I2C_CR2_LAST;
    i2c_regs[obj->i2c]->CR1 |= I2C_CR1_STOP;
    
    obj->irq_res = i2c::res::ok;
    vTaskNotifyGiveFromISR(obj->task, hi_task_woken);
    portYIELD_FROM_ISR(*hi_task_woken);
}

extern "C" void error_hndlr(i2c_stm32f4 *obj, i2c::res err, BaseType_t *hi_task_woken)
{
    *hi_task_woken = 0;
    
    obj->write_dma.stop();
    obj->read_dma.stop();
    
    obj->write_buff = nullptr;
    obj->write_size = 0;
    obj->read_buff = nullptr;
    obj->read_size = 0;
    
    //i2c_list[obj->_i2c]->CR2 &= ~I2C_CR2_LAST;
    i2c_regs[obj->i2c]->CR1 |= I2C_CR1_STOP;
    
    obj->irq_res = err;
    vTaskNotifyGiveFromISR(obj->task, hi_task_woken);
    portYIELD_FROM_ISR(*hi_task_woken);
}

extern "C" void i2c_event_irq_hndlr(i2c_stm32f4 *obj)
{
    I2C_TypeDef *i2c_reg = i2c_regs[obj->i2c];
    uint16_t sr1 = i2c_reg->SR1;
    
    if(sr1 & I2C_SR1_SB)
    {
        // Start condition is sent. Need to send device address
        i2c_reg->DR = (obj->addr << 1) | (obj->write_buff ? 0 : 1);
    }
    else if(sr1 & (I2C_SR1_ADDR | I2C_SR1_ADD10))
    {
        uint16_t sr2 = (uint16_t)i2c_reg->SR2;
        // Device address is sent. Need to send/receive data
        if(obj->write_buff)
        {
            obj->write_dma.source(obj->write_buff);
            obj->write_dma.size(obj->write_size);
            obj->write_dma.set_callback([obj](dma_stm32f4::event event) { obj->on_dma_write(event); });
            obj->write_dma.start();
        }
        else if(obj->read_buff)
        {
            if(obj->read_size > 1)
            {
                i2c_reg->CR1 |= I2C_CR1_ACK;
            }
            else
            {
                i2c_reg->CR1 &= ~I2C_CR1_ACK;
            }
            
            obj->read_dma.destination(obj->read_buff);
            obj->read_dma.size(obj->read_size);
            obj->read_dma.set_callback([obj](dma_stm32f4::event event) { obj->on_dma_read(event); });
            obj->read_dma.start();
        }
    }
    else if(sr1 & I2C_SR1_BTF)
    {
        // Clear BTF flag
        uint32_t dr = i2c_reg->DR;
    }
}

extern "C" void i2c_error_irq_hndlr(i2c_stm32f4 *obj)
{
    I2C_TypeDef *i2c_reg = i2c_regs[obj->i2c];
    uint32_t sr1 = i2c_reg->SR1;
    uint32_t sr2 = i2c_reg->SR2;
    uint32_t dr = i2c_reg->DR;
    
    BaseType_t hi_task_woken = 0;
    if(sr1 & I2C_SR1_AF)
    {
        // No ACK from device
        // NAK irq flag doesn't work during receiving using DMA.
        // When slave sends NAK, master still wait for some data.
        // Maybe problem in I2C_CR2_LAST?
        i2c_reg->SR1 &= ~I2C_SR1_AF;
        error_hndlr(obj, i2c::res::no_ack, &hi_task_woken);
    }
    else if(sr1 & I2C_SR1_OVR)
    {
        i2c_reg->SR1 &= ~I2C_SR1_OVR;
        error_hndlr(obj, i2c::res::read_error, &hi_task_woken);
    }
    else if(sr1 & I2C_SR1_ARLO)
    {
        i2c_reg->SR1 &= ~I2C_SR1_ARLO;
        error_hndlr(obj, i2c::res::write_error, &hi_task_woken);
    }
    else if(sr1 & I2C_SR1_BERR)
    {
        // Error: bus error is detected (misplaced start or stop condition)
        i2c_reg->SR1 &= ~I2C_SR1_BERR;
        error_hndlr(obj, i2c::res::write_error, &hi_task_woken);
    }
}

extern "C" void I2C1_EV_IRQHandler(void)
{
    i2c_event_irq_hndlr(obj_list[0]);
}

extern "C" void I2C1_ER_IRQHandler(void)
{
    i2c_error_irq_hndlr(obj_list[0]);
}

extern "C" void I2C2_EV_IRQHandler(void)
{
    i2c_event_irq_hndlr(obj_list[1]);
}

extern "C" void I2C2_ER_IRQHandler(void)
{
    i2c_error_irq_hndlr(obj_list[1]);
}

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
    defined(STM32F407xx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
    defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
extern "C" void I2C3_EV_IRQHandler(void)
{
    i2c_event_irq_hndlr(obj_list[2]);
}

extern "C" void I2C3_ER_IRQHandler(void)
{
    i2c_error_irq_hndlr(obj_list[2]);
}
#endif
