#include <cassert>
#include "periph/dma_stm32f1.hpp"
#include "stm32f1xx.h"
#include "core_cm3.h"

using namespace periph;

static constexpr auto dmas = 2; // Total number of DMA controllers
static constexpr auto channels = 7; // Total number of DMA channels

static dma_stm32f1 *obj_list[dmas][channels];

constexpr DMA_Channel_TypeDef *const dma_channel[dmas][channels] =
{
    {
        DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4,
        DMA1_Channel5, DMA1_Channel6, DMA1_Channel7
    },
    {
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
        DMA2_Channel1, DMA2_Channel2, DMA2_Channel3,
#else
        nullptr, nullptr, nullptr,
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC) || defined(STM32F105xC) || defined(STM32F107xC)
        DMA2_Channel4, DMA2_Channel5,
#else
        nullptr, nullptr,
#endif
        // DMA2 doesn't have 6th and 7th channels
        nullptr, nullptr
    }
};

constexpr IRQn_Type irqn_num[dmas][channels] =
{
    {
        DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn,
        DMA1_Channel4_IRQn, DMA1_Channel5_IRQn, DMA1_Channel6_IRQn,
        DMA1_Channel7_IRQn
    },
    {
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
        DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn,
#else
        static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
        static_cast<IRQn_Type>(0),
#endif
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG)
        DMA2_Channel4_5_IRQn, DMA2_Channel4_5_IRQn,
#elif defined(STM32F105xC) || defined(STM32F107xC)
        DMA2_Channel4_IRQn, DMA2_Channel5_IRQn,
#else
        static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0),
#endif
        // DMA2 doesn't have 6th and 7th channels
        static_cast<IRQn_Type>(0), static_cast<IRQn_Type>(0)
    }
};

dma_stm32f1::dma_stm32f1(uint8_t dma, uint8_t channel, enum direction direction,
    uint8_t increment_size):
    _direction(direction),
    inc_size(increment_size),
    src(0),
    dst(0),
    _size(0)
{
    assert(dma >= 1 && dma <= dmas);
    assert(channel >= 1 && channel <= channels);
    assert(dma_channel[dma - 1][channel - 1]);
    // Only DMA2 is able to perform memory-to-memory transfers
    assert(_direction != direction::memory_to_memory || dma != 1);
    assert(inc_size == 8 || inc_size == 16 || inc_size == 32);
    
    this->dma = dma - 1;
    _channel = channel - 1;
    
    obj_list[this->dma][_channel] = this;
    
    if(this->dma == 0)
    {
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;
        DMA1->IFCR = DMA_IFCR_CGIF1 << (_channel * DMA_IFCR_CGIF2_Pos);
    }
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    else
    {
        RCC->AHBENR |= RCC_AHBENR_DMA2EN;
        DMA2->IFCR = DMA_IFCR_CGIF1 << (_channel * DMA_IFCR_CGIF2_Pos);
    }
#endif
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[this->dma][_channel];
    
    // Setup data direction
    channel_reg->CCR &= ~(DMA_CCR_DIR | DMA_CCR_MEM2MEM);
    if(_direction == direction::memory_to_periph)
    {
        channel_reg->CCR |= DMA_CCR_DIR;
    }
    else if(_direction == direction::memory_to_memory)
    {
        channel_reg->CCR |= DMA_CCR_MEM2MEM;
    }
    
    // Setup data size
    channel_reg->CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE);
    if(inc_size == 16)
    {
        channel_reg->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
    }
    else if(inc_size == 32)
    {
        channel_reg->CCR |= DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
    }
    
    // Setup incremental mode
    channel_reg->CCR |= DMA_CCR_MINC;
    if(_direction == direction::memory_to_memory)
    {
        channel_reg->CCR |= DMA_CCR_PINC;
    }
    else
    {
        channel_reg->CCR &= ~DMA_CCR_PINC;
    }
    
    channel_reg->CCR |= DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE;
    
    NVIC_SetPriority(irqn_num[this->dma][_channel], 3);
    NVIC_EnableIRQ(irqn_num[this->dma][_channel]);
}

dma_stm32f1::~dma_stm32f1()
{
    NVIC_DisableIRQ(irqn_num[dma][_channel]);
    dma_channel[dma][_channel]->CCR &= ~DMA_CCR_EN;
    obj_list[dma][_channel] = nullptr;
}

void dma_stm32f1::source(const void *source)
{
    assert(source);
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[dma][_channel];
    // This action allowed only when DMA is disabled
    assert(!(channel_reg->CCR & DMA_CCR_EN));
    
    src = (uint32_t)source;
    if(_direction == direction::memory_to_periph)
    {
        channel_reg->CMAR = src;
    }
    else
    {
        channel_reg->CPAR = src;
    }
}

void dma_stm32f1::destination(void *destination)
{
    assert(destination);
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[dma][_channel];
    // This action allowed only when DMA is disabled
    assert(!(channel_reg->CCR & DMA_CCR_EN));
    
    dst = (uint32_t)destination;
    if(_direction == direction::memory_to_periph)
    {
        channel_reg->CPAR = dst;
    }
    else
    {
        channel_reg->CMAR = dst;
    }
}

void dma_stm32f1::size(uint16_t size)
{
    assert(size > 0);
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[dma][_channel];
    // This action allowed only when DMA is disabled
    assert(!(channel_reg->CCR & DMA_CCR_EN));
    
    _size = size;
    channel_reg->CNDTR = _size;
}

void dma_stm32f1::direction(enum direction direction)
{
    // Only DMA2 is able to perform memory-to-memory transfers
    assert(direction != direction::memory_to_memory || dma != 0);
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[dma][_channel];
    // This action allowed only when DMA is disabled
    assert(!(channel_reg->CCR & DMA_CCR_EN));
    
    _direction = direction;
    // Setup data direction
    channel_reg->CCR &= ~(DMA_CCR_DIR | DMA_CCR_MEM2MEM);
    if(_direction == direction::memory_to_periph)
    {
        channel_reg->CCR |= DMA_CCR_DIR;
    }
    else if(_direction == direction::memory_to_memory)
    {
        channel_reg->CCR |= DMA_CCR_MEM2MEM;
    }
    
    // Setup incremental mode
    channel_reg->CCR |= DMA_CCR_MINC;
    if(_direction == direction::memory_to_memory)
    {
        channel_reg->CCR |= DMA_CCR_PINC;
    }
    else
    {
        channel_reg->CCR &= ~DMA_CCR_PINC;
    }
}

void dma_stm32f1::increment_size(uint8_t increment_size)
{
    assert(inc_size == 8 || inc_size == 16 || inc_size == 32);
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[dma][_channel];
    // This action allowed only when DMA is disabled
    assert(!(channel_reg->CCR & DMA_CCR_EN));
    
    inc_size = increment_size;
    // Setup data size
    channel_reg->CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE);
    if(inc_size == 16)
    {
        channel_reg->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
    }
    else if(inc_size == 32)
    {
        channel_reg->CCR |= DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1;
    }
}

uint16_t dma_stm32f1::transfered() const
{
    return _size - dma_channel[dma][_channel]->CNDTR;
}

uint16_t dma_stm32f1::remains() const
{
    return dma_channel[dma][_channel]->CNDTR;
}

void dma_stm32f1::set_callback(std::function<void(event event)> on_event)
{
    this->on_event = std::move(on_event);
}

void dma_stm32f1::start(bool is_cyclic)
{
    assert(_size > 0);
    assert(src);
    assert(dst);
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[dma][_channel];
    // This action allowed only when DMA is disabled
    assert(!(channel_reg->CCR & DMA_CCR_EN));
    
    // Disable circular mode
    channel_reg->CCR &= ~DMA_CCR_CIRC;
    
    if(dma == 0)
    {
        // Clear interrupt flag to prevent transfer complete interrupt
        DMA1->IFCR = DMA_IFCR_CTCIF1 << (_channel * DMA_IFCR_CGIF2_Pos);
    }
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    else
    {
        DMA2->IFCR = DMA_IFCR_CTCIF1 << (_channel * DMA_IFCR_CGIF2_Pos);
    }
#endif
    
    NVIC_EnableIRQ(irqn_num[dma][_channel]);
    channel_reg->CCR |= DMA_CCR_EN | (is_cyclic ? DMA_CCR_CIRC : 0);
}

void dma_stm32f1::stop()
{
    NVIC_DisableIRQ(irqn_num[dma][_channel]);
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[dma][_channel];
    channel_reg->CCR &= ~DMA_CCR_EN;
    
    // Waiting for end of DMA transmission
    while(channel_reg->CCR & DMA_CCR_EN)
    {
    }
}

bool dma_stm32f1::is_busy() const
{
    return dma_channel[dma][_channel]->CCR & DMA_CCR_EN;
}

extern "C" void dma_irq_hndlr(dma_stm32f1 *obj)
{
    uint32_t isr;
    volatile uint32_t *iclr;
    if(obj->dma == 0)
    {
        isr = DMA1->ISR;
        iclr = &DMA1->IFCR;
    }
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
    else
    {
        isr = DMA2->ISR;
        iclr = &DMA2->IFCR;
    }
#endif
    
    DMA_Channel_TypeDef *channel_reg = dma_channel[obj->dma][obj->_channel];
    uint8_t isr_offset = obj->_channel * DMA_ISR_GIF2_Pos;
    
    if((channel_reg->CCR & DMA_CCR_TCIE) && (isr & (DMA_ISR_TCIF1 << isr_offset)))
    {
        *iclr = DMA_IFCR_CTCIF1 << isr_offset;
        
        // Do not stop DMA because of it was started in circular mode
        if(!(channel_reg->CCR & DMA_CCR_CIRC))
        {
            channel_reg->CCR &= ~DMA_CCR_EN;
        }
        
        if(obj->on_event)
        {
            obj->on_event(dma_stm32f1::event::complete);
        }
    }
    else if((channel_reg->CCR & DMA_CCR_HTIE) && (isr & (DMA_ISR_HTIF1 << isr_offset)))
    {
        *iclr = DMA_IFCR_CHTIF1 << isr_offset;
        if(obj->on_event)
        {
            obj->on_event(dma_stm32f1::event::half_complete);
        }
    }
    else if((channel_reg->CCR & DMA_CCR_TEIE) && (isr & (DMA_ISR_TEIF1 << isr_offset)))
    {
        *iclr = DMA_IFCR_CTEIF1 << isr_offset;
        
        // Do not stop DMA because of it was started in circular mode
        if(!(channel_reg->CCR & DMA_CCR_CIRC))
        {
            channel_reg->CCR &= ~DMA_CCR_EN;
        }
        
        if(obj->on_event)
        {
            obj->on_event(dma_stm32f1::event::error);
        }
    }
}

extern "C" void DMA1_Channel1_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][0]);
}

extern "C" void DMA1_Channel2_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][1]);
}

extern "C" void DMA1_Channel3_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][2]);
}

extern "C" void DMA1_Channel4_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][3]);
}

extern "C" void DMA1_Channel5_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][4]);
}

extern "C" void DMA1_Channel6_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][5]);
}

extern "C" void DMA1_Channel7_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][6]);
}

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || \
    defined(STM32F107xC)
extern "C" void DMA2_Channel1_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][0]);
}

extern "C" void DMA2_Channel2_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][1]);
}

extern "C" void DMA2_Channel3_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][2]);
}
#endif

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F103xE) || defined(STM32F103xG)
extern "C" void DMA2_Channel4_5_IRQHandler(void)
{
    uint32_t isr = DMA2->ISR;
    
    if(isr & DMA_ISR_GIF4)
    {
        dma_irq_hndlr(obj_list[1][3]);
    }
    else if(isr & DMA_ISR_GIF5)
    {
        dma_irq_hndlr(obj_list[1][4]);
    }
}
#elif defined(STM32F105xC) || defined(STM32F107xC)
extern "C" void DMA2_Channel4_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][3]);
}

extern "C" void DMA2_Channel5_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][4]);
}
#endif
