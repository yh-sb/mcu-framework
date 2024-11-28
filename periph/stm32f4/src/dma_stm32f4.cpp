#include <cassert>
#include "periph/dma_stm32f4.hpp"
#include "stm32f4xx.h"
#include "core_cm4.h"

using namespace periph;

static constexpr auto dmas = 2; // Total number of DMA controllers
static constexpr auto streams = 8; // Total number of DMA streams
static constexpr auto channels = 8; // Total number of DMA channels in one stream

static dma_stm32f4 *obj_list[dmas][streams];

constexpr DMA_Stream_TypeDef *const dma_stream[dmas][streams] =
{
    {
        DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4,
        DMA1_Stream5, DMA1_Stream6, DMA1_Stream7
    },
    {
        DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4,
        DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
    }
};

constexpr IRQn_Type irqn_num[dmas][streams] =
{
    {
        DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn,
        DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn,
        DMA1_Stream6_IRQn, DMA1_Stream7_IRQn
    },
    {
        DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn,
        DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn,
        DMA2_Stream6_IRQn, DMA2_Stream7_IRQn
    }
};

constexpr volatile uint32_t *const isr_reg[dmas][streams] =
{
    {
        &DMA1->LISR, &DMA1->LISR, &DMA1->LISR, &DMA1->LISR, &DMA1->HISR,
        &DMA1->HISR, &DMA1->HISR, &DMA1->HISR
    },
    {
        &DMA2->LISR, &DMA2->LISR, &DMA2->LISR, &DMA2->LISR, &DMA2->HISR,
        &DMA2->HISR, &DMA2->HISR, &DMA2->HISR
    }
};

constexpr volatile uint32_t *const iclr_reg[dmas][streams] =
{
    {
        &DMA1->LIFCR, &DMA1->LIFCR, &DMA1->LIFCR, &DMA1->LIFCR, &DMA1->HIFCR,
        &DMA1->HIFCR, &DMA1->HIFCR, &DMA1->HIFCR
    },
    {
        &DMA2->LIFCR, &DMA2->LIFCR, &DMA2->LIFCR, &DMA2->LIFCR, &DMA2->HIFCR,
        &DMA2->HIFCR, &DMA2->HIFCR, &DMA2->HIFCR
    }
};

// DMA ISR registers offset for each stream
constexpr uint32_t isr_offsets[streams] =
{
    DMA_LISR_FEIF0_Pos, DMA_LISR_FEIF1_Pos, DMA_LISR_FEIF2_Pos,
    DMA_LISR_FEIF3_Pos, DMA_HISR_FEIF4_Pos, DMA_HISR_FEIF5_Pos,
    DMA_HISR_FEIF6_Pos, DMA_HISR_FEIF7_Pos
};

dma_stm32f4::dma_stm32f4(uint8_t dma, uint8_t stream, uint8_t channel,
    enum direction direction, uint8_t increment_size):
    stream(stream),
    channel(channel),
    _direction(direction),
    inc_size(increment_size),
    src(0),
    dst(0),
    _size(0)
{
    assert(dma >= 1 && dma <= dmas);
    assert(stream < streams);
    assert(channel < channels);
    // Only DMA2 is able to perform memory-to-memory transfers
    assert(_direction != direction::memory_to_memory || dma != 1);
    assert(inc_size == 8 || inc_size == 16 || inc_size == 32);
    
    this->dma = dma - 1;
    
    obj_list[this->dma][stream] = this;
    
    RCC->AHB1ENR |= (this->dma == 0) ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
    
    DMA_Stream_TypeDef *stream_reg = dma_stream[this->dma][stream];
    
    *iclr_reg[this->dma][stream] = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
        DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0) << isr_offsets[stream];
    
    stream_reg->CR &= ~DMA_SxCR_CHSEL;
    stream_reg->CR |= channel << DMA_SxCR_CHSEL_Pos;
    
    // Setup data direction
    stream_reg->CR &= ~DMA_SxCR_DIR;
    if(_direction == direction::memory_to_periph)
    {
        stream_reg->CR |= DMA_SxCR_DIR_0;
    }
    else if(_direction == direction::memory_to_memory)
    {
        stream_reg->CR |= DMA_SxCR_DIR_1;
    }
    
    // Setup data size
    stream_reg->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
    if(inc_size == 16)
    {
        stream_reg->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;
    }
    else if(inc_size == 32)
    {
        stream_reg->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
    }
    
    // Setup incremental mode
    stream_reg->CR |= DMA_SxCR_MINC;
    if(_direction == direction::memory_to_memory)
    {
        stream_reg->CR |= DMA_SxCR_PINC;
    }
    else
    {
        stream_reg->CR &= ~DMA_SxCR_PINC;
    }
    
    stream_reg->CR |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    
    NVIC_SetPriority(irqn_num[this->dma][stream], 3);
    NVIC_EnableIRQ(irqn_num[this->dma][stream]);
}

dma_stm32f4::~dma_stm32f4()
{
    NVIC_DisableIRQ(irqn_num[dma][stream]);
    dma_stream[dma][stream]->CR &= ~DMA_SxCR_EN;
    obj_list[dma][stream] = nullptr;
}

void dma_stm32f4::source(const void *source)
{
    assert(source);
    
    DMA_Stream_TypeDef *stream_reg = dma_stream[dma][stream];
    // This action allowed only when DMA is disabled
    assert(!(stream_reg->CR & DMA_SxCR_EN));
    
    src = (uint32_t)source;
    if(_direction == direction::memory_to_periph)
    {
        stream_reg->M0AR = src;
    }
    else
    {
        stream_reg->PAR = src;
    }
}

void dma_stm32f4::destination(void *destination)
{
    assert(destination);
    
    DMA_Stream_TypeDef *stream_reg = dma_stream[dma][stream];
    // This action allowed only when DMA is disabled
    assert(!(stream_reg->CR & DMA_SxCR_EN));
    
    dst = (uint32_t)destination;
    if(_direction == direction::memory_to_periph)
    {
        stream_reg->PAR = dst;
    }
    else
    {
        stream_reg->M0AR = dst;
    }
}

void dma_stm32f4::size(uint16_t size)
{
    assert(size > 0);
    
    DMA_Stream_TypeDef *stream_reg = dma_stream[dma][stream];
    // This action allowed only when DMA is disabled
    assert(!(stream_reg->CR & DMA_SxCR_EN));
    
    _size = size;
    stream_reg->NDTR = _size;
}

void dma_stm32f4::direction(enum direction direction)
{
    // Only DMA2 is able to perform memory-to-memory transfers
    assert(direction != direction::memory_to_memory || dma != 0);
    
    DMA_Stream_TypeDef *stream_reg = dma_stream[dma][stream];
    // This action allowed only when DMA is disabled
    assert(!(stream_reg->CR & DMA_SxCR_EN));
    
    _direction = direction;
    // Setup data direction
    stream_reg->CR &= ~DMA_SxCR_DIR;
    if(_direction == direction::memory_to_periph)
    {
        stream_reg->CR |= DMA_SxCR_DIR_0;
    }
    else if(_direction == direction::memory_to_memory)
    {
        stream_reg->CR |= DMA_SxCR_DIR_1;
    }
    
    // Setup incremental mode
    stream_reg->CR |= DMA_SxCR_MINC;
    if(_direction == direction::memory_to_memory)
    {
        stream_reg->CR |= DMA_SxCR_PINC;
    }
    else
    {
        stream_reg->CR &= ~DMA_SxCR_PINC;
    }
}

void dma_stm32f4::increment_size(uint8_t increment_size)
{
    assert(inc_size == 8 || inc_size == 16 || inc_size == 32);
    
    DMA_Stream_TypeDef *stream_reg = dma_stream[dma][stream];
    // This action allowed only when DMA is disabled
    assert(!(stream_reg->CR & DMA_SxCR_EN));
    
    inc_size = increment_size;
    // Setup data size
    stream_reg->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
    if(inc_size == 16)
    {
        stream_reg->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;
    }
    else if(inc_size == 32)
    {
        stream_reg->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
    }
}

uint16_t dma_stm32f4::transfered() const
{
    return _size - dma_stream[dma][stream]->NDTR;
}

uint16_t dma_stm32f4::remains() const
{
    return dma_stream[dma][stream]->NDTR;
}

void dma_stm32f4::set_callback(std::function<void(event event)> on_event)
{
    this->on_event = std::move(on_event);
}

void dma_stm32f4::start(bool is_cyclic)
{
    assert(_size > 0);
    assert(src);
    assert(dst);
    
    DMA_Stream_TypeDef *stream_reg = dma_stream[dma][stream];
    // This action allowed only when DMA is disabled
    assert(!(stream_reg->CR & DMA_SxCR_EN));
    
    // Disable circular mode
    stream_reg->CR &= ~DMA_SxCR_CIRC;
    
    // Clear interrupt flag to prevent transfer complete interrupt
    *iclr_reg[dma][stream] = DMA_LIFCR_CTCIF0 << isr_offsets[stream];
    
    NVIC_EnableIRQ(irqn_num[dma][stream]);
    stream_reg->CR |= DMA_SxCR_EN | (is_cyclic ? DMA_SxCR_CIRC : 0);
}

void dma_stm32f4::stop()
{
    NVIC_DisableIRQ(irqn_num[dma][stream]);
    
    DMA_Stream_TypeDef *stream_reg = dma_stream[dma][stream];
    stream_reg->CR &= ~DMA_SxCR_EN;
    
    // Waiting for end of DMA transmission
    while(stream_reg->CR & DMA_SxCR_EN)
    {
    }
}

bool dma_stm32f4::is_busy() const
{
    return dma_stream[dma][stream]->CR & DMA_SxCR_EN;
}

extern "C" void dma_irq_hndlr(dma_stm32f4 *obj)
{
    DMA_Stream_TypeDef *stream_reg = dma_stream[obj->dma][obj->stream];
    uint32_t isr = *isr_reg[obj->dma][obj->stream];
    volatile uint32_t *iclr = iclr_reg[obj->dma][obj->stream];
    uint8_t isr_offset = isr_offsets[obj->stream];
    
    if((stream_reg->CR & DMA_SxCR_TCIE) && (isr & (DMA_LISR_TCIF0 << isr_offset)))
    {
        *iclr = DMA_LIFCR_CTCIF0 << isr_offset;
        
        // Do not stop DMA because of it was started in circular mode
        if(!(stream_reg->CR & DMA_SxCR_CIRC))
        {
            stream_reg->CR &= ~DMA_SxCR_EN;
        }
        
        if(obj->on_event)
        {
            obj->on_event(dma_stm32f4::event::complete);
        }
    }
    else if((stream_reg->CR & DMA_SxCR_HTIE) && (isr & (DMA_LISR_HTIF0 << isr_offset)))
    {
        *iclr = DMA_LIFCR_CHTIF0 << isr_offset;
        if(obj->on_event)
        {
            obj->on_event(dma_stm32f4::event::half_complete);
        }
    }
    else if((stream_reg->CR & DMA_SxCR_TEIE) && (isr & (DMA_LISR_TEIF0 << isr_offset)))
    {
        *iclr = DMA_LIFCR_CTEIF0 << isr_offset;
        
        // Do not stop DMA because of it was started in circular mode
        if(!(stream_reg->CR & DMA_SxCR_CIRC))
        {
            stream_reg->CR &= ~DMA_SxCR_EN;
        }
        
        if(obj->on_event)
        {
            obj->on_event(dma_stm32f4::event::error);
        }
    }
}

extern "C" void DMA1_Stream0_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][0]);
}

extern "C" void DMA1_Stream1_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][1]);
}

extern "C" void DMA1_Stream2_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][2]);
}

extern "C" void DMA1_Stream3_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][3]);
}

extern "C" void DMA1_Stream4_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][4]);
}

extern "C" void DMA1_Stream5_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][5]);
}

extern "C" void DMA1_Stream6_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][6]);
}

extern "C" void DMA1_Stream7_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[0][7]);
}

extern "C" void DMA2_Stream0_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][0]);
}

extern "C" void DMA2_Stream1_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][1]);
}

extern "C" void DMA2_Stream2_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][2]);
}

extern "C" void DMA2_Stream3_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][3]);
}

extern "C" void DMA2_Stream4_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][4]);
}

extern "C" void DMA2_Stream5_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][5]);
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][6]);
}

extern "C" void DMA2_Stream7_IRQHandler(void)
{
    dma_irq_hndlr(obj_list[1][7]);
}
