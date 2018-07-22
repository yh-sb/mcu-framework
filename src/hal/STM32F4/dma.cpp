#include <stddef.h>

#include "common/macros.h"

#include "dma.hpp"

#include "hal/STM32F4/CMSIS/device-support/include/stm32f4xx.h"
#include "hal/STM32F4/CMSIS/core-support/core_cm4.h"

using namespace hal;

#define IRQ_PRIORITY 2
#define CHSEL_OFFSET 25

static DMA_Stream_TypeDef *const stream_list[DMA_END][DMA_STREAM_END] =
{
	{
		DMA1_Stream0, DMA1_Stream1, DMA1_Stream2,
		DMA1_Stream3, DMA1_Stream4, DMA1_Stream5,
		DMA1_Stream6, DMA1_Stream7
	},
	{
		DMA2_Stream0, DMA2_Stream1, DMA2_Stream2,
		DMA2_Stream3, DMA2_Stream4, DMA2_Stream5,
		DMA2_Stream6, DMA2_Stream7
	}
};

static IRQn_Type const irq_list[DMA_END][DMA_STREAM_END] =
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

static volatile uint32_t *const isr_list[DMA_END][DMA_STREAM_END] =
{
	{
		&DMA1->LISR, &DMA1->LISR, &DMA1->LISR,
		&DMA1->LISR, &DMA1->HISR, &DMA1->HISR,
		&DMA1->HISR, &DMA1->HISR
	},
	{
		&DMA2->LISR, &DMA2->LISR, &DMA2->LISR,
		&DMA2->LISR, &DMA2->HISR, &DMA2->HISR,
		&DMA2->HISR, &DMA2->HISR
	}
};

static volatile uint32_t *const isr_clr_list[DMA_END][DMA_STREAM_END] =
{
	{
		&DMA1->LIFCR, &DMA1->LIFCR, &DMA1->LIFCR,
		&DMA1->LIFCR, &DMA1->HIFCR, &DMA1->HIFCR,
		&DMA1->HIFCR, &DMA1->HIFCR
	},
	{
		&DMA2->LIFCR, &DMA2->LIFCR, &DMA2->LIFCR,
		&DMA2->LIFCR, &DMA2->HIFCR, &DMA2->HIFCR,
		&DMA2->HIFCR, &DMA2->HIFCR
	}
};

/* DMA_LISR, DMA_HISR, DMA_LIFCR and DMA_HIFCR regs offset for each stream */
static uint32_t const isr_shift_list[DMA_STREAM_END] =
{
	0, 6, 16, 22, 0, 6, 16, 22
};

static dma *obj_list[DMA_END][DMA_STREAM_END];

dma::dma(dma_t dma, dma_stream_t stream, dma_ch_t ch, dma_dir_t dir,
	dma_inc_size_t inc_size):
	_dma(dma),
	_stream(stream),
	_ch(ch),
	_dir(dir),
	_inc_size(inc_size),
	_src(0),
	_dst(0),
	_size(0),
	_ctx(NULL),
	_cb(NULL)
{
	ASSERT(_dma < DMA_END);
	ASSERT(_stream < DMA_STREAM_END);
	ASSERT(_ch < DMA_CH_END);
	/* Only DMA2 is able to perform memory-to-memory transfers */
	ASSERT(_dir != DMA_DIR_MEM_TO_MEM || _dma != DMA_1);
	
	obj_list[_dma][_stream] = this;
	
	RCC->AHB1ENR |= (_dma == DMA_1) ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
	
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	dma_stream->CR &= ~DMA_SxCR_EN;
	
	/* Clear isr reg flags */
	volatile uint32_t *isr_clr_reg = isr_clr_list[dma][stream];
	*isr_clr_reg = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
		DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0) << isr_shift_list[_stream];
	
	/* Setup channel */
	dma_stream->CR &= ~DMA_SxCR_CHSEL;
	dma_stream->CR |= ch << CHSEL_OFFSET;
	
	/* Setup low priority */
	dma_stream->CR &= ~DMA_SxCR_PL;
	
	/* Setup data direction. Default is peripheral to memory */
	dma_stream->CR &= ~DMA_SxCR_DIR;
	if(_dir == DMA_DIR_MEM_TO_PERIPH)
		dma_stream->CR |= DMA_SxCR_DIR_0;
	else if(_dir == DMA_DIR_MEM_TO_MEM)
		dma_stream->CR |= DMA_SxCR_DIR_1;
	
	/* Setup data size. Default is 8 */
	dma_stream->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
	if(_inc_size == DMA_INC_SIZE_16)
		dma_stream->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;
	else if(_inc_size == DMA_INC_SIZE_32)
		dma_stream->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
	
	/* Setup incremental mode */
	dma_stream->CR |= DMA_SxCR_MINC;
	if(_dir == DMA_DIR_MEM_TO_MEM)
		dma_stream->CR |= DMA_SxCR_PINC;
	else
		dma_stream->CR &= ~DMA_SxCR_PINC;
	
	/* Single transfer (without memory burst) */
	dma_stream->CR &= ~(DMA_SxCR_MBURST | DMA_SxCR_PBURST);
	
	dma_stream->CR |= DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE |
		DMA_SxCR_DMEIE;
	
	NVIC_SetPriority(irq_list[_dma][_stream], IRQ_PRIORITY);
	NVIC_EnableIRQ(irq_list[_dma][_stream]);
}

dma::~dma()
{
	
}

void dma::src(void *src)
{
	ASSERT(src);

	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_stream->CR & DMA_SxCR_EN));
	
	_src = (uint32_t)src;
	if(_dir == DMA_DIR_MEM_TO_PERIPH)
		dma_stream->M0AR = _src;
	else
		dma_stream->PAR = _src;
}

void dma::dst(void *dst)
{
	ASSERT(dst);
	
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_stream->CR & DMA_SxCR_EN));
	
	_dst = (uint32_t)dst;
	if(_dir == DMA_DIR_MEM_TO_PERIPH)
		dma_stream->PAR = _dst;
	else
		dma_stream->M0AR = _dst;
}

void dma::size(uint16_t size)
{
	ASSERT(size > 0);
	
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_stream->CR & DMA_SxCR_EN));
	
	_size = size;
	dma_stream->NDTR = _size;
}

void dma::dir(dma_dir_t dir)
{
	/* Only DMA2 is able to perform memory-to-memory transfers */
	ASSERT(dir != DMA_DIR_MEM_TO_MEM || _dma != DMA_1);
	
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	/* this action allowed only when DMA is disabled */
	ASSERT(!(dma_stream->CR & DMA_SxCR_EN));
	
	_dir = dir;
	/* Setup data direction. Default is peripheral to memory */
	dma_stream->CR &= ~DMA_SxCR_DIR;
	if(_dir == DMA_DIR_MEM_TO_PERIPH)
		dma_stream->CR |= DMA_SxCR_DIR_0;
	else if(_dir == DMA_DIR_MEM_TO_MEM)
		dma_stream->CR |= DMA_SxCR_DIR_1;
	
	/* Setup incremental mode */
	if(_dir == DMA_DIR_MEM_TO_MEM)
		dma_stream->CR |= DMA_SxCR_PINC;
	else
		dma_stream->CR &= ~DMA_SxCR_PINC;
}

void dma::inc_size(dma_inc_size_t inc_size)
{
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_stream->CR & DMA_SxCR_EN));
	
	_inc_size = inc_size;
	/* Setup data size. Default is 8 */
	dma_stream->CR &= ~(DMA_SxCR_MSIZE | DMA_SxCR_PSIZE);
	if(_inc_size == DMA_INC_SIZE_16)
		dma_stream->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;
	else if(_inc_size == DMA_INC_SIZE_32)
		dma_stream->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
}

uint16_t dma::transfered() const
{
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	return _size - dma_stream->NDTR;
}

uint16_t dma::remain() const
{
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	return (uint16_t)dma_stream->NDTR;
}

void dma::start_once(dma_cb_t cb, void *ctx)
{
	ASSERT(_size > 0);
	ASSERT(_src);
	ASSERT(_dst);
	
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_stream->CR & DMA_SxCR_EN));
	
	_ctx = ctx;
	_cb = cb;
	
	/* Disable circular mode */
	dma_stream->CR &= ~DMA_SxCR_CIRC;
	/* Clear interrupt flag to prevent transfer complete interrupt */
	volatile uint32_t *isr_clr_reg = isr_clr_list[_dma][_stream];
	*isr_clr_reg = DMA_LIFCR_CTCIF0 << isr_shift_list[_stream];
	NVIC_EnableIRQ(irq_list[_dma][_stream]);
	dma_stream->CR |= DMA_SxCR_EN;
}

void dma::start_cyclic(dma_cb_t cb, void *ctx)
{
	ASSERT(_size > 0);
	ASSERT(_src);
	ASSERT(_dst);
	
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	/* This action allowed only when DMA is disabled */
	ASSERT(!(dma_stream->CR & DMA_SxCR_EN));
	
	_ctx = ctx;
	_cb = cb;
	
	/* Enable circular mode */
	dma_stream->CR |= DMA_SxCR_CIRC;
	/* Clear interrupt flag to prevent transfer complete interrupt */
	volatile uint32_t *isr_clr_reg = isr_clr_list[_dma][_stream];
	*isr_clr_reg = DMA_LIFCR_CTCIF0 << isr_shift_list[_stream];
	NVIC_EnableIRQ(irq_list[_dma][_stream]);
	dma_stream->CR |= DMA_SxCR_EN;
}

void dma::stop()
{
	_cb = NULL;
	_ctx = NULL;
	
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	
	NVIC_DisableIRQ(irq_list[_dma][_stream]);
	dma_stream->CR &= ~DMA_SxCR_EN;
	
	/* Waiting for end of DMA transmission */
	while(dma_stream->CR & DMA_SxCR_EN);
}

bool dma::busy()
{
	DMA_Stream_TypeDef *dma_stream = stream_list[_dma][_stream];
	return (bool)(dma_stream->CR & DMA_SxCR_EN);
}

extern "C" void dma_irq_hndlr(hal::dma *obj)
{
	DMA_Stream_TypeDef *dma_stream = stream_list[obj->_dma][obj->_stream];
	uint32_t isr = *isr_list[obj->_dma][obj->_stream];
	volatile uint32_t *isr_clr_reg = isr_clr_list[obj->_dma][obj->_stream];
	uint8_t shift = isr_shift_list[obj->_stream];
	
	if((dma_stream->CR & DMA_SxCR_TCIE) && (isr & (DMA_LISR_TCIF0 << shift)))
	{
		*isr_clr_reg = DMA_LIFCR_CTCIF0 << shift;
		dma_stream->CR &= ~DMA_SxCR_EN;
		if(obj->_cb)
			obj->_cb(obj, DMA_EVENT_CMPLT, obj->_ctx);
	}
	else if((dma_stream->CR & DMA_SxCR_HTIE) &&
		(isr & (DMA_LISR_HTIF0 << shift)))
	{
		*isr_clr_reg = DMA_LIFCR_CHTIF0 << shift;
		if(obj->_cb)
			obj->_cb(obj, DMA_EVENT_HALF, obj->_ctx);
	}
	else if((dma_stream->CR & DMA_SxCR_TEIE) &&
		(isr & (DMA_LISR_TEIF0 << shift)))
	{
		*isr_clr_reg = DMA_LIFCR_CTEIF0 << shift;
		dma_stream->CR &= ~DMA_SxCR_EN;
		if(obj->_cb)
			obj->_cb(obj, DMA_EVENT_ERROR, obj->_ctx);
	}
}

extern "C" void DMA1_Stream0_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_1][DMA_STREAM_0]);
}

extern "C" void DMA1_Stream1_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_1][DMA_STREAM_1]);
}

extern "C" void DMA1_Stream2_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_1][DMA_STREAM_2]);
}

extern "C" void DMA1_Stream3_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_1][DMA_STREAM_3]);
}

extern "C" void DMA1_Stream4_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_1][DMA_STREAM_4]);
}

extern "C" void DMA1_Stream5_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_1][DMA_STREAM_5]);
}

extern "C" void DMA1_Stream6_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_1][DMA_STREAM_6]);
}

extern "C" void DMA1_Stream7_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_1][DMA_STREAM_7]);
}

extern "C" void DMA2_Stream0_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_2][DMA_STREAM_0]);
}

extern "C" void DMA2_Stream1_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_2][DMA_STREAM_1]);
}

extern "C" void DMA2_Stream2_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_2][DMA_STREAM_2]);
}

extern "C" void DMA2_Stream3_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_2][DMA_STREAM_3]);
}

extern "C" void DMA2_Stream4_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_2][DMA_STREAM_4]);
}

extern "C" void DMA2_Stream5_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_2][DMA_STREAM_5]);
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_2][DMA_STREAM_6]);
}

extern "C" void DMA2_Stream7_IRQHandler(void)
{
	dma_irq_hndlr(obj_list[DMA_2][DMA_STREAM_7]);
}
