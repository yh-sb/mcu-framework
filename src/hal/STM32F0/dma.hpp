#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace hal { class dma; }
// For internal use only! (called from ISR)
extern "C" void dma_irq_hndlr(hal::dma *obj);

namespace hal
{
typedef enum
{
	DMA_1,
	DMA_2, // for STM32F09x only
	DMA_END
} dma_t;

typedef enum
{
	DMA_CH_1,
	DMA_CH_2,
	DMA_CH_3,
	DMA_CH_4,
	DMA_CH_5,
	DMA_CH_6,
	DMA_CH_7,
	DMA_CH_END
} dma_ch_t;

typedef enum
{
	DMA_DIR_PERIPH_TO_MEM,
	DMA_DIR_MEM_TO_PERIPH,
	DMA_DIR_MEM_TO_MEM
} dma_dir_t;

typedef enum
{
	DMA_INC_SIZE_8,
	DMA_INC_SIZE_16,
	DMA_INC_SIZE_32
} dma_inc_size_t;

typedef enum
{
	DMA_EVENT_CMPLT,
	DMA_EVENT_HALF,
	DMA_EVENT_ERROR
} dma_event_t;

typedef void (*dma_cb_t)(dma *dma, dma_event_t event, void *ctx);

class dma
{
	public:
		dma(dma_t dma, dma_ch_t ch, dma_dir_t dir, dma_inc_size_t inc_size);
		~dma();
		
		void src(void *src);
		void dst(void *dst);
		void size(uint16_t size);
		void dir(dma_dir_t dir);
		dma_dir_t dir() const { return _dir; }
		void inc_size(dma_inc_size_t inc_size);
		dma_inc_size_t inc_size() const { return _inc_size; }
		uint16_t transfered() const;
		uint16_t remain() const;
		
		void start_once(dma_cb_t cb, void *ctx);
		void start_cyclic(dma_cb_t cb, void *ctx);
		void stop();
		
		bool busy();
		
	private:
		dma_t _dma;
		dma_ch_t _ch;
		dma_dir_t _dir;
		dma_inc_size_t _inc_size;
		uint32_t _src;
		uint32_t _dst;
		uint16_t _size;
		void *_ctx;
		dma_cb_t _cb;
		friend void ::dma_irq_hndlr(dma *obj);
};
}
