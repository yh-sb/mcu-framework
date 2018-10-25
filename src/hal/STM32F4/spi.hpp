#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "hal/STM32F4/gpio.hpp"
#include "hal/STM32F4/dma.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

namespace hal { class spi; }
// For internal use only! (called from ISR)
extern "C" void spi_irq_hndlr(hal::spi *obj);

namespace hal
{
typedef enum
{
	SPI_1,
	SPI_2,
	SPI_3,
	SPI_4,
	SPI_5,
	SPI_6,
	SPI_END
} spi_t;

typedef enum
{
	SPI_CPOL_1,
	SPI_CPOL_0
} spi_cpol_t;

typedef enum
{
	SPI_CPHA_0,
	SPI_CPHA_1
} spi_cpha_t;

typedef enum
{
	SPI_BIT_ORDER_MSB,
	SPI_BIT_ORDER_LSB
} spi_bit_order_t;

enum spi_err_t
{
	SPI_ERR_NONE =  0,
	SPI_ERR_FAIL = -1
};

class spi
{
	public:
		spi(spi_t spi, uint32_t baud, spi_cpol_t cpol, spi_cpha_t cpha,
			spi_bit_order_t bit_order, dma &dma_tx, dma &dma_rx, gpio &mosi,
			gpio &miso, gpio &clk);
		~spi();
		
		void baud(uint32_t baud);
		uint32_t baud() const { return _baud; }
		void cpol(spi_cpol_t cpol);
		spi_cpol_t cpol() const { return _cpol; }
		void cpha(spi_cpha_t cpha);
		spi_cpha_t cpha() const { return _cpha; }
		void bit_order(spi_bit_order_t bit_order);
		spi_bit_order_t bit_order() const { return _bit_order; };
		int8_t tx(void *buff, uint16_t size, gpio *cs = NULL);
		int8_t tx(uint8_t byte, gpio *cs = NULL);
		int8_t rx(void *buff, uint16_t size, gpio *cs = NULL);
		int8_t exch(void *buff_tx, void *buff_rx, uint16_t size, gpio *cs = NULL);
		
		spi &operator = (const spi &);
	
	private:
		spi_t _spi;
		uint32_t _baud;
		spi_cpol_t _cpol;
		spi_cpha_t _cpha;
		spi_bit_order_t _bit_order;
		
		SemaphoreHandle_t api_lock;
		SemaphoreHandle_t irq_lock;
		int8_t irq_res;
		
		gpio &_mosi;
		gpio &_miso;
		gpio &_clk;
		gpio *_cs;
		
		dma &tx_dma;
		void *tx_buff;
		
		dma &rx_dma;
		void *rx_buff;
		int8_t rx_irq_res;
		
		static void on_dma_tx(dma *dma, dma::event_t event, void *ctx);
		static void on_dma_rx(dma *dma, dma::event_t event, void *ctx);
		friend void ::spi_irq_hndlr(spi *obj);
};
}
