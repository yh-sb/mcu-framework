#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio/gpio.hpp"
#include "dma/dma.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace hal { class uart; }
// For internal use only! (called from ISR)
extern "C" void uart_irq_hndlr(hal::uart *obj);

namespace hal
{
class uart
{
	public:
		enum uart_t
		{
			UART_1,
			UART_2,
			UART_3,
			UART_4,
			UART_5,
			UART_6,
			UART_END
		};

		enum stopbit_t
		{
			STOPBIT_0_5,
			STOPBIT_1,
			STOPBIT_1_5,
			STOPBIT_2
		};

		enum parity_t
		{
			PARITY_NONE,
			PARITY_EVEN,
			PARITY_ODD
		};

		enum res_t
		{
			RES_OK         =  0,
			RES_RX_TIMEOUT = -1,
			RES_TX_FAIL    = -2,
			RES_RX_FAIL    = -3
		};
		
		uart(uart_t uart, uint32_t baud, stopbit_t stopbit, parity_t parity,
			dma &dma_tx, dma &dma_rx, gpio &gpio_tx, gpio &gpio_rx);
		~uart();
		
		void baud(uint32_t baud);
		uint32_t baud() const { return _baud; }
		int8_t write(const uint8_t *buff, uint16_t size);
		int8_t read(uint8_t *buff, uint16_t *size, uint32_t timeout);
		int8_t exch(uint8_t *tx_buff, uint16_t tx_size, uint8_t *rx_buff,
			uint16_t *rx_size, uint32_t timeout);
	
	private:
		uart_t _uart;
		uint32_t _baud;
		stopbit_t _stopbit;
		parity_t _parity;
		
		dma &tx_dma;
		gpio &tx_gpio;
		SemaphoreHandle_t tx_api_lock;
		SemaphoreHandle_t tx_irq_lock;
		int8_t tx_irq_res;
		
		dma &rx_dma;
		gpio &rx_gpio;
		uint16_t *rx_cnt;
		SemaphoreHandle_t rx_api_lock;
		SemaphoreHandle_t rx_irq_lock;
		int8_t rx_irq_res;
		
		static void on_dma_tx(dma *dma, dma::event_t event, void *ctx);
		static void on_dma_rx(dma *dma, dma::event_t event, void *ctx);
		friend void ::uart_irq_hndlr(uart *obj);
};
}
