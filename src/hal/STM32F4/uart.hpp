#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gpio.hpp"
#include "dma.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

namespace hal { class uart; }
// For internal use only! (called from ISR)
extern "C" void uart_irq_hndlr(hal::uart *obj);

namespace hal
{
typedef enum
{
	UART_1,
	UART_2,
	UART_3,
	UART_4,
	UART_5,
	UART_6,
	UART_END
} uart_t;

typedef enum
{
	UART_STOPBIT_0_5,
	UART_STOPBIT_1,
	UART_STOPBIT_1_5,
	UART_STOPBIT_2
} uart_stopbit_t;

typedef enum
{
	UART_PARITY_NONE,
	UART_PARITY_EVEN,
	UART_PARITY_ODD
} uart_parity_t;

enum uart_err_t
{
	UART_ERR_NONE       =  0,
	UART_ERR_RX_TIMEOUT = -1,
	UART_ERR_TX_FAIL    = -2,
	UART_ERR_RX_FAIL    = -3
};

class uart
{
	public:
		uart(uart_t uart, uint32_t baud, uart_stopbit_t stopbit,
			uart_parity_t parity, dma &dma_tx, dma &dma_rx, gpio &gpio_tx,
			gpio &gpio_rx);
		~uart();
		
		void baud(uint32_t baud);
		uint32_t baud() const { return _baud; }
		int8_t tx(const uint8_t *buff, uint16_t size);
		int8_t rx(uint8_t *buff, uint16_t *size, uint32_t timeout);
		int8_t exch(uint8_t *tx_buff, uint16_t tx_size, uint8_t *rx_buff,
			uint16_t *rx_size, uint32_t timeout);
	
	private:
		uart_t _uart;
		uint32_t _baud;
		uart_stopbit_t _stopbit;
		uart_parity_t _parity;
		
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
		
		static void on_dma_tx(dma *dma, dma_event_t event, void *ctx);
		static void on_dma_rx(dma *dma, dma_event_t event, void *ctx);
		friend void ::uart_irq_hndlr(uart *obj);
};
}
