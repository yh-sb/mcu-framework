#pragma once

#include <stdint.h>

#include "uart.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

namespace drv
{
enum onewire_err_t
{
	ONEWIRE_ERR_NONE      =  0,
	ONEWIRE_ERR_LINE_BUSY = -1,
	ONEWIRE_ERR_NO_DEV    = -2,
	ONEWIRE_ERR_TX_FAIL   = -4,
	ONEWIRE_ERR_RX_FAIL   = -5
};

class onewire
{
	public:
		onewire(hal::uart &uart);
		~onewire();
		
		int8_t tx(uint64_t rom, void *tx_buff, uint16_t tx_size);
		int8_t rx(uint64_t rom, void *rx_buff, uint16_t rx_size);
		int8_t exch(uint64_t rom, void *tx_buff, uint16_t tx_size,
			void *rx_buff, uint16_t rx_size);
		int8_t read_rom(uint64_t *rom);
		//int8_t search(uint64_t *rom_list, size_t *rom_list_size);
	
	private:
		int8_t do_reset();
		int8_t send_buff(void *buff, uint8_t size);
		int8_t read_buff(void *buff, uint8_t size);
		int8_t send_byte(uint8_t byte);
		int8_t read_byte(uint8_t *byte);
		
		hal::uart &_uart;
		SemaphoreHandle_t api_lock;
};
}
