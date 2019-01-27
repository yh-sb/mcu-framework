#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "drv/onewire/onewire.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

namespace drv
{
class ds18b20
{
	public:
		enum resol_t
		{
			RESOL_9_BIT,  /* 0,5    °C,  94 ms */
			RESOL_10_BIT, /* 0,25   °C, 188 ms */
			RESOL_11_BIT, /* 0,125  °C, 375 ms */
			RESOL_12_BIT  /* 0,0625 °C, 750 ms. Default */
		};

		enum res_t
		{
			RES_OK           =  0,
			RES_NO_DEV       = -1,
			RES_CRC_ERR      = -2,
			RES_ONEWIRE_BUSY = -3,
		};
		
		ds18b20(onewire &onewire);
		~ds18b20();
		
		int8_t get_temp(uint64_t rom, float *temp);
		int8_t set_resol(uint64_t rom, resol_t resol);  // 10 ms
		int8_t get_resol(uint64_t rom, resol_t *resol); // 6  ms
		int8_t set_th(uint64_t rom, uint8_t th);        // 10 ms
		int8_t get_th(uint64_t rom, uint8_t *th);       // 6  ms
		int8_t set_tl(uint64_t rom, uint8_t tl);        // 10 ms
		int8_t get_tl(uint64_t rom, uint8_t *tl);       // 6  ms
		int8_t write_eeprom(uint64_t rom);              // 1  ms
		int8_t restore_eeprom(uint64_t rom);            // 1  ms
	
	private:
		int8_t write_scratchpad(uint64_t rom, uint8_t th, uint8_t tl, uint8_t conf);
		int8_t read_scratchpad(uint64_t rom, void *rx_buff, uint8_t rx_size);
		
		onewire &_onewire;
		SemaphoreHandle_t api_lock;
		resol_t _resol;
};
}
