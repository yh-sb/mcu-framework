#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "drv/onewire/onewire.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

namespace drv
{
typedef enum
{
	DS18B20_RESOL_9_BIT,  /* 0,5    °C,  94 ms */
	DS18B20_RESOL_10_BIT, /* 0,25   °C, 188 ms */
	DS18B20_RESOL_11_BIT, /* 0,125  °C, 375 ms */
	DS18B20_RESOL_12_BIT  /* 0,0625 °C, 750 ms. Default */
} ds18b20_resol_t;

enum ds18b20_err_t
{
	DS18B20_ERR_NONE         =  0,
	DS18B20_ERR_NO_DEV       = -1,
	DS18B20_ERR_CRC_ERR      = -2,
	DS18B20_ERR_ONEWIRE_BUSY = -3,
};

class ds18b20
{
	public:
		ds18b20(onewire &onewire);
		~ds18b20();
		
		int8_t get_temp(uint64_t rom, float *temp);
		int8_t set_resol(uint64_t rom, ds18b20_resol_t resol);  // 10 ms
		int8_t get_resol(uint64_t rom, ds18b20_resol_t *resol); // 6  ms
		int8_t set_th(uint64_t rom, uint8_t th);                // 10 ms
		int8_t get_th(uint64_t rom, uint8_t *th);               // 6  ms
		int8_t set_tl(uint64_t rom, uint8_t tl);                // 10 ms
		int8_t get_tl(uint64_t rom, uint8_t *tl);               // 6  ms
		int8_t write_eeprom(uint64_t rom);                      // 1  ms
		int8_t restore_eeprom(uint64_t rom);                    // 1  ms
	
	private:
		int8_t write_scratchpad(uint64_t rom, uint8_t th, uint8_t tl, uint8_t conf);
		int8_t read_scratchpad(uint64_t rom, void *rx_buff, uint8_t rx_size);
		
		onewire &_onewire;
		SemaphoreHandle_t api_lock;
		ds18b20_resol_t _resol;
};
}
