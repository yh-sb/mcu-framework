#pragma once

#include "drv/singlewire/singlewire.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class dht
{
	public:
		enum res_t
		{
			RES_OK      =  0,
			RES_NODEV   = -1,
			RES_DEVERR  = -2,
			RES_BUSY    = -3,
			RES_CRC_ERR = -4
		};
		
		enum dht_dev_t
		{
			DHT11,
			DHT22
		};
		
		struct val_t
		{
			uint16_t rh_x10;
			int16_t t_x10;
		};
		
		dht(singlewire &singlewire, dht_dev_t dht_dev);
		~dht();
		
		int8_t get(val_t *val);
	
	private:
		singlewire &_singlewire;
		dht_dev_t _dht_dev;
		SemaphoreHandle_t api_lock;
		TickType_t recent_time;
		bool is_first_measurement;
		
		void wait_ready();
		bool is_crc_valid(uint8_t *buff);
		val_t parce_data(uint8_t *buff);
};
}
