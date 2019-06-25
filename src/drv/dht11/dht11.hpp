#pragma once

#include "drv/singlewire/singlewire.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class dht11
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
		
		struct val_t
		{
			uint8_t rh;
			uint8_t t;
		};
		
		dht11(singlewire &singlewire);
		~dht11();
		
		int8_t get(val_t *val);
	
	private:
		singlewire &_singlewire;
		SemaphoreHandle_t api_lock;
};
}
