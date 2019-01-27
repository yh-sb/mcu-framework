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
			RES_OK     =  0,
			RES_NODEV  = -1,
			RES_DEVERR = -2,
			RES_BUSY   = -3,
			RES_CRCERR = -4
		};

		dht11(singlewire &singlewire);
		~dht11();
		
		int8_t get(uint8_t *rh, uint8_t *t = NULL);
	
	private:
		singlewire &_singlewire;
		SemaphoreHandle_t api_lock;
};
}
