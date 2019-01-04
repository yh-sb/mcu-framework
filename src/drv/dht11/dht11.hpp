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
			OK     =  0,
			NODEV  = -1,
			DEVERR = -2,
			BUSY   = -3,
			CRCERR = -4
		};

		dht11(singlewire &singlewire);
		~dht11();
		
		int8_t get(uint8_t *rh, uint8_t *t = NULL);
	
	private:
		singlewire &_singlewire;
		SemaphoreHandle_t api_lock;
};
}
