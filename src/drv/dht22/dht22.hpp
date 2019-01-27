#pragma once

#include "drv/singlewire/singlewire.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

namespace drv
{
class dht22
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

		dht22(singlewire &singlewire);
		~dht22();
		
		int8_t get(uint16_t *rh_x10, int16_t *t_x10 = NULL);
	
	private:
		singlewire &_singlewire;
		SemaphoreHandle_t api_lock;
};
}
