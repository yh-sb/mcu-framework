#include "common/assert.h"
#include "dht22.hpp"

using namespace drv;

// Byte map of response from DHT22
enum
{
	RH_INT,
	RH_DEC,
	T_INT,
	T_DEC,
	CHECKSUM,
	BYTES_TOTAL // Total number of bytes in response from DHT22
};

dht22::dht22(singlewire &singlewire):
	_singlewire(singlewire)
{
	api_lock = xSemaphoreCreateMutex();
	ASSERT(api_lock);
}

dht22::~dht22()
{
	vSemaphoreDelete(api_lock);
}

int8_t dht22::get(uint16_t *rh_x10, int16_t *t_x10)
{
	ASSERT(rh_x10 || t_x10);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = RES_OK;
	uint8_t buff[BYTES_TOTAL];
	
	switch(_singlewire.read(buff, sizeof(buff)))
	{
		case singlewire::OK: break;
		case singlewire::NODEV: res = RES_NODEV; goto Exit;
		case singlewire::BUSY: res = RES_BUSY; goto Exit;
		default: res = RES_DEVERR; goto Exit;
	}
	
	if(buff[CHECKSUM] != (uint8_t)(buff[RH_INT] + buff[RH_DEC] + buff[T_INT] +
		buff[T_DEC]))
	{
		res = RES_CRCERR;
		goto Exit;
	}
	
	if(rh_x10)
		*rh_x10 = (buff[RH_INT] << 8) | buff[RH_DEC];
	
	if(t_x10)
	{
		*t_x10 = (buff[T_INT] << 8) | buff[T_DEC];
		
		if(buff[T_INT] >> 7) // Determine the sign
		{
			*t_x10 &= ~(1 << 15); // Remove sign bit from the DHT22 response
			*t_x10 *= -1;
		}
	}
	
Exit:
	xSemaphoreGive(api_lock);
	return res;
}
