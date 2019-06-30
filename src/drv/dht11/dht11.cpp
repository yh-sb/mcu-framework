#include "common/assert.h"
#include "dht11.hpp"

using namespace drv;

// Byte map of response from DHT11
enum
{
	RH_INT,
	RH_DEC,
	T_INT,
	T_DEC,
	CHECKSUM,
	BYTES_TOTAL // Total number of bytes in response from DHT11
};

dht11::dht11(singlewire &singlewire):
	_singlewire(singlewire)
{
	ASSERT(api_lock = xSemaphoreCreateMutex());
}

dht11::~dht11()
{
	vSemaphoreDelete(api_lock);
}

int8_t dht11::get(val_t *val)
{
	ASSERT(val);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = RES_OK;
	uint8_t buff[BYTES_TOTAL];
	
	switch(_singlewire.read(buff, sizeof(buff)))
	{
		case singlewire::RES_OK: break;
		case singlewire::RES_NODEV: res = RES_NODEV; goto Exit;
		case singlewire::RES_BUSY: res = RES_BUSY; goto Exit;
		default: res = RES_DEVERR; goto Exit;
	}
	
	if(buff[CHECKSUM] != (uint8_t)(buff[RH_INT] + buff[RH_DEC] + buff[T_INT] +
		buff[T_DEC]))
	{
		res = RES_CRC_ERR;
		goto Exit;
	}
	
	/* Relative humidity range of DHT11 is 20-90 % and accuracy is ±4 or ±5 %.
	   It means that decimal part of measurement always equal to 0.
	   So ignore it and use only integral part.
	*/
	val->rh = buff[RH_INT];
	
	/* Temperature range of DHT11 is 0-50 °C and accuracy is ±1 or ±2 °C.
	   It means that decimal part of measurement always equal to 0.
	   So ignore it and use only integral part.
	*/
	val->t = buff[T_INT];
	
Exit:
	xSemaphoreGive(api_lock);
	return res;
}
