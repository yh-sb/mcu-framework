#include "common/assert.h"
#include "dht.hpp"

using namespace drv;

static const uint16_t timeout[dht::dht_dev_t::DHT22 + 1] = {1200, 2200};

// Byte map of response from DHTxx
enum
{
	RH_INT,
	RH_DEC,
	T_INT,
	T_DEC,
	CHECKSUM,
	BYTES_TOTAL // Total number of bytes in response
};

dht::dht(singlewire &singlewire, dht_dev_t dht_dev):
	_singlewire(singlewire),
	_dht_dev(dht_dev),
	recent_time(xTaskGetTickCount()),
	is_first_measurement(true)
{
	ASSERT(api_lock = xSemaphoreCreateMutex());
}

dht::~dht()
{
	vSemaphoreDelete(api_lock);
}

int8_t dht::get(val_t *val)
{
	ASSERT(val);
	
	wait_ready();
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
	
	if(!is_crc_valid(buff))
	{
		res = RES_CRC_ERR;
		goto Exit;
	}
	
	*val = parce_data(buff);
	
Exit:
	xSemaphoreGive(api_lock);
	return res;
}

void dht::wait_ready(void)
{
	if(!is_first_measurement)
		vTaskDelayUntil(&recent_time, timeout[_dht_dev]);
	else
	{
		vTaskDelayUntil(&recent_time, timeout[_dht_dev] * 2);
		is_first_measurement = false;
	}
}

bool dht::is_crc_valid(uint8_t *buff)
{
	return buff[CHECKSUM] == buff[RH_INT] + buff[RH_DEC] + buff[T_INT] +
		buff[T_DEC];
}

dht::val_t dht::parce_data(uint8_t *buff)
{
	val_t val =
	{
		.rh_x10 = (uint16_t)((buff[RH_INT] * 10) + buff[RH_DEC]),
		.t_x10 = (int16_t)((buff[T_INT] * 10) + buff[T_DEC])
	};
	
	return val;
}
