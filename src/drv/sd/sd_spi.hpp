#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "sd.hpp"
#include "gpio.hpp"
#include "spi.hpp"

namespace drv
{
class sd_spi : public sd
{
	public:
		sd_spi(hal::spi &spi, hal::gpio &cs, hal::gpio *cd = NULL);
		~sd_spi();
	
	private:
		virtual void select(bool is_selected);
		virtual int8_t init_sd();
		virtual void set_speed(uint32_t speed);
		virtual int8_t send_cmd(cmd_t cmd, uint32_t arg, resp_t resp_type, uint8_t *resp);
		virtual int8_t read_data(void *data, uint16_t size);
		virtual int8_t write_data(void *data, uint16_t size);
		
		int8_t wait_ready();
		
		hal::spi &_spi;
		hal::gpio &_cs;
};
}
