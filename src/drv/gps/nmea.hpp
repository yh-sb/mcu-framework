#pragma once

#include <stdint.h>
#include "uart/uart.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "third_party/libnmea/src/nmea/nmea.h"
#include "third_party/libnmea/src/parsers/gpgga.h"
#include "third_party/libnmea/src/parsers/gpgll.h"
#include "third_party/libnmea/src/parsers/gpgsv.h"
#include "third_party/libnmea/src/parsers/gprmc.h"

namespace drv
{
class nmea
{
	public:
		enum res_t
		{
			RES_OK = 0,
			RES_READ_ERR = -1,
			RES_PARSE_ERR = -2,
			RES_UNKNOWN_MSG_ERR = -3,
		};
		
		struct queue_t
		{
			res_t res;
			nmea_t type;
			union
			{
				nmea_gpgga_s gpgga;
				nmea_gpgll_s gpgll;
				nmea_gprmc_s gprmc;
				nmea_gpgsv_s gpgsv;
			};
		};
		
		nmea(hal::uart &uart, UBaseType_t task_priority);
		~nmea();
		
		QueueHandle_t get_queue() { return queue; };
	
	private:
		hal::uart &_uart;
		TaskHandle_t task_hndlr;
		QueueHandle_t queue;
		
		static void task(void *pvParameters);
		void on_recv(char *buff, size_t size);
		queue_t format_queue(nmea_s *data);
		bool get_line(char *buff, size_t size, char **line, size_t *line_size);
};
}
