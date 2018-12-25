
#include <stdint.h>

#include "FreeRTOS.h"

void assert_failed(uint8_t* file, uint32_t line)
{
	portDISABLE_INTERRUPTS();
	while(1);
}
