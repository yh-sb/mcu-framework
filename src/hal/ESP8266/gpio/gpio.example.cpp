#include "FreeRTOS.h"
#include "task.h"

#include "hal/ESP8266/ESP8266_RTOS_SDK/include/espressif/esp_system.h"

#include "gpio/gpio.hpp"

extern "C" uint32_t user_rf_cal_sector_set(void)
{
	flash_size_map size_map = system_get_flash_size_map();
	uint32_t rf_cal_sec = 0;
	switch(size_map)
	{
		case FLASH_SIZE_4M_MAP_256_256:
			rf_cal_sec = 128 - 5;
			break;
		
		case FLASH_SIZE_8M_MAP_512_512:
			rf_cal_sec = 256 - 5;
			break;
		
		case FLASH_SIZE_16M_MAP_512_512:
		case FLASH_SIZE_16M_MAP_1024_1024:
			rf_cal_sec = 512 - 5;
			break;
		
		case FLASH_SIZE_32M_MAP_512_512:
		case FLASH_SIZE_32M_MAP_1024_1024:
			rf_cal_sec = 1024 - 5;
			break;
		
		case FLASH_SIZE_64M_MAP_1024_1024:
			rf_cal_sec = 2048 - 5;
			break;
		
		case FLASH_SIZE_128M_MAP_1024_1024:
			rf_cal_sec = 4096 - 5;
			break;
		
		default:
			rf_cal_sec = 0;
			break;
	}
	
	return rf_cal_sec;
}

extern "C" void user_rf_pre_init(void)
{
	
}

static void main_task(void *pvParameters)
{
	hal::gpio *led = static_cast<hal::gpio *>(pvParameters);
	
	while(1)
	{
		led->toggle();
		
		vTaskDelay(500 / portTICK_RATE_MS);
		printf("Blue led toggled");
	}
}

extern "C" void user_init(void)
{
	printf("SDK version:%s %d\n", system_get_sdk_version(), system_get_free_heap_size());
	
	static hal::gpio blue_led(0, 2, hal::gpio::MODE_DO, 1);
	
	xTaskCreate(main_task, (const signed char *)"main",
		configMINIMAL_STACK_SIZE * 1, &blue_led, tskIDLE_PRIORITY + 1, NULL);
}
