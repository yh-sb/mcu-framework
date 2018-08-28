#pragma once

#include <stdint.h>

#include "ESP8266_RTOS_SDK/include/espressif/c_types.h"

/*
List of ESP8266 pin functions from Espressif documentation:
| GPIO |    Name    |  Func1   |   Func2   |   Func3    |   Func4   |   Func5    |
| ---: | ---------- | -------- | --------- | ---------- | --------- | ---------- |
|    0 | GPIO0_U    | GPIO0    | SPICS2    | -          | -         | CLK_OUT    |
|    1 | U0TXD_U    | U0TXD    | SPICS1    | -          | GPIO1     | CLK_RTC    |
|    2 | GPIO2_U    | GPIO2    | I2SO_WS   | U1TXD      | -         | U0TXD      |
|    3 | U0RXD_U    | U0RXD    | I2SO_DATA | -          | GPIO3     | CLK_XTAL   |
|    4 | GPIO4_U    | GPIO4    | CLK_XTAL  | -          | -         | -          |
|    5 | GPIO5_U    | GPIO5    | CLK_RTC   | -          | -         | -          |
|   *6 | SD_CLK_U   | SD_CLK   | SPICLK    | -          | GPIO6     | U1CTS      |
|   *7 | SD_DATA0_U | SD_DATA0 | SPIQ      | -          | GPIO7     | U1TXD      |
|   *8 | SD_DATA1_U | SD_DATA1 | SPID      | -          | GPIO8     | U1RXD      |
|   *9 | SD_DATA2_U | SD_DATA2 | SPIHD     | -          | GPIO9     | HSPIHD     |
|  *10 | SD_DATA3_U | SD_DATA3 | SPIWP     | -          | GPIO10    | HSPIWP     |
|  *11 | SD_CMD_U   | SD_CMD   | SPICS0    | -          | GPIO11    | U1RTS      |
|   12 | MTDI_U     | MTDI     | I2SI_DATA | HSPIQ MISO | GPIO12    | U0DTR      |
|   13 | MTCK_U     | MTCK     | I2SI_BCK  | HSPID MOSI | GPIO13    | U0CTS      |
|   14 | MTMS_U     | MTMS     | I2SI_WS   | HSPICLK    | GPIO14    | U0DSR      |
|   15 | MTDO_U     | MTDO     | I2SO_BCK  | HSPICS     | GPIO15    | U0RTS      |
|   16 | XPD_DCDC   | XPD_DCDC | RTC_GPIO0 | EXT_WAKEUP | DEEPSLEEP | BT_XTAL_EN |
| ---- | ---------- | -------- | --------- | ---------- | --------- | ---------- |

List of ESP8266 pin functions according to this hal_gpio:
| GPIO |    AF1    |    AF2     |    AF3     |    AF4     |
| ---: | --------- | ---------- | ---------- | ---------- |
|    0 | SPICS2    | -          | -          | CLK_OUT    |
|    1 | U0TXD     | SPICS1     | -          | CLK_RTC1   |
|    2 | I2SO_WS   | U1TXD      | -          | U0TXD      |
|    3 | U0RXD     | I2SO_DATA  | -          | CLK_XTAL1  |
|    4 | CLK_XTAL2 | -          | -          | -          |
|    5 | CLK_RTC2  | -          | -          | -          |
|   *6 | SD_CLK    | SPICLK     | -          | U1CTS      |
|   *7 | SD_DATA0  | SPIQ       | -          | U1TXD      |
|   *8 | SD_DATA1  | SPID       | -          | U1RXD      |
|   *9 | SD_DATA2  | SPIHD      | -          | HSPIHD     |
|  *10 | SD_DATA3  | SPIWP      | -          | HSPIWP     |
|  *11 | SD_CMD    | SPICS0     | -          | U1RTS      |
|   12 | MTDI      | I2SI_DATA  | HSPIQ MISO | U0DTR      |
|   13 | MTCK      | I2SI_BCK   | HSPID MOSI | U0CTS      |
|   14 | MTMS      | I2SI_WS    | HSPICLK    | U0DSR      |
|   15 | MTDO      | I2SO_BCK   | HSPICS     | U0RTS      |
|   16 | XPD_DCDC  | EXT_WAKEUP | DEEPSLEEP  | BT_XTAL_EN |
| ---- | --------- | ---------- | ---------- | ---------- |
*/

namespace hal
{
#define PORT_QTY 1
#define PIN_QTY  17

typedef enum
{
	GPIO_MODE_DO,
	GPIO_MODE_DI,
	GPIO_MODE_AF1,
	GPIO_MODE_AF2,
	GPIO_MODE_AF3,
	GPIO_MODE_AF4
} gpio_mode_t;

class gpio
{
	public:
		gpio(uint8_t port, uint8_t pin, gpio_mode_t mode, bool state);
		~gpio();
		
		void set(bool state) const;
		bool get() const;
		void toggle() const;
		
		gpio_mode_t mode() const { return _mode; }
		uint8_t port() const { return _port; }
		uint8_t pin() const { return _pin; }
	
	private:
		uint8_t _port;
		uint8_t _pin;
		gpio_mode_t _mode;
};
}
