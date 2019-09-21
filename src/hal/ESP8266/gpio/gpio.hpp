#pragma once

#include <stdint.h>

/* List of ESP8266 pin functions
| GPIO | Default  |   Func1   |   Func2    |   Func3    |    Func4    |
| ---: | -------- | --------- | ---------- | ---------- | ----------- |
|    0 | GPIO0    | SPICS2    | -          | -          | CLK_OUT     |
|    1 | U0TXD    | SPICS1    | -          | GPIO1      | CLK_RTC_BK  |
|    2 | GPIO2    | I2SO_WS   | U1TXD_BK   | -          | U0TXD_BK    |
|    3 | U0RXD    | I2SO_DATA | -          | GPIO3      | CLK_XTAL_BK |
|    4 | GPIO4    | CLK_XTAL  | -          | -          | -           |
|    5 | GPIO5    | CLK_RTC   | -          | -          | -           |
|   *6 | SDCLK    | SPICLK    | -          | GPIO6      | U1CTS       |
|   *7 | SDDATA0  | SPIQ_MISO | -          | GPIO7      | U1TXD       |
|   *8 | SDDATA1  | SPID_MOSI | -          | GPIO8      | U1RXD       |
|   *9 | SDDATA2  | SPIHD     | -          | GPIO9      | HSPIHD      |
|  *10 | SDDATA3  | SPIWP     | -          | GPIO10     | HSPIWP      |
|  *11 | SDCMD    | SPICS0    | -          | GPIO11     | U1RTS       |
|   12 | MTDI     | I2SI_DATA | HSPIQ_MISO | GPIO12     | UART0_DTR   |
|   13 | MTCK     | I2SI_BCK  | HSPID_MOSI | GPIO13     | UART0_CTS   |
|   14 | MTMS     | I2SI_WS   | HSPI_CLK   | GPIO14     | UART0_DSR   |
|   15 | MTDO     | I2SO_BCK  | HSPI_CS0   | GPIO15     | U0RTS       |
|   16 | XPD_DCDC | RTC_GPIO0 | EXT_WAKEUP | DEEPSLEEP  | BT_XTAL_EN  |
| ---- | -------- | --------- | ---------- | ---------- | ----------- |
* - pins used for accessing external flash memory */

namespace hal
{
#define PORT_QTY 1
#define PIN_QTY  17

class gpio
{
	public:
		enum mode_t
		{
			MODE_DO,
			MODE_OD,
			MODE_DI,
			MODE_AF1,
			MODE_AF2,
			MODE_AF3,
			MODE_AF4
		};
		
		gpio(uint8_t port, uint8_t pin, mode_t mode, bool state = false);
		~gpio();
		
		void set(bool state) const;
		bool get() const;
		void toggle() const;
		void mode(mode_t mode, bool state = false);
		
		mode_t mode() const { return _mode; }
		uint8_t port() const { return _port; }
		uint8_t pin() const { return _pin; }
	
	private:
		uint8_t _port;
		uint8_t _pin;
		mode_t _mode;
};
}
