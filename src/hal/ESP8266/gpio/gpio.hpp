#pragma once

#include <stdint.h>

/* List of ESP8266 pin functions according to this hal:
| DO,OD,DI |    AF1   |    AF2     |    AF3     |    AF4      |
| -------- | -------- | ---------- | ---------- | ----------- |
| GPIO0    | SPICS2   | -          | -          | CLK_OUT     |
| GPIO1    | U0TXD    | SPICS1     | -          | CLK_RTC_BK  |
| GPIO2    | I2SO_WS  | U1TXD_BK   | -          | U0TXD_BK    |
| GPIO3    | U0RXD    | I2SO_DATA  | -          | CLK_XTAL_BK |
| GPIO4    | CLK_XTAL | -          | -          | -           |
| GPIO5    | CLK_RTC  | -          | -          | -           |
| GPIO6*   | SDCLK    | SPICLK     | -          | U1CTS       |
| GPIO7*   | SDDATA0  | SPIQ_MISO  | -          | U1TXD       |
| GPIO8*   | SDDATA1  | SPID_MOSI  | -          | U1RXD       |
| GPIO9*   | SDDATA2  | SPIHD      | -          | HSPIHD      |
| GPIO10*  | SDDATA3  | SPIWP      | -          | HSPIWP      |
| GPIO11*  | SDCMD    | SPICS0     | -          | U1RTS       |
| GPIO12   | MTDI     | I2SI_DATA  | HSPIQ_MISO | U0DTR       |
| GPIO13   | MTCK     | I2SI_BCK   | HSPID_MOSI | U0CTS       |
| GPIO14   | MTMS     | I2SI_WS    | HSPI_CLK   | U0DSR       |
| GPIO15   | MTDO     | I2SO_BCK   | HSPI_CS    | U0RTS       |
| GPIO16   | XPD_DCDC | EXT_WAKEUP | DEEPSLEEP  | BT_XTAL_EN  |
| -------- | -------- | ---------- | ---------- | ----------- |
* - pins used for accessing external flash memory */

namespace hal
{
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
