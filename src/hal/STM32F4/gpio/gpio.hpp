#pragma once

#include <stdint.h>

namespace hal
{
#define PORT_QTY 11 // GPIOK
#define PIN_QTY  16

class gpio
{
	public:
		enum mode_t
		{
			MODE_DO,
			MODE_OD,
			MODE_DI,
			MODE_AN,
			MODE_AF
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
