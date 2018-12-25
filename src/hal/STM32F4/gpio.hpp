#pragma once

#include <stdint.h>

namespace hal
{
#define PORT_QTY 11 // GPIOK
#define PIN_QTY  16

class gpio
{
	public:
		enum class mode
		{
			DO,
			OD,
			DI,
			AN,
			AF
		};

		gpio(uint8_t port, uint8_t pin, enum mode mode, bool state = false);
		~gpio();
		
		void set(bool state) const;
		bool get() const;
		void toggle() const;
		void mode(enum mode mode, bool state = false);
		
		enum mode mode() const { return _mode; }
		uint8_t port() const { return _port; }
		uint8_t pin() const { return _pin; }
	
	private:
		uint8_t _port;
		uint8_t _pin;
		enum mode _mode;
};
}
