#pragma once

#include <stdint.h>

namespace hal
{
#define PORT_QTY 7 // GPIOG
#define PIN_QTY  16

typedef enum
{
	GPIO_MODE_DO,
	GPIO_MODE_OD,
	GPIO_MODE_DI,
	GPIO_MODE_AN,
	GPIO_MODE_AF
} gpio_mode_t;

class gpio
{
	public:
		gpio(uint8_t port, uint8_t pin, gpio_mode_t mode, bool state = false);
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
