#include <stdint.h>
#include <stddef.h>
#include "common/assert.h"
#include "gpio.hpp"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/pin_mux_register.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/gpio_register.h"
#include "ESP8266_RTOS_SDK/components/esp8266/include/esp8266/gpio_struct.h"

using namespace hal;

constexpr uint8_t ports = 1;
constexpr uint8_t pins = 17;
constexpr uint8_t rtc_pin = 16;

static const uint32_t mux_list[pins] =
{
	PERIPHS_IO_MUX_GPIO0_U,
	PERIPHS_IO_MUX_U0TXD_U,
	PERIPHS_IO_MUX_GPIO2_U,
	PERIPHS_IO_MUX_U0RXD_U,
	PERIPHS_IO_MUX_GPIO4_U,
	PERIPHS_IO_MUX_GPIO5_U,
	PERIPHS_IO_MUX_SD_CLK_U,
	PERIPHS_IO_MUX_SD_DATA0_U,
	PERIPHS_IO_MUX_SD_DATA1_U,
	PERIPHS_IO_MUX_SD_DATA2_U,
	PERIPHS_IO_MUX_SD_DATA3_U,
	PERIPHS_IO_MUX_SD_CMD_U,
	PERIPHS_IO_MUX_MTDI_U,
	PERIPHS_IO_MUX_MTCK_U,
	PERIPHS_IO_MUX_MTMS_U,
	PERIPHS_IO_MUX_MTDO_U,
	PAD_XPD_DCDC_CONF
};

// RTC pin functions definitions
#define FUNC_RTC_XPD_DCDC 0
#define FUNC_RTC_GPIO0    1
#define FUNC_EXT_WAKEUP   2
#define FUNC_DEEPSLEEP    3
#define FUNC_BT_XTAL_EN   4

static const uint32_t func_list[pins][5] =
{
//  DO/DI            AF1,               AF2,             AF3,             AF4
	{FUNC_GPIO0,     FUNC_SPICS2,       0,               0,               FUNC_CLK_OUT},
	{FUNC_GPIO1,     FUNC_U0TXD,        FUNC_SPICS1,     0,               FUNC_CLK_RTC_BK},
	{FUNC_GPIO2,     FUNC_I2SO_WS,      FUNC_U1TXD_BK,   0,               FUNC_U0TXD_BK},
	{FUNC_GPIO3,     FUNC_U0RXD,        FUNC_I2SO_DATA,  0,               FUNC_CLK_XTAL_BK},
	{FUNC_GPIO4,     FUNC_CLK_XTAL,     0,               0,               0},
	{FUNC_GPIO5,     FUNC_CLK_RTC,      0,               0,               0},
	{FUNC_GPIO6,     FUNC_SDCLK,        FUNC_SPICLK,     0,               UART1_CTS},
	{FUNC_GPIO7,     FUNC_SDDATA0,      FUNC_SPIQ_MISO,  0,               FUNC_U1TXD},
	{FUNC_GPIO8,     FUNC_SDDATA1,      FUNC_SPID_MOSI,  0,               FUNC_U1RXD},
	{FUNC_GPIO9,     FUNC_SDDATA2,      FUNC_SPIHD,      0,               UFNC_HSPIHD},
	{FUNC_GPIO10,    FUNC_SDDATA3,      FUNC_SPIWP,      0,               FUNC_HSPIWP},
	{FUNC_GPIO11,    FUNC_SDCMD,        FUNC_SPICS0,     0,               U1RTS},
	{FUNC_GPIO12,    FUNC_MTDI,         FUNC_I2SI_DATA,  FUNC_HSPIQ_MISO, FUNC_UART0_DTR},
	{FUNC_GPIO13,    FUNC_MTCK,         FUNC_I2SI_BCK,   FUNC_HSPID_MOSI, FUNC_UART0_CTS},
	{FUNC_GPIO14,    FUNC_MTMS,         FUNC_I2SI_WS,    FUNC_HSPI_CLK,   FUNC_UART0_DSR},
	{FUNC_GPIO15,    FUNC_MTDO,         FUNC_I2SO_BCK,   FUNC_HSPI_CS0,   FUNC_U0RTS},
	{FUNC_RTC_GPIO0, FUNC_RTC_XPD_DCDC, FUNC_EXT_WAKEUP, FUNC_DEEPSLEEP,  FUNC_BT_XTAL_EN}
};

gpio::gpio(uint8_t port, uint8_t pin, mode_t mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode)
{
	ASSERT(_port < ports);
	ASSERT(_pin < pins);
	
	gpio::mode(_mode, state);
}

gpio::~gpio()
{
	
}

void gpio::set(bool state) const
{
	ASSERT(_mode == MODE_DO);
	
	if(_pin != rtc_pin)
	{
		if(state)
			GPIO.out_w1ts |= 1 << _pin;
		else
			GPIO.out_w1tc |= 1 << _pin;
	}
	else
	{
		if(state)
			SET_PERI_REG_MASK(RTC_GPIO_OUT, 1);
		else
			CLEAR_PERI_REG_MASK(RTC_GPIO_OUT, 1);
	}
}

bool gpio::get() const
{
	ASSERT(_mode == MODE_DI && _mode == MODE_DO);
	
	if(_pin != rtc_pin)
		return GPIO.in & (1 << _pin);
	else
		return READ_PERI_REG(RTC_GPIO_IN_DATA) & 1;
}

void gpio::toggle() const
{
	ASSERT(_mode == MODE_DO);
	
	if(_pin != rtc_pin)
	{
		if(GPIO.in & (1 << _pin))
			GPIO.out_w1tc |= 1 << _pin;
		else
			GPIO.out_w1ts |= 1 << _pin;
	}
	else
	{
		if(READ_PERI_REG(RTC_GPIO_IN_DATA) & 1)
			CLEAR_PERI_REG_MASK(RTC_GPIO_OUT, 1);
		else
			SET_PERI_REG_MASK(RTC_GPIO_OUT, 1);
	}
}

void gpio::mode(mode_t mode, bool state)
{
	ASSERT(_mode <= MODE_AF4);
	if(_pin == rtc_pin)
	{
		ASSERT(_mode != MODE_DI || !state); // GPIO16 has only pulldown input mode
		ASSERT(_mode != MODE_OD); // GPIO16 doesn't have open drain mode
	}
	
	switch(_mode)
	{
		case MODE_DO:
			if(_pin != rtc_pin)
			{
				PIN_FUNC_SELECT(mux_list[_pin], func_list[_pin][0]);
				GPIO.enable_w1ts |= 1 << _pin;
				GPIO.pin[_pin].driver = 0;
				set(state);
			}
			else
			{
				// mux configuration for XPD_DCDC to output rtc_gpio0
				uint32_t tmp_val = READ_PERI_REG(mux_list[_pin]) & 0xffffffbc;
				WRITE_PERI_REG(mux_list[_pin], tmp_val | 1);
				
				CLEAR_PERI_REG_MASK(RTC_GPIO_CONF, 1);
				SET_PERI_REG_MASK(RTC_GPIO_ENABLE, 1);
				set(state);
			}
			break;
		
		case MODE_OD:
			PIN_FUNC_SELECT(mux_list[_pin], func_list[_pin][0]);
			GPIO.enable_w1ts |= 1 << _pin;
			GPIO.pin[_pin].driver = 1;
			set(state);
			break;
		
		case MODE_DI:
			if(_pin != rtc_pin)
			{
				PIN_FUNC_SELECT(mux_list[_pin], func_list[_pin][0]);
				GPIO.enable_w1tc |= 1 << _pin;
				GPIO.pin[_pin].driver = 0;
				
				// GPIO 0-15 has pullup/floating input modes only
				if(state)
					PIN_PULLUP_EN(mux_list[_pin]);
				else
					PIN_PULLUP_DIS(mux_list[_pin]);
			}
			else
			{
				// mux configuration for XPD_DCDC and rtc_gpio0 connection
				uint32_t tmp_val = READ_PERI_REG(mux_list[_pin]) & 0xffffffbc;
				WRITE_PERI_REG(mux_list[_pin], tmp_val | 1);
				
				CLEAR_PERI_REG_MASK(RTC_GPIO_CONF, 1);
				CLEAR_PERI_REG_MASK(RTC_GPIO_ENABLE, 1);
				
				// GPIO 16 has pulldown/floating input modes only
				gpio_pin_reg_t pin_reg = {.val = READ_PERI_REG(mux_list[_pin])};
				pin_reg.rtc_pin.pulldown = !state;
				WRITE_PERI_REG(mux_list[_pin], pin_reg.val);
			}
			break;
		
		case MODE_AF1:
		case MODE_AF2:
		case MODE_AF3:
		case MODE_AF4:
			if(_pin != rtc_pin)
			{
				// Normalize _mode (- MODE_DI) to access proper row from func_list
				PIN_FUNC_SELECT(mux_list[_pin], func_list[_pin][_mode - MODE_DI]);
			}
			else
			{
				// Not implemented yet
				ASSERT(0);
			}
			break;
	}
}