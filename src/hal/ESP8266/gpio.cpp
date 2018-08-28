#include <stdint.h>
#include <stddef.h>

#include "common/macros.h"

#include "gpio.hpp"

#include "ESP8266_RTOS_SDK/include/espressif/c_types.h"
#include "ESP8266_RTOS_SDK/include/espressif/esp8266/pin_mux_register.h"
#include "ESP8266_RTOS_SDK/include/espressif/esp8266/gpio_register.h"
#include "ESP8266_RTOS_SDK/include/espressif/esp8266/eagle_soc.h"

using namespace hal;

static const uint32_t mux_list[PIN_QTY] =
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
	0                           /*< GPIO_16 should be configured other way, 
	                             because it related to RTC module */
};

// Definition of absent in SDK pin functions
// According to the table in hal_gpio.h file
#define FUNC_GPIO6        3
#define FUNC_GPIO7        3
#define FUNC_GPIO8        3
#define FUNC_GPIO11       3
#define FUNC_SPICS2       1
#define FUNC_CLK_OUT      4
#define FUNC_SPICS1       1
#define FUNC_CLK_RTC1     4
#define FUNC_I2SO_WS      1
#define FUNC_U0RXD        0
#define FUNC_I2SO_DATA    1
#define FUNC_CLK_XTAL1    4
#define FUNC_CLK_XTAL2    1
#define FUNC_CLK_RTC2     1
#define FUNC_U1CTS        4
#define FUNC_HSPIHD       4
#define FUNC_HSPIWP       4
#define FUNC_U1RTS        4
#define FUNC_I2SI_DATA    1
#define FUNC_MTDI         0
#define FUNC_HSPIQ_MISO   2
#define FUNC_U0DTR        4
#define FUNC_MTCK         0
#define FUNC_I2SI_BCK     1
#define FUNC_HSPID_MOSI   2
#define FUNC_U0CTS        4
#define FUNC_MTMS         0
#define FUNC_I2SI_WS      1
#define FUNC_HSPICLK      2
#define FUNC_U0DSR        4
#define FUNC_MTDO         0
#define FUNC_I2SO_BCK     1
#define FUNC_HSPICS       2

// Absent RTC module definitions
#define FUNC_RTC_XPD_DCDC 0
#define FUNC_RTC_GPIO0    1
#define FUNC_EXT_WAKEUP   2
#define FUNC_DEEPSLEEP    3
#define FUNC_BT_XTAL_EN   4

static const uint32_t func_list[PIN_QTY][5] =
{
//  DO/DI         AF1,            AF2,            AF3,             AF4
	{FUNC_GPIO0,  FUNC_SPICS2,    0,              0,               FUNC_CLK_OUT},
	{FUNC_GPIO1,  FUNC_U0TXD,     FUNC_SPICS1,    0,               FUNC_CLK_RTC1},
	{FUNC_GPIO2,  FUNC_I2SO_WS,   FUNC_U1TXD_BK,  0,               FUNC_U0TXD_BK},
	{FUNC_GPIO3,  FUNC_U0RXD,     FUNC_I2SO_DATA, 0,               FUNC_CLK_XTAL1},
	{FUNC_GPIO4,  FUNC_CLK_XTAL2, 0,              0,               0},
	{FUNC_GPIO5,  FUNC_CLK_RTC2,  0,              0,               0},
	{FUNC_GPIO6,  FUNC_SDCLK,     FUNC_SPICLK,    0,               FUNC_U1CTS},
	{FUNC_GPIO7,  FUNC_SDDATA0,   FUNC_SPIQ_MISO, 0,               FUNC_U1TXD},
	{FUNC_GPIO8,  FUNC_SDDATA1,   FUNC_SPID_MOSI, 0,               FUNC_U1RXD},
	{FUNC_GPIO9,  FUNC_SDDATA2,   FUNC_SPIHD,     0,               FUNC_HSPIHD},
	{FUNC_GPIO10, FUNC_SDDATA3,   FUNC_SPIWP,     0,               FUNC_HSPIWP},
	{FUNC_GPIO11, FUNC_SDCMD,     FUNC_SPICS0,    0,               FUNC_U1RTS},
	{FUNC_GPIO12, FUNC_MTDI,      FUNC_I2SI_DATA, FUNC_HSPIQ_MISO, FUNC_U0DTR},
	{FUNC_GPIO13, FUNC_MTCK,      FUNC_I2SI_BCK,  FUNC_HSPID_MOSI, FUNC_U0CTS},
	{FUNC_GPIO14, FUNC_MTMS,      FUNC_I2SI_WS,   FUNC_HSPICLK,    FUNC_U0DSR},
	{FUNC_GPIO15, FUNC_MTDO,      FUNC_I2SO_BCK,  FUNC_HSPICS,     FUNC_U0RTS},
	{0,           0,              0,              0,               0}
};

gpio::gpio(uint8_t port, uint8_t pin, gpio_mode_t mode, bool state):
	_port(port),
	_pin(pin),
	_mode(mode)
{
	ASSERT(_port < PORT_QTY);
	ASSERT(_pin < PIN_QTY);
	
	// GPIO0-15 have only pulldown in input mode
	ASSERT(_mode != GPIO_MODE_DI || _pin > 15 || state);
	
	// GPIO16 has only pullup in input mode
	ASSERT(_mode != GPIO_MODE_DI || _pin < 16 || !state);
	
	switch(_mode)
	{
		case GPIO_MODE_DO:
			if(_pin < 16)
			{
				// Set as GPIO
				PIN_FUNC_SELECT(mux_list[_pin], func_list[_pin][0]);
				
				// Switch to output
				GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, 1 << _pin);
				
				// Setup default state
				GPIO_REG_WRITE(state ? GPIO_OUT_W1TS_ADDRESS :
					GPIO_OUT_W1TC_ADDRESS, 1 << _pin);
			}
			else
			{
				// mux configuration for XPD_DCDC to output rtc_gpio0
				uint32 tmp_val = READ_PERI_REG(PAD_XPD_DCDC_CONF);
				WRITE_PERI_REG(PAD_XPD_DCDC_CONF, (tmp_val & 0xffffffbc) | 0x1);
				
				// mux configuration for out enable
				tmp_val = READ_PERI_REG(RTC_GPIO_CONF);
				WRITE_PERI_REG(RTC_GPIO_CONF, tmp_val & 0xfffffffe);
				
				// Out enable
				tmp_val = READ_PERI_REG(RTC_GPIO_ENABLE);
				WRITE_PERI_REG(RTC_GPIO_ENABLE, (tmp_val & 0xfffffffe) | 0x1);
				
				// Setup default state
				tmp_val = READ_PERI_REG(RTC_GPIO_OUT);
				WRITE_PERI_REG(RTC_GPIO_OUT, (tmp_val & 0xfffffffe) | state);
			}
			break;
		
		case GPIO_MODE_DI:
			if(_pin < 16)
			{
				// Set as GPIO
				PIN_FUNC_SELECT(mux_list[_pin], func_list[_pin][0]);
				
				// Switch to input
				GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, 1 << _pin);
				
				// Setup pullup or pulldown
				if(state)
					PIN_PULLUP_EN(mux_list[_pin]);
				else
					PIN_PULLUP_DIS(mux_list[_pin]);
			}
			else
			{
				// mux configuration for XPD_DCDC and rtc_gpio0 connection
				uint32 tmp_val = READ_PERI_REG(PAD_XPD_DCDC_CONF);
				WRITE_PERI_REG(PAD_XPD_DCDC_CONF, (tmp_val & 0xffffffbc) | 0x1);
				
				//mux configuration for out enable
				tmp_val = READ_PERI_REG(RTC_GPIO_CONF);
				WRITE_PERI_REG(RTC_GPIO_CONF, tmp_val & 0xfffffffe);
				
				// Out disable
				tmp_val = READ_PERI_REG(RTC_GPIO_ENABLE);
				WRITE_PERI_REG(RTC_GPIO_ENABLE, tmp_val & 0xfffffffe);
			}
			break;
		
		case GPIO_MODE_AF1:
		case GPIO_MODE_AF2:
		case GPIO_MODE_AF3:
		case GPIO_MODE_AF4:
			if(_pin < 16)
			{
				// Set as AFx
				PIN_FUNC_SELECT(mux_list[_pin], func_list[_pin][_mode - 1]);
			}
			else
			{
				// Not implemented yet
				ASSERT(0);
			}
			break;
	}
}

gpio::~gpio()
{
	
}

void gpio::set(bool state) const
{
	ASSERT(_mode == GPIO_MODE_DO);
	
	if(_pin < 16)
	{
		GPIO_REG_WRITE(state ? GPIO_OUT_W1TS_ADDRESS : GPIO_OUT_W1TC_ADDRESS,
			1 << _pin);
	}
	else
	{
		uint32 tmp_val = READ_PERI_REG(RTC_GPIO_OUT);
		WRITE_PERI_REG(RTC_GPIO_OUT, (tmp_val & 0xfffffffe) | state);
	}
}

bool gpio::get() const
{
	ASSERT(_mode == GPIO_MODE_DI && _mode == GPIO_MODE_DO);
	
	if(_pin < 16)
		return (bool)(GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << _pin));
	else
		return (bool)(READ_PERI_REG(RTC_GPIO_IN_DATA) & 1);
}

void gpio::toggle() const
{
	ASSERT(_mode == GPIO_MODE_DO);
	
	if(_pin < 16)
	{
		if(GPIO_REG_READ(GPIO_IN_ADDRESS) & (1 << _pin))
			GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << _pin);
		else
			GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << _pin);
	}
	else
	{
		if(READ_PERI_REG(RTC_GPIO_IN_DATA) & 1)
		{
			uint32 tmp_val = READ_PERI_REG(RTC_GPIO_OUT);
			WRITE_PERI_REG(RTC_GPIO_OUT, (tmp_val & 0xfffffffe) | 0);
		}
		else
		{
			uint32 tmp_val = READ_PERI_REG(RTC_GPIO_OUT);
			WRITE_PERI_REG(RTC_GPIO_OUT, (tmp_val & 0xfffffffe) | 1);
		}
	}
}
