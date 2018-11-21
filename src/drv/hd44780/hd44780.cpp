#include <stddef.h>

#include "common/assert.h"
#include "hd44780.hpp"

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/task.h"

using namespace drv;
using namespace hal;

#define X_MAX 19
#define Y_MAX 3

hd44780::hd44780(gpio &e, gpio &rw, gpio &rs, gpio &db4, gpio &db5, gpio &db6,
	gpio &db7, tim &tim):
	_e(e),
	_rw(rw),
	_rs(rs),
	_db{&db4, &db5, &db6, &db7},
	_tim(tim),
	_x(0),
	_y(0)
{
	ASSERT(_e.mode() == gpio::mode::DO);
	ASSERT(_rw.mode() == gpio::mode::DO);
	ASSERT(_rs.mode() == gpio::mode::DO);
	
	for(uint8_t i = 4; (i < sizeof(_db)/sizeof(_db[0])) && _db[i]; i++)
		ASSERT(_db[i]->mode() == gpio::mode::DO);
}

hd44780::~hd44780()
{
	
}

void hd44780::init()
{
	_rw.set(0);
	_rs.set(0);
	
	write_4bit(0b00000011);
	delay(4100);
	write_4bit(0b00000011);
	delay(100);
	write_4bit(0b00000011);
	
	write_4bit(0x28 >> 4);
	write(CMD, 0x28); // 2 lines, 5x8 character matrix
	write(CMD, 0x0C); // Display ctrl: Disp=ON, Curs/Blnk=OFF
	write(CMD, 0x06); // Entry mode: Move right, no shift
	
	//write(CMD, 0x01); // Clear
	x(0);
	delay(1530);
}

void hd44780::print(const char *str)
{
	while(*str != '\0')
	{
		write(DATA, *str);
		str++;
	}
}

void hd44780::print(char byte)
{
	write(DATA, byte);
}

void hd44780::x(uint8_t x)
{
	ASSERT(x <= X_MAX);
	
	uint8_t y_shift[] = {0x00, 0x40, 0x14, 0x54};
	
	write(CMD, (y_shift[_y] + x) | 0b10000000);
}

void hd44780::y(uint8_t y)
{
	ASSERT(y <= Y_MAX);
	
	uint8_t y_shift[] = {0x00, 0x40, 0x14, 0x54};
	
	write(CMD, (y_shift[y] + _x) | 0b10000000);
}

void hd44780::write_4bit(uint8_t half_byte)
{
	for(uint8_t i = 0; i < 4; i++)
		_db[i]->set((half_byte >> i) & 1);
	
	_e.set(1);
	delay(40);
	_e.set(0);
	delay(40);
}

void hd44780::write(write_t type, uint8_t byte)
{
	_rw.set(0);
	_rs.set(type == CMD ? 0 : 1);
	
	write_4bit(byte >> 4);
	write_4bit(byte);
}

void hd44780::delay(uint32_t us)
{
	_tim.us(us);
	_tim.start_once(NULL, NULL);
	while(_tim.is_running());
}
