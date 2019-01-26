#include <stddef.h>
#include <cstring>

#include "common/assert.h"
#include "onewire.hpp"

using namespace drv;
using namespace hal;

#define RX_WAIT_TIMEOUT 5 /* ms */

enum cmd_t
{
	CMD_SEARCH_ROM   = 0xF0, /* Search ROM */
	CMD_ALARM_SEARCH = 0xEC, /* Search ROM with alarm */
	CMD_READ_ROM     = 0x33, /* Read ROM of 1wire device. Only if single device
                                is present on 1 wire bus */
	CMD_MATCH_ROM    = 0x55, /* Select specific 1 wire device by it's ROM */
	CMD_SKIP_ROM     = 0xCC  /* Send/receive data to all devices on the bus */
};

onewire::onewire(uart &uart):
	_uart(uart)
{
	api_lock = xSemaphoreCreateMutex();
	ASSERT(api_lock);
}

onewire::~onewire()
{
	
}

int8_t onewire::tx(uint64_t rom, void *tx_buff, uint16_t tx_size)
{
	ASSERT(tx_buff);
	ASSERT(tx_size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = do_reset();
	if(res)
		goto Exit;
	
	if(!rom)
		res = send_byte(CMD_SKIP_ROM);
	else
	{
		uint8_t rom_buff[9];
		rom_buff[0] = CMD_MATCH_ROM;
		for(uint8_t i = 1; i < sizeof(rom_buff); i++, rom = rom >> 8)
			rom_buff[i] = rom;
		res = send_buff(rom_buff, sizeof(rom_buff));
	}
	if(res)
		goto Exit;
	
	res = send_buff(tx_buff, tx_size);
	
Exit:
	xSemaphoreGive(api_lock);
	return res;
}

int8_t onewire::rx(uint64_t rom, void *rx_buff, uint16_t rx_size)
{
	ASSERT(rx_buff);
	ASSERT(rx_size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = do_reset();
	if(res)
		goto Exit;
	
	if(!rom)
		res = send_byte(CMD_SKIP_ROM);
	else
	{
		uint8_t rom_buff[9];
		rom_buff[0] = CMD_MATCH_ROM;
		for(uint8_t i = 1; i < sizeof(rom_buff); i++, rom = rom >> 8)
			rom_buff[i] = rom;
		res = send_buff(rom_buff, sizeof(rom_buff));
	}
	if(res)
		goto Exit;
	
	res = read_buff(rx_buff, rx_size);
	
Exit:
	xSemaphoreGive(api_lock);
	return res;
}

int8_t onewire::exch(uint64_t rom, void *tx_buff, uint16_t tx_size, void *rx_buff,
	uint16_t rx_size)
{
	ASSERT(tx_buff);
	ASSERT(tx_size > 0);
	ASSERT(rx_buff);
	ASSERT(rx_size > 0);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = do_reset();
	if(res)
		goto Exit;
	
	if(!rom)
		res = send_byte(CMD_SKIP_ROM);
	else
	{
		uint8_t rom_buff[9];
		rom_buff[0] = CMD_MATCH_ROM;
		for(uint8_t i = 1; i < sizeof(rom_buff); i++, rom = rom >> 8)
			rom_buff[i] = rom;
		res = send_buff(rom_buff, sizeof(rom_buff));
	}
	if(res)
		goto Exit;
	
	if(tx_buff)
	{
		res = send_buff(tx_buff, tx_size);
		if(res)
			goto Exit;
	}
	
	if(rx_buff)
		res = read_buff(rx_buff, rx_size);
	
Exit:
	xSemaphoreGive(api_lock);
	return res;
}

int8_t onewire::read_rom(uint64_t *rom)
{
	ASSERT(rom);
	
	xSemaphoreTake(api_lock, portMAX_DELAY);
	
	int8_t res = do_reset();
	if(res)
		goto Exit;
	
	res = send_byte(CMD_READ_ROM);
	if(res)
		goto Exit;
	
	uint8_t rx_buff[8];
	res = read_buff(rx_buff, 8);
	if(res)
		goto Exit;
	
	*rom = 0;
	for(uint8_t i = 0; i < sizeof(*rom); i++)
		*rom = (*rom << 8) | rx_buff[7 - i];
	
Exit:
	xSemaphoreGive(api_lock);
	return res;
}

/*int8_t onewire::search(uint64_t *rom_list, size_t *rom_list_size)
{
	
}*/

int8_t onewire::do_reset()
{
	int8_t res = ONEWIRE_ERR_NONE, uart_res;
	uint8_t tx_buff = 0xF0, rx_buff = 0x00;
	uint16_t size = sizeof(rx_buff);
	
	if(_uart.baud() != 9600)
		_uart.baud(9600);
	
	if(_uart.tx(&tx_buff, 1) != uart::RES_OK)
	{
		res = ONEWIRE_ERR_TX_FAIL;
		goto Exit;
	}
	
	uart_res = _uart.rx(&rx_buff, &size, RX_WAIT_TIMEOUT);
	if(uart_res == uart::RES_RX_TIMEOUT)
		res = ONEWIRE_ERR_NO_DEV;
	else if(uart_res == uart::RES_RX_FAIL || size != sizeof(rx_buff))
		res = ONEWIRE_ERR_RX_FAIL;
	else if(uart_res == uart::RES_TX_FAIL)
		res = ONEWIRE_ERR_TX_FAIL;
	else if(rx_buff == 0x00)
		res = ONEWIRE_ERR_LINE_BUSY;
	
Exit:
	_uart.baud(115200);
	return res;
}

int8_t onewire::send_buff(void *buff, uint8_t size)
{
	int8_t res = ONEWIRE_ERR_NONE;
	
	for(uint8_t i = 0; (i < size) && !res; i++)
		res = send_byte(static_cast<uint8_t *>(buff)[i]);
		
	
	return res;
}

int8_t onewire::read_buff(void *buff, uint8_t size)
{
	int8_t res = ONEWIRE_ERR_NONE;
	
	for(uint8_t i = 0; (i < size) && !res; i++)
		res = read_byte(&static_cast<uint8_t *>(buff)[i]);
	
	return res;
}

int8_t onewire::send_byte(uint8_t byte)
{
	uint8_t tx_buff[8], rx_buff[8];
	uint16_t size = sizeof(rx_buff);
	
	for(uint8_t i = 0; i < size; i++)
		tx_buff[i] = ((byte >> i) & 1) ? 0xFF : 0x00;
	
	int8_t uart_res = _uart.exch(tx_buff, size, rx_buff, &size, RX_WAIT_TIMEOUT);
	
	if(uart_res == uart::RES_RX_TIMEOUT)
		return ONEWIRE_ERR_NO_DEV;
	else if(uart_res == uart::RES_RX_FAIL || size != sizeof(rx_buff))
		return ONEWIRE_ERR_RX_FAIL;
	else if(uart_res == uart::RES_TX_FAIL)
		return ONEWIRE_ERR_TX_FAIL;
	
	if(memcmp(tx_buff, rx_buff, sizeof(tx_buff)))
		return ONEWIRE_ERR_LINE_BUSY;
	
	return ONEWIRE_ERR_NONE;
}

int8_t onewire::read_byte(uint8_t *byte)
{
	uint8_t tx_buff[8], rx_buff[8];
	uint16_t size = sizeof(rx_buff);
	
	memset(tx_buff, 0xFF, sizeof(tx_buff));
	int8_t uart_res = _uart.exch(tx_buff, size, rx_buff, &size, RX_WAIT_TIMEOUT);
	
	if(uart_res == uart::RES_RX_TIMEOUT)
		return ONEWIRE_ERR_NO_DEV;
	else if(uart_res == uart::RES_RX_FAIL || size != sizeof(rx_buff))
		return ONEWIRE_ERR_RX_FAIL;
	else if(uart_res == uart::RES_TX_FAIL)
		return ONEWIRE_ERR_TX_FAIL;
	
	*byte = 0;
	for(uint8_t i = 0; i < size; i++)
		*byte |= (rx_buff[i] == 0xFF) << i;
	
	return ONEWIRE_ERR_NONE;
}
