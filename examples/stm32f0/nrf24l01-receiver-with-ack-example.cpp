// Example for STM32F072DISCOVERY development board
// NRF24L01 module connected to SPI1: MOSI - PB5, MISO - PB4, CLK - PB3, CSN - PB8, CE - PB6, IRQ - PB7
// See transmitter example in ../stm32f1/nrf24l01-transmitter-with-ack-example.cpp

#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f0.hpp"
#include "periph/dma_stm32f0.hpp"
#include "periph/spi_stm32f0.hpp"
#include "periph/exti_stm32f0.hpp"
#include "periph/timer_stm32f0.hpp"
#include "drivers/nrf24l01.hpp"

struct nrf_task_params_t
{
    drv::nrf24l01 &nrf;
    periph::gpio &led;
};

static void nrf_task(void *pvParameters)
{
    nrf_task_params_t *task_params = (nrf_task_params_t *)pvParameters;
    drv::nrf24l01 &nrf = task_params->nrf;
    periph::gpio &green_led = task_params->led;
    
    auto res = nrf.init();
    assert(res == drv::nrf24l01::res::ok);
    
    drv::nrf24l01::config_t nrf_config {};
    res = nrf.get_config(nrf_config);
    assert(res == drv::nrf24l01::res::ok);
    
    nrf_config.pipe[1].enable = true;
    nrf_config.pipe[1].addr = 0xA5A5;
    nrf_config.pipe[1].auto_ack = true;
    nrf_config.pipe[1].size = drv::nrf24l01::fifo_size;
    nrf_config.pipe[1].dyn_payload = true;
    nrf_config.dyn_payload = true;
    nrf_config.datarate = drv::nrf24l01::datarate::_250_kbps;
    nrf_config.channel = 0;
    nrf_config.retransmit_delay = drv::nrf24l01::ard::_4000_US;
    nrf_config.retransmit_count = 15;
    
    res = nrf.set_config(nrf_config);
    assert(res == drv::nrf24l01::res::ok);
    
    while(1)
    {
        drv::nrf24l01::packet_t packet {}, ack {};
        strncpy((char *)packet.buff, "my ack", sizeof(packet.buff));
        ack.size = sizeof("my ack") - 1;
        ack.pipe = 1;
        
        res = nrf.read(packet, &ack);
        if(res == drv::nrf24l01::res::ok && !strncmp((const char *)packet.buff,
            "Hello!", sizeof("Hello!") - 1)) // Message from transmitter
        {
            green_led.toggle();
        }
    }
    
    res = nrf.get_config(nrf_config);
    assert(res == drv::nrf24l01::res::ok);
    
    nrf_config.pipe[1].enable = false;
    
    res = nrf.set_config(nrf_config);
    assert(res == drv::nrf24l01::res::ok);
}

int main(int argc, char *argv[])
{
    periph::systick::init();
    
    // Green LED
    periph::gpio_stm32f0 green_led(periph::gpio_stm32f0::port::c, 9, periph::gpio::mode::digital_output);
    
    // SPI1 interface
    periph::gpio_stm32f0 spi1_mosi(periph::gpio_stm32f0::port::b, 5, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f0 spi1_miso(periph::gpio_stm32f0::port::b, 4, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f0 spi1_clk(periph::gpio_stm32f0::port::b, 3, periph::gpio::mode::alternate_function, 1);
    periph::dma_stm32f0 spi1_write_dma(1, 3, periph::dma_stm32f0::direction::memory_to_periph, 8);
    periph::dma_stm32f0 spi1_read_dma(1, 2, periph::dma_stm32f0::direction::periph_to_memory, 8);
    periph::spi_stm32f0 spi1(1, 4000000, periph::spi::cpol::low, periph::spi::cpha::leading,
        periph::spi::bit_order::msb, spi1_write_dma, spi1_read_dma, spi1_mosi, spi1_miso, spi1_clk);
    
    // NRF24L01
    periph::gpio_stm32f0 nrf24l01_csn(periph::gpio_stm32f0::port::b, 8, periph::gpio::mode::digital_output, 1);
    periph::gpio_stm32f0 nrf24l01_ce(periph::gpio_stm32f0::port::b, 6, periph::gpio::mode::digital_output, 0);
    periph::gpio_stm32f0 nrf24l01_irq(periph::gpio_stm32f0::port::b, 7, periph::gpio::mode::digital_input, 1);
    periph::exti_stm32f0 nrf24l01_exti7(nrf24l01_irq, periph::exti::trigger::falling);
    periph::timer_stm32f0 nrf24l01_timer6(6);
    drv::nrf24l01 nrf(spi1, nrf24l01_csn, nrf24l01_ce, nrf24l01_exti7, nrf24l01_timer6);
    
    nrf_task_params_t nrf_task_params = {nrf, green_led};
    xTaskCreate(nrf_task, "nrf", configMINIMAL_STACK_SIZE + 20, &nrf_task_params, 1, nullptr);
    
    vTaskStartScheduler();
}
