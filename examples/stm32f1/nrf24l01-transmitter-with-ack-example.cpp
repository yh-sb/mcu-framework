// Example for STM32VLDISCOVERY development board
// NRF24L01 module connected to SPI1: MOSI - PA7, MISO - PA6, CLK - PA5, CSN - PA4, CE - PA3, IRQ - PA2
// See receiver example in ../stm32f0/nrf24l01-receiver-with-ack-example.cpp

#include <cstring>
#include "FreeRTOS.h"
#include "task.h"
#include "periph/systick.hpp"
#include "periph/gpio_stm32f1.hpp"
#include "periph/dma_stm32f1.hpp"
#include "periph/spi_stm32f1.hpp"
#include "periph/exti_stm32f1.hpp"
#include "periph/timer_stm32f1.hpp"
#include "drivers/gpio_pin_debouncer.hpp"
#include "drivers/nrf24l01.hpp"

struct nrf_task_params_t
{
    drv::gpio_pin_debouncer &button;
    drv::nrf24l01 &nrf;
    periph::gpio &led;
};

static void nrf_task(void *pvParameters)
{
    nrf_task_params_t *task_params = (nrf_task_params_t *)pvParameters;
    drv::gpio_pin_debouncer &button_1 = task_params->button;
    drv::nrf24l01 &nrf = task_params->nrf;
    periph::gpio &green_led = task_params->led;
    
    auto res = nrf.init();
    assert(res == drv::nrf24l01::res::ok);
    
    drv::nrf24l01::config_t nrf_config {};
    res = nrf.get_config(nrf_config);
    assert(res == drv::nrf24l01::res::ok);
    
    nrf_config.tx_addr = 0xA5A5;
    nrf_config.tx_auto_ack = true;
    nrf_config.dyn_payload = true;
    nrf_config.datarate = drv::nrf24l01::datarate::_250_kbps;
    nrf_config.channel = 0;
    nrf_config.retransmit_delay = drv::nrf24l01::ard::_4000_US;
    nrf_config.retransmit_count = 15;
    
    res = nrf.set_config(nrf_config);
    assert(res == drv::nrf24l01::res::ok);
    
    while(1)
    {
        bool new_state;
        if(button_1.poll_1ms(new_state))
        {
            if(new_state)
            {
                char buff[] = "Hello!";
                drv::nrf24l01::packet_t ack {};
                
                res = nrf.write(buff, sizeof(buff), &ack);
                if(res == drv::nrf24l01::res::ok)
                {
                    green_led.toggle();
                }
                nrf.power_down();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int main(int argc, char *argv[])
{
    periph::systick::init();
    
    // Green LED
    periph::gpio_stm32f1 green_led(periph::gpio_stm32f1::port::c, 9, periph::gpio::mode::digital_output);
    
    // Button 1
    periph::gpio_stm32f1 button_1_gpio(periph::gpio_stm32f1::port::a, 0, periph::gpio::mode::digital_input);
    drv::gpio_pin_debouncer button_1(button_1_gpio, std::chrono::milliseconds(50), 1);
    
    // SPI1 interface
    periph::gpio_stm32f1 spi1_mosi(periph::gpio_stm32f1::port::a, 7, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f1 spi1_miso(periph::gpio_stm32f1::port::a, 6, periph::gpio::mode::alternate_function, 1);
    periph::gpio_stm32f1 spi1_clk(periph::gpio_stm32f1::port::a, 5, periph::gpio::mode::alternate_function, 1);
    periph::dma_stm32f1 spi1_write_dma(1, 3, periph::dma_stm32f1::direction::memory_to_periph, 8);
    periph::dma_stm32f1 spi1_read_dma(1, 2, periph::dma_stm32f1::direction::periph_to_memory, 8);
    periph::spi_stm32f1 spi1(1, 1000000, periph::spi::cpol::low, periph::spi::cpha::leading,
        periph::spi::bit_order::msb, spi1_write_dma, spi1_read_dma, spi1_mosi, spi1_miso, spi1_clk);
    
    // NRF24L01
    periph::gpio_stm32f1 nrf24l01_csn(periph::gpio_stm32f1::port::a, 4, periph::gpio::mode::digital_output, 1);
    periph::gpio_stm32f1 nrf24l01_ce(periph::gpio_stm32f1::port::a, 3, periph::gpio::mode::digital_output, 0);
    periph::gpio_stm32f1 nrf24l01_irq(periph::gpio_stm32f1::port::a, 2, periph::gpio::mode::digital_input, 1);
    periph::exti_stm32f1 nrf24l01_exti2(nrf24l01_irq, periph::exti::trigger::falling);
    periph::timer_stm32f1 nrf24l01_timer6(6);
    drv::nrf24l01 nrf(spi1, nrf24l01_csn, nrf24l01_ce, nrf24l01_exti2, nrf24l01_timer6);
    
    nrf_task_params_t nrf_task_params = {button_1, nrf, green_led};
    xTaskCreate(nrf_task, "nrf", configMINIMAL_STACK_SIZE + 20, &nrf_task_params, 1, nullptr);
    
    vTaskStartScheduler();
}
