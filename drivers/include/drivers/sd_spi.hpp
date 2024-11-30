#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sd.hpp"
#include "periph/gpio.hpp"
#include "periph/spi.hpp"

namespace drv
{
class sd_spi : public sd
{
public:
    sd_spi(periph::spi &spi, periph::gpio &cs, periph::gpio *cd = nullptr);
    ~sd_spi();
    
    // Delete copy constructor and copy assignment operator
    sd_spi(const sd_spi&) = delete;
    sd_spi& operator=(const sd_spi&) = delete;
    
    // Delete move constructor and move assignment operator
    sd_spi(sd_spi&&) = delete;
    sd_spi& operator=(sd_spi&&) = delete;
    
private:
    void select(bool is_selected) final;
    enum res init_sd() final;
    void set_speed(uint32_t speed) final;
    enum res send_cmd(cmd_t cmd, uint32_t arg, resp_t resp_type, uint8_t *resp) final;
    enum res read_data(void *data, uint16_t size) final;
    enum res write_data(void *data, uint16_t size) final;
    
    enum res wait_ready();
    
    periph::spi &spi;
    periph::gpio &cs;
};
} // namespace drv
