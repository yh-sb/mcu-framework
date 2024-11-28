#include <cstring>
#include <cassert>
#include "freertos_wrappers.hpp"
#include "drivers/nrf24l01.hpp"
#include "FreeRTOS.h"
#include "task.h"

using namespace drv;

constexpr std::chrono::milliseconds power_on_reset_timeout(100);
constexpr std::chrono::microseconds powerdown_to_standby1_timeout(1500);
constexpr std::chrono::microseconds standby1_to_rxtx_timeout(130);
constexpr std::chrono::milliseconds transmit_max_timeout(60); // ARD_4000_US * 15 retries = 60 ms
constexpr auto addr_max_size = 5;
constexpr auto addr_min_size = 1;

enum prim_rx_t
{
    PTX,
    PRX
};

enum crco_t
{
    _1_BYTE,
    _2_BYTE
};

enum aw_t
{
    _3_BYTES = 1,
    _4_BYTES = 2,
    _5_BYTES = 3
};

enum ard_t
{
    _250_US,
    _500_US,
    _750_US,
    _1000_US,
    _1250_US,
    _1500_US,
    _1750_US,
    _2000_US,
    _2250_US,
    _2500_US,
    _2750_US,
    _3000_US,
    _3250_US,
    _3500_US,
    _3750_US,
    _4000_US
};

enum rf_pwr_t
{
    _18_DBM,
    _12_DBM,
    _6_DBM,
    _0_DBM
};

#pragma pack(push, 1)
struct config_reg_t
{
    prim_rx_t prim_rx:1;
    bool pwr_up:1;
    crco_t crco:1;
    bool en_crc:1;
    bool mask_max_rt:1;
    bool mask_tx_ds:1;
    bool mask_rx_dr:1;
    bool reserved:1;
};

struct setup_aw_reg_t
{
    aw_t aw:2;
    uint8_t reserved:6;
};

struct setup_retr_reg_t
{
    uint8_t arc:4;
    ard_t ard:4;
};


union rf_setup_reg_t
{
    struct
    {
        bool lna_hcurr:1;
        rf_pwr_t rf_pwr:2;
        bool rf_dr:1;
        bool pll_lock:1;
        uint8_t reserved:3;
    } nrf24l01;
    struct
    {
        bool obselete:1;
        rf_pwr_t rf_pwr:2;
        bool rf_dr_high:1;
        bool pll_lock:1;
        bool rf_dr_low:1;
        bool reserved:1;
        bool cont_wave:1;
    } nrf24l01_plus;
};

struct status_reg_t
{
    bool tx_full:1;
    uint8_t rx_p_no:3;
    bool max_rt:1;
    bool tx_ds:1;
    bool rx_dr:1;
    bool reserved:1;
};

struct observe_tx_reg_t
{
    uint8_t arc_cnt:4;
    uint8_t plos_cnt:4;
};

struct fifo_status_reg_t
{
    bool rx_empty:1;
    bool rx_full:1;
    uint8_t reserved1:2;
    bool tx_empty:1;
    bool tx_full:1;
    bool tx_reuse:1;
    bool reserved2:1;
};

struct feature_reg_t // Only for nrf24l01+
{
    bool en_dyn_ack:1;
    bool en_ack_pay:1;
    bool en_dpl:1;
    uint8_t reserved:5;
};
#pragma pack(pop)

nrf24l01::nrf24l01(periph::spi &spi, periph::gpio &cs, periph::gpio &ce,
    periph::exti &exti, periph::timer &timer, enum dev dev):
    spi(spi),
    cs(cs),
    ce(ce),
    exti(exti),
    timer(timer),
    conf({.dev = dev})
{
    assert(spi.cpol() == periph::spi::cpol::low);
    assert(spi.cpha() == periph::spi::cpha::leading);
    assert(spi.bit_order() == periph::spi::bit_order::msb);
    assert(cs.mode() == periph::gpio::mode::digital_output);
    assert(ce.mode() == periph::gpio::mode::digital_output);
    
    cs.set(1);
    ce.set(0);
    
    assert(api_lock = xSemaphoreCreateMutex());
    
    exti.set_callback([this]() {
        BaseType_t hi_task_woken = 0;
        vTaskNotifyGiveFromISR(task, &hi_task_woken);
        portYIELD_FROM_ISR(hi_task_woken);
    });
    
    timer.set_callback([this]() {
        BaseType_t hi_task_woken = 0;
        vTaskNotifyGiveFromISR(task, &hi_task_woken);
        portYIELD_FROM_ISR(hi_task_woken);
    });
}

nrf24l01::~nrf24l01()
{
    // Reset IRQ flags
    status_reg_t status = {.max_rt = 1, .tx_ds = 1, .rx_dr = 1};
    write_reg(reg::STATUS, &status);
    
    cs.set(1);
    ce.set(0);
    exti.disable();
    timer.stop();
    xSemaphoreGive(api_lock);
    vSemaphoreDelete(api_lock);
}

enum nrf24l01::res nrf24l01::init()
{
    freertos::semaphore_take(api_lock, portMAX_DELAY);
    
    ce.set(0);
    vTaskDelay(pdMS_TO_TICKS(power_on_reset_timeout.count()));
    
    auto res = set_mode(mode::PWR_DOWN);
    if(res != res::ok)
    {
        return res;
    }
    
    setup_aw_reg_t setup_aw;
    res = read_reg(reg::SETUP_AW, &setup_aw);
    if(res != res::ok)
    {
        return res;
    }
    
    // Check for invalid response from nrf24l01
    if(setup_aw.aw == 0 || setup_aw.reserved != 0)
    {
        return res::no_response;
    }
    
    if(conf.dev == dev::NRF24L01_PLUS)
    {
        feature_reg_t feature;
        res = read_reg(reg::CONFIG, &feature);
        if(res != res::ok)
        {
            return res;
        }
        
        conf.dyn_payload = feature.en_dpl;
        conf.ack_payload = feature.en_ack_pay && feature.en_dpl;
    }
    
    config_reg_t config = {.crco = _2_BYTE, .en_crc = 1};
    res = write_reg(reg::CONFIG, &config);
    if(res != res::ok)
    {
        return res;
    }
    
    // Disable all rx data pipes, since pipe 0 and 1 are enabled by default
    uint8_t en_rxaddr = 0;
    res = write_reg(reg::EN_RXADDR, &en_rxaddr);
    if(res != res::ok)
    {
        return res;
    }
    memset(conf.pipe, 0, sizeof(conf.pipe));
    
    res = exec_instruction(instruction::FLUSH_TX);
    if(res != res::ok)
    {
        return res;
    }
    
    res = exec_instruction(instruction::FLUSH_RX);
    if(res != res::ok)
    {
        return res;
    }
    
    // Reset IRQ flags
    status_reg_t status = {.max_rt = 1, .tx_ds = 1, .rx_dr = 1};
    res = write_reg(reg::STATUS, &status);
    if(res != res::ok)
    {
        return res;
    }
    
    return res;
}

enum nrf24l01::res nrf24l01::get_config(config_t &config)
{
    freertos::semaphore_take(api_lock, portMAX_DELAY);
    
    memset(&config, 0, sizeof(config));
    
    uint8_t en_rxaddr;
    auto res = read_reg(reg::EN_RXADDR, &en_rxaddr);
    if(res != res::ok)
    {
        return res;
    }
    
    uint8_t en_aa;
    res = read_reg(reg::EN_AA, &en_aa);
    if(res != res::ok)
    {
        return res;
    }
    
    rf_setup_reg_t rf_setup;
    res = read_reg(reg::RF_SETUP, &rf_setup);
    if(res != res::ok)
    {
        return res;
    }
    
    uint8_t dynpd;
    if(conf.dev == dev::NRF24L01_PLUS)
    {
        res = read_reg(reg::DYNPD, &dynpd);
        if(res != res::ok)
        {
            return res;
        }
        
        feature_reg_t feature;
        res = read_reg(reg::FEATURE, &feature);
        if(res != res::ok)
        {
            return res;
        }
        conf.dyn_payload = feature.en_dpl;
        
        if(rf_setup.nrf24l01_plus.rf_dr_low)
            config.datarate = datarate::_250_kbps;
        else if(rf_setup.nrf24l01_plus.rf_dr_high)
            config.datarate = datarate::_2_Mbps;
        else
            config.datarate = datarate::_1_Mbps;
    }
    else
    {
        config.datarate = rf_setup.nrf24l01.rf_dr ? datarate::_2_Mbps : datarate::_1_Mbps;
    }
    switch(rf_setup.nrf24l01.rf_pwr)
    {
        case _18_DBM: config.power = pwr::_18_dBm; break;
        case _12_DBM: config.power = pwr::_12_dBm; break;
        case _6_DBM: config.power = pwr::_6_dBm; break;
        case _0_DBM: config.power = pwr::_0_dBm; break;
    }
    
    for(uint8_t i = 0; i < pipes; i++)
    {
        config.pipe[i].enable = en_rxaddr & (1 << i);
        
        uint8_t rx_pw = static_cast<uint8_t>(reg::RX_PW_P0) + i;
        uint8_t rx_pw_val = 0;
        res = read_reg(static_cast<enum reg>(rx_pw), &rx_pw_val);
        if(res != res::ok)
        {
            return res;
        }
        config.pipe[i].size = rx_pw_val;
        conf.pipe[i].size = rx_pw_val;
        
        uint8_t rx_addr = static_cast<uint8_t>(reg::RX_ADDR_P0) + i;
        uint64_t rx_addr_val = 0;
        res = read_reg(static_cast<enum reg>(rx_addr), &rx_addr_val, i < 2 ? addr_max_size : addr_min_size);
        if(res != res::ok)
        {
            return res;
        }
        config.pipe[i].addr = rx_addr_val;
        
        config.pipe[i].auto_ack = en_aa & (1 << i);
        
        if(conf.dev == dev::NRF24L01_PLUS)
        {
            config.pipe[i].dyn_payload = dynpd & (1 << i);
            conf.pipe[i].dyn_payload = dynpd & (1 << i);
        }
    }
    
    uint64_t tx_addr_val = 0;
    res = read_reg(reg::TX_ADDR, &tx_addr_val, addr_max_size);
    if(res != res::ok)
    {
        return res;
    }
    config.tx_addr = tx_addr_val;
    
    config.tx_auto_ack = (config.tx_addr == config.pipe[0].addr && config.pipe[0].enable);
    
    res = read_reg(reg::RF_CH, &config.channel);
    if(res != res::ok)
    {
        return res;
    }
    
    setup_retr_reg_t retr_reg;
    res = read_reg(reg::SETUP_RETR, &retr_reg);
    if(res != res::ok)
    {
        return res;
    }
    config.retransmit_count = retr_reg.arc;
    config.retransmit_delay = static_cast<nrf24l01::ard>(retr_reg.ard);
    
    return res;
}

enum nrf24l01::res nrf24l01::set_config(config_t &config)
{
    is_config_valid(config);
    
    freertos::semaphore_take(api_lock, portMAX_DELAY);
    
    if(config.tx_auto_ack)
    {
        config.pipe[0].enable = true;
        config.pipe[0].size = fifo_size;
        config.pipe[0].addr = config.tx_addr;
        config.pipe[0].auto_ack = true;
        config.pipe[0].dyn_payload = config.dyn_payload;
    }
    
    res res;
    uint8_t en_rxaddr = 0, en_aa = 0, dynpd = 0;
    for(uint8_t i = 0; i < pipes; i++)
    {
        en_rxaddr |= config.pipe[i].enable << i;
        en_aa |= config.pipe[i].auto_ack << i;
        if(conf.dev == dev::NRF24L01_PLUS)
        {
            dynpd |= conf.pipe[i].dyn_payload << i;
            conf.pipe[i].dyn_payload = conf.pipe[i].dyn_payload;
        }
        
        uint8_t rx_pw = static_cast<uint8_t>(reg::RX_PW_P0) + i;
        uint8_t rx_pw_val = conf.pipe[i].size;
        res = write_reg(static_cast<enum reg>(rx_pw), &rx_pw_val);
        if(res != res::ok)
        {
            return res;
        }
        conf.pipe[i].size = conf.pipe[i].size;
        
        uint8_t rx_addr = static_cast<uint8_t>(reg::RX_ADDR_P0) + i;
        uint64_t rx_addr_val = config.pipe[i].addr;
        res = write_reg(static_cast<enum reg>(rx_addr), &rx_addr_val, i < 2 ? addr_max_size : addr_min_size);
        if(res != res::ok)
        {
            return res;
        }
    }
    
    res = write_reg(reg::EN_RXADDR, &en_rxaddr);
    if(res != res::ok)
    {
        return res;
    }
    
    res = write_reg(reg::EN_AA, &en_aa);
    if(res != res::ok)
    {
        return res;
    }
    
    uint64_t tx_addr_val = config.tx_addr;
    res = write_reg(reg::TX_ADDR, &tx_addr_val, addr_max_size);
    if(res != res::ok)
    {
        return res;
    }
    
    rf_setup_reg_t rf_setup;
    res = read_reg(reg::RF_SETUP, &rf_setup);
    if(res != res::ok)
    {
        return res;
    }
    
    switch(config.power)
    {
        case pwr::_0_dBm: rf_setup.nrf24l01.rf_pwr = _0_DBM; break;
        case pwr::_6_dBm: rf_setup.nrf24l01.rf_pwr = _6_DBM; break;
        case pwr::_12_dBm: rf_setup.nrf24l01.rf_pwr = _12_DBM; break;
        case pwr::_18_dBm: rf_setup.nrf24l01.rf_pwr = _18_DBM; break;
    }
    
    if(conf.dev == dev::NRF24L01_PLUS)
    {
        res = write_reg(reg::DYNPD, &dynpd);
        if(res != res::ok)
        {
            return res;
        }
        
        feature_reg_t feature;
        res = read_reg(reg::FEATURE, &feature);
        if(res != res::ok)
        {
            return res;
        }
        
        if(feature.en_dpl != conf.dyn_payload)
        {
            feature.en_dpl = conf.dyn_payload;
            res = write_reg(reg::FEATURE, &feature);
            if(res != res::ok)
            {
                return res;
            }
        }
        conf.dyn_payload = feature.en_dpl;
        conf.ack_payload = feature.en_dpl && feature.en_ack_pay;
        
        if(config.datarate == datarate::_250_kbps)
        {
            rf_setup.nrf24l01_plus.rf_dr_low = true;
            rf_setup.nrf24l01_plus.rf_dr_high = false;
        }
        else if(config.datarate == datarate::_2_Mbps)
        {
            rf_setup.nrf24l01_plus.rf_dr_low = false;
            rf_setup.nrf24l01_plus.rf_dr_high = true;
        }
        else
        {
            rf_setup.nrf24l01_plus.rf_dr_low = false;
            rf_setup.nrf24l01_plus.rf_dr_high = false;
        }
    }
    else
    {
        rf_setup.nrf24l01.rf_dr = config.datarate == datarate::_2_Mbps;
    }
    
    res = write_reg(reg::RF_SETUP, &rf_setup);
    if(res != res::ok)
    {
        return res;
    }
    
    res = write_reg(reg::RF_CH, &config.channel);
    if(res != res::ok)
    {
        return res;
    }
    
    setup_retr_reg_t retr_reg =
    {
        .arc = config.retransmit_count,
        .ard = static_cast<ard_t>(config.retransmit_delay)
    };
    res = write_reg(reg::SETUP_RETR, &retr_reg);
    if(res != res::ok)
    {
        return res;
    }
    
    res = exec_instruction(instruction::FLUSH_TX);
    if(res != res::ok)
    {
        return res;
    }
    
    res = exec_instruction(instruction::FLUSH_RX);
    if(res != res::ok)
    {
        return res;
    }
    
    if(!en_rxaddr && conf.mode != mode::PWR_DOWN)
    {
        res = set_mode(mode::PWR_DOWN);
    }
    
    return res;
}

bool nrf24l01::is_config_valid(config_t &config)
{
    if(conf.dev != dev::NRF24L01_PLUS && config.dyn_payload)
    {
        // Dynamic payload size available only for nrf24l01+
        assert(0);
        return false;
    }
    
    if(conf.dev == dev::NRF24L01 && config.datarate == datarate::_250_kbps)
    {
        // 250 kbps datarate isn't available for nrf24l01
        assert(0);
        return false;
    }
    
    if(config.channel > 125)
    {
        assert(0);
        return false;
    }
    
    for(uint8_t i = 0; i < pipes; i++)
    {
        if(config.pipe[i].size > fifo_size)
        {
            assert(0);
            return false;
        }
        
        if(i > 1 && config.pipe[i].addr > 0xFF)
        {
            /* Only byte 0 can be configured in pipe rx address for pipe number
            2-5 */
            assert(0);
            return false;
        }
        
        if(conf.dev != dev::NRF24L01_PLUS && config.pipe[i].dyn_payload)
        {
            // Dynamic payload size available only for nrf24l01+
            assert(0);
            return false;
        }
        
        if(config.pipe[i].dyn_payload && !config.pipe[i].auto_ack)
        {
            // Auto ack is requrements for dynamic payload
            assert(0);
            return false;
        }
    }
    return true;
}

enum nrf24l01::res nrf24l01::read(packet_t &packet, packet_t *ack)
{
    /* If ack payload is provided, dynamic payload size should be configured and
    ack payload size must be also valid */
    assert(!ack || (conf.dyn_payload && ack->size > 0 &&
        ack->size <= fifo_size && ack->pipe < pipes));
    
    freertos::semaphore_take(api_lock, portMAX_DELAY);
    
    // Check FIFO_STATUS first to read the data received earlier is exist
    fifo_status_reg_t fifo_status;
    auto res = read_reg(reg::FIFO_STATUS, &fifo_status);
    if(res != res::ok)
    {
        return res;
    }
    
    if(fifo_status.rx_empty)
    {
        if(conf.mode != mode::RX)
        {
            res = set_mode(mode::RX);
            if(res != res::ok)
            {
                return res;
            }
        }
        
        if(static_cast<bool>(ack) != conf.ack_payload)
        {
            res = ack_payload(static_cast<bool>(ack));
            if(res != res::ok)
            {
                return res;
            }
        }
        if(ack)
        {
            // Write ack payload to be transmitted after reception
            uint8_t w_ack_payload_with_pipe =
                static_cast<uint8_t>(instruction::W_ACK_PAYLOAD) | ack->pipe;
            res = exec_instruction_with_write(static_cast<instruction>(w_ack_payload_with_pipe),
                ack->buff, ack->size);
            if(res != res::ok)
            {
                return res;
            }
        }
        
        // Wait for data
        task = xTaskGetCurrentTaskHandle();
        ce.set(1);
        exti.enable();
        ulTaskNotifyTake(true, portMAX_DELAY);
        exti.disable();
    }
    
    // Read received data
    status_reg_t status;
    res = read_reg(reg::STATUS, &status);
    if(res != res::ok)
    {
        goto exit;
    }
    
    if(ack && status.max_rt) // Payload ack wasn't transmitted
    {
        res = res::tx_max_retries;
        goto exit;
    }
    
    if(status.rx_p_no > pipes - 1)
    {
        res = res::spi_error;
        goto exit;
    }
    
    if(conf.pipe[status.rx_p_no].dyn_payload)
    {
        res = exec_instruction_with_read(instruction::R_RX_PL_WID, &packet.size,
            sizeof(packet.size));
        if(res != res::ok)
        {
            goto exit;
        }
        if(!packet.size || packet.size > fifo_size)
        {
            res = res::spi_error;
            goto exit;
        }
    }
    else
    {
        packet.size = conf.pipe[status.rx_p_no].size;
    }
    
    res = exec_instruction_with_read(instruction::R_RX_PAYLOAD, packet.buff, packet.size);
    packet.pipe = status.rx_p_no;
    
exit:
    status.rx_dr = 1;
    status.tx_ds = 1;
    status.max_rt = 1;
    auto res_final = write_reg(reg::STATUS, &status);
    return res != res::ok ? res : res_final;
}

enum nrf24l01::res nrf24l01::write(void *buff, size_t size, packet_t *ack, bool is_continuous_tx)
{
    assert(buff);
    assert(size > 0 && size <= fifo_size);
    assert(size == fifo_size || conf.dyn_payload);
    // Ack payload supported only for nrf24l01+
    assert(!ack || conf.dyn_payload);
    
    freertos::semaphore_take(api_lock, portMAX_DELAY);
    
    auto res = res::ok;
    if(conf.mode != mode::TX)
    {
        res = set_mode(mode::TX);
        if(res != res::ok)
        {
            ce.set(0);
            return res;
        }
    }
    
    if(static_cast<bool>(ack) != conf.ack_payload)
    {
        res = ack_payload(static_cast<bool>(ack));
        if(res != res::ok)
        {
            ce.set(0);
            return res;
        }
    }
    
    res = exec_instruction_with_write(instruction::W_TX_PAYLOAD, buff, size);
    if(res != res::ok)
    {
        ce.set(0);
        return res;
    }
    
    task = xTaskGetCurrentTaskHandle();
    exti.enable();
    
    ce.set(1);
    /* is_continuous_tx 0: go to standby-1 mode after transmitting one packet
       is_continuous_tx 1: go to standby-2 mode when fifo will be empty
    */
    if(!is_continuous_tx)
    {
        delay(std::chrono::microseconds(10));
        ce.set(0);
    }
    
    // TODO: Calculate presize timeout based on values of arc, ard and datarate
    if(!ulTaskNotifyTake(true, transmit_max_timeout.count()))
    {
        if(is_continuous_tx)
        {
            ce.set(0);
        }
        exti.disable();
        return res::no_response;
    }
    exti.disable();
    
    status_reg_t status;
    res = read_reg(reg::STATUS, &status);
    if(res != res::ok)
    {
        if(is_continuous_tx)
        {
            ce.set(0);
        }
        memset(&status, 0, sizeof(status));
    }
    
    auto txrx_res = res::ok;
    if(status.max_rt)
    {
        // Tx fifo payload isn't removed in case of max_rt. So flush it manually
        exec_instruction(instruction::FLUSH_TX);
        txrx_res = res::tx_max_retries;
    }
    else if(status.rx_dr) // Payload was received
    {
        if(ack)
        {
            // Get ack payload length and read it
            txrx_res = exec_instruction_with_read(instruction::R_RX_PL_WID,
                &ack->size, sizeof(ack->size));
            if(!ack->size || ack->size > fifo_size || status.rx_p_no > pipes - 1)
            {
                txrx_res = res::spi_error;
            }
            
            ack->pipe = status.rx_p_no;
            if(txrx_res == res::ok)
            {
                txrx_res = exec_instruction_with_read(instruction::R_RX_PAYLOAD,
                    ack->buff, ack->size);
            }
            else
            {
                exec_instruction(instruction::FLUSH_RX);
            }
        }
        else
        {
            exec_instruction(instruction::FLUSH_RX);
        }
    }
    
    status.max_rt = 1;
    status.tx_ds = 1;
    status.rx_dr = 1;
    res = write_reg(reg::STATUS, &status);
    
    return txrx_res != res::ok ? txrx_res : res;
}

enum nrf24l01::res nrf24l01::power_down()
{
    freertos::semaphore_take(api_lock, portMAX_DELAY);
    
    return set_mode(mode::PWR_DOWN);
}

enum nrf24l01::res nrf24l01::read_reg(reg reg, void *data, size_t size)
{
    periph::spi_cs cs(cs);
    
    if(reg == reg::STATUS)
    {
        instruction instruction = instruction::NOP;
        if(spi.write_read(&instruction, data, 1) != periph::spi::res::ok)
        {
            return res::spi_error;
        }
        
        return res::ok;
    }
    
    if(spi.write(static_cast<uint8_t>(instruction::R_REGISTER) | static_cast<uint8_t>(reg)) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    if(spi.read(data, size) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    return res::ok;
}

enum nrf24l01::res nrf24l01::write_reg(reg reg, void *data, size_t size)
{
    periph::spi_cs cs(cs);
    
    if(spi.write(static_cast<uint8_t>(instruction::W_REGISTER) |
        static_cast<uint8_t>(reg)) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    if(spi.write(data, size) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    return res::ok;
}

enum nrf24l01::res nrf24l01::exec_instruction(instruction instruction)
{
    periph::spi_cs cs(cs);
    
    if(spi.write(static_cast<uint8_t>(instruction)) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    return res::ok;
}

enum nrf24l01::res nrf24l01::exec_instruction_with_read(instruction instruction, void *buff, size_t size)
{
    periph::spi_cs cs(cs);
    
    if(spi.write(static_cast<uint8_t>(instruction)) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    if(spi.read(buff, size) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    return res::ok;
}

enum nrf24l01::res nrf24l01::exec_instruction_with_write(instruction instruction, void *buff, size_t size)
{
    periph::spi_cs cs(cs);
    
    if(spi.write(static_cast<uint8_t>(instruction)) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    if(spi.write(buff, size) != periph::spi::res::ok)
    {
        return res::spi_error;
    }
    
    return res::ok;
}

enum nrf24l01::res nrf24l01::set_mode(mode mode)
{
    auto res = res::ok;
    config_reg_t config;
    std::chrono::microseconds wait_timeout;
    
    switch(mode)
    {
        case mode::PWR_DOWN:
            ce.set(0);
            res = read_reg(reg::CONFIG, &config);
            if(res != res::ok)
            {
                return res;
            }
            
            config.pwr_up = false;
            res = write_reg(reg::CONFIG, &config);
            if(res != res::ok)
            {
                return res;
            }
            break;
        
        case mode::STANDBY_1:
            ce.set(0);
            if(conf.mode == mode::PWR_DOWN)
            {
                res = read_reg(reg::CONFIG, &config);
                if(res != res::ok)
                {
                    return res;
                }
                
                config.pwr_up = true;
                res = write_reg(reg::CONFIG, &config);
                if(res != res::ok)
                {
                    return res;
                }
                wait_timeout += powerdown_to_standby1_timeout;
            }
            break;
        
        case mode::RX:
            ce.set(0); // Disable CE to switch into Standby-1 mode first
            res = read_reg(reg::CONFIG, &config);
            if(res != res::ok)
            {
                return res;
            }
            
            config.pwr_up = true;
            config.prim_rx = PRX;
            res = write_reg(reg::CONFIG, &config);
            if(res != res::ok)
            {
                return res;
            }
            /* Don't wait any timeout here since reading is blocking operation.
            Will wait inside read() method */
            
            res = exec_instruction(instruction::FLUSH_RX);
            if(res != res::ok)
            {
                return res;
            }
            /* Don't set CE=1 here. Manage it in read() method to avoid
            situation when rx IRQ already happened but we haven't had time to
            start waiting for it */
            break;
        
        case mode::TX:
            ce.set(0); // Disable CE to switch into Standby-1 mode first
            res = read_reg(reg::CONFIG, &config);
            if(res != res::ok)
            {
                return res;
            }
            if(!config.pwr_up)
            {
                config.pwr_up = true;
                wait_timeout += powerdown_to_standby1_timeout;
            }
            config.prim_rx = PTX;
            res = write_reg(reg::CONFIG, &config);
            if(res != res::ok)
            {
                return res;
            }
            wait_timeout += standby1_to_rxtx_timeout;
            
            res = exec_instruction(instruction::FLUSH_TX);
            if(res != res::ok)
            {
                return res;
            }
            /* Don't set CE=1 here. Manage CE it in write() method to support
            continuous tx operation */
            break;
    }
    
    if(wait_timeout.count())
    {
        delay(wait_timeout);
    }
    
    conf.mode = mode;
    return res::ok;
}

enum nrf24l01::res nrf24l01::ack_payload(bool enable)
{
    feature_reg_t feature;
    auto res = read_reg(reg::FEATURE, &feature);
    if(res != res::ok)
    {
        return res;
    }
    
    if((feature.en_dpl != enable) || (feature.en_ack_pay != enable))
    {
        /* Do not turn-off dynamic payload size if it was explicitly enabled by
        user */
        feature.en_dpl = enable || conf.dyn_payload;
        
        feature.en_ack_pay = enable;
        
        res = write_reg(reg::FEATURE, &feature);
        if(res != res::ok)
        {
            return res;
        }
    }
    conf.ack_payload = enable;
    
    // Setup dynamic payload for pipe0 which is used for tx
    if(conf.mode == mode::TX)
    {
        uint8_t dynpd;
        res = read_reg(reg::DYNPD, &dynpd);
        if(res != res::ok)
        {
            return res;
        }
        
        if(enable != (dynpd & 1))
        {
            if(enable)
            {
                dynpd |= 1;
            }
            else
            {
                dynpd &= ~1;
            }
            
            res = write_reg(reg::DYNPD, &dynpd);
            if(res != res::ok)
            {
                return res;
            }
        }
    }
    
    return res;
}

void nrf24l01::delay(std::chrono::microseconds timeout)
{
    task = xTaskGetCurrentTaskHandle();
    timer.timeout(timeout);
    timer.start();
    ulTaskNotifyTake(true, portMAX_DELAY);
}
