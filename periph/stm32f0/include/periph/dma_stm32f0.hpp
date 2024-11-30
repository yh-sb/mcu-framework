#pragma once

#include <cstdint>
#include <functional>

namespace periph { class dma_stm32f0; }
// For internal use only! (called from ISR)
extern "C" void dma_irq_hndlr(periph::dma_stm32f0 *obj);

namespace periph
{
class dma_stm32f0
{
public:
    enum class direction : uint8_t
    {
        periph_to_memory,
        memory_to_periph,
        memory_to_memory
    };
    
    enum class event : uint8_t
    {
        complete,
        half_complete,
        error
    };
    
    /**
     * @brief  Construct dma (direct memory access) object
     * 
     * @param  dma            Number of DMA controller. Can be 1 or 2
     * @param  channel        Number of DMA channel. Can be 1 to 7
     * @param  direction      Direction of data transfer
     * @param  increment_size Increment size of the source and destination addresses in bits. Can be 8, 16 or 32
     */
    dma_stm32f0(uint8_t dma, uint8_t channel, enum direction direction, uint8_t increment_size);
    ~dma_stm32f0();
    
    void source(const void *source);
    void destination(void *destination);
    void size(uint16_t size);
    void direction(enum direction direction);
    enum direction direction() const { return _direction; }
    
    /**
     * @brief  Set the increment size of the source and destination addresses
     * 
     * @param  increment_size Can be 8, 16 or 32 bits
     */
    void increment_size(uint8_t increment_size);
    
    /**
     * @brief  Get the increment size of the source and destination addresses
     * 
     * @return uint8_t The increment size in bits. Can be 8, 16 or 32 bits.
     */
    uint8_t increment_size() const { return inc_size; }
    
    uint16_t transfered() const;
    uint16_t remains() const;
    
    void set_callback(std::function<void(event event)> on_event);
    
    void start(bool is_cyclic = false);
    
    void stop();
    
    bool is_busy() const;
    
    uint8_t channel() const { return _channel; }
    
    // Delete copy constructor and copy assignment operator
    dma_stm32f0(const dma_stm32f0&) = delete;
    dma_stm32f0& operator=(const dma_stm32f0&) = delete;
    
    // Delete move constructor and move assignment operator
    dma_stm32f0(dma_stm32f0&&) = delete;
    dma_stm32f0& operator=(dma_stm32f0&&) = delete;
    
private:
    uint8_t dma;
    uint8_t _channel;
    enum direction _direction;
    uint8_t inc_size;
    uint32_t src;
    uint32_t dst;
    size_t _size;
    std::function<void(event event)> on_event;
    friend void ::dma_irq_hndlr(dma_stm32f0 *obj);
};
} // namespace periph
