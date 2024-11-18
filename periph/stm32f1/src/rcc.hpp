#pragma once

#include <cstdint>
#include "stm32f1xx.h"
#include "system_stm32f1xx.h"
#include "core_cm3.h"

namespace periph
{
class rcc
{
public:
    enum class clk_source : uint8_t
    {
        sysclk,
        ahb,
        apb1,
        apb2
    };
    
    enum class reset_reason : uint8_t
    {
        low_power,
        external,
        internal,
        wdt,
        unknown
    };
    
    static void init()
    {
        // Save the previous reset reason
        csr = RCC->CSR;
        
        // Clear the reset reason
        RCC->CSR |= RCC_CSR_RMVF;
        
        // Enable UsageFault_Handler(), BusFault_Handler() and MemManage_Handler()
        SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
            SCB_SHCSR_MEMFAULTENA_Msk;
    }
    
    // Get frequency of specific clock source in Hz
    static uint32_t frequency(clk_source clk_source)
    {
        uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
        uint32_t ahb_presc = ahb_prescallers[tmp];
        
        tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
        uint32_t apb1_presc = apb_prescallers[tmp];
        
        tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
        uint32_t apb2_presc = apb_prescallers[tmp];
        
        SystemCoreClockUpdate();
        uint32_t frequency = SystemCoreClock;
        
        switch(clk_source)
        {
            case clk_source::ahb:
                if(ahb_presc > 0)
                {
                    frequency /= ahb_presc;
                }
                break;
            
            case clk_source::apb1:
                if(ahb_presc > 0)
                {
                    frequency /= ahb_presc;
                }
                if(apb1_presc > 0)
                {
                    frequency /= apb1_presc;
                }
                break;
            
            case clk_source::apb2:
                if(ahb_presc > 0)
                {
                    frequency /= ahb_presc;
                }
                if(apb2_presc > 0)
                {
                    frequency /= apb2_presc;
                }
                break;
        }
        return frequency;
    }
    
    // Reset MCU
    static void reset()
    {
        NVIC_SystemReset();
    }
    
    // Get reason of previous reset
    static reset_reason reset_reason()
    {
        if(!csr)
        {
            init();
        }
        
        if(csr & RCC_CSR_PORRSTF || csr & RCC_CSR_LPWRRSTF)
        {
            return reset_reason::low_power;
        }
        else if(csr & RCC_CSR_PINRSTF)
        {
            return reset_reason::external;
        }
        else if(csr & RCC_CSR_SFTRSTF)
        {
            return reset_reason::internal;
        }
        else if(csr & RCC_CSR_IWDGRSTF || csr & RCC_CSR_WWDGRSTF)
        {
            return reset_reason::wdt;
        }
        return reset_reason::unknown;
    }
    
private:
    rcc() {}
    
    static uint32_t csr;
    
    // Converting CFGR[7:4] HPRE value to prescaller
    static constexpr uint32_t ahb_prescallers[16] =
    {
        0, 0, 0, 0, 0, 0, 0, 0, // 0-7 AHB prescaller 1
        2,                      // 8   AHB prescaller 2
        4,                      // 9   AHB prescaller 4
        8,                      // 10  AHB prescaller 8
        16,                     // 11  AHB prescaller 16
        64,                     // 12  AHB prescaller 64
        128,                    // 13  AHB prescaller 128
        256,                    // 14  AHB prescaller 256
        512                     // 15  AHB prescaller 512
    };
    
    // Converting CFGR[12:10] (the same as CFGR[15:13]) PPRE1 value to prescaller
    static constexpr uint32_t apb_prescallers[8] =
    {
        0, 0, 0, 0, // APB (1,2) prescaller 1
        2,          // APB (1,2) prescaller 2
        4,          // APB (1,2) prescaller 4
        8,          // APB (1,2) prescaller 8
        16          // APB (1,2) prescaller 16
    };
};
} // namespace periph
