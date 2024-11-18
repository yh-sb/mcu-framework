#include "stm32f1xx.h"
#include "core_cm3.h"
#include "FreeRTOS.h"

void HardFault_Handler()
{
    uint32_t hfsr = SCB->HFSR;
    
    __asm("bkpt");
    portDISABLE_INTERRUPTS();
    while(1);
}

void MemManage_Handler()
{
    __asm("bkpt");
    portDISABLE_INTERRUPTS();
    while(1);
}

void BusFault_Handler()
{
    __asm("bkpt");
    portDISABLE_INTERRUPTS();
    while(1);
}

void UsageFault_Handler()
{
    __asm("bkpt");
    portDISABLE_INTERRUPTS();
    while(1);
}
