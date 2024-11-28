// Custom __assert_func() function. It is called when assert() from stdlib fails

#include "FreeRTOS.h"

void __assert_func(const char *file, int line, const char *func,
    const char *failedexpr)
{
    __asm("bkpt");
    portDISABLE_INTERRUPTS();
    while(1)
    {
    }
}
