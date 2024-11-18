// Custom __assert_func() function. It is called when assert() from stdlib fails

#include "FreeRTOS.h"

void __assert_func(const char *file, int line, const char *func,
    const char *failedexpr)
{
#if defined (__arm__) || defined (__aarch64__)
    __asm("bkpt");
#elif __XTENSA__
    printf("ASSERT FAILED: %s %s:%d (%s)\n", func, file, line, failedexpr);
#endif
    portDISABLE_INTERRUPTS();
    while(1);
}
