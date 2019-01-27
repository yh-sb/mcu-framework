#pragma once

#include <stdint.h>

namespace hal
{
/**
 * @brief      Initialization of WDT timer
 *
 * @param[in]  ms    WDT timeout in ms
 */
void wdt_init(uint16_t ms);

/**
 * @brief      Start WDT timer
 */
void wdt_on(void);

/**
 * @brief      Reload WDT timer
 */
void wdt_reload(void);
}
