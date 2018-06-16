#pragma once

#include <stdbool.h>
#include <stdint.h>

namespace hal
{
typedef enum
{
	RCC_SRC_SYSCLK,
	RCC_SRC_AHB,
	RCC_SRC_APB1,
	RCC_SRC_APB2
} rcc_src_t;

typedef enum
{
	RCC_RST_REASON_LOW_POWER,
	RCC_RST_REASON_EXTERNAL,
	RCC_RST_REASON_INTERNAL,
	RCC_RST_REASON_WDT,
	RCC_RST_REASON_UNKNOWN
} rcc_rst_reason_t;

/**
 * @brief      Specific RCC initialization
 *
 * @return     True in case of success, otherwise - false
 */
bool rcc_init(void);

/**
 * @brief      Get frequency of specific clock source
 *
 * @param[in]  src   Source of clock
 *
 * @return     Frequency of specific clock source in Hz
 */
uint32_t rcc_get_freq(rcc_src_t src);

/**
 * @brief      Reset MCU
 */
void rcc_reset(void);

/**
 * @brief      Get reason of previous reset
 *
 * @return     Reset reason
 */
rcc_rst_reason_t rcc_get_rst_reason(void);
}
