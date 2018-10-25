#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" void assert_failed(const char *file, uint32_t line);
#else
extern void assert_failed(const char *file, uint32_t line);
#endif

#define ASSERT(expr) ((expr) ? (void)0 : assert_failed(__FILE__, __LINE__))
