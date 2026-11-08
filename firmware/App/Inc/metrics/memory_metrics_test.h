#ifndef MEMORY_METRICS_TEST_H
#define MEMORY_METRICS_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MEMORY_METRICS_TEST_LAYOUT_OK   (1UL << 0)
#define MEMORY_METRICS_TEST_STACK_OK    (1UL << 1)
#define MEMORY_METRICS_TEST_HEAP_OK     (1UL << 2)

uint32_t MemoryMetricsTest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* MEMORY_METRICS_TEST_H */
