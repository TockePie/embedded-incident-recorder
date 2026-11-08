#ifndef METRICS_TEST_H
#define METRICS_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	METRICS_TEST_PASS = 0U,
	METRICS_TEST_FAIL_NOT_INITIALIZED = (1U << 0),
	METRICS_TEST_FAIL_CLOCK_NOT_MONOTONIC = (1U << 1),
	METRICS_TEST_FAIL_EVENT_QUEUE = (1U << 2),
	METRICS_TEST_FAIL_LOOP_STATS = (1U << 3),
	METRICS_TEST_FAIL_EXEC_STATS = (1U << 4),
	METRICS_TEST_FAIL_LOAD_STATS = (1U << 5),
	METRICS_TEST_FAIL_PROBE_INIT = (1U << 6)
} MetricsTest_Result_t;

/** Run a short smoke test in a dedicated test build before normal operation.
 *  It resets event, loop, and one execution-statistics entry.
 */
uint32_t MetricsTest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* METRICS_TEST_H */
