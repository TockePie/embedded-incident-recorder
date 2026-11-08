#include "metrics/metrics_test.h"
#include "metrics/metrics.h"
#include "metrics/metrics_config.h"

#include <stddef.h>

uint32_t MetricsTest_Run(void) {
	uint32_t result = (uint32_t) METRICS_TEST_PASS;
	uint64_t time_before;
	uint64_t time_after;
	Metrics_Event_t event;
	Metrics_EventStatus_t event_status;
	Metrics_Stats_t loop_stats;
	Metrics_ExecStats_t exec_stats;
	Metrics_LoadStats_t load_stats;
	volatile uint32_t workload;
	uint32_t i;

	if (!Metrics_IsReady()) {
		result |= (uint32_t) METRICS_TEST_FAIL_NOT_INITIALIZED;
		return result;
	}
	if (!Metrics_ProbeIsReady()) {
		result |= (uint32_t) METRICS_TEST_FAIL_PROBE_INIT;
	}
	if (!Metrics_LoadStatsGet(&load_stats)
			|| (load_stats.current_permille > 1000U)
			|| (load_stats.average_permille > 1000U)
			|| (load_stats.max_permille > 1000U)) {
		result |= (uint32_t) METRICS_TEST_FAIL_LOAD_STATS;
	}

	time_before = Metrics_NowCycles();
	for (i = 0U; i < 1000U; i++) {
		workload = i;
		(void) workload;
	}
	time_after = Metrics_NowCycles();
	if (time_after < time_before) {
		result |= (uint32_t) METRICS_TEST_FAIL_CLOCK_NOT_MONOTONIC;
	}

	Metrics_EventClear();
	if (!Metrics_EventPush(METRICS_TEST_EVENT_ID, METRICS_TEST_EVENT_ARG)
			|| !Metrics_EventPop(&event) || (event.id != METRICS_TEST_EVENT_ID)
			|| (event.argument != METRICS_TEST_EVENT_ARG)) {
		result |= (uint32_t) METRICS_TEST_FAIL_EVENT_QUEUE;
	}

	Metrics_EventClear();
	for (i = 0U; i < (METRICS_EVENT_CAPACITY + 1U); ++i) {
		if (!Metrics_EventPush(METRICS_TEST_EVENT_ID, i)) {
			result |= (uint32_t) METRICS_TEST_FAIL_EVENT_QUEUE;
			break;
		}
	}
	Metrics_EventStatusGet(&event_status);
	if ((event_status.count != METRICS_EVENT_CAPACITY)
			|| (event_status.dropped != 1U)) {
		result |= (uint32_t) METRICS_TEST_FAIL_EVENT_QUEUE;
	}

	Metrics_LoopStatsReset();
	Metrics_LoopBegin();
	for (i = 0U; i < 1000U; i++) {
		workload = i * 3U;
		(void) workload;
	}
	Metrics_LoopEnd();
	if (!Metrics_LoopStatsGet(&loop_stats) || (loop_stats.count != 1U)
			|| (loop_stats.max_cycles < loop_stats.min_cycles)) {
		result |= (uint32_t) METRICS_TEST_FAIL_LOOP_STATS;
	}

	Metrics_ExecStatsReset(METRICS_EXEC_FOREGROUND_CAMERA);
	Metrics_ExecBegin(METRICS_EXEC_FOREGROUND_CAMERA, 1U);
	for (i = 0U; i < 1000U; i++) {
		workload = i + 7U;
		(void) workload;
	}
	Metrics_ExecEnd(METRICS_EXEC_FOREGROUND_CAMERA);
	Metrics_ExecBegin(METRICS_EXEC_FOREGROUND_CAMERA, 1U);
	Metrics_ExecEnd(METRICS_EXEC_FOREGROUND_CAMERA);
	Metrics_ExecMissedPeriods(METRICS_EXEC_FOREGROUND_CAMERA, 2U);

	if (!Metrics_ExecStatsGet(METRICS_EXEC_FOREGROUND_CAMERA, &exec_stats)
			|| (exec_stats.count != 2U)
			|| (exec_stats.max_cycles < exec_stats.min_cycles)
			|| (exec_stats.jitter_count != 1U)
			|| (exec_stats.missed_periods != 2U)) {
		result |= (uint32_t) METRICS_TEST_FAIL_EXEC_STATS;
	}

	Metrics_LoopStatsReset();
	Metrics_ExecStatsReset(METRICS_EXEC_FOREGROUND_CAMERA);
	Metrics_EventClear();
	return result;
}
