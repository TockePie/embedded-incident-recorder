#ifndef RTOS_METRICS_H
#define RTOS_METRICS_H

/* Metrics specific to the FreeRTOS build (lab 4): CPU time per task, stack
 * high-water marks, OS heap, tick cost, context-switch cost and a task
 * timeline. Everything here is read-only diagnostics; the lab 3 metrics
 * (metrics.h / memory_metrics.h) keep their behaviour and output format. */

#include <stdbool.h>
#include <stdint.h>

#include "metrics/metrics_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RTOS_METRICS_NAME_LEN 16U

typedef struct {
	uint32_t count;
	uint32_t min_cycles;
	uint32_t max_cycles;
	uint64_t total_cycles;
} RtosMetrics_CycleStats_t;

typedef struct {
	char name[RTOS_METRICS_NAME_LEN];
	uint32_t task_number;        /* kernel task number (matches the trace) */
	uint32_t priority;           /* base priority, 0 = idle */
	char state;                  /* X running, R ready, B blocked, S suspended, D deleted */
	uint64_t run_cycles;         /* CPU time since boot, DWT cycles */
	uint32_t stack_free_min_words; /* high-water mark: minimum free stack, words */
	uint32_t stack_size_words;   /* configured stack size, words; 0 = unknown */
} RtosMetrics_TaskInfo_t;

typedef struct {
	uint32_t total_bytes;        /* configTOTAL_HEAP_SIZE */
	uint32_t free_bytes;         /* free now */
	uint32_t min_ever_free_bytes;
	uint32_t used_peak_bytes;    /* total - min_ever_free */
} RtosMetrics_Heap_t;

typedef struct {
	uint32_t cycles;             /* low 32 bits of DWT->CYCCNT at switch-in */
	uint32_t task_number;
} RtosMetrics_TraceEntry_t;

/** Remember the configured stack size (words) of a task by name, so that the
 *  report can show used = size - high-water mark. Call right before or after
 *  xTaskCreate(). IDLE and the timer task are known automatically. */
void RtosMetrics_RegisterStack(const char *name, uint32_t words);

/** Snapshot of all tasks (name, priority, state, CPU time, stack). Wraps
 *  uxTaskGetSystemState(), which suspends the scheduler while it walks the
 *  task lists: do not call it while timings are being measured. Returns the
 *  number of tasks stored. total_run_cycles may be NULL. */
uint32_t RtosMetrics_TaskSnapshot(RtosMetrics_TaskInfo_t *out,
		uint32_t max_tasks, uint64_t *total_run_cycles);

void RtosMetrics_HeapGet(RtosMetrics_Heap_t *heap);

/* ---- called from FreeRTOSConfig.h trace macros and from SysTick wrapper ---- */
void RtosMetrics_OnSwitchedOut(void);
void RtosMetrics_OnSwitchedIn(uint32_t task_number);
void RtosMetrics_TickRecord(uint32_t cycles);

/** Duration of the SysTick handler (xPortSysTickHandler only, without the
 *  hardware exception entry/exit). Needs METRICS_RTOS_TICK_PROBE. */
void RtosMetrics_TickStatsGet(RtosMetrics_CycleStats_t *stats);

/** Time between traceTASK_SWITCHED_OUT and traceTASK_SWITCHED_IN, i.e. the
 *  scheduler part of a context switch (vTaskSwitchContext); the register
 *  save/restore in PendSV is not included. Needs METRICS_RTOS_STATS_ENABLE. */
void RtosMetrics_SwitchStatsGet(RtosMetrics_CycleStats_t *gap,
		uint32_t *switch_count);

/* ---- task timeline: records the first METRICS_RTOS_TRACE_CAPACITY switches ---- */
void RtosMetrics_TraceStart(void);
void RtosMetrics_TraceStop(void);
bool RtosMetrics_TraceIsActive(void);
uint32_t RtosMetrics_TraceCount(void);
uint32_t RtosMetrics_TraceDropped(void);
bool RtosMetrics_TraceGet(uint32_t index, RtosMetrics_TraceEntry_t *entry);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_METRICS_H */
