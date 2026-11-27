#include "metrics/rtos_metrics.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"

#ifndef configRUN_TIME_COUNTER_TYPE
#define configRUN_TIME_COUNTER_TYPE uint32_t
#endif

#define RTOS_METRICS_STACK_REGISTRY_SIZE 12U

typedef struct {
	char name[RTOS_METRICS_NAME_LEN];
	uint32_t words;
} RtosMetrics_StackEntry_t;

static RtosMetrics_StackEntry_t s_stack_registry[RTOS_METRICS_STACK_REGISTRY_SIZE];
static uint32_t s_stack_registry_count;

/* The hooks below run in PendSV/SysTick (lowest priority). Readers copy the
 * data with interrupts masked, so a reader always sees a consistent snapshot. */
static RtosMetrics_CycleStats_t s_tick_stats;
static RtosMetrics_CycleStats_t s_switch_stats;
static uint32_t s_switch_count;
static uint32_t s_switch_out_cycles;

static RtosMetrics_TraceEntry_t s_trace[METRICS_RTOS_TRACE_CAPACITY];
static volatile uint32_t s_trace_count;
static volatile uint32_t s_trace_dropped;
static volatile bool s_trace_active;

static void RtosMetrics_StatsAccumulate(RtosMetrics_CycleStats_t *stats,
		uint32_t cycles) {
	if ((stats->count == 0U) || (cycles < stats->min_cycles)) {
		stats->min_cycles = cycles;
	}
	if ((stats->count == 0U) || (cycles > stats->max_cycles)) {
		stats->max_cycles = cycles;
	}
	stats->total_cycles += (uint64_t) cycles;
	if (stats->count < UINT32_MAX) {
		stats->count++;
	}
}

void RtosMetrics_RegisterStack(const char *name, uint32_t words) {
	uint32_t i;

	if ((name == NULL)
			|| (s_stack_registry_count >= RTOS_METRICS_STACK_REGISTRY_SIZE)) {
		return;
	}

	for (i = 0U; i < s_stack_registry_count; ++i) {
		if (strncmp(s_stack_registry[i].name, name, RTOS_METRICS_NAME_LEN) == 0) {
			s_stack_registry[i].words = words;
			return;
		}
	}

	(void) strncpy(s_stack_registry[s_stack_registry_count].name, name,
			RTOS_METRICS_NAME_LEN - 1U);
	s_stack_registry[s_stack_registry_count].name[RTOS_METRICS_NAME_LEN - 1U] =
			'\0';
	s_stack_registry[s_stack_registry_count].words = words;
	s_stack_registry_count++;
}

static uint32_t RtosMetrics_StackSizeWords(const char *name) {
	uint32_t i;

	for (i = 0U; i < s_stack_registry_count; ++i) {
		if (strncmp(s_stack_registry[i].name, name, RTOS_METRICS_NAME_LEN) == 0) {
			return s_stack_registry[i].words;
		}
	}

	/* Tasks created by the kernel itself. */
	if (strncmp(name, "IDLE", RTOS_METRICS_NAME_LEN) == 0) {
		return (uint32_t) configMINIMAL_STACK_SIZE;
	}
#if (configUSE_TIMERS == 1)
	if (strncmp(name, "Tmr Svc", RTOS_METRICS_NAME_LEN) == 0) {
		return (uint32_t) configTIMER_TASK_STACK_DEPTH;
	}
#endif
	return 0U;
}

static char RtosMetrics_StateChar(eTaskState state) {
	switch (state) {
	case eRunning:
		return 'X';
	case eReady:
		return 'R';
	case eBlocked:
		return 'B';
	case eSuspended:
		return 'S';
	case eDeleted:
		return 'D';
	default:
		return '?';
	}
}

uint32_t RtosMetrics_TaskSnapshot(RtosMetrics_TaskInfo_t *out,
		uint32_t max_tasks, uint64_t *total_run_cycles) {
	TaskStatus_t status[METRICS_RTOS_MAX_TASKS];
	configRUN_TIME_COUNTER_TYPE total_run = 0;
	UBaseType_t found;
	uint32_t i;
	uint32_t count;

	if ((out == NULL) || (max_tasks == 0U)) {
		return 0U;
	}

	found = uxTaskGetSystemState(status, (UBaseType_t) METRICS_RTOS_MAX_TASKS,
			&total_run);
	count = ((uint32_t) found < max_tasks) ? (uint32_t) found : max_tasks;

	for (i = 0U; i < count; ++i) {
		(void) memset(&out[i], 0, sizeof(out[i]));
		(void) strncpy(out[i].name, status[i].pcTaskName,
				RTOS_METRICS_NAME_LEN - 1U);
		out[i].task_number = (uint32_t) status[i].xTaskNumber;
		out[i].priority = (uint32_t) status[i].uxBasePriority;
		out[i].state = RtosMetrics_StateChar(status[i].eCurrentState);
		out[i].run_cycles = (uint64_t) status[i].ulRunTimeCounter;
		out[i].stack_free_min_words = (uint32_t) status[i].usStackHighWaterMark;
		out[i].stack_size_words = RtosMetrics_StackSizeWords(out[i].name);
	}

	if (total_run_cycles != NULL) {
		*total_run_cycles = (uint64_t) total_run;
	}
	return count;
}

void RtosMetrics_HeapGet(RtosMetrics_Heap_t *heap) {
	if (heap == NULL) {
		return;
	}

	heap->total_bytes = (uint32_t) configTOTAL_HEAP_SIZE;
	heap->free_bytes = (uint32_t) xPortGetFreeHeapSize();
	heap->min_ever_free_bytes = (uint32_t) xPortGetMinimumEverFreeHeapSize();
	heap->used_peak_bytes = (heap->total_bytes >= heap->min_ever_free_bytes) ?
			(heap->total_bytes - heap->min_ever_free_bytes) : 0U;
}

void RtosMetrics_OnSwitchedOut(void) {
	s_switch_out_cycles = DWT->CYCCNT;
}

void RtosMetrics_OnSwitchedIn(uint32_t task_number) {
	const uint32_t now = DWT->CYCCNT;

	RtosMetrics_StatsAccumulate(&s_switch_stats,
			(uint32_t) (now - s_switch_out_cycles));
	if (s_switch_count < UINT32_MAX) {
		s_switch_count++;
	}

	if (s_trace_active) {
		if (s_trace_count < METRICS_RTOS_TRACE_CAPACITY) {
			s_trace[s_trace_count].cycles = now;
			s_trace[s_trace_count].task_number = task_number;
			s_trace_count++;
		} else {
			s_trace_dropped++;
			s_trace_active = false; /* buffer full: stop paying for the hook */
		}
	}
}

void RtosMetrics_TickRecord(uint32_t cycles) {
	RtosMetrics_StatsAccumulate(&s_tick_stats, cycles);
}

void RtosMetrics_TickStatsGet(RtosMetrics_CycleStats_t *stats) {
	uint32_t primask;

	if (stats == NULL) {
		return;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	*stats = s_tick_stats;
	__set_PRIMASK(primask);
}

void RtosMetrics_SwitchStatsGet(RtosMetrics_CycleStats_t *gap,
		uint32_t *switch_count) {
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	if (gap != NULL) {
		*gap = s_switch_stats;
	}
	if (switch_count != NULL) {
		*switch_count = s_switch_count;
	}
	__set_PRIMASK(primask);
}

void RtosMetrics_TraceStart(void) {
	const uint32_t primask = __get_PRIMASK();

	__disable_irq();
	s_trace_active = false;
	s_trace_count = 0U;
	s_trace_dropped = 0U;
	s_trace_active = true;
	__set_PRIMASK(primask);
}

void RtosMetrics_TraceStop(void) {
	s_trace_active = false;
}

bool RtosMetrics_TraceIsActive(void) {
	return s_trace_active;
}

uint32_t RtosMetrics_TraceCount(void) {
	return s_trace_count;
}

uint32_t RtosMetrics_TraceDropped(void) {
	return s_trace_dropped;
}

bool RtosMetrics_TraceGet(uint32_t index, RtosMetrics_TraceEntry_t *entry) {
	if ((entry == NULL) || (index >= s_trace_count)) {
		return false;
	}

	*entry = s_trace[index];
	return true;
}
