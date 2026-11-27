#include "metrics/metrics.h"
#include "metrics/metrics_config.h"

#include "stm32f4xx.h"
#include "main.h"

#include <stddef.h>
#include <string.h>

extern uint32_t SystemCoreClock;

static uint32_t s_last_cycle;
static uint64_t s_cycle_epoch;
static uint32_t s_core_clock_hz;
static bool s_initialized;
static bool s_probe_initialized;

static uint32_t s_loop_start_cycle;
static bool s_loop_running;
static Metrics_Stats_t s_loop_stats;
static uint64_t s_load_window_start_cycles;
static uint64_t s_load_window_busy_cycles;
static Metrics_LoadStats_t s_load_stats;

static Metrics_ExecStats_t s_exec_stats[METRICS_EXEC_ID_COUNT];
static uint32_t s_exec_start_cycle[METRICS_EXEC_ID_COUNT];
static bool s_exec_running[METRICS_EXEC_ID_COUNT];
static uint64_t s_exec_last_start[METRICS_EXEC_ID_COUNT];
static bool s_exec_has_last_start[METRICS_EXEC_ID_COUNT];

static Metrics_Event_t s_events[METRICS_EVENT_CAPACITY];
static uint32_t s_event_read;
static uint32_t s_event_write;
static uint32_t s_event_dropped;

static uint32_t s_dwt_anomaly_count;
static Metrics_ImuSampleStats_t s_imu_stats;
static uint32_t s_imu_last_sequence;
static uint64_t s_imu_last_cycles;
static bool s_imu_have_last;
static bool s_imu_first_seen;

static const uint16_t s_probe_pins[METRICS_PROBE_COUNT] = {
GPIO_PIN_4, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10 };
static GPIO_TypeDef *const s_probe_ports[METRICS_PROBE_COUNT] = {
GPIOA, GPIOB, GPIOB, GPIOB };

static const char *const s_event_names[] = { "invalid", "button_press",
		"state_change", "incident_count", "fault_entered", "scheduler_missed" };

static const char *const s_exec_names[METRICS_EXEC_ID_COUNT] = { "button",
		"state_manager", "led", "cli", "config", "journal", "health_monitor",
		"foreground_camera", "foreground_imu", "foreground_detector" };

/* Metrics are used from several RTOS tasks, from PendSV (run-time stats) and
 * from the CLI task. Shared state is therefore updated in very short critical
 * sections that only mask interrupts through PRIMASK (a few cycles). This
 * works from task, ISR and PendSV context and does not use the FreeRTOS API. */
static inline uint32_t Metrics_CriticalEnter(void) {
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	return primask;
}

static inline void Metrics_CriticalExit(uint32_t primask) {
	__set_PRIMASK(primask);
}

/* Must be called with interrupts masked. */
static void Metrics_UpdateCycleEpochLocked(void) {
	const uint32_t current = DWT->CYCCNT;

	/* Unsigned wrap detection is valid because Metrics_Poll() is called much
	 * more frequently than once per 2^32 cycles. A decrease from a small
	 * previous value cannot be a wrap: count it as an anomaly instead of
	 * adding a bogus 2^32 cycles to the timeline. */
	if (current < s_last_cycle) {
		if (s_last_cycle >= METRICS_DWT_WRAP_MIN_LAST) {
			s_cycle_epoch += (1ULL << 32);
		} else if (s_dwt_anomaly_count < UINT32_MAX) {
			s_dwt_anomaly_count++;
		}
	}

	s_last_cycle = current;
}

static void Metrics_UpdateCycleEpoch(void) {
	const uint32_t primask = Metrics_CriticalEnter();
	Metrics_UpdateCycleEpochLocked();
	Metrics_CriticalExit(primask);
}

/* Epoch and last-cycle value are combined inside one critical section, so a
 * preempting context cannot make the 64-bit value jump by 2^32. */
static uint64_t Metrics_ReadCycles64(void) {
	uint64_t value;
	const uint32_t primask = Metrics_CriticalEnter();
	Metrics_UpdateCycleEpochLocked();
	value = s_cycle_epoch + (uint64_t) s_last_cycle;
	Metrics_CriticalExit(primask);
	return value;
}

static bool Metrics_IsValidExecId(Metrics_ExecId_t id) {
	return ((uint32_t) id < (uint32_t) METRICS_EXEC_ID_COUNT);
}

static int64_t Metrics_SaturatingAddInt64(int64_t left, int64_t right) {
	if ((right > 0) && (left > (INT64_MAX - right))) {
		return INT64_MAX;
	}
	if ((right < 0) && (left < (INT64_MIN - right))) {
		return INT64_MIN;
	}
	return left + right;
}

static void Metrics_StatsAccumulate(Metrics_Stats_t *stats, uint32_t elapsed) {
	if ((stats->count == 0U) || (elapsed < stats->min_cycles)) {
		stats->min_cycles = elapsed;
	}
	if ((stats->count == 0U) || (elapsed > stats->max_cycles)) {
		stats->max_cycles = elapsed;
	}

	stats->total_cycles += (uint64_t) elapsed;
	if (stats->count < UINT32_MAX) {
		stats->count++;
	}
}

static uint64_t Metrics_SaturatingAddUint64(uint64_t left, uint64_t right) {
	if (right > (UINT64_MAX - left)) {
		return UINT64_MAX;
	}
	return left + right;
}

static uint16_t Metrics_RatioPermille(uint64_t numerator, uint64_t denominator) {
	uint64_t value;

	if (denominator == 0ULL) {
		return 0U;
	}

	if (numerator >= denominator) {
		return 1000U;
	}

	value = (numerator * 1000ULL) / denominator;
	return (uint16_t) value;
}

static void Metrics_UpdateLoadWindow(uint64_t now_cycles) {
	const uint64_t window_cycles = ((uint64_t) s_core_clock_hz
			* (uint64_t) METRICS_LOAD_WINDOW_MS) / 1000ULL;
	Metrics_LoadStats_t next;
	uint64_t elapsed_cycles;
	uint64_t busy_cycles;
	uint32_t primask;

	if (window_cycles == 0ULL) {
		return;
	}

	/* The busy counter is also updated by Metrics_ExecEnd() in other tasks:
	 * take a snapshot and restart the window atomically. */
	primask = Metrics_CriticalEnter();
	elapsed_cycles = now_cycles - s_load_window_start_cycles;
	if (elapsed_cycles < window_cycles) {
		Metrics_CriticalExit(primask);
		return;
	}
	busy_cycles = s_load_window_busy_cycles;
	/* One update per poll, even if the foreground was stalled for multiple
	 * windows. The measured interval remains explicit in elapsed_cycles. */
	s_load_window_start_cycles = now_cycles;
	s_load_window_busy_cycles = 0ULL;
	next = s_load_stats;
	Metrics_CriticalExit(primask);

	next.current_permille = Metrics_RatioPermille(busy_cycles, elapsed_cycles);
	if (next.current_permille > next.max_permille) {
		next.max_permille = next.current_permille;
	}

	next.total_busy_cycles = Metrics_SaturatingAddUint64(
			next.total_busy_cycles, busy_cycles);
	next.total_elapsed_cycles = Metrics_SaturatingAddUint64(
			next.total_elapsed_cycles, elapsed_cycles);
	if (next.total_elapsed_cycles != 0ULL) {
		next.average_permille = Metrics_RatioPermille(next.total_busy_cycles,
				next.total_elapsed_cycles);
	}
	if (next.window_count < UINT32_MAX) {
		next.window_count++;
	}

	primask = Metrics_CriticalEnter();
	s_load_stats = next;
	Metrics_CriticalExit(primask);
}

bool Metrics_ProbeInit(void) {
	GPIO_InitTypeDef gpio = { 0 };

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10,
			GPIO_PIN_RESET);

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	gpio.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10;
	HAL_GPIO_Init(GPIOB, &gpio);

	s_probe_initialized = true;
	return true;
}

bool Metrics_ProbeIsReady(void) {
	return s_probe_initialized;
}

void Metrics_ProbeSet(Metrics_ProbeId_t probe, bool active) {
	GPIO_TypeDef *port;
	uint32_t pin;

	if (!s_probe_initialized
			|| ((uint32_t) probe >= (uint32_t) METRICS_PROBE_COUNT)) {
		return;
	}

	port = s_probe_ports[(uint32_t) probe];
	pin = (uint32_t) s_probe_pins[(uint32_t) probe];
	port->BSRR = active ? pin : (pin << 16U);
}

bool Metrics_Init(void) {
	if (SystemCoreClock == 0U) {
		return false;
	}

	/* Do not clear CYCCNT here: the camera driver also uses DWT for delays
	 * and timing. Metrics_Init() is deliberately called after App_Init(). */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
		return false;
	}

	s_core_clock_hz = SystemCoreClock;
	s_last_cycle = DWT->CYCCNT;
	s_cycle_epoch = 0ULL;
	s_loop_start_cycle = 0U;
	s_loop_running = false;
	(void) memset(&s_loop_stats, 0, sizeof(s_loop_stats));
	s_load_window_start_cycles = s_cycle_epoch + (uint64_t) s_last_cycle;
	s_load_window_busy_cycles = 0ULL;
	(void) memset(&s_load_stats, 0, sizeof(s_load_stats));
	(void) memset(s_exec_stats, 0, sizeof(s_exec_stats));
	(void) memset(s_exec_start_cycle, 0, sizeof(s_exec_start_cycle));
	(void) memset(s_exec_running, 0, sizeof(s_exec_running));
	(void) memset(s_exec_last_start, 0, sizeof(s_exec_last_start));
	(void) memset(s_exec_has_last_start, 0, sizeof(s_exec_has_last_start));
	(void) memset(s_events, 0, sizeof(s_events));
	s_event_read = 0U;
	s_event_write = 0U;
	s_event_dropped = 0U;
	s_dwt_anomaly_count = 0U;
	(void) memset(&s_imu_stats, 0, sizeof(s_imu_stats));
	s_imu_last_sequence = 0U;
	s_imu_last_cycles = 0ULL;
	s_imu_have_last = false;
	s_imu_first_seen = false;
	s_initialized = true;

	return true;
}

bool Metrics_IsReady(void) {
	return s_initialized;
}

void Metrics_Poll(void) {
	if (s_initialized) {
		Metrics_UpdateLoadWindow(Metrics_ReadCycles64());
	}
}

uint64_t Metrics_NowCycles(void) {
	if (!s_initialized) {
		return 0ULL;
	}

	return Metrics_ReadCycles64();
}

uint64_t Metrics_CyclesToUs(uint64_t cycles) {
	uint64_t whole_seconds;
	uint64_t remaining_cycles;

	if (s_core_clock_hz == 0U) {
		return 0ULL;
	}

	whole_seconds = cycles / (uint64_t) s_core_clock_hz;
	remaining_cycles = cycles % (uint64_t) s_core_clock_hz;

	return (whole_seconds * 1000000ULL)
			+ ((remaining_cycles * 1000000ULL) / (uint64_t) s_core_clock_hz);
}

void Metrics_LoopBegin(void) {
	if (!s_initialized || s_loop_running) {
		return;
	}

	Metrics_ProbeSet(METRICS_PROBE_LOOP, true);
	Metrics_UpdateCycleEpoch();
	s_loop_start_cycle = DWT->CYCCNT;
	s_loop_running = true;
}

void Metrics_LoopEnd(void) {
	uint32_t elapsed;

	if (!s_initialized || !s_loop_running) {
		return;
	}

	elapsed = (uint32_t) (DWT->CYCCNT - s_loop_start_cycle);
	s_loop_running = false;
	Metrics_StatsAccumulate(&s_loop_stats, elapsed);
	Metrics_ProbeSet(METRICS_PROBE_LOOP, false);

	/* Extend the timestamp after each iteration, preventing loss of a wrap. */
	Metrics_UpdateCycleEpoch();
}

bool Metrics_LoadStatsGet(Metrics_LoadStats_t *stats) {
	if ((stats == NULL) || !s_initialized) {
		return false;
	}

	{
		const uint32_t primask = Metrics_CriticalEnter();
		*stats = s_load_stats;
		Metrics_CriticalExit(primask);
	}
	return true;
}

void Metrics_LoopStatsReset(void) {
	if (s_initialized) {
		(void) memset(&s_loop_stats, 0, sizeof(s_loop_stats));
	}
}

bool Metrics_LoopStatsGet(Metrics_Stats_t *stats) {
	if ((stats == NULL) || !s_initialized) {
		return false;
	}

	{
		const uint32_t primask = Metrics_CriticalEnter();
		*stats = s_loop_stats;
		Metrics_CriticalExit(primask);
	}
	return true;
}

void Metrics_ExecBegin(Metrics_ExecId_t id, uint32_t period_ms) {
	uint32_t index = (uint32_t) id;
	uint64_t start_timestamp;
	uint64_t interval_cycles;
	uint64_t interval_us;
	int64_t jitter_us;
	uint32_t missed_periods = 0U;
	uint32_t primask;
	Metrics_ExecStats_t *stats;

	if (!s_initialized || !Metrics_IsValidExecId(id) || s_exec_running[index]) {
		return;
	}

	start_timestamp = Metrics_NowCycles();
	stats = &s_exec_stats[index];

	if ((period_ms != 0U) && s_exec_has_last_start[index]) {
		interval_cycles = start_timestamp - s_exec_last_start[index];
		interval_us = Metrics_CyclesToUs(interval_cycles);
		jitter_us = (int64_t) interval_us - ((int64_t) period_ms * 1000LL);

		primask = Metrics_CriticalEnter();
		if ((stats->jitter_count == 0U) || (jitter_us < stats->min_jitter_us)) {
			stats->min_jitter_us = jitter_us;
		}
		if ((stats->jitter_count == 0U) || (jitter_us > stats->max_jitter_us)) {
			stats->max_jitter_us = jitter_us;
		}
		stats->total_jitter_us = Metrics_SaturatingAddInt64(
				stats->total_jitter_us, jitter_us);
		if (stats->jitter_count < UINT32_MAX) {
			stats->jitter_count++;
		}
		Metrics_CriticalExit(primask);

#if (METRICS_MISSED_FROM_EXEC_BEGIN != 0U)
		/* Same definition as AppScheduler_Run() in lab 3: the interval is
		 * truncated to whole milliseconds, and only an interval longer than
		 * one period counts: missed = elapsed_ms / period_ms - 1. */
		{
			const uint32_t elapsed_ms = (uint32_t) (interval_us / 1000ULL);
			if (elapsed_ms > period_ms) {
				missed_periods = (elapsed_ms / period_ms) - 1U;
			}
		}
#endif
	}

	if (period_ms != 0U) {
		s_exec_last_start[index] = start_timestamp;
		s_exec_has_last_start[index] = true;
	}

	/* Done before the start stamp so that it is not part of the measured
	 * region (the lab 3 scheduler did the same before Metrics_ExecBegin). */
	if (missed_periods != 0U) {
		Metrics_ExecMissedPeriods(id, missed_periods);
		(void) Metrics_EventPush(METRICS_EVENT_SCHEDULER_MISSED_PERIODS,
				((index & 0xFFU) << 24U) | (missed_periods & 0x00FFFFFFUL));
	}

	s_exec_start_cycle[index] = DWT->CYCCNT;
	s_exec_running[index] = true;
}

void Metrics_ExecEnd(Metrics_ExecId_t id) {
	uint32_t index = (uint32_t) id;
	uint32_t elapsed;
	uint32_t primask;
	Metrics_ExecStats_t *stats;

	if (!s_initialized || !Metrics_IsValidExecId(id)
			|| !s_exec_running[index]) {
		return;
	}

	elapsed = (uint32_t) (DWT->CYCCNT - s_exec_start_cycle[index]);
	s_exec_running[index] = false;
	stats = &s_exec_stats[index];

	primask = Metrics_CriticalEnter();
	/* CPU-load numerator counts only instrumented work regions, not the
	 * scheduler's periodic eligibility checks or super-loop bookkeeping. */
	s_load_window_busy_cycles = Metrics_SaturatingAddUint64(
			s_load_window_busy_cycles, (uint64_t) elapsed);
	if ((stats->count == 0U) || (elapsed < stats->min_cycles)) {
		stats->min_cycles = elapsed;
	}
	if ((stats->count == 0U) || (elapsed > stats->max_cycles)) {
		stats->max_cycles = elapsed;
	}
	stats->total_cycles += (uint64_t) elapsed;
	if (stats->count < UINT32_MAX) {
		stats->count++;
	}
	Metrics_CriticalExit(primask);
}

void Metrics_ExecMissedPeriods(Metrics_ExecId_t id, uint32_t missed_periods) {
	uint32_t index = (uint32_t) id;
	uint32_t remaining;
	uint32_t primask;

	if (!s_initialized || !Metrics_IsValidExecId(id)
			|| (missed_periods == 0U)) {
		return;
	}

	primask = Metrics_CriticalEnter();
	remaining = UINT32_MAX - s_exec_stats[index].missed_periods;
	if (missed_periods > remaining) {
		s_exec_stats[index].missed_periods = UINT32_MAX;
	} else {
		s_exec_stats[index].missed_periods += missed_periods;
	}
	Metrics_CriticalExit(primask);
}

void Metrics_ExecStatsReset(Metrics_ExecId_t id) {
	uint32_t index = (uint32_t) id;

	uint32_t primask;

	if (!s_initialized || !Metrics_IsValidExecId(id)) {
		return;
	}

	primask = Metrics_CriticalEnter();
	(void) memset(&s_exec_stats[index], 0, sizeof(s_exec_stats[index]));
	s_exec_running[index] = false;
	s_exec_last_start[index] = 0ULL;
	s_exec_has_last_start[index] = false;
	Metrics_CriticalExit(primask);
}

bool Metrics_ExecStatsGet(Metrics_ExecId_t id, Metrics_ExecStats_t *stats) {
	uint32_t index = (uint32_t) id;

	if (!s_initialized || !Metrics_IsValidExecId(id) || (stats == NULL)) {
		return false;
	}

	{
		const uint32_t primask = Metrics_CriticalEnter();
		*stats = s_exec_stats[index];
		Metrics_CriticalExit(primask);
	}
	return true;
}

const char* Metrics_ExecIdName(Metrics_ExecId_t id) {
	if (!Metrics_IsValidExecId(id)) {
		return "invalid";
	}

	return s_exec_names[(uint32_t) id];
}

bool Metrics_EventPush(uint16_t id, uint32_t argument) {
	uint32_t slot;
	uint32_t primask;
	uint64_t timestamp;
	Metrics_Event_t *event;

	if (!s_initialized) {
		return false;
	}

	timestamp = Metrics_NowCycles();

	primask = Metrics_CriticalEnter();
	if ((uint32_t) (s_event_write - s_event_read) >= METRICS_EVENT_CAPACITY) {
		s_event_read++;
		if (s_event_dropped < UINT32_MAX) {
			s_event_dropped++;
		}
	}

	slot = s_event_write % METRICS_EVENT_CAPACITY;
	event = &s_events[slot];
	event->id = id;
	event->reserved = 0U;
	event->argument = argument;
	event->timestamp_cycles = timestamp;
	s_event_write++;
	Metrics_CriticalExit(primask);

	return true;
}

bool Metrics_EventPop(Metrics_Event_t *event) {
	uint32_t slot;
	uint32_t primask;

	if ((event == NULL) || !s_initialized) {
		return false;
	}

	primask = Metrics_CriticalEnter();
	if (s_event_read == s_event_write) {
		Metrics_CriticalExit(primask);
		return false;
	}

	slot = s_event_read % METRICS_EVENT_CAPACITY;
	*event = s_events[slot];
	s_event_read++;
	Metrics_CriticalExit(primask);

	return true;
}

void Metrics_EventClear(void) {
	if (s_initialized) {
		const uint32_t primask = Metrics_CriticalEnter();
		s_event_read = s_event_write;
		s_event_dropped = 0U;
		Metrics_CriticalExit(primask);
	}
}

void Metrics_EventStatusGet(Metrics_EventStatus_t *status) {
	uint32_t primask;

	if (status == NULL) {
		return;
	}

	if (!s_initialized) {
		status->count = 0U;
		status->dropped = 0U;
		return;
	}

	primask = Metrics_CriticalEnter();
	status->count = (uint32_t) (s_event_write - s_event_read);
	status->dropped = s_event_dropped;
	Metrics_CriticalExit(primask);
}

const char* Metrics_EventIdName(uint16_t id) {
	if ((uint32_t) id
			>= (uint32_t) (sizeof(s_event_names) / sizeof(s_event_names[0]))) {
		return "unknown";
	}
	return s_event_names[id];
}

uint32_t Metrics_DwtAnomalyCountGet(void) {
	return s_dwt_anomaly_count;
}

void Metrics_ImuSampleObserve(uint32_t sequence, uint32_t sample_tick_ms,
		uint32_t period_ms) {
	uint64_t now_cycles;
	uint64_t interval_us = 0ULL;
	uint32_t elapsed_ms = 0U;
	uint32_t primask;
	const bool measure = s_imu_have_last && (period_ms != 0U);

	/* sequence == 0: no valid sample yet. Same sequence: no new sample. */
	if (!s_initialized || (sequence == 0U)
			|| (s_imu_have_last && (sequence == s_imu_last_sequence))) {
		return;
	}

	now_cycles = Metrics_NowCycles();

	/* 64-bit divisions stay outside the critical section. This function is
	 * the only writer of s_imu_last_cycles and runs in a single task. */
	if (measure) {
		interval_us = Metrics_CyclesToUs(now_cycles - s_imu_last_cycles);
		elapsed_ms = (uint32_t) (interval_us / 1000ULL);
	}

	primask = Metrics_CriticalEnter();
	if (!s_imu_first_seen) {
		s_imu_stats.first_sample_tick_ms = sample_tick_ms;
		s_imu_first_seen = true;
	}
	s_imu_stats.period_ms = period_ms;
	if (measure) {
		if ((s_imu_stats.count == 0U)
				|| (interval_us < s_imu_stats.min_interval_us)) {
			s_imu_stats.min_interval_us = interval_us;
		}
		if ((s_imu_stats.count == 0U)
				|| (interval_us > s_imu_stats.max_interval_us)) {
			s_imu_stats.max_interval_us = interval_us;
		}
		s_imu_stats.total_interval_us += interval_us;
		if (s_imu_stats.count < UINT32_MAX) {
			s_imu_stats.count++;
		}
		if (elapsed_ms > period_ms) {
			const uint32_t missed = (elapsed_ms / period_ms) - 1U;
			const uint32_t room = UINT32_MAX - s_imu_stats.missed;
			s_imu_stats.missed += (missed > room) ? room : missed;
		}
	}
	s_imu_last_sequence = sequence;
	s_imu_last_cycles = now_cycles;
	s_imu_have_last = true;
	Metrics_CriticalExit(primask);
}

bool Metrics_ImuSampleStatsGet(Metrics_ImuSampleStats_t *stats) {
	uint32_t primask;

	if ((stats == NULL) || !s_initialized) {
		return false;
	}

	primask = Metrics_CriticalEnter();
	*stats = s_imu_stats;
	Metrics_CriticalExit(primask);
	return true;
}

void Metrics_ImuSampleStatsReset(void) {
	uint32_t primask;

	if (!s_initialized) {
		return;
	}

	primask = Metrics_CriticalEnter();
	{
		/* The time of the very first sample (boot-time metric) survives. */
		const uint32_t first_tick = s_imu_stats.first_sample_tick_ms;
		(void) memset(&s_imu_stats, 0, sizeof(s_imu_stats));
		s_imu_stats.first_sample_tick_ms = first_tick;
	}
	s_imu_have_last = false;
	s_imu_last_sequence = 0U;
	Metrics_CriticalExit(primask);
}
