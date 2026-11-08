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

static const uint16_t s_probe_pins[METRICS_PROBE_COUNT] = {
GPIO_PIN_4, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10 };
static GPIO_TypeDef *const s_probe_ports[METRICS_PROBE_COUNT] = {
GPIOA, GPIOB, GPIOB, GPIOB };

static const char *const s_event_names[] = { "invalid", "button_press",
		"state_change", "incident_count", "fault_entered", "scheduler_missed" };

static const char *const s_exec_names[METRICS_EXEC_ID_COUNT] = { "button",
		"state_manager", "led", "cli", "config", "journal", "health_monitor",
		"foreground_camera", "foreground_imu", "foreground_detector" };

static void Metrics_UpdateCycleEpoch(void) {
	const uint32_t current = DWT->CYCCNT;

	/* Unsigned wrap detection is valid because Metrics_Poll() is called much
	 * more frequently than once per 2^32 cycles. */
	if (current < s_last_cycle) {
		s_cycle_epoch += (1ULL << 32);
	}

	s_last_cycle = current;
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
	const uint64_t elapsed_cycles = now_cycles - s_load_window_start_cycles;
	const uint64_t window_cycles = ((uint64_t) s_core_clock_hz
			* (uint64_t) METRICS_LOAD_WINDOW_MS) / 1000ULL;

	if ((window_cycles == 0ULL) || (elapsed_cycles < window_cycles)) {
		return;
	}

	s_load_stats.current_permille = Metrics_RatioPermille(
			s_load_window_busy_cycles, elapsed_cycles);
	if (s_load_stats.current_permille > s_load_stats.max_permille) {
		s_load_stats.max_permille = s_load_stats.current_permille;
	}

	s_load_stats.total_busy_cycles = Metrics_SaturatingAddUint64(
			s_load_stats.total_busy_cycles, s_load_window_busy_cycles);
	s_load_stats.total_elapsed_cycles = Metrics_SaturatingAddUint64(
			s_load_stats.total_elapsed_cycles, elapsed_cycles);
	if (s_load_stats.total_elapsed_cycles != 0ULL) {
		s_load_stats.average_permille = Metrics_RatioPermille(
				s_load_stats.total_busy_cycles,
				s_load_stats.total_elapsed_cycles);
	}
	if (s_load_stats.window_count < UINT32_MAX) {
		s_load_stats.window_count++;
	}

	/* One update per poll, even if the foreground was stalled for multiple
	 * windows. The measured interval remains explicit in elapsed_cycles. */
	s_load_window_start_cycles = now_cycles;
	s_load_window_busy_cycles = 0ULL;
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
	s_initialized = true;

	return true;
}

bool Metrics_IsReady(void) {
	return s_initialized;
}

void Metrics_Poll(void) {
	if (s_initialized) {
		uint64_t now_cycles;
		Metrics_UpdateCycleEpoch();
		now_cycles = s_cycle_epoch + (uint64_t) s_last_cycle;
		Metrics_UpdateLoadWindow(now_cycles);
	}
}

uint64_t Metrics_NowCycles(void) {
	if (!s_initialized) {
		return 0ULL;
	}

	Metrics_UpdateCycleEpoch();
	return s_cycle_epoch + (uint64_t) s_last_cycle;
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

	*stats = s_load_stats;
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

	*stats = s_loop_stats;
	return true;
}

void Metrics_ExecBegin(Metrics_ExecId_t id, uint32_t period_ms) {
	uint32_t index = (uint32_t) id;
	uint64_t start_timestamp;
	uint64_t interval_cycles;
	uint64_t interval_us;
	int64_t jitter_us;
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
	}

	if (period_ms != 0U) {
		s_exec_last_start[index] = start_timestamp;
		s_exec_has_last_start[index] = true;
	}

	s_exec_start_cycle[index] = DWT->CYCCNT;
	s_exec_running[index] = true;
}

void Metrics_ExecEnd(Metrics_ExecId_t id) {
	uint32_t index = (uint32_t) id;
	uint32_t elapsed;

	if (!s_initialized || !Metrics_IsValidExecId(id)
			|| !s_exec_running[index]) {
		return;
	}

	elapsed = (uint32_t) (DWT->CYCCNT - s_exec_start_cycle[index]);
	s_exec_running[index] = false;
	/* CPU-load numerator counts only instrumented work regions, not the
	 * scheduler's periodic eligibility checks or super-loop bookkeeping. */
	s_load_window_busy_cycles = Metrics_SaturatingAddUint64(
			s_load_window_busy_cycles, (uint64_t) elapsed);
	if ((s_exec_stats[index].count == 0U)
			|| (elapsed < s_exec_stats[index].min_cycles)) {
		s_exec_stats[index].min_cycles = elapsed;
	}
	if ((s_exec_stats[index].count == 0U)
			|| (elapsed > s_exec_stats[index].max_cycles)) {
		s_exec_stats[index].max_cycles = elapsed;
	}
	s_exec_stats[index].total_cycles += (uint64_t) elapsed;
	if (s_exec_stats[index].count < UINT32_MAX) {
		s_exec_stats[index].count++;
	}
}

void Metrics_ExecMissedPeriods(Metrics_ExecId_t id, uint32_t missed_periods) {
	uint32_t index = (uint32_t) id;
	uint32_t remaining;

	if (!s_initialized || !Metrics_IsValidExecId(id)
			|| (missed_periods == 0U)) {
		return;
	}

	remaining = UINT32_MAX - s_exec_stats[index].missed_periods;
	if (missed_periods > remaining) {
		s_exec_stats[index].missed_periods = UINT32_MAX;
	} else {
		s_exec_stats[index].missed_periods += missed_periods;
	}
}

void Metrics_ExecStatsReset(Metrics_ExecId_t id) {
	uint32_t index = (uint32_t) id;

	if (!s_initialized || !Metrics_IsValidExecId(id)) {
		return;
	}

	(void) memset(&s_exec_stats[index], 0, sizeof(s_exec_stats[index]));
	s_exec_running[index] = false;
	s_exec_last_start[index] = 0ULL;
	s_exec_has_last_start[index] = false;
}

bool Metrics_ExecStatsGet(Metrics_ExecId_t id, Metrics_ExecStats_t *stats) {
	uint32_t index = (uint32_t) id;

	if (!s_initialized || !Metrics_IsValidExecId(id) || (stats == NULL)) {
		return false;
	}

	*stats = s_exec_stats[index];
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
	Metrics_Event_t *event;

	if (!s_initialized) {
		return false;
	}

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
	event->timestamp_cycles = Metrics_NowCycles();
	s_event_write++;

	return true;
}

bool Metrics_EventPop(Metrics_Event_t *event) {
	uint32_t slot;

	if ((event == NULL) || !s_initialized || (s_event_read == s_event_write)) {
		return false;
	}

	slot = s_event_read % METRICS_EVENT_CAPACITY;
	*event = s_events[slot];
	s_event_read++;

	return true;
}

void Metrics_EventClear(void) {
	if (s_initialized) {
		s_event_read = s_event_write;
		s_event_dropped = 0U;
	}
}

void Metrics_EventStatusGet(Metrics_EventStatus_t *status) {
	if (status == NULL) {
		return;
	}

	if (!s_initialized) {
		status->count = 0U;
		status->dropped = 0U;
		return;
	}

	status->count = (uint32_t) (s_event_write - s_event_read);
	status->dropped = s_event_dropped;
}

const char* Metrics_EventIdName(uint16_t id) {
	if ((uint32_t) id
			>= (uint32_t) (sizeof(s_event_names) / sizeof(s_event_names[0]))) {
		return "unknown";
	}
	return s_event_names[id];
}
