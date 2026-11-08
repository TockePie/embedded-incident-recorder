#ifndef METRICS_H
#define METRICS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Stable IDs for the seven cooperative scheduler tasks and the three
 * foreground operations executed on every App_RunOnce() pass. */
typedef enum {
	METRICS_EXEC_TASK_BUTTON = 0U,
	METRICS_EXEC_TASK_STATE_MANAGER,
	METRICS_EXEC_TASK_LED,
	METRICS_EXEC_TASK_CLI,
	METRICS_EXEC_TASK_CONFIG,
	METRICS_EXEC_TASK_JOURNAL,
	METRICS_EXEC_TASK_HEALTH_MONITOR,
	METRICS_EXEC_FOREGROUND_CAMERA,
	METRICS_EXEC_FOREGROUND_IMU,
	METRICS_EXEC_FOREGROUND_DETECTOR,
	METRICS_EXEC_ID_COUNT
} Metrics_ExecId_t;

typedef struct {
	uint32_t count;
	uint32_t min_cycles;
	uint32_t max_cycles;
	uint64_t total_cycles;
} Metrics_Stats_t;

typedef struct {
	uint32_t count;
	uint32_t min_cycles;
	uint32_t max_cycles;
	uint64_t total_cycles;
	int64_t min_jitter_us;
	int64_t max_jitter_us;
	int64_t total_jitter_us;
	uint32_t jitter_count;
	uint32_t missed_periods;
} Metrics_ExecStats_t;

typedef struct {
	uint16_t id;
	uint16_t reserved;
	uint32_t argument;
	uint64_t timestamp_cycles;
} Metrics_Event_t;

typedef struct {
	uint32_t count;
	uint32_t dropped;
} Metrics_EventStatus_t;

typedef struct {
	uint16_t current_permille;
	uint16_t average_permille;
	uint16_t max_permille;
	uint16_t reserved;
	uint32_t window_count;
	uint64_t total_busy_cycles;
	uint64_t total_elapsed_cycles;
} Metrics_LoadStats_t;

typedef enum {
	METRICS_PROBE_LOOP = 0U, /* PA4: complete App_RunOnce() */
	METRICS_PROBE_IMU, /* PB0: ImuService_RunOnce() */
	METRICS_PROBE_CAMERA, /* PB1: OV7670_DvpCapture_Process() */
	METRICS_PROBE_BUTTON, /* PB10: debounced button response */
	METRICS_PROBE_COUNT
} Metrics_ProbeId_t;

typedef enum {
	METRICS_EVENT_BUTTON_PRESS = 1U,
	METRICS_EVENT_STATE_CHANGE,
	METRICS_EVENT_INCIDENT_COUNT_CHANGED,
	METRICS_EVENT_FAULT_ENTERED,
	METRICS_EVENT_SCHEDULER_MISSED_PERIODS
} Metrics_EventId_t;

/** Enable DWT CYCCNT and initialize the metrics core. Call after all
 *  initialization code that may reset DWT->CYCCNT (including camera init).
 */
bool Metrics_Init(void);
bool Metrics_IsReady(void);

/** Update the 64-bit cycle-time extension. Must be called at least once per
 *  32-bit CYCCNT wrap (about 42.95 s at 100 MHz). Foreground context only.
 */
void Metrics_Poll(void);
uint64_t Metrics_NowCycles(void);
uint64_t Metrics_CyclesToUs(uint64_t cycles);

/** Configure the four reserved measurement pins. Call after MX_GPIO_Init(). */
bool Metrics_ProbeInit(void);
bool Metrics_ProbeIsReady(void);
void Metrics_ProbeSet(Metrics_ProbeId_t probe, bool active);

/** Foreground/main-loop occupancy estimate, updated in one-second windows.
 *  It is a measured-work estimate: the numerator sums the instrumented
 *  foreground regions and scheduler callbacks. It excludes scheduler scans,
 *  super-loop bookkeeping and uninstrumented ISR time. ISR preemption occurring
 *  inside a measured region is included in that region's wall-time duration.
 */
bool Metrics_LoadStatsGet(Metrics_LoadStats_t *stats);

/** Whole-App_RunOnce execution-time measurement. */
void Metrics_LoopBegin(void);
void Metrics_LoopEnd(void);
void Metrics_LoopStatsReset(void);
bool Metrics_LoopStatsGet(Metrics_Stats_t *stats);

/** Measure one non-nested foreground operation or scheduler callback.
 *  period_ms == 0 disables periodic-jitter calculation (foreground regions).
 */
void Metrics_ExecBegin(Metrics_ExecId_t id, uint32_t period_ms);
void Metrics_ExecEnd(Metrics_ExecId_t id);
void Metrics_ExecMissedPeriods(Metrics_ExecId_t id, uint32_t missed_periods);
void Metrics_ExecStatsReset(Metrics_ExecId_t id);
bool Metrics_ExecStatsGet(Metrics_ExecId_t id, Metrics_ExecStats_t *stats);
const char* Metrics_ExecIdName(Metrics_ExecId_t id);

/** Fixed-size event FIFO. On overflow, the oldest event is discarded. */
bool Metrics_EventPush(uint16_t id, uint32_t argument);
bool Metrics_EventPop(Metrics_Event_t *event);
void Metrics_EventClear(void);
void Metrics_EventStatusGet(Metrics_EventStatus_t *status);
const char* Metrics_EventIdName(uint16_t id);

#ifdef __cplusplus
}
#endif

#endif /* METRICS_H */
