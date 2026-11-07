#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_SCHEDULER_MAX_TASKS 8U

typedef void (*AppScheduler_TaskFunction_t)(uint32_t now_ms, void *context);

typedef struct {
	AppScheduler_TaskFunction_t function;
	void *context;
	uint32_t period_ms;
	uint32_t last_run_ms;
	bool enabled;
} AppScheduler_Task_t;

typedef struct {
	AppScheduler_Task_t tasks[APP_SCHEDULER_MAX_TASKS];
	uint8_t task_count;
	bool initialized;
} AppScheduler_t;

typedef enum {
	APP_SCHEDULER_OK = 0,
	APP_SCHEDULER_ERROR_ARGUMENT,
	APP_SCHEDULER_ERROR_FULL
} AppScheduler_Status_t;

/** Initialize an empty cooperative task scheduler. */
void AppScheduler_Init(AppScheduler_t *scheduler);

/** Add a periodic task. The scheduler does not take ownership of context. */
AppScheduler_Status_t AppScheduler_AddPeriodic(
		AppScheduler_t *scheduler,
		AppScheduler_TaskFunction_t function,
		void *context,
		uint32_t period_ms,
		uint32_t now_ms);

/** Run each enabled task whose period has elapsed. */
void AppScheduler_Run(AppScheduler_t *scheduler, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCHEDULER_H */
