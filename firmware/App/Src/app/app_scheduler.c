#include "app/app_scheduler.h"
#include <stddef.h>
#include <string.h>

void AppScheduler_Init(AppScheduler_t *scheduler) {
	if (scheduler == NULL) {
		return;
	}

	memset(scheduler, 0, sizeof(*scheduler));
	scheduler->initialized = true;
}

AppScheduler_Status_t AppScheduler_AddPeriodic(AppScheduler_t *scheduler,
		AppScheduler_TaskFunction_t function, void *context, uint32_t period_ms,
		uint32_t now_ms) {
	AppScheduler_Task_t *task;

	if ((scheduler == NULL) || (function == NULL) || (period_ms == 0U)
			|| (!scheduler->initialized)) {
		return APP_SCHEDULER_ERROR_ARGUMENT;
	}

	if (scheduler->task_count >= APP_SCHEDULER_MAX_TASKS) {
		return APP_SCHEDULER_ERROR_FULL;
	}

	task = &scheduler->tasks[scheduler->task_count];
	task->function = function;
	task->context = context;
	task->period_ms = period_ms;
	task->last_run_ms = now_ms;
	task->enabled = true;
	scheduler->task_count++;

	return APP_SCHEDULER_OK;
}

void AppScheduler_Run(AppScheduler_t *scheduler, uint32_t now_ms) {
	uint8_t index;

	if ((scheduler == NULL) || (!scheduler->initialized)) {
		return;
	}

	for (index = 0U; index < scheduler->task_count; ++index) {
		AppScheduler_Task_t *task = &scheduler->tasks[index];

		if ((task->enabled != false)
				&& ((uint32_t) (now_ms - task->last_run_ms) >= task->period_ms)) {
			/* Skip missed periods instead of executing a burst of catch-up calls. */
			task->last_run_ms = now_ms;
			task->function(now_ms, task->context);
		}
	}
}
