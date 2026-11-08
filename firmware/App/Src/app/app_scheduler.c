#include "app/app_scheduler.h"

#include <stddef.h>
#include <string.h>

void AppScheduler_Init(AppScheduler_t *scheduler)
{
    if (scheduler == NULL) {
        return;
    }

    (void) memset(scheduler, 0, sizeof(*scheduler));
    scheduler->initialized = true;
}

AppScheduler_Status_t AppScheduler_AddPeriodic(AppScheduler_t *scheduler,
        AppScheduler_TaskFunction_t function, void *context, uint32_t period_ms,
        uint32_t now_ms, Metrics_ExecId_t metrics_id)
{
    AppScheduler_Task_t *task;
    uint8_t index;

    if ((scheduler == NULL) || (function == NULL) || (period_ms == 0U)
            || (!scheduler->initialized)
            || (metrics_id >= METRICS_EXEC_ID_COUNT)
            || (metrics_id > METRICS_EXEC_TASK_HEALTH_MONITOR)) {
        return APP_SCHEDULER_ERROR_ARGUMENT;
    }

    if (scheduler->task_count >= APP_SCHEDULER_MAX_TASKS) {
        return APP_SCHEDULER_ERROR_FULL;
    }

    for (index = 0U; index < scheduler->task_count; ++index) {
        if (scheduler->tasks[index].metrics_id == metrics_id) {
            return APP_SCHEDULER_ERROR_ARGUMENT;
        }
    }

    task = &scheduler->tasks[scheduler->task_count];
    task->function = function;
    task->context = context;
    task->period_ms = period_ms;
    task->last_run_ms = now_ms;
    task->metrics_id = metrics_id;
    task->enabled = true;
    scheduler->task_count++;

    return APP_SCHEDULER_OK;
}

void AppScheduler_Run(AppScheduler_t *scheduler, uint32_t now_ms)
{
    uint8_t index;

    if ((scheduler == NULL) || (!scheduler->initialized)) {
        return;
    }

    for (index = 0U; index < scheduler->task_count; ++index) {
        AppScheduler_Task_t *task = &scheduler->tasks[index];
        const uint32_t elapsed_ms = (uint32_t) (now_ms - task->last_run_ms);

        if ((task->enabled != false) && (elapsed_ms >= task->period_ms)) {
            uint32_t missed_periods = 0U;

            /* Preserve the existing skip/catch-up policy. For example, a
             * 16 ms delay on a 5 ms task means two elapsed periods were
             * skipped after the one invocation that is executed now. */
            if (elapsed_ms > task->period_ms) {
                missed_periods = (elapsed_ms / task->period_ms) - 1U;
            }

            task->last_run_ms = now_ms;
            Metrics_ExecMissedPeriods(task->metrics_id, missed_periods);
            if (missed_periods != 0U) {
                const uint32_t event_argument =
                        (((uint32_t) task->metrics_id & 0xFFU) << 24U)
                        | (missed_periods & 0x00FFFFFFUL);
                (void) Metrics_EventPush(
                        METRICS_EVENT_SCHEDULER_MISSED_PERIODS,
                        event_argument);
            }
            Metrics_ExecBegin(task->metrics_id, task->period_ms);
            task->function(now_ms, task->context);
            Metrics_ExecEnd(task->metrics_id);
        }
    }
}
