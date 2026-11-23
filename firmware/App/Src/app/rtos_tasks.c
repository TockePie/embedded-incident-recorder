#include "app/rtos_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app/app.h"
#include "metrics/metrics.h"
#include "services/led_service.h"
#include "stm32f4xx_hal.h"

/* Розміри стеків у словах (1 слово = 4 байти на Cortex-M4) */
#define APP_MAIN_TASK_STACK_SIZE    1024U
#define CONTROL_TASK_STACK_SIZE      256U
#define LED_TASK_STACK_SIZE          256U
#define HEALTH_TASK_STACK_SIZE       256U

/* Тимчасові пріоритети (configMAX_PRIORITIES = 6, отже діапазон 0..5).
 * Поки AppMainTask крутиться без затримок, її пріоритет має бути нижчим,
 * ніж у періодичних задач, інакше вони ніколи не отримають процесор. */
#define APP_MAIN_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)
#define LED_TASK_PRIORITY           (tskIDLE_PRIORITY + 2U)
#define HEALTH_TASK_PRIORITY        (tskIDLE_PRIORITY + 2U)
#define CONTROL_TASK_PRIORITY       (tskIDLE_PRIORITY + 3U)

#define CONTROL_TASK_PERIOD_MS       5U
#define LED_TASK_PERIOD_MS          10U
#define HEALTH_TASK_PERIOD_MS      100U

static void AppMainTask(void *argument)
{
    (void)argument;

    for (;;) {
        Metrics_LoopBegin();
        App_RunOnce();
        Metrics_LoopEnd();
        Metrics_Poll();
    }
}

static void ControlTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        uint32_t now_ms = HAL_GetTick();

        Metrics_ExecBegin(METRICS_EXEC_TASK_BUTTON, CONTROL_TASK_PERIOD_MS);
        App_StepButton(now_ms);
        Metrics_ExecEnd(METRICS_EXEC_TASK_BUTTON);

        Metrics_ExecBegin(METRICS_EXEC_TASK_STATE_MANAGER, CONTROL_TASK_PERIOD_MS);
        App_StepStateManager(now_ms);
        Metrics_ExecEnd(METRICS_EXEC_TASK_STATE_MANAGER);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

static void LedTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        Metrics_ExecBegin(METRICS_EXEC_TASK_LED, LED_TASK_PERIOD_MS);
        LedService_Update(App_GetState(), HAL_GetTick());
        Metrics_ExecEnd(METRICS_EXEC_TASK_LED);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
    }
}

static void HealthTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        Metrics_ExecBegin(METRICS_EXEC_TASK_HEALTH_MONITOR, HEALTH_TASK_PERIOD_MS);
        App_StepHealthMonitor(HAL_GetTick());
        Metrics_ExecEnd(METRICS_EXEC_TASK_HEALTH_MONITOR);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(HEALTH_TASK_PERIOD_MS));
    }
}

bool RtosTasks_Start(void)
{
    if (xTaskCreate(AppMainTask, "AppMain", APP_MAIN_TASK_STACK_SIZE,
        NULL, APP_MAIN_TASK_PRIORITY, NULL) != pdPASS) {
        return false;
        }

        if (xTaskCreate(ControlTask, "ControlTask", CONTROL_TASK_STACK_SIZE,
            NULL, CONTROL_TASK_PRIORITY, NULL) != pdPASS) {
            return false;
            }

            if (xTaskCreate(LedTask, "LedTask", LED_TASK_STACK_SIZE,
                NULL, LED_TASK_PRIORITY, NULL) != pdPASS) {
                return false;
                }

                if (xTaskCreate(HealthTask, "HealthTask", HEALTH_TASK_STACK_SIZE,
                    NULL, HEALTH_TASK_PRIORITY, NULL) != pdPASS) {
                    return false;
                    }

                    vTaskStartScheduler();

                /* Сюди виконання дійде лише якщо не вистачило RAM на Idle/Timer задачі */
                return false;
}
