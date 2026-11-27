#include "app/rtos_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app/app.h"
#include "app/rtos_sync.h"
#include "metrics/metrics.h"
#include "metrics/rtos_metrics.h"
#include "services/led_service.h"
#include "stm32f4xx_hal.h"

/* Розміри стеків у словах (1 слово = 4 байти) */
#define IMU_TASK_STACK_SIZE          512U
#define CONTROL_TASK_STACK_SIZE      256U
#define STORAGE_TASK_STACK_SIZE      512U
#define CAMERA_TASK_STACK_SIZE       512U
#define CLI_TASK_STACK_SIZE         1024U
#define LED_TASK_STACK_SIZE          256U
#define HEALTH_TASK_STACK_SIZE       256U

/* Фінальні пріоритети (configMAX_PRIORITIES = 6 -> 0..5) */
#define IMU_TASK_PRIORITY           (tskIDLE_PRIORITY + 5U)
#define CONTROL_TASK_PRIORITY       (tskIDLE_PRIORITY + 4U)
#define STORAGE_TASK_PRIORITY       (tskIDLE_PRIORITY + 3U)
#define CAMERA_TASK_PRIORITY        (tskIDLE_PRIORITY + 2U)
#define CLI_TASK_PRIORITY           (tskIDLE_PRIORITY + 2U)
#define LED_TASK_PRIORITY           (tskIDLE_PRIORITY + 1U)
#define HEALTH_TASK_PRIORITY        (tskIDLE_PRIORITY + 1U)

#define IMU_TASK_PERIOD_MS            5U
#define CONTROL_TASK_PERIOD_MS        5U
#define STORAGE_TASK_PERIOD_MS        5U
#define CAMERA_TASK_PERIOD_MS         2U
#define CLI_TASK_PERIOD_MS            5U
#define LED_TASK_PERIOD_MS           10U
#define HEALTH_TASK_PERIOD_MS       100U

static void ImuTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        App_StepImu();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
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

static void StorageTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        App_StepStorage();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STORAGE_TASK_PERIOD_MS));
    }
}

static void CameraTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        App_StepCamera();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAMERA_TASK_PERIOD_MS));
    }
}

static void CliTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* Прокидаємось миттєво при отриманні байта з UART1 (FromISR)
         * або раз на 5 мс для фонових стейт-машин CLI (фото/експорт). */
        RtosSync_WaitCliEvent(CLI_TASK_PERIOD_MS);
        App_StepCli();
    }
}

static void LedTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        Metrics_ExecBegin(METRICS_EXEC_TASK_LED, LED_TASK_PERIOD_MS);
        RtosSync_LockState();
        LedService_Update(App_GetState(), HAL_GetTick());
        RtosSync_UnlockState();
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

        /* Оновлюємо 32-бітний DWT лічильник, щоб не було переповнення (~42.9 с) */
        Metrics_Poll();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(HEALTH_TASK_PERIOD_MS));
    }
}

bool RtosTasks_Start(void)
{
    TaskHandle_t cli_task_handle = NULL;

    if (!RtosSync_Init()) {
        return false;
    }

    /* [metrics] configured stack sizes, so that 'rtos stacks' can show used = size - high-water mark */
    RtosMetrics_RegisterStack("ImuTask", IMU_TASK_STACK_SIZE);
    RtosMetrics_RegisterStack("ControlTask", CONTROL_TASK_STACK_SIZE);
    RtosMetrics_RegisterStack("StorageTask", STORAGE_TASK_STACK_SIZE);
    RtosMetrics_RegisterStack("CameraTask", CAMERA_TASK_STACK_SIZE);
    RtosMetrics_RegisterStack("CliTask", CLI_TASK_STACK_SIZE);
    RtosMetrics_RegisterStack("LedTask", LED_TASK_STACK_SIZE);
    RtosMetrics_RegisterStack("HealthTask", HEALTH_TASK_STACK_SIZE);

    if (xTaskCreate(ImuTask, "ImuTask", IMU_TASK_STACK_SIZE,
        NULL, IMU_TASK_PRIORITY, NULL) != pdPASS) {
        return false;
        }

        if (xTaskCreate(ControlTask, "ControlTask", CONTROL_TASK_STACK_SIZE,
            NULL, CONTROL_TASK_PRIORITY, NULL) != pdPASS) {
            return false;
            }

            if (xTaskCreate(StorageTask, "StorageTask", STORAGE_TASK_STACK_SIZE,
                NULL, STORAGE_TASK_PRIORITY, NULL) != pdPASS) {
                return false;
                }

                if (xTaskCreate(CameraTask, "CameraTask", CAMERA_TASK_STACK_SIZE,
                    NULL, CAMERA_TASK_PRIORITY, NULL) != pdPASS) {
                    return false;
                    }

                    if (xTaskCreate(CliTask, "CliTask", CLI_TASK_STACK_SIZE,
                        NULL, CLI_TASK_PRIORITY, &cli_task_handle) != pdPASS) {
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

                                /* [FIX, role 1] without this call s_cli_task stays NULL and
                                 * RtosSync_NotifyCliFromISR() never wakes CliTask: it is only
                                 * woken by the 5 ms timeout. */
                                RtosSync_RegisterCliTask((void *)cli_task_handle);

                                vTaskStartScheduler();
                            return false;
}