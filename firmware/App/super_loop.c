#include "super_loop.h"
#include "drivers_stub.h"
#include "main.h"
#include <stddef.h>

__attribute__((weak)) void Metrics_OnLoopStart(void) {}
__attribute__((weak)) void Metrics_OnLoopEnd(void) {}
__attribute__((weak)) void Metrics_OnTaskStart(SuperLoop_TaskID_t task_id) { (void)task_id; }
__attribute__((weak)) void Metrics_OnTaskEnd(SuperLoop_TaskID_t task_id) { (void)task_id; }

static Bmp180Data_t g_bmp_data;
static bool g_incident_pending = false;

static void Task_ProcessUART(void) {
    uint8_t c;
    while (Drivers_UART_DequeueRx(&c)) {
        if (c == 'R' || c == 'r') {
            g_incident_pending = true;
        }
    }
}

static void Task_ReadSensors(void) {
    if (Drivers_BMP180_Read(&g_bmp_data)) {
        if (g_bmp_data.threshold_exceeded) {
            g_incident_pending = true;
        }
    }
}

static void Task_CameraPoll(void) {
    if (g_incident_pending) {
        Drivers_Camera_TriggerCapture();
    }
}

static void Task_FlashFlush(void) {
    static uint8_t fsm_state = 0;
    static uint8_t log_payload[32];

    switch (fsm_state) {
        case 0:
            if (g_incident_pending) {
                log_payload[0] = 0xAA;
                log_payload[1] = (uint8_t)((uint32_t)g_bmp_data.temperature);
                fsm_state = 1;
            }
            break;

        case 1:
            if (Drivers_W25Q64_WritePage(0x000000, log_payload, sizeof(log_payload))) {
                g_incident_pending = false;
                fsm_state = 0;
            }
            break;
    }
}

static void Task_HeartbeatLED(void) {
#ifdef LD2_GPIO_Port
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
#endif
}

static SuperLoop_Task_t g_tasks[TASK_COUNT] = {
    [TASK_ID_UART_RX]   = { TASK_ID_UART_RX,   "UART_Process", Task_ProcessUART,   10,  0, true },
    [TASK_ID_SENSORS]   = { TASK_ID_SENSORS,   "Sensors_Read", Task_ReadSensors,   100, 0, true },
    [TASK_ID_CAMERA]    = { TASK_ID_CAMERA,    "Camera_Poll",  Task_CameraPoll,    50,  0, true },
    [TASK_ID_FLASH_LOG] = { TASK_ID_FLASH_LOG, "Flash_Flush",  Task_FlashFlush,    200, 0, true },
    [TASK_ID_HEARTBEAT] = { TASK_ID_HEARTBEAT, "Heartbeat",    Task_HeartbeatLED,  500, 0, true },
};

void SuperLoop_Init(void) {
    Drivers_InitAll();
    uint32_t current_time = HAL_GetTick();
    for (int i = 0; i < TASK_COUNT; i++) {
        g_tasks[i].last_run_ms = current_time;
    }
}

void SuperLoop_EnableTask(SuperLoop_TaskID_t task_id, bool enable) {
    if (task_id < TASK_COUNT) {
        g_tasks[task_id].enabled = enable;
    }
}

void SuperLoop_Run(void) {
    while (1) {
        Metrics_OnLoopStart();
        uint32_t now = HAL_GetTick();

        for (int i = 0; i < TASK_COUNT; i++) {
            SuperLoop_Task_t *task = &g_tasks[i];
            if (!task->enabled || task->function == NULL) {
                continue;
            }

            if (task->period_ms == 0 || ((uint32_t)(now - task->last_run_ms) >= task->period_ms)) {
                Metrics_OnTaskStart(task->id);
                
                task->function();
                
                Metrics_OnTaskEnd(task->id);
                task->last_run_ms = now;
            }
        }
        Metrics_OnLoopEnd();
    }
}
