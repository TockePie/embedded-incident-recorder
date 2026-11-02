#include "super_loop.h"
#include "perf.h"
#include "cli.h"
#include "main.h"
#include <stdio.h>

static perf_stat_t stat_loop    = { .name = "loop" };[cite: 7]
static perf_stat_t stat_sensor  = { .name = "sensor_task" };
static perf_stat_t stat_fsm     = { .name = "fsm_task" };
static perf_stat_t stat_cli     = { .name = "cli_task" };
static perf_stat_t stat_latency = { .name = "event_latency" };[cite: 7]

static volatile uint8_t  g_motion_flag = 0;
static volatile uint32_t g_event_timestamp = 0;

static AppState_t g_current_state = STATE_IDLE;

// Драйверні заглушки 
static bool Camera_StartCapture(void) { return true; }
static bool Camera_IsCaptureComplete(void) { return true; }
static bool Flash_WriteEventData(void) { return true; }

void SuperLoop_Init(void) {
    perf_init();[cite: 6]
    CLI_Init();
    g_current_state = STATE_IDLE;
}

void SuperLoop_OnGPIO_Interrupt(uint16_t pin) {
    if (pin == MOTION_SENSOR_Pin) {
        g_event_timestamp = perf_now();[cite: 7]
        g_motion_flag = 1;[cite: 7]
    }
}

static void Task_Sensors(void) {
    if (g_motion_flag) {
        g_motion_flag = 0;
        uint32_t latency = perf_now() - g_event_timestamp;
        perf_add(&stat_latency, latency);

        if (g_current_state == STATE_IDLE) {
            g_current_state = STATE_TRIGGERED;
        }
    }
}

static void Task_FSM(void) {
    switch (g_current_state) {
        case STATE_IDLE:
            break;
            
        case STATE_TRIGGERED:
            if (Camera_StartCapture()) {
                g_current_state = STATE_CAPTURING;
            } else {
                g_current_state = STATE_ERROR;
            }
            break;

        case STATE_CAPTURING:
            if (Camera_IsCaptureComplete()) {
                g_current_state = STATE_SAVING_TO_FLASH;
            }
            break;

        case STATE_SAVING_TO_FLASH:
            if (Flash_WriteEventData()) {
                printf("[EVT] Photo & Metadata written to SPI Flash!\r\n");
                g_current_state = STATE_IDLE;
            } else {
                g_current_state = STATE_ERROR;
            }
            break;

        case STATE_ERROR:
            printf("[ERR] System error encountered. Resetting state.\r\n");
            g_current_state = STATE_IDLE;
            break;
    }
}

static void Task_ReportStats(void) {
    printf("\r\n=== METRICS REPORT (DWT Cycles) ===\r\n");
    printf("Loop:    n=%lu, min=%lu us, max=%lu us\r\n", stat_loop.count, stat_loop.min, stat_loop.max);
    printf("Sensors: n=%lu, min=%lu us, max=%lu us\r\n", stat_sensor.count, stat_sensor.min, stat_sensor.max);
    printf("FSM:     n=%lu, min=%lu us, max=%lu us\r\n", stat_fsm.count, stat_fsm.min, stat_fsm.max);
    printf("Latency: n=%lu, min=%lu us, max=%lu us\r\n", stat_latency.count, stat_latency.min, stat_latency.max);
    printf("===================================\r\n");
}

void SuperLoop_Run(void) {
    uint32_t last_sensor_time = perf_now();
    uint32_t last_stats_time = perf_now();

    while (1) {
        uint32_t t_loop_start = perf_now();[cite: 7]

        // 1. опитування/обробкa сенсорів
        if ((perf_now() - last_sensor_time) >= SENSOR_POLL_PERIOD_US) {
            uint32_t t_start = perf_now();
            Task_Sensors();
            perf_add(&stat_sensor, perf_now() - t_start);
            last_sensor_time = perf_now();
        }

        // 2. FSM 
        uint32_t t_fsm_start = perf_now();
        Task_FSM();
        perf_add(&stat_fsm, perf_now() - t_fsm_start);

        // 3. CLI
        uint32_t t_cli_start = perf_now();
        Task_CLI();
        perf_add(&stat_cli, perf_now() - t_cli_start);

        // 4. звіт метрик
        if ((perf_now() - last_stats_time) >= STATS_REPORT_PERIOD_US) {
            Task_ReportStats();
            last_stats_time = perf_now();
        }

        perf_add(&stat_loop, perf_now() - t_loop_start);[cite: 7]
    }
}
