#ifndef METRICS_CONFIG_H
#define METRICS_CONFIG_H

/* Core clock is read from SystemCoreClock at runtime. */
#define METRICS_EVENT_CAPACITY 64U
#define METRICS_LOAD_WINDOW_MS 1000U

/* 1: пропущені періоди рахуються всередині Metrics_ExecBegin() за тією самою
 * формулою, що й в AppScheduler_Run() у ЛР3: elapsed_ms > period_ms ->
 * missed = elapsed_ms / period_ms - 1. 0: пропуски вже рахує AppScheduler. */
#define METRICS_MISSED_FROM_EXEC_BEGIN 1U

/* Wrap CYCCNT вважається справжнім, лише якщо попереднє значення було не
 * меншим за цей поріг. Інше "зменшення" лічильника вважається аномалією
 * (наприклад, хтось скинув DWT->CYCCNT) і лише рахується. */
#define METRICS_DWT_WRAP_MIN_LAST 0x80000000UL

/* 1: run-time stats, лічильник перемикань і trace FreeRTOS. 0: вимкнути, щоб
 * виміряти вартість самих вимірювань. */
#define METRICS_RTOS_STATS_ENABLE 1U

/* 1: обгортка над SysTick_Handler, що міряє тривалість обробника тіка. */
#define METRICS_RTOS_TICK_PROBE 1U

/* Кільцевий буфер хронології перемикань задач (8 байтів на запис). */
#define METRICS_RTOS_TRACE_CAPACITY 256U

/* Скільки задач одночасно читаємо через uxTaskGetSystemState(). */
#define METRICS_RTOS_MAX_TASKS 12U

/* Очікуваний період нових зразків MPU-6050 (200 Гц) для Metrics_ImuSampleObserve(). */
#define METRICS_IMU_SAMPLE_PERIOD_MS 5U

/* Set to 1 only for the dedicated startup smoke-test build. */
#define METRICS_ENABLE_STARTUP_SELF_TEST 0U
#define METRICS_TEST_EVENT_ID  0xA55AU
#define METRICS_TEST_EVENT_ARG 0x51A7C0DEUL

#endif /* METRICS_CONFIG_H */