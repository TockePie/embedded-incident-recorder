#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"

#include "metrics/metrics_config.h"
#include "metrics/rtos_metrics.h"

/* Fatal-error state for the debugger: kind 1 = stack overflow, 2 = malloc failed. */
volatile uint32_t g_rtos_fault_kind;
volatile char g_rtos_fault_task[configMAX_TASK_NAME_LEN];

/* Fatal path: write straight to the USART2 registers. No mutex, no HAL, no
 * heap, no interrupts: the kernel is already in an undefined state here. */
static void Hooks_FatalWrite(const char *text)
{
    while (*text != '\0') {
        while ((USART2->SR & USART_SR_TXE) == 0U) {
        }
        USART2->DR = (uint8_t)*text;
        text++;
    }
    while ((USART2->SR & USART_SR_TC) == 0U) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    uint32_t i;
    char name[configMAX_TASK_NAME_LEN];

    (void)xTask;

    taskDISABLE_INTERRUPTS();

    for (i = 0U; i < (configMAX_TASK_NAME_LEN - 1U); i++) {
        name[i] = pcTaskName[i];
        g_rtos_fault_task[i] = pcTaskName[i];
        if (pcTaskName[i] == '\0') {
            break;
        }
    }
    name[i] = '\0';
    g_rtos_fault_task[i] = '\0';
    g_rtos_fault_kind = 1U;

    Hooks_FatalWrite("\r\nFATAL stack overflow in ");
    Hooks_FatalWrite(name);
    Hooks_FatalWrite("\r\n");

    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();

    g_rtos_fault_kind = 2U;
    Hooks_FatalWrite("\r\nFATAL FreeRTOS heap exhausted (malloc failed)\r\n");

    for (;;)
    {
    }
}

#if (METRICS_RTOS_TICK_PROBE != 0U)
/* Times the kernel tick handler. The measured interval starts at the first
 * instruction of this handler and ends after xPortSysTickHandler() returns;
 * the hardware exception entry/exit (about 12 cycles each way) is not included. */
extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
    const uint32_t start = DWT->CYCCNT;

    xPortSysTickHandler();

    RtosMetrics_TickRecord((uint32_t)(DWT->CYCCNT - start));
}
#endif
