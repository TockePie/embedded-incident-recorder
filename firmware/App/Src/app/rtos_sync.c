#include "app/rtos_sync.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static SemaphoreHandle_t s_flash_mutex = NULL;
static SemaphoreHandle_t s_uart_mutex  = NULL;
static SemaphoreHandle_t s_state_mutex = NULL;

static bool RtosSync_IsSchedulerRunning(void)
{
    return (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
}

bool RtosSync_Init(void)
{
    /* Рекурсивні м'ютекси безпечні, навіть якщо одна задача візьме замок двічі */
    s_flash_mutex = xSemaphoreCreateRecursiveMutex();
    s_uart_mutex  = xSemaphoreCreateRecursiveMutex();
    s_state_mutex = xSemaphoreCreateRecursiveMutex();

    return (s_flash_mutex != NULL) && (s_uart_mutex != NULL) && (s_state_mutex != NULL);
}

void RtosSync_LockFlash(void)
{
    if ((s_flash_mutex != NULL) && RtosSync_IsSchedulerRunning()) {
        (void)xSemaphoreTakeRecursive(s_flash_mutex, portMAX_DELAY);
    }
}

void RtosSync_UnlockFlash(void)
{
    if ((s_flash_mutex != NULL) && RtosSync_IsSchedulerRunning()) {
        (void)xSemaphoreGiveRecursive(s_flash_mutex);
    }
}

void RtosSync_LockDiagUart(void)
{
    if ((s_uart_mutex != NULL) && RtosSync_IsSchedulerRunning()) {
        (void)xSemaphoreTakeRecursive(s_uart_mutex, portMAX_DELAY);
    }
}

void RtosSync_UnlockDiagUart(void)
{
    if ((s_uart_mutex != NULL) && RtosSync_IsSchedulerRunning()) {
        (void)xSemaphoreGiveRecursive(s_uart_mutex);
    }
}

void RtosSync_LockState(void)
{
    if ((s_state_mutex != NULL) && RtosSync_IsSchedulerRunning()) {
        (void)xSemaphoreTakeRecursive(s_state_mutex, portMAX_DELAY);
    }
}

void RtosSync_UnlockState(void)
{
    if ((s_state_mutex != NULL) && RtosSync_IsSchedulerRunning()) {
        (void)xSemaphoreGiveRecursive(s_state_mutex);
    }
}
