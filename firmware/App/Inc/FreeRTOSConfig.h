
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* Clock and tick configuration */
#define configCPU_CLOCK_HZ                  (100000000UL)
#define configTICK_RATE_HZ                  (1000U)
#define configTICK_TYPE_WIDTH_IN_BITS       TICK_TYPE_WIDTH_32_BITS

/* Scheduler */
#define configUSE_PREEMPTION                 1
#define configUSE_TIME_SLICING               1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configMAX_PRIORITIES                 6
#define configMINIMAL_STACK_SIZE             128
#define configMAX_TASK_NAME_LEN              16
#define configIDLE_SHOULD_YIELD              1

/* Memory */
#define configSUPPORT_STATIC_ALLOCATION      0
#define configSUPPORT_DYNAMIC_ALLOCATION     1
#define configTOTAL_HEAP_SIZE                (16U * 1024U)
#define configAPPLICATION_ALLOCATED_HEAP     0

/* Hooks */
#define configUSE_IDLE_HOOK                  0
#define configUSE_TICK_HOOK                  0
#define configCHECK_FOR_STACK_OVERFLOW       2
#define configUSE_MALLOC_FAILED_HOOK         1

/* Synchronization and communication */
#define configUSE_MUTEXES                    1
#define configUSE_RECURSIVE_MUTEXES          1
#define configUSE_COUNTING_SEMAPHORES        1
#define configUSE_TASK_NOTIFICATIONS         1
#define configUSE_QUEUE_SETS                 0
#define configQUEUE_REGISTRY_SIZE            8

/* Software timers */
#define configUSE_TIMERS                     1
#define configTIMER_TASK_PRIORITY            2
#define configTIMER_QUEUE_LENGTH             8
#define configTIMER_TASK_STACK_DEPTH         256

/* Cortex-M interrupt priorities */
#define configPRIO_BITS                      4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY       15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

#define configKERNEL_INTERRUPT_PRIORITY \
(15U << (8U - configPRIO_BITS))

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
(5U << (8U - configPRIO_BITS))

/* Handler names */
#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler

/* Runtime statistics */
#define configGENERATE_RUN_TIME_STATS       0

/* API inclusion */
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_vTaskDelayUntil              1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskResume                 1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

#endif /* FREERTOS_CONFIG_H */
