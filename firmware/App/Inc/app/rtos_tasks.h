#ifndef APP_RTOS_TASKS_H
#define APP_RTOS_TASKS_H

#include <stdbool.h>

/* Створює всі задачі FreeRTOS та запускає планувальник.
 * Повертає false лише якщо не вдалося створити задачу (не вистачило heap). */
bool RtosTasks_Start(void);

#endif /* APP_RTOS_TASKS_H */
