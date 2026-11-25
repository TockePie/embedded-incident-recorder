# Архітектура FreeRTOS (ЛР4 — Embedded Incident Recorder)

## 1. Базова конфігурація ОСРЧ
* **Ядро:** FreeRTOS-Kernel V11.3.3 (Git submodule у `firmware/Middlewares/FreeRTOS/FreeRTOS-Kernel/`).
* **Порт:** `GCC/ARM_CM4F` (STM32F411RE, 100 МГц SYSCLK, апаратний FPU).
* **Менеджер пам'яті:** `heap_4`, `configTOTAL_HEAP_SIZE = 32 КБ` (`32U * 1024U`).
* **Конфігурація:** `firmware/App/Inc/FreeRTOSConfig.h`.
* **Часова база:** `SysTick` (1000 Гц, тік = 1 мс) використовується виключно FreeRTOS; часова база STM32 HAL перенесена на апаратний таймер `TIM4` (`stm32f4xx_hal_timebase_tim.c`).
* **Захисні механізми:** `configCHECK_FOR_STACK_OVERFLOW = 2`, `configUSE_MALLOC_FAILED_HOOK = 1` (обробники у `firmware/App/Src/app/freertos_hooks.c`), `INCLUDE_uxTaskGetStackHighWaterMark = 1`.
* **Рівні пріоритетів:** `configMAX_PRIORITIES = 6` (діапазон `0..5`, де `0` — `IDLE`).

## 2. Архітектура задач

Замість кооперативного Super Loop (`AppScheduler`) прикладна логіка розділена на 7 незалежних задач у `firmware/App/Src/app/rtos_tasks.c`.

| Назва задачі | Модулі всередині | Пріоритет | Період / Запуск | Стек (слова / байти) | Обґрунтування |
| :--- | :--- | :---: | :--- | :---: | :--- |
| **`ImuTask`** | `ImuService`, `IncidentDetector` | **5** (найвищий) | 5 мс (`vTaskDelayUntil`) | 512 / 2048 Б | Частота опитування MPU-6050 — 200 Гц (5 мс). Критичний шлях реального часу: не можна пропускати удар або нахил. |
| **`ControlTask`** | `UserButton`, `AppStateManager` | **4** | 5 мс (`vTaskDelayUntil`) | 256 / 1024 Б | Кнопка та менеджер станів об'єднані в одну задачу, оскільки працюють з однаковим періодом (5 мс) і спільним станом `s_state_manager`. |
| **`StorageTask`** | `ConfigService`, `StorageMaintenance`, `IncidentJournal`, `PhotoStore`, `PhotoCaptureService` | **3** | 5 мс (`vTaskDelayUntil`) | 512 / 2048 Б | Централізоване обслуговування операцій запису/стирання у SPI Flash W25Q64 (`SPI2`) та сервісу збереження фото під єдиним м'ютексом Flash, щоб уникнути гонки на шині та не блокувати `ImuTask`. |
| **`CameraTask`** | `OV7670_DvpCapture` | **2** | 2 мс (`vTaskDelayUntil`) | 512 / 2048 Б | Полінг стану захоплення кадру OV7670 та таймаутів DVP. Лінії HREF фіналізуються в EXTI ISR; низький пріоритет гарантує, що камера не затримує IMU. |
| **`CliTask`** | `CliService` (USART1) | **2** | Подія з ISR (`ulTaskNotifyTake`) + таймаут 5 мс | 1024 / 4096 Б | Миттєва реакція на вхідні команди та ACK/NACK передачі фото. Великий стек (4 КБ) виділено через парсинг команд і форматування відповідей. |
| **`LedTask`** | `LedService` | **1** | 10 мс (`vTaskDelayUntil`) | 256 / 1024 Б | Фонова світлодіодна індикація поточного стану системи та патернів інцидентів. |
| **`HealthTask`** | `HealthMonitor`, `Metrics_Poll` | **1** | 100 мс (`vTaskDelayUntil`) | 256 / 1024 Б | Фоновий моніторинг справності та регулярне оновлення 32-бітного лічильника DWT (період переповнення ~42.9 с). |

## 3. Синхронізація та міжзадачна взаємодія (`rtos_sync.c`)

* **М'ютекс SPI Flash (`s_flash_mutex`, recursive mutex):** захищає доступ до W25Q64 (`SPI2`) та структур конфігурації/журналу між `StorageTask`, `CliTask` (команди читання/експорту/збереження) та `ImuTask` (читання активної конфігурації).
* **М'ютекс діагностичного UART (`s_uart_mutex`, recursive mutex):** захищає `DiagUART_Printf()` (`USART2`) від перемішування рядків логу при одночасному виводі з різних задач.
* **М'ютекс стану системи (`s_state_mutex`, recursive mutex):** захищає `s_state_manager` і тригер патернів `LedService` між `ControlTask`, `ImuTask` та `LedTask`.
* **Сповіщення із переривання (Direct-to-Task Notification):** `HAL_UART_RxCpltCallback()` (USART1) викликає `RtosSync_NotifyCliFromISR()` (`vTaskNotifyGiveFromISR` + `portYIELD_FROM_ISR`), пробуджуючи `CliTask` одразу після прибуття байта.

## 4. Пріоритети переривань NVIC

У `FreeRTOSConfig.h` встановлено `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`. Усі периферійні переривання мають числовий пріоритет ≥ 5, що гарантує безпечний виклик `FromISR` API:

* `DMA1_Stream2_IRQn`, `DMA2_Stream4_IRQn` — пріоритет `5`
* `EXTI15_10_IRQn` (HREF/VSYNC камери) — пріоритет `6`
* `USART1_IRQn` (CLI), `USART2_IRQn` (Diag UART) — пріоритет `7`

## 5. Точки вимірювання для метрик (контракт для Хімко А.)

* Збережено всі існуючі маркери `Metrics_ExecBegin` / `Metrics_ExecEnd` (`METRICS_EXEC_FOREGROUND_IMU`, `METRICS_EXEC_FOREGROUND_DETECTOR`, `METRICS_EXEC_FOREGROUND_CAMERA`, `METRICS_EXEC_TASK_BUTTON`, `METRICS_EXEC_TASK_STATE_MANAGER`, `METRICS_EXEC_TASK_LED`, `METRICS_EXEC_TASK_CLI`, `METRICS_EXEC_TASK_CONFIG`, `METRICS_EXEC_TASK_JOURNAL`, `METRICS_EXEC_TASK_HEALTH_MONITOR`) та апаратні проби (`PA4`, `PB0`, `PB1`, `PB10`).
* Для зняття статистики стеків доступний `uxTaskGetStackHighWaterMark()`, для купи — `xPortGetFreeHeapSize()` / `xPortGetMinimumEverFreeHeapSize()`.
* Для ввімкнення `vTaskGetRunTimeStats()` достатньо виставити `configGENERATE_RUN_TIME_STATS = 1` у `FreeRTOSConfig.h` та прив'язати `portGET_RUN_TIME_COUNTER_VALUE()` до DWT.
