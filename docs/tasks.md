# Опис структури Super Loop та перелік задач (ЛР3)

## 1. Схема виконання та порядок обробки
Прошивка побудована за архітектурою **Time-Triggered / Event-Driven Super Loop**[cite: 5]. 

1. **Event-driven:** Події високого пріоритету (рух/переривання від сенсора) фіксуються в ISR (`HAL_GPIO_EXTI_Callback`) з фіксацією точного часу DWT `CYCCNT`[cite: 6, 7].
2. **Time-triggered:** Перевірка періодичних задач проводиться за допомогою лічильника `perf_now()` без використання блокуючих затримок (`HAL_Delay`)[cite: 5, 6].
3. **FSM (Non-blocking):** Довготривалі операції (камера, запис Flash) розбиті на кроки машини станів[cite: 5].

---

## 2. Таблиця задач головного циклу

| Назва задачі | Тип запуску | Період / Умова | Очікувана тривалість | Дедлайн | Залежності |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `Task_Sensors` | Time-Triggered / Event | 10 мс / ISR flag | < 100 мкс | 1 мс | Датчик руху, EXTI |
| `Task_FSM` | Continuous / Event | Кожен цикл | 50 мкс (без блокування) | 50 ммс | DCMI Camera, SPI Flash |
| `Task_CLI` | Continuous | Кожен цикл (при наявності даних) | < 200 мкс | 100 мс | USART RX ISR (RingBuffer) |
| `Task_ReportStats` | Time-Triggered | 2000 мс | < 2 мс | N/A | DWT CYCCNT, USART TX |

---

## 3. Машина станів (FSM)

[STATE_IDLE] ──(PIR Interrupt)──> [STATE_TRIGGERED]
▲                                   │
│                               (Start Cam)
│                                   ▼
[STATE_SAVING_TO_FLASH] <──(Captured)── [STATE_CAPTURING]

