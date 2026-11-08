#ifndef METRICS_CONFIG_H
#define METRICS_CONFIG_H

/* Core clock is read from SystemCoreClock at runtime. */
#define METRICS_EVENT_CAPACITY 64U
#define METRICS_LOAD_WINDOW_MS 1000U

/* Set to 1 only for the dedicated startup smoke-test build. */
#define METRICS_ENABLE_STARTUP_SELF_TEST 0U
#define METRICS_TEST_EVENT_ID  0xA55AU
#define METRICS_TEST_EVENT_ARG 0x51A7C0DEUL

#endif /* METRICS_CONFIG_H */
