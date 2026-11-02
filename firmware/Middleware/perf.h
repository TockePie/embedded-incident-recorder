#ifndef PERF_H
#define PERF_H

#include "stm32f4xx.h"
#include <stdint.h>

static inline void perf_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;         
}

static inline uint32_t perf_now(void) {
    return DWT->CYCCNT;[cite: 6]
}

static inline uint32_t perf_cycles_to_us(uint32_t cyc) {
    return cyc / (SystemCoreClock / 1000000u);[cite: 6]
}

typedef struct {
    const char *name;
    uint32_t count;
    uint32_t min;
    uint32_t max;
    uint64_t sum;
} perf_stat_t;[cite: 6]

static inline void perf_add(perf_stat_t *s, uint32_t cyc) {
    uint32_t us = perf_cycles_to_us(cyc);
    if (s->count == 0 || us < s->min) s->min = us;[cite: 6]
    if (us > s->max) s->max = us;[cite: 7]
    s->sum += us;[cite: 7]
    s->count++;[cite: 7]
}

#endif
