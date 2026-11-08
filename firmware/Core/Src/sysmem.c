/**
 ******************************************************************************
 * @file      sysmem.c
 * @brief     Newlib heap support with current/high-water accounting.
 *
 * The heap grows upward from _end and is prevented from entering the linker
 * reserved MSP stack region. This project is bare-metal and does not call
 * malloc/free from interrupt context; _sbrk therefore needs no RTOS lock.
 ******************************************************************************
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include "metrics/memory_metrics.h"

static uint8_t *s_heap_start;
static uint8_t *s_heap_end;
static uint32_t s_heap_peak_bytes;
static uint32_t s_heap_sbrk_calls;
static uint32_t s_heap_failed_calls;

void *_sbrk(ptrdiff_t increment)
{
    extern uint8_t _end;
    extern uint8_t _estack;
    extern uint32_t _Min_Stack_Size;

    const uintptr_t stack_limit = (uintptr_t) &_estack
            - (uintptr_t) &_Min_Stack_Size;
    uintptr_t current;
    uintptr_t next;
    uint32_t current_bytes;
    uint8_t *previous;

    if (s_heap_start == NULL) {
        s_heap_start = &_end;
        s_heap_end = &_end;
    }

    s_heap_sbrk_calls++;
    current = (uintptr_t) s_heap_end;

    if (increment >= 0) {
        const uintptr_t amount = (uintptr_t) increment;
        if ((amount > UINTPTR_MAX - current)
                || (current + amount > stack_limit)) {
            s_heap_failed_calls++;
            errno = ENOMEM;
            MemoryMetrics_HeapAccount((uint32_t) (current - (uintptr_t) s_heap_start),
                    s_heap_peak_bytes, s_heap_sbrk_calls, s_heap_failed_calls);
            return (void *) -1;
        }
        next = current + amount;
    } else {
        const uintptr_t amount = (uintptr_t) (-(increment + 1)) + 1U;
        if (amount > (current - (uintptr_t) s_heap_start)) {
            s_heap_failed_calls++;
            errno = EINVAL;
            MemoryMetrics_HeapAccount((uint32_t) (current - (uintptr_t) s_heap_start),
                    s_heap_peak_bytes, s_heap_sbrk_calls, s_heap_failed_calls);
            return (void *) -1;
        }
        next = current - amount;
    }

    previous = s_heap_end;
    s_heap_end = (uint8_t *) next;
    current_bytes = (uint32_t) (next - (uintptr_t) s_heap_start);
    if (current_bytes > s_heap_peak_bytes) {
        s_heap_peak_bytes = current_bytes;
    }
    MemoryMetrics_HeapAccount(current_bytes, s_heap_peak_bytes,
            s_heap_sbrk_calls, s_heap_failed_calls);
    return previous;
}

#if defined(__PICOLIBC__)
__strong_reference(_sbrk, sbrk);
#endif
