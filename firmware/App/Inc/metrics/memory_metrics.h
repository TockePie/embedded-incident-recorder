#ifndef MEMORY_METRICS_H
#define MEMORY_METRICS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t flash_used_bytes;
	uint32_t flash_capacity_bytes;
	uint32_t ram_data_bytes;
	uint32_t ram_bss_bytes;
	uint32_t ram_static_bytes;
	uint32_t ram_capacity_bytes;
	uint32_t ram_free_after_static_bytes;
	uint32_t heap_current_bytes;
	uint32_t heap_peak_bytes;
	uint32_t heap_capacity_bytes;
	uint32_t heap_sbrk_calls;
	uint32_t heap_failed_calls;
	uint32_t heap_min_reserve_bytes;
	uint32_t stack_used_peak_bytes;
	uint32_t stack_reserved_bytes;
	uint32_t stack_free_watermark_bytes;
	uint32_t stack_min_reserve_bytes;
	bool linker_layout_valid;
	bool stack_paint_valid;
} MemoryMetrics_Snapshot_t;

/* Must be called as the first operation in main(), before HAL_Init(). */
void MemoryMetrics_StackPaint(void);

/* Read-only snapshot. Stack usage is a watermark since StackPaint(). */
bool MemoryMetrics_GetSnapshot(MemoryMetrics_Snapshot_t *snapshot);

/* Internal heap accounting hooks called by the _sbrk implementation. */
void MemoryMetrics_HeapAccount(uint32_t current_bytes, uint32_t peak_bytes,
		uint32_t sbrk_calls, uint32_t failed_calls);

#ifdef __cplusplus
}
#endif

#endif /* MEMORY_METRICS_H */
