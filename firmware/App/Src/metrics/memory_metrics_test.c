#include "metrics/memory_metrics.h"
#include "metrics/memory_metrics_test.h"

uint32_t MemoryMetricsTest_Run(void) {
	MemoryMetrics_Snapshot_t snapshot;
	uint32_t result = 0U;

	if (!MemoryMetrics_GetSnapshot(&snapshot)) {
		return result;
	}

	if (snapshot.linker_layout_valid
			&& (snapshot.flash_used_bytes <= snapshot.flash_capacity_bytes)
			&& (snapshot.ram_static_bytes <= snapshot.ram_capacity_bytes)) {
		result |= MEMORY_METRICS_TEST_LAYOUT_OK;
	}

	if (snapshot.stack_paint_valid && (snapshot.stack_reserved_bytes != 0U)
			&& (snapshot.stack_used_peak_bytes <= snapshot.stack_reserved_bytes)
			&& (snapshot.stack_free_watermark_bytes >= 64U)) {
		result |= MEMORY_METRICS_TEST_STACK_OK;
	}

	if ((snapshot.heap_current_bytes <= snapshot.heap_peak_bytes)
			&& (snapshot.heap_peak_bytes <= snapshot.heap_capacity_bytes)) {
		result |= MEMORY_METRICS_TEST_HEAP_OK;
	}

	return result;
}
