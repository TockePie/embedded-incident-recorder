#include "metrics/memory_metrics.h"

#include <stddef.h>
#include <stdint.h>

#define MEMORY_FLASH_BASE       0x08000000UL
#define MEMORY_FLASH_BYTES      (512UL * 1024UL)
#define MEMORY_RAM_BASE         0x20000000UL
#define MEMORY_RAM_BYTES        (128UL * 1024UL)
#define MEMORY_STACK_PATTERN    0xDEADBEEFUL

extern uint8_t _sidata;
extern uint8_t _sdata;
extern uint8_t _edata;
extern uint8_t _sbss;
extern uint8_t _ebss;
extern uint8_t _end;
extern uint8_t _estack;
extern uint32_t _Min_Heap_Size;
extern uint32_t _Min_Stack_Size;

static uintptr_t s_stack_bottom;
static uintptr_t s_stack_paint_start;
static uintptr_t s_stack_initial_sp;
static bool s_stack_paint_valid;
static uint32_t s_heap_current;
static uint32_t s_heap_peak;
static uint32_t s_heap_sbrk_calls;
static uint32_t s_heap_failed_calls;

void MemoryMetrics_StackPaint(void) {
	uintptr_t stack_pointer;
	uintptr_t paint_end;
	uintptr_t address;
	const uintptr_t stack_top = (uintptr_t) &_estack;
	const uintptr_t stack_size = (uintptr_t) &_Min_Stack_Size;

	__asm volatile ("mrs %0, msp" : "=r" (stack_pointer));

	s_stack_bottom = stack_top - stack_size;
	s_stack_initial_sp = stack_pointer;
	s_stack_paint_start = 0U;
	s_stack_paint_valid = false;

	/* Preserve the active C frames and a 64-byte safety margin. */
	paint_end = (stack_pointer > 64U) ? (stack_pointer - 64U) : 0U;
	paint_end &= ~(uintptr_t) 0x3U;

	if ((stack_size >= 64U) && (paint_end > s_stack_bottom)
			&& (paint_end <= stack_top) && (s_stack_bottom >= MEMORY_RAM_BASE)
			&& (stack_top <= (MEMORY_RAM_BASE + MEMORY_RAM_BYTES))) {
		for (address = s_stack_bottom; address < paint_end; address +=
				sizeof(uint32_t)) {
			*((volatile uint32_t*) address) = MEMORY_STACK_PATTERN;
		}
		s_stack_paint_start = s_stack_bottom;
		s_stack_paint_valid = true;
	}
}

void MemoryMetrics_HeapAccount(uint32_t current_bytes, uint32_t peak_bytes,
		uint32_t sbrk_calls, uint32_t failed_calls) {
	s_heap_current = current_bytes;
	s_heap_peak = peak_bytes;
	s_heap_sbrk_calls = sbrk_calls;
	s_heap_failed_calls = failed_calls;
}

bool MemoryMetrics_GetSnapshot(MemoryMetrics_Snapshot_t *snapshot) {
	uintptr_t flash_end;
	uintptr_t data_size;
	uintptr_t bss_size;
	uintptr_t stack_top;
	uintptr_t stack_size;
	uintptr_t stack_bottom;
	uintptr_t first_modified;
	uintptr_t address;
	uintptr_t heap_start;
	uintptr_t stack_limit;
	uint32_t stack_used;

	if (snapshot == NULL) {
		return false;
	}

	snapshot->flash_capacity_bytes = MEMORY_FLASH_BYTES;
	snapshot->ram_capacity_bytes = MEMORY_RAM_BYTES;

	data_size = (uintptr_t) &_edata - (uintptr_t) &_sdata;
	bss_size = (uintptr_t) &_ebss - (uintptr_t) &_sbss;
	flash_end = (uintptr_t) &_sidata + data_size;

	snapshot->ram_data_bytes = (uint32_t) data_size;
	snapshot->ram_bss_bytes = (uint32_t) bss_size;
	snapshot->ram_static_bytes =
			((uintptr_t) &_end >= MEMORY_RAM_BASE) ?
					(uint32_t) ((uintptr_t) &_end - MEMORY_RAM_BASE) : 0U;
	snapshot->ram_free_after_static_bytes =
			(snapshot->ram_static_bytes <= MEMORY_RAM_BYTES) ?
					(MEMORY_RAM_BYTES - snapshot->ram_static_bytes) : 0U;

	snapshot->linker_layout_valid = (((uintptr_t) &_sdata >= MEMORY_RAM_BASE)
			&& ((uintptr_t) &_edata >= (uintptr_t) &_sdata)
			&& ((uintptr_t) &_edata <= (MEMORY_RAM_BASE + MEMORY_RAM_BYTES))
			&& ((uintptr_t) &_sbss >= (uintptr_t) &_edata)
			&& ((uintptr_t) &_ebss >= (uintptr_t) &_sbss)
			&& ((uintptr_t) &_ebss <= (MEMORY_RAM_BASE + MEMORY_RAM_BYTES))
			&& ((uintptr_t) &_end >= (uintptr_t) &_ebss)
			&& ((uintptr_t) &_end
					< ((uintptr_t) &_estack - (uintptr_t) &_Min_Stack_Size))
			&& ((uintptr_t) &_sidata >= MEMORY_FLASH_BASE)
			&& (flash_end >= MEMORY_FLASH_BASE)
			&& (flash_end <= (MEMORY_FLASH_BASE + MEMORY_FLASH_BYTES))
			&& ((uintptr_t) &_estack == (MEMORY_RAM_BASE + MEMORY_RAM_BYTES)));
	snapshot->flash_used_bytes =
			snapshot->linker_layout_valid ?
					(uint32_t) (flash_end - MEMORY_FLASH_BASE) : 0U;

	stack_top = (uintptr_t) &_estack;
	stack_size = (uintptr_t) &_Min_Stack_Size;
	stack_bottom = stack_top - stack_size;
	heap_start = (uintptr_t) &_end;
	stack_limit = stack_bottom;

	snapshot->heap_min_reserve_bytes = (uint32_t) (uintptr_t) &_Min_Heap_Size;
	snapshot->stack_min_reserve_bytes = (uint32_t) stack_size;
	snapshot->heap_capacity_bytes =
			(stack_limit > heap_start) ?
					(uint32_t) (stack_limit - heap_start) : 0U;
	snapshot->heap_current_bytes = s_heap_current;
	snapshot->heap_peak_bytes = s_heap_peak;
	snapshot->heap_sbrk_calls = s_heap_sbrk_calls;
	snapshot->heap_failed_calls = s_heap_failed_calls;

	snapshot->stack_reserved_bytes = (uint32_t) stack_size;
	snapshot->stack_paint_valid = s_stack_paint_valid
			&& (s_stack_paint_start == stack_bottom)
			&& (s_stack_initial_sp > stack_bottom)
			&& (s_stack_initial_sp <= stack_top);
	snapshot->stack_used_peak_bytes = 0U;
	snapshot->stack_free_watermark_bytes = 0U;

	if (snapshot->stack_paint_valid) {
		first_modified = s_stack_paint_start;
		for (address = s_stack_paint_start; address < stack_top; address +=
				sizeof(uint32_t)) {
			if (*((volatile const uint32_t*) address) != MEMORY_STACK_PATTERN) {
				first_modified = address;
				break;
			}
		}

		/* The initial SP is the baseline occupied region not painted. */
		if (first_modified == stack_top) {
			first_modified = s_stack_initial_sp;
		} else if (first_modified > s_stack_initial_sp) {
			first_modified = s_stack_initial_sp;
		}

		stack_used =
				(first_modified < stack_top) ?
						(uint32_t) (stack_top - first_modified) : 0U;
		if (stack_used > (uint32_t) stack_size) {
			stack_used = (uint32_t) stack_size;
		}
		snapshot->stack_used_peak_bytes = stack_used;
		snapshot->stack_free_watermark_bytes = (uint32_t) stack_size
				- stack_used;
	}

	return snapshot->linker_layout_valid;
}
