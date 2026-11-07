#ifndef OV7670_DVP_DIAG_H
#define OV7670_DVP_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY 12U

typedef enum {
	OV7670_DVP_DIAG_OK = 0,
	OV7670_DVP_DIAG_BAD_ARGUMENT,
	OV7670_DVP_DIAG_BAD_COM10,
	OV7670_DVP_DIAG_XCLK_CONFIG_ERROR,
	OV7670_DVP_DIAG_TIM2_CONFIG_ERROR,
	OV7670_DVP_DIAG_TIM2_CLOCK_ERROR,
	OV7670_DVP_DIAG_TIM3_CONFIG_ERROR,
	OV7670_DVP_DIAG_EXTI_CONFIG_ERROR,
	OV7670_DVP_DIAG_DMA_START_ERROR,
	OV7670_DVP_DIAG_PCLK_TIMEOUT,
	OV7670_DVP_DIAG_DMA_TRANSFER_ERROR,
	OV7670_DVP_DIAG_PCLK_OUT_OF_RANGE,
	OV7670_DVP_DIAG_PCLK_INTERVAL_ERROR,
	OV7670_DVP_DIAG_HREF_MISSING,
	OV7670_DVP_DIAG_HREF_TIMING_ERROR,
	OV7670_DVP_DIAG_VSYNC_MISSING,
	OV7670_DVP_DIAG_VSYNC_LEVEL_ERROR,
	OV7670_DVP_DIAG_FRAME_LINES_ERROR,
	OV7670_DVP_DIAG_POLL_INTEGRITY_ERROR
} OV7670_DvpDiagStatus_t;

typedef struct {
	/* Total HREF pulses between consecutive VSYNC rising edges. */
	uint16_t href_lines;
	/* Classify each HREF rising edge by the OV7670 VGA frame timing window. */
	uint16_t leading_blank_href_lines;
	uint16_t active_image_href_lines;
	uint16_t trailing_blank_href_lines;
	uint8_t classification_valid;
	uint8_t vsync_level_high;
	uint8_t href_active_at_boundary;
	uint32_t period_us;
	uint32_t active_window_start_us;
	uint32_t active_window_end_us;
} OV7670_DvpFrameTraceEntry_t;

typedef struct {
	uint32_t apb1_timer_clock_hz;
	uint32_t apb2_timer_clock_hz;

	uint32_t tim2_counter_clock_hz;
	uint32_t tim2_probe_delta_ticks;
	uint16_t tim2_prescaler_used;
	uint8_t tim2_clock_ok;

	uint32_t xclk_hz_from_timer;
	uint16_t xclk_duty_permille;
	uint8_t xclk_timer_enabled;
	uint8_t xclk_channel_enabled;
	uint8_t xclk_main_output_enabled;
	uint8_t xclk_pin_alternate_function;

	uint8_t com10;
	uint8_t href_active_low;
	uint8_t vsync_active_low;
	uint8_t pclk_gated_by_href;
	uint8_t pclk_reversed;
	uint8_t com10_baseline_ok;

	uint32_t pclk_sample_count;
	uint32_t pclk_hz;
	uint16_t pclk_period_min_ticks;
	uint16_t pclk_period_max_ticks;
	uint32_t pclk_period_average_millTicks;
	uint16_t pclk_interval_outliers;
	uint8_t pclk_dma_overcapture;

	uint32_t href_active_pulses;
	uint32_t href_measured_pulses;
	uint32_t href_average_pclk_cycles_milli;
	uint32_t href_min_pclk_cycles_milli;
	uint32_t href_max_pclk_cycles_milli;
	uint32_t href_active_edge_callbacks;
	uint32_t href_inactive_edge_callbacks;
	uint32_t href_state_mismatch_callbacks;
	uint32_t href_state_mismatch_at_vsync;
	uint32_t href_lines_before_first_vsync;
	uint32_t href_lines_in_complete_intervals;
	uint32_t href_lines_after_last_vsync;
	uint8_t href_accounting_consistent;

	uint32_t vsync_rising_callbacks;
	uint32_t vsync_gpio_low_callbacks;
	uint32_t vsync_boundary_while_href_active;
	uint32_t complete_frames;
	uint16_t frame_lines_min;
	uint16_t frame_lines_max;
	uint16_t frame_line_trace_count;
	OV7670_DvpFrameTraceEntry_t frame_line_trace[OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY];
	uint32_t vsync_period_min_us;
	uint32_t vsync_period_average_us;
	uint32_t vsync_period_max_us;
	uint32_t frame_rate_millihz;

	uint32_t exti_imr_before;
	uint32_t exti_rtsr_before;
	uint32_t exti_ftsr_before;
	uint32_t exti_rtsr_after;
	uint32_t exti_ftsr_after;
	uint8_t exti_restore_ok;
	uint8_t exti_href_both_edges_before;
	uint8_t exti_vsync_rising_only_verified;
	uint8_t exti_vsync_rising_was_enabled;
	uint8_t exti_vsync_falling_was_enabled;
	uint8_t gpio_href_mode;
	uint8_t gpio_vsync_mode;
	uint8_t gpio_href_pupd;
	uint8_t gpio_vsync_pupd;

	uint8_t xclk_ok;
	uint8_t pclk_ok;
	uint8_t href_ok;
	uint8_t vsync_ok;
	uint8_t vsync_level_ok;
	uint8_t frame_lines_ok;

	/* Independent foreground GPIO sampling, concurrent with EXTI counting. */
	uint32_t poll_sample_count;
	uint32_t poll_max_gap_us;
	uint32_t poll_href_rising_edges;
	uint32_t poll_href_falling_edges;
	uint32_t poll_vsync_rising_edges;
	uint32_t poll_vsync_falling_edges;
	uint32_t poll_href_measured_pulses;
	uint32_t poll_href_width_min_us;
	uint32_t poll_href_width_average_us;
	uint32_t poll_href_width_max_us;
	uint32_t poll_vsync_high_min_us;
	uint32_t poll_vsync_high_average_us;
	uint32_t poll_vsync_high_max_us;
	/* Total HREF pulses (including vertical blanking) per VSYNC interval. */
	uint16_t poll_frame_lines_min;
	uint16_t poll_frame_lines_max;
	/* HREF pulses classified inside the expected 480-line active image window. */
	uint16_t poll_active_image_lines_min;
	uint16_t poll_active_image_lines_max;
	uint16_t poll_frame_trace_count;
	OV7670_DvpFrameTraceEntry_t poll_frame_trace[OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY];
	uint8_t poll_integrity_ok;
	uint8_t poll_frame_lines_ok;
	uint8_t exti_poll_href_edges_match;
	uint8_t exti_poll_vsync_edges_match;
	uint16_t exti_poll_frame_trace_mismatches;
} OV7670_DvpDiagReport_t;

/**
 * Observe HREF, VSYNC, and PCLK timing without changing camera registers.
 * TIM3_CH4 with DMA1 Stream2 measures PCLK periods, while TIM2 provides
 * timestamps. EXTI callbacks and foreground GPIO polling independently track
 * frame boundaries. The QQVGA active image window is expected to contain
 * 120 HREF pulses; blanking intervals may increase the total count. The EXTI
 * configuration is restored before returning. D0-D7 are not sampled.
 */
OV7670_DvpDiagStatus_t OV7670_DvpDiag_Run(uint8_t com10,
		OV7670_DvpDiagReport_t *report);

const char * OV7670_DvpDiag_StatusString(OV7670_DvpDiagStatus_t status);

#ifdef __cplusplus
}
#endif

#endif /* OV7670_DVP_DIAG_H */
