#include "drivers/ov7670_dvp_diag.h"
#include "drivers/ov7670_dvp_capture.h"

#include <stddef.h>
#include <string.h>

#include "main.h"
#include "tim.h"

/* STM32F411RE: TIM3_CH4 -> DMA1 Stream2 / Channel5. */
extern DMA_HandleTypeDef hdma_tim3_ch4_up;

#define DVP_COM10_PCLK_GATING        (1U << 5)
#define DVP_COM10_PCLK_REVERSE       (1U << 4)
#define DVP_COM10_HREF_REVERSE       (1U << 3)
#define DVP_COM10_VSYNC_NEGATIVE     (1U << 1)

#define DVP_PCLK_CAPTURE_COUNT       256U
#define DVP_PCLK_CAPTURE_DECIMATION  4U
#define DVP_PCLK_INTERVAL_TOLERANCE_PERCENT 25U
#define DVP_TIM_CCMR2_IC4PSC_MASK    (3UL << 10U)
#define DVP_TIM_CCMR2_IC4PSC_DIV4    (2UL << 10U)
#define DVP_PCLK_CAPTURE_TIMEOUT_MS  500U
#define DVP_SYNC_MONITOR_WINDOW_MS   2500U
#define DVP_TIM2_PROBE_DELAY_MS      50U
#define DVP_TIM2_PROBE_TOLERANCE_PCT 20U
/* Keep the polling gap below one quarter of the measured HREF pulse width
 * (~408 us) so bounded interrupt latency does not hide an edge transition. */
#define DVP_POLL_MAX_GAP_US           100U

#define DVP_EXPECTED_XCLK_HZ         12500000UL
#define DVP_PCLK_MIN_HZ              100000UL
#define DVP_PCLK_MAX_HZ              25000000UL
#define DVP_LINE_CYCLES_MIN          280U
#define DVP_LINE_CYCLES_MAX          360U
#define DVP_MIN_MEASURED_LINES       100U
/* OV7670 VGA timing: 3 VSYNC lines + 17 front-porch lines + 480 active
 * lines + 10 trailing lines = 510 source lines. QQVGA is 1-from-4 => 120. */
#define DVP_VGA_FRAME_LINES           510U
#define DVP_VGA_ACTIVE_FIRST_LINE     20U
#define DVP_VGA_ACTIVE_LINE_COUNT     480U
#define DVP_VGA_ACTIVE_END_LINE       (DVP_VGA_ACTIVE_FIRST_LINE + DVP_VGA_ACTIVE_LINE_COUNT)
#define DVP_QQVGA_VERTICAL_DECIMATION 4U
#define DVP_EXPECTED_ACTIVE_HREF_LINES 120U
#define DVP_MIN_CLASSIFIED_FRAMES      3U
#define DVP_MAX_HREF_EVENTS_PER_FRAME  160U

#define DVP_EXTI_CAMERA_MASK         ((uint32_t)CAM_HREF_Pin | (uint32_t)CAM_VSYNC_Pin)

typedef struct {
	uint32_t rtsr_bit;
	uint32_t ftsr_bit;
	uint8_t valid;
} DvpVsyncExtiState_t;

typedef struct {
	uint32_t sample_count;
	uint32_t max_gap_ticks;
	uint32_t href_rising_edges;
	uint32_t href_falling_edges;
	uint32_t href_measured_pulses;
	uint32_t href_start_tick;
	uint64_t href_width_sum_ticks;
	uint32_t href_width_min_ticks;
	uint32_t href_width_max_ticks;
	uint8_t href_measure_started;
	uint32_t vsync_rising_edges;
	uint32_t vsync_falling_edges;
	uint32_t vsync_high_samples;
	uint64_t vsync_high_sum_ticks;
	uint32_t vsync_high_min_ticks;
	uint32_t vsync_high_max_ticks;
	uint32_t vsync_last_rising_tick;
	uint8_t vsync_have_rising;
	uint16_t lines_since_vsync_rising;
	uint16_t frame_lines_min;
	uint16_t frame_lines_max;
	uint16_t frame_trace_count;
	uint16_t active_image_lines_min;
	uint16_t active_image_lines_max;
	uint16_t frame_href_offset_count;
	uint8_t frame_href_offset_overflow;
	uint32_t frame_href_offsets[DVP_MAX_HREF_EVENTS_PER_FRAME];
	OV7670_DvpFrameTraceEntry_t frame_trace[OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY];
} DvpPollStats_t;

static volatile uint32_t s_pclk_capture[DVP_PCLK_CAPTURE_COUNT];
static volatile uint8_t s_dma_complete;
static volatile uint8_t s_dma_error;
static volatile uint8_t s_monitor_active;

static volatile GPIO_PinState s_href_active_level;
static volatile uint8_t s_href_is_active;
static volatile uint8_t s_href_measure_started;
static volatile uint32_t s_href_start_tick;
static volatile uint32_t s_href_active_pulses;
static volatile uint32_t s_href_measured_pulses;
static volatile uint32_t s_href_active_edge_callbacks;
static volatile uint32_t s_href_inactive_edge_callbacks;
static volatile uint32_t s_href_state_mismatch_callbacks;
static volatile uint32_t s_href_state_mismatch_at_vsync;
static volatile uint32_t s_href_lines_before_first_vsync;
static volatile uint32_t s_href_lines_in_complete_intervals;
static volatile uint32_t s_tim2_counter_clock_hz;
static volatile uint32_t s_href_min_width_ticks;
static volatile uint32_t s_href_max_width_ticks;
static volatile uint64_t s_href_sum_width_ticks;

static volatile uint32_t s_vsync_rising_callbacks;
static volatile uint32_t s_vsync_gpio_low_callbacks;
static volatile uint32_t s_vsync_boundary_while_href_active;
static volatile uint32_t s_vsync_last_rising_tick;
static volatile uint8_t s_vsync_have_last_rising;
static volatile uint32_t s_vsync_period_min_ticks;
static volatile uint32_t s_vsync_period_max_ticks;
static volatile uint64_t s_vsync_period_sum_ticks;
static volatile uint32_t s_complete_frames;
static volatile uint16_t s_lines_since_vsync_rising;
static volatile uint16_t s_frame_lines_min;
static volatile uint16_t s_frame_lines_max;
static volatile uint16_t s_frame_line_trace_count;
static volatile OV7670_DvpFrameTraceEntry_t s_frame_line_trace[OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY];

static uint32_t apb1_timer_clock_hz(void) {
	uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();

	/* STM32F4 doubles the APB timer clock when the APB prescaler is not 1. */
	if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) {
		clock_hz *= 2U;
	}
	return clock_hz;
}

static uint32_t apb2_timer_clock_hz(void) {
	uint32_t clock_hz = HAL_RCC_GetPCLK2Freq();

	if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) {
		clock_hz *= 2U;
	}
	return clock_hz;
}

static void reset_sync_statistics(GPIO_PinState href_active_level,
		GPIO_PinState href_initial_state) {
	s_href_active_level = href_active_level;
	s_href_is_active = (uint8_t) (href_initial_state == href_active_level);
	/* If the monitor starts mid-pulse, do not count that partial pulse. */
	s_href_measure_started = 0U;
	s_href_start_tick = 0U;
	s_href_active_pulses = 0U;
	s_href_measured_pulses = 0U;
	s_href_active_edge_callbacks = 0U;
	s_href_inactive_edge_callbacks = 0U;
	s_href_state_mismatch_callbacks = 0U;
	s_href_state_mismatch_at_vsync = 0U;
	s_href_lines_before_first_vsync = 0U;
	s_href_lines_in_complete_intervals = 0U;
	s_href_min_width_ticks = 0xFFFFFFFFUL;
	s_href_max_width_ticks = 0U;
	s_href_sum_width_ticks = 0ULL;

	s_vsync_rising_callbacks = 0U;
	s_vsync_gpio_low_callbacks = 0U;
	s_vsync_boundary_while_href_active = 0U;
	s_vsync_last_rising_tick = 0U;
	s_vsync_have_last_rising = 0U;
	s_vsync_period_min_ticks = 0xFFFFFFFFUL;
	s_vsync_period_max_ticks = 0U;
	s_vsync_period_sum_ticks = 0ULL;
	s_complete_frames = 0U;
	s_lines_since_vsync_rising = 0U;
	s_frame_lines_min = 0xFFFFU;
	s_frame_lines_max = 0U;
	s_frame_line_trace_count = 0U;
	memset((void*) s_frame_line_trace, 0, sizeof(s_frame_line_trace));
}

static OV7670_DvpDiagStatus_t start_tim2_timestamp(uint32_t prescaler,
		uint32_t *counter_clock_hz) {
	uint32_t timer_clock;

	if ((counter_clock_hz == NULL)
			|| ((htim2.Instance->CR1 & TIM_CR1_UDIS) != 0U)
			|| (prescaler == 0xFFFFFFFFUL)) {
		return OV7670_DVP_DIAG_TIM2_CONFIG_ERROR;
	}

	timer_clock = apb1_timer_clock_hz();
	if (timer_clock == 0U) {
		return OV7670_DVP_DIAG_TIM2_CONFIG_ERROR;
	}

	__HAL_TIM_DISABLE(&htim2);
	__HAL_TIM_SET_PRESCALER(&htim2, prescaler);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 0xFFFFFFFFUL);
	__HAL_TIM_SET_COUNTER(&htim2, 0U);

	/* PSC is buffered. Apply it via the documented HAL software update event. */
	if (HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
		return OV7670_DVP_DIAG_TIM2_CONFIG_ERROR;
	}

	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim2, 0U);
	__HAL_TIM_ENABLE(&htim2);

	*counter_clock_hz = timer_clock / (prescaler + 1U);
	if (*counter_clock_hz == 0U) {
		__HAL_TIM_DISABLE(&htim2);
		return OV7670_DVP_DIAG_TIM2_CONFIG_ERROR;
	}
	return OV7670_DVP_DIAG_OK;
}

static OV7670_DvpDiagStatus_t stop_tim2_timestamp(uint32_t old_prescaler,
		uint32_t old_arr, uint32_t old_counter, uint8_t was_enabled) {
	HAL_StatusTypeDef hal_status;

	__HAL_TIM_DISABLE(&htim2);
	__HAL_TIM_SET_PRESCALER(&htim2, old_prescaler);
	__HAL_TIM_SET_AUTORELOAD(&htim2, old_arr);
	__HAL_TIM_SET_COUNTER(&htim2, 0U);
	hal_status = HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim2, old_counter);
	if (was_enabled != 0U) {
		__HAL_TIM_ENABLE(&htim2);
	}
	return (hal_status == HAL_OK) ?
			OV7670_DVP_DIAG_OK : OV7670_DVP_DIAG_TIM2_CONFIG_ERROR;
}

static HAL_StatusTypeDef prepare_tim3_capture(void) {
	__HAL_TIM_DISABLE(&htim3);
	__HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_CC4);
	htim3.Instance->CCER &= ~TIM_CCER_CC4E;
	__HAL_TIM_SET_PRESCALER(&htim3, 0U);
	__HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFFU);
	__HAL_TIM_SET_COUNTER(&htim3, 0U);
	htim3.Instance->CCMR2 = (htim3.Instance->CCMR2 & ~DVP_TIM_CCMR2_IC4PSC_MASK)
			|
			DVP_TIM_CCMR2_IC4PSC_DIV4;
	if (HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
		return HAL_ERROR;
	}
	htim3.Instance->SR = 0U;
	return HAL_OK;
}

static HAL_StatusTypeDef restore_tim3(uint32_t old_prescaler, uint32_t old_arr,
		uint32_t old_counter, uint32_t old_ccmr2, uint32_t old_ccer,
		uint32_t old_dier, uint8_t was_enabled) {
	HAL_StatusTypeDef hal_status;

	__HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_CC4);
	__HAL_TIM_DISABLE(&htim3);
	htim3.Instance->CCER &= ~TIM_CCER_CC4E;
	__HAL_TIM_SET_PRESCALER(&htim3, old_prescaler);
	__HAL_TIM_SET_AUTORELOAD(&htim3, old_arr);
	htim3.Instance->CCMR2 = old_ccmr2;
	__HAL_TIM_SET_COUNTER(&htim3, 0U);
	hal_status = HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim3, old_counter);
	htim3.Instance->CCER = old_ccer;
	htim3.Instance->DIER = old_dier;
	htim3.Instance->SR = 0U;
	if (was_enabled != 0U) {
		__HAL_TIM_ENABLE(&htim3);
	}
	return hal_status;
}

static void dvp_dma_complete(DMA_HandleTypeDef *hdma) {
	if (hdma == &hdma_tim3_ch4_up) {
		__HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_CC4);
		htim3.Instance->CCER &= ~TIM_CCER_CC4E;
		s_dma_complete = 1U;
	}
}

static void dvp_dma_error(DMA_HandleTypeDef *hdma) {
	if (hdma == &hdma_tim3_ch4_up) {
		__HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_CC4);
		htim3.Instance->CCER &= ~TIM_CCER_CC4E;
		s_dma_error = 1U;
	}
}

static OV7670_DvpDiagStatus_t capture_pclk_periods(uint32_t *timer_clock_hz,
		uint64_t *period_sum_ticks, uint16_t *period_min_ticks,
		uint16_t *period_max_ticks, uint16_t *interval_outlier_count,
		uint8_t *overcapture) {
	HAL_StatusTypeDef hal_status;
	HAL_StatusTypeDef restore_status;
	uint32_t start_tick;
	uint32_t old_tim3_psc = htim3.Instance->PSC;
	uint32_t old_tim3_arr = htim3.Instance->ARR;
	uint32_t old_tim3_counter = htim3.Instance->CNT;
	uint32_t old_tim3_ccmr2 = htim3.Instance->CCMR2;
	uint32_t old_tim3_ccer = htim3.Instance->CCER;
	uint32_t old_tim3_dier = htim3.Instance->DIER;
	uint8_t tim3_was_enabled = (uint8_t) ((htim3.Instance->CR1 & TIM_CR1_CEN)
			!= 0U);
	void (*old_complete_callback)(
			DMA_HandleTypeDef*) = hdma_tim3_ch4_up.XferCpltCallback;
	void (*old_error_callback)(
			DMA_HandleTypeDef*) = hdma_tim3_ch4_up.XferErrorCallback;
	uint32_t i;
	uint64_t sum = 0ULL;
	uint64_t average_interval;
	uint64_t lower_interval_limit;
	uint64_t upper_interval_limit;
	uint16_t min_period = 0xFFFFU;
	uint16_t max_period = 0U;
	uint16_t interval_outliers = 0U;
	OV7670_DvpDiagStatus_t result = OV7670_DVP_DIAG_OK;

	s_dma_complete = 0U;
	s_dma_error = 0U;

	hal_status = prepare_tim3_capture();
	if (hal_status != HAL_OK) {
		(void) restore_tim3(old_tim3_psc, old_tim3_arr, old_tim3_counter,
				old_tim3_ccmr2, old_tim3_ccer, old_tim3_dier, tim3_was_enabled);
		return OV7670_DVP_DIAG_TIM3_CONFIG_ERROR;
	}

	*timer_clock_hz = apb1_timer_clock_hz();
	if (*timer_clock_hz == 0U) {
		(void) restore_tim3(old_tim3_psc, old_tim3_arr, old_tim3_counter,
				old_tim3_ccmr2, old_tim3_ccer, old_tim3_dier, tim3_was_enabled);
		return OV7670_DVP_DIAG_PCLK_OUT_OF_RANGE;
	}

	hdma_tim3_ch4_up.XferCpltCallback = dvp_dma_complete;
	hdma_tim3_ch4_up.XferErrorCallback = dvp_dma_error;
	hal_status = HAL_DMA_Start_IT(&hdma_tim3_ch4_up,
			(uint32_t) (uintptr_t) &htim3.Instance->CCR4,
			(uint32_t) (uintptr_t) &s_pclk_capture[0],
			DVP_PCLK_CAPTURE_COUNT);
	if (hal_status != HAL_OK) {
		result = OV7670_DVP_DIAG_DMA_START_ERROR;
		goto cleanup;
	}

	__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC4);
	hal_status = HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_4);
	if (hal_status != HAL_OK) {
		__HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_CC4);
		(void) HAL_DMA_Abort(&hdma_tim3_ch4_up);
		result = OV7670_DVP_DIAG_DMA_START_ERROR;
		goto cleanup;
	}

	start_tick = HAL_GetTick();
	while ((s_dma_complete == 0U) && (s_dma_error == 0U)
			&& ((HAL_GetTick() - start_tick) < DVP_PCLK_CAPTURE_TIMEOUT_MS)) {
		/* Bounded foreground wait. DMA completion uses DMA1_Stream2_IRQHandler. */
	}

	__HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_CC4);
	(void) HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_4);
	if (s_dma_complete == 0U) {
		if (hdma_tim3_ch4_up.State != HAL_DMA_STATE_READY) {
			(void) HAL_DMA_Abort(&hdma_tim3_ch4_up);
		}
		result =
				(s_dma_error != 0U) ?
						OV7670_DVP_DIAG_DMA_TRANSFER_ERROR :
						OV7670_DVP_DIAG_PCLK_TIMEOUT;
		goto cleanup;
	}

	__DMB();
	*overcapture = (uint8_t) ((htim3.Instance->SR & TIM_SR_CC4OF) != 0U);

	for (i = 1U; i < DVP_PCLK_CAPTURE_COUNT; ++i) {
		const uint16_t previous = (uint16_t) s_pclk_capture[i - 1U];
		const uint16_t current = (uint16_t) s_pclk_capture[i];
		const uint16_t delta = (uint16_t) (current - previous);
		const uint16_t pclk_period = (uint16_t) ((delta
				+ (DVP_PCLK_CAPTURE_DECIMATION / 2U))
				/ DVP_PCLK_CAPTURE_DECIMATION);

		if (delta == 0U) {
			result = OV7670_DVP_DIAG_PCLK_OUT_OF_RANGE;
			goto cleanup;
		}
		if (pclk_period < min_period) {
			min_period = pclk_period;
		}
		if (pclk_period > max_period) {
			max_period = pclk_period;
		}
		sum += delta;
	}

	average_interval = sum / (DVP_PCLK_CAPTURE_COUNT - 1U);
	lower_interval_limit = (average_interval
			* (100U - DVP_PCLK_INTERVAL_TOLERANCE_PERCENT)) / 100U;
	upper_interval_limit = (average_interval
			* (100U + DVP_PCLK_INTERVAL_TOLERANCE_PERCENT)) / 100U;

	for (i = 1U; i < DVP_PCLK_CAPTURE_COUNT; ++i) {
		const uint16_t previous = (uint16_t) s_pclk_capture[i - 1U];
		const uint16_t current = (uint16_t) s_pclk_capture[i];
		const uint16_t delta = (uint16_t) (current - previous);
		if (((uint64_t) delta < lower_interval_limit)
				|| ((uint64_t) delta > upper_interval_limit)) {
			interval_outliers++;
		}
	}

	*period_sum_ticks = sum;
	*period_min_ticks = min_period;
	*period_max_ticks = max_period;
	*interval_outlier_count = interval_outliers;

	cleanup: hdma_tim3_ch4_up.XferCpltCallback = old_complete_callback;
	hdma_tim3_ch4_up.XferErrorCallback = old_error_callback;
	restore_status = restore_tim3(old_tim3_psc, old_tim3_arr, old_tim3_counter,
			old_tim3_ccmr2, old_tim3_ccer, old_tim3_dier, tim3_was_enabled);
	if (restore_status != HAL_OK) {
		return OV7670_DVP_DIAG_TIM3_CONFIG_ERROR;
	}
	return result;
}

static void calculate_xclk_report(OV7670_DvpDiagReport_t *report) {
	const uint32_t gpio_mode = (GPIOA->MODER >> (8U * 2U)) & 0x03U;
	const uint32_t gpio_speed = (GPIOA->OSPEEDR >> (8U * 2U)) & 0x03U;
	const uint32_t gpio_af = GPIOA->AFR[1] & 0x0FU;
	const uint32_t arr = htim1.Instance->ARR;
	const uint32_t psc = htim1.Instance->PSC;
	const uint64_t period = ((uint64_t) arr + 1ULL) * ((uint64_t) psc + 1ULL);
	const uint32_t pulse = htim1.Instance->CCR1;

	report->apb2_timer_clock_hz = apb2_timer_clock_hz();
	report->xclk_timer_enabled = (uint8_t) ((htim1.Instance->CR1 & TIM_CR1_CEN)
			!= 0U);
	report->xclk_channel_enabled = (uint8_t) ((htim1.Instance->CCER
			& TIM_CCER_CC1E) != 0U);
	report->xclk_main_output_enabled = (uint8_t) ((htim1.Instance->BDTR
			& TIM_BDTR_MOE) != 0U);
	report->xclk_pin_alternate_function = (uint8_t) ((gpio_mode == 2U)
			&& (gpio_af == GPIO_AF1_TIM1) && (gpio_speed >= 2U));

	if ((period != 0ULL) && (report->apb2_timer_clock_hz != 0U)) {
		report->xclk_hz_from_timer = (uint32_t) (report->apb2_timer_clock_hz
				/ period);
		report->xclk_duty_permille = (uint16_t) (((uint64_t) pulse * 1000ULL)
				/ ((uint64_t) arr + 1ULL));
	}

	report->xclk_ok = (uint8_t) ((report->xclk_timer_enabled != 0U)
			&& (report->xclk_channel_enabled != 0U)
			&& (report->xclk_main_output_enabled != 0U)
			&& (report->xclk_pin_alternate_function != 0U)
			&& (report->xclk_hz_from_timer == DVP_EXPECTED_XCLK_HZ)
			&& (report->xclk_duty_permille >= 450U)
			&& (report->xclk_duty_permille <= 550U));
}

static uint32_t timer_ticks_to_us(uint32_t ticks, uint32_t timer_clock_hz) {
	if (timer_clock_hz == 0U) {
		return 0U;
	}
	return (uint32_t) (((uint64_t) ticks * 1000000ULL) / timer_clock_hz);
}

static void configure_vsync_rising_only(OV7670_DvpDiagReport_t *report,
		DvpVsyncExtiState_t *saved_state) {
	uint32_t primask;
	const uint32_t href_mask = (uint32_t) CAM_HREF_Pin;
	const uint32_t vsync_mask = (uint32_t) CAM_VSYNC_Pin;

	primask = __get_PRIMASK();
	__disable_irq();

	report->exti_imr_before = EXTI->IMR & DVP_EXTI_CAMERA_MASK;
	report->exti_rtsr_before = EXTI->RTSR & DVP_EXTI_CAMERA_MASK;
	report->exti_ftsr_before = EXTI->FTSR & DVP_EXTI_CAMERA_MASK;
	report->exti_href_both_edges_before = (uint8_t) (((EXTI->IMR & href_mask)
			!= 0U) && ((EXTI->RTSR & href_mask) != 0U)
			&& ((EXTI->FTSR & href_mask) != 0U));

	saved_state->rtsr_bit = EXTI->RTSR & vsync_mask;
	saved_state->ftsr_bit = EXTI->FTSR & vsync_mask;
	saved_state->valid = 1U;
	report->exti_vsync_rising_was_enabled = (uint8_t) (saved_state->rtsr_bit
			!= 0U);
	report->exti_vsync_falling_was_enabled = (uint8_t) (saved_state->ftsr_bit
			!= 0U);

	/* Do not change EXTI IMR or PC11. Only make PC12 rising-edge-triggered. */
	EXTI->RTSR |= vsync_mask;
	EXTI->FTSR &= ~vsync_mask;
	EXTI->PR = vsync_mask;

	report->exti_vsync_rising_only_verified =
			(uint8_t) (((EXTI->IMR & vsync_mask) != 0U)
					&& ((EXTI->RTSR & vsync_mask) != 0U)
					&& ((EXTI->FTSR & vsync_mask) == 0U)
					&& (report->exti_href_both_edges_before != 0U)
					&& (report->gpio_href_mode == 0U)
					&& (report->gpio_vsync_mode == 0U));

	__set_PRIMASK(primask);
}

static void restore_vsync_exti(OV7670_DvpDiagReport_t *report,
		const DvpVsyncExtiState_t *saved_state) {
	uint32_t primask;
	const uint32_t vsync_mask = (uint32_t) CAM_VSYNC_Pin;

	if ((saved_state == NULL) || (saved_state->valid == 0U)) {
		report->exti_restore_ok = 0U;
		return;
	}

	primask = __get_PRIMASK();
	__disable_irq();

	if (saved_state->rtsr_bit != 0U) {
		EXTI->RTSR |= vsync_mask;
	} else {
		EXTI->RTSR &= ~vsync_mask;
	}
	if (saved_state->ftsr_bit != 0U) {
		EXTI->FTSR |= vsync_mask;
	} else {
		EXTI->FTSR &= ~vsync_mask;
	}
	EXTI->PR = vsync_mask;

	report->exti_rtsr_after = EXTI->RTSR & DVP_EXTI_CAMERA_MASK;
	report->exti_ftsr_after = EXTI->FTSR & DVP_EXTI_CAMERA_MASK;
	report->exti_restore_ok =
			(uint8_t) (((report->exti_rtsr_after & vsync_mask)
					== saved_state->rtsr_bit)
					&& ((report->exti_ftsr_after & vsync_mask)
							== saved_state->ftsr_bit));

	__set_PRIMASK(primask);
}

static void poll_camera_gpio_edges(DvpPollStats_t *stats, uint32_t window_ms,
		GPIO_PinState href_active_level, uint32_t counter_clock_hz) {
	uint32_t start_tick;
	uint32_t last_sample_tick;
	uint32_t previous_idr;
	GPIO_PinState previous_href;
	GPIO_PinState previous_vsync;
	const uint32_t window_ticks = (uint32_t) (((uint64_t) counter_clock_hz
			* window_ms) / 1000ULL);

	if ((stats == NULL) || (window_ticks == 0U) || (counter_clock_hz == 0U)) {
		return;
	}

	memset(stats, 0, sizeof(*stats));
	stats->href_width_min_ticks = 0xFFFFFFFFUL;
	stats->vsync_high_min_ticks = 0xFFFFFFFFUL;
	stats->frame_lines_min = 0xFFFFU;
	stats->active_image_lines_min = 0xFFFFU;
	stats->frame_href_offset_count = 0U;
	stats->frame_href_offset_overflow = 0U;

	start_tick = TIM2->CNT;
	last_sample_tick = start_tick;
	previous_idr = GPIOC->IDR;
	previous_href =
			((previous_idr & (uint32_t) CAM_HREF_Pin) != 0U) ?
					GPIO_PIN_SET : GPIO_PIN_RESET;
	previous_vsync =
			((previous_idr & (uint32_t) CAM_VSYNC_Pin) != 0U) ?
					GPIO_PIN_SET : GPIO_PIN_RESET;

	while ((uint32_t) (TIM2->CNT - start_tick) < window_ticks) {
		const uint32_t now_tick = TIM2->CNT;
		const uint32_t idr = GPIOC->IDR;
		const uint32_t gap_ticks = now_tick - last_sample_tick;
		const GPIO_PinState href_now =
				((idr & (uint32_t) CAM_HREF_Pin) != 0U) ?
						GPIO_PIN_SET : GPIO_PIN_RESET;
		const GPIO_PinState vsync_now =
				((idr & (uint32_t) CAM_VSYNC_Pin) != 0U) ?
						GPIO_PIN_SET : GPIO_PIN_RESET;

		stats->sample_count++;
		if (gap_ticks > stats->max_gap_ticks) {
			stats->max_gap_ticks = gap_ticks;
		}

		/* Process HREF before VSYNC if both levels changed between samples. */
		if (href_now != previous_href) {
			if (href_now == href_active_level) {
				stats->href_rising_edges++;
				stats->href_start_tick = now_tick;
				stats->href_measure_started = 1U;
				if (stats->vsync_have_rising != 0U) {
					stats->lines_since_vsync_rising++;
					if (stats->frame_href_offset_count
							< DVP_MAX_HREF_EVENTS_PER_FRAME) {
						stats->frame_href_offsets[stats->frame_href_offset_count] =
								now_tick - stats->vsync_last_rising_tick;
						stats->frame_href_offset_count++;
					} else {
						stats->frame_href_offset_overflow = 1U;
					}
				}
			} else {
				stats->href_falling_edges++;
				if (stats->href_measure_started != 0U) {
					const uint32_t width_ticks = now_tick
							- stats->href_start_tick;
					stats->href_width_sum_ticks += width_ticks;
					stats->href_measured_pulses++;
					if (width_ticks < stats->href_width_min_ticks) {
						stats->href_width_min_ticks = width_ticks;
					}
					if (width_ticks > stats->href_width_max_ticks) {
						stats->href_width_max_ticks = width_ticks;
					}
					stats->href_measure_started = 0U;
				}
			}
			previous_href = href_now;
		}

		if (vsync_now != previous_vsync) {
			if (vsync_now == GPIO_PIN_SET) {
				stats->vsync_rising_edges++;
				if (stats->vsync_have_rising != 0U) {
					const uint32_t period_ticks = now_tick
							- stats->vsync_last_rising_tick;
					const uint16_t frame_lines = stats->lines_since_vsync_rising;
					const uint32_t active_start_ticks =
							(uint32_t) (((uint64_t) period_ticks
									* DVP_VGA_ACTIVE_FIRST_LINE) /
							DVP_VGA_FRAME_LINES);
					const uint32_t active_end_ticks =
							(uint32_t) (((uint64_t) period_ticks
									* DVP_VGA_ACTIVE_END_LINE) /
							DVP_VGA_FRAME_LINES);
					uint16_t leading_blank = 0U;
					uint16_t active_image = 0U;
					uint16_t trailing_blank = 0U;
					uint16_t j;

					for (j = 0U; j < stats->frame_href_offset_count; ++j) {
						const uint32_t href_offset =
								stats->frame_href_offsets[j];
						if (href_offset < active_start_ticks) {
							leading_blank++;
						} else if (href_offset < active_end_ticks) {
							active_image++;
						} else {
							trailing_blank++;
						}
					}

					if (frame_lines < stats->frame_lines_min) {
						stats->frame_lines_min = frame_lines;
					}
					if (frame_lines > stats->frame_lines_max) {
						stats->frame_lines_max = frame_lines;
					}
					if (stats->frame_href_offset_overflow == 0U) {
						if (active_image < stats->active_image_lines_min) {
							stats->active_image_lines_min = active_image;
						}
						if (active_image > stats->active_image_lines_max) {
							stats->active_image_lines_max = active_image;
						}
					}
					if (stats->frame_trace_count <
					OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY) {
						OV7670_DvpFrameTraceEntry_t *entry =
								&stats->frame_trace[stats->frame_trace_count];
						entry->href_lines = frame_lines;
						entry->leading_blank_href_lines = leading_blank;
						entry->active_image_href_lines = active_image;
						entry->trailing_blank_href_lines = trailing_blank;
						entry->classification_valid =
								(uint8_t) (stats->frame_href_offset_overflow
										== 0U);
						entry->vsync_level_high = 1U;
						entry->href_active_at_boundary = (uint8_t) (href_now
								== href_active_level);
						entry->period_us = timer_ticks_to_us(period_ticks,
								counter_clock_hz);
						entry->active_window_start_us = timer_ticks_to_us(
								active_start_ticks, counter_clock_hz);
						entry->active_window_end_us = timer_ticks_to_us(
								active_end_ticks, counter_clock_hz);
						stats->frame_trace_count++;
					}
				}
				stats->vsync_last_rising_tick = now_tick;
				stats->vsync_have_rising = 1U;
				stats->lines_since_vsync_rising = 0U;
				stats->frame_href_offset_count = 0U;
				stats->frame_href_offset_overflow = 0U;
			} else {
				stats->vsync_falling_edges++;
				if (stats->vsync_have_rising != 0U) {
					const uint32_t high_ticks = now_tick
							- stats->vsync_last_rising_tick;
					stats->vsync_high_sum_ticks += high_ticks;
					stats->vsync_high_samples++;
					if (high_ticks < stats->vsync_high_min_ticks) {
						stats->vsync_high_min_ticks = high_ticks;
					}
					if (high_ticks > stats->vsync_high_max_ticks) {
						stats->vsync_high_max_ticks = high_ticks;
					}
				}
			}
			previous_vsync = vsync_now;
		}

		last_sample_tick = now_tick;
	}
}

static void calculate_report_metrics(OV7670_DvpDiagReport_t *report,
		uint64_t pclk_period_sum_ticks, uint32_t pclk_timer_clock_hz) {
	const uint64_t pclk_intervals = DVP_PCLK_CAPTURE_COUNT - 1U;
	const uint64_t href_count = s_href_measured_pulses;
	uint16_t i;

	report->href_active_pulses = s_href_active_pulses;
	report->href_measured_pulses = s_href_measured_pulses;
	report->href_active_edge_callbacks = s_href_active_edge_callbacks;
	report->href_inactive_edge_callbacks = s_href_inactive_edge_callbacks;
	report->href_state_mismatch_callbacks = s_href_state_mismatch_callbacks;
	report->href_state_mismatch_at_vsync = s_href_state_mismatch_at_vsync;
	report->href_lines_before_first_vsync = s_href_lines_before_first_vsync;
	report->href_lines_in_complete_intervals =
			s_href_lines_in_complete_intervals;
	report->href_lines_after_last_vsync = s_lines_since_vsync_rising;
	report->href_accounting_consistent =
			(uint8_t) (((uint64_t) report->href_active_pulses)
					== ((uint64_t) report->href_lines_before_first_vsync
							+ (uint64_t) report->href_lines_in_complete_intervals
							+ (uint64_t) report->href_lines_after_last_vsync));

	report->vsync_rising_callbacks = s_vsync_rising_callbacks;
	report->vsync_gpio_low_callbacks = s_vsync_gpio_low_callbacks;
	report->vsync_boundary_while_href_active =
			s_vsync_boundary_while_href_active;
	report->complete_frames = s_complete_frames;
	report->frame_lines_min =
			(s_frame_lines_min == 0xFFFFU) ? 0U : s_frame_lines_min;
	report->frame_lines_max = s_frame_lines_max;
	report->frame_line_trace_count = s_frame_line_trace_count;
	if (report->frame_line_trace_count > OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY) {
		report->frame_line_trace_count = OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY;
	}
	for (i = 0U; i < report->frame_line_trace_count; ++i) {
		report->frame_line_trace[i] = s_frame_line_trace[i];
	}

	if ((pclk_period_sum_ticks != 0ULL) && (report->pclk_hz != 0U)
			&& (report->tim2_counter_clock_hz != 0U)) {
		report->pclk_period_average_millTicks =
				(uint32_t) ((pclk_period_sum_ticks * 1000ULL)
						/ (pclk_intervals * DVP_PCLK_CAPTURE_DECIMATION));

		if (href_count != 0ULL) {
			const uint64_t href_scale = pclk_intervals *
			DVP_PCLK_CAPTURE_DECIMATION * (uint64_t) pclk_timer_clock_hz
					* 1000ULL;
			const uint64_t denominator = pclk_period_sum_ticks
					* (uint64_t) report->tim2_counter_clock_hz;
			const uint64_t average_width_ticks = s_href_sum_width_ticks
					/ href_count;

			report->href_average_pclk_cycles_milli =
					(uint32_t) ((average_width_ticks * pclk_intervals *
					DVP_PCLK_CAPTURE_DECIMATION * (uint64_t) pclk_timer_clock_hz
							* 1000ULL) / denominator);
			report->href_min_pclk_cycles_milli =
					(uint32_t) (((uint64_t) s_href_min_width_ticks * href_scale)
							/ denominator);
			report->href_max_pclk_cycles_milli =
					(uint32_t) (((uint64_t) s_href_max_width_ticks * href_scale)
							/ denominator);
		}
	}

	if ((s_complete_frames != 0U) && (s_vsync_period_sum_ticks != 0ULL)
			&& (report->tim2_counter_clock_hz != 0U)) {
		report->frame_rate_millihz =
				(uint32_t) (((uint64_t) report->tim2_counter_clock_hz * 1000ULL
						* s_complete_frames) / s_vsync_period_sum_ticks);
		report->vsync_period_average_us =
				(uint32_t) ((s_vsync_period_sum_ticks * 1000000ULL)
						/ ((uint64_t) s_complete_frames
								* report->tim2_counter_clock_hz));
		report->vsync_period_min_us = timer_ticks_to_us(
				s_vsync_period_min_ticks, report->tim2_counter_clock_hz);
		report->vsync_period_max_us = timer_ticks_to_us(
				s_vsync_period_max_ticks, report->tim2_counter_clock_hz);
	}
}

static void copy_poll_stats_to_report(OV7670_DvpDiagReport_t *report,
		const DvpPollStats_t *stats, uint32_t counter_clock_hz) {
	uint16_t i;
	uint32_t poll_href_rising_delta;
	uint32_t poll_href_falling_delta;
	uint32_t poll_vsync_delta;
	uint16_t trace_common_count;

	report->poll_sample_count = stats->sample_count;
	report->poll_max_gap_us = timer_ticks_to_us(stats->max_gap_ticks,
			counter_clock_hz);
	report->poll_href_rising_edges = stats->href_rising_edges;
	report->poll_href_falling_edges = stats->href_falling_edges;
	report->poll_vsync_rising_edges = stats->vsync_rising_edges;
	report->poll_vsync_falling_edges = stats->vsync_falling_edges;
	report->poll_href_measured_pulses = stats->href_measured_pulses;
	report->poll_href_width_min_us =
			(stats->href_width_min_ticks == 0xFFFFFFFFUL) ?
					0U :
					timer_ticks_to_us(stats->href_width_min_ticks,
							counter_clock_hz);
	report->poll_href_width_average_us =
			((stats->href_measured_pulses != 0U) && (counter_clock_hz != 0U)) ?
					(uint32_t) ((stats->href_width_sum_ticks * 1000000ULL)
							/ ((uint64_t) stats->href_measured_pulses
									* counter_clock_hz)) :
					0U;
	report->poll_href_width_max_us = timer_ticks_to_us(
			stats->href_width_max_ticks, counter_clock_hz);
	report->poll_vsync_high_min_us =
			(stats->vsync_high_min_ticks == 0xFFFFFFFFUL) ?
					0U :
					timer_ticks_to_us(stats->vsync_high_min_ticks,
							counter_clock_hz);
	report->poll_vsync_high_max_us = timer_ticks_to_us(
			stats->vsync_high_max_ticks, counter_clock_hz);
	if ((stats->vsync_high_samples != 0U) && (counter_clock_hz != 0U)) {
		report->poll_vsync_high_average_us =
				(uint32_t) ((stats->vsync_high_sum_ticks * 1000000ULL)
						/ ((uint64_t) stats->vsync_high_samples
								* counter_clock_hz));
	}
	report->poll_frame_lines_min =
			(stats->frame_lines_min == 0xFFFFU) ? 0U : stats->frame_lines_min;
	report->poll_frame_lines_max = stats->frame_lines_max;
	report->poll_active_image_lines_min =
			(stats->active_image_lines_min == 0xFFFFU) ?
					0U : stats->active_image_lines_min;
	report->poll_active_image_lines_max = stats->active_image_lines_max;
	report->poll_frame_trace_count = stats->frame_trace_count;
	for (i = 0U; i < report->poll_frame_trace_count; ++i) {
		report->poll_frame_trace[i] = stats->frame_trace[i];
	}

	report->poll_integrity_ok = (uint8_t) ((stats->sample_count > 1000U)
			&& (report->poll_max_gap_us <= DVP_POLL_MAX_GAP_US)
			&& (stats->href_measured_pulses >= DVP_MIN_MEASURED_LINES)
			&& (report->poll_frame_trace_count
					== ((stats->vsync_rising_edges > 0U) ?
							(stats->vsync_rising_edges - 1U) : 0U)));
	report->poll_frame_lines_ok = (uint8_t) ((report->poll_integrity_ok != 0U)
			&& (report->poll_frame_trace_count >= DVP_MIN_CLASSIFIED_FRAMES)
			&& (report->poll_active_image_lines_min
					== DVP_EXPECTED_ACTIVE_HREF_LINES)
			&& (report->poll_active_image_lines_max
					== DVP_EXPECTED_ACTIVE_HREF_LINES));
	for (i = 0U; i < report->poll_frame_trace_count; ++i) {
		if ((report->poll_frame_trace[i].classification_valid == 0U)
				|| (report->poll_frame_trace[i].active_image_href_lines !=
				DVP_EXPECTED_ACTIVE_HREF_LINES)) {
			report->poll_frame_lines_ok = 0U;
		}
	}

	poll_href_rising_delta =
			(report->href_active_edge_callbacks > report->poll_href_rising_edges) ?
					(report->href_active_edge_callbacks
							- report->poll_href_rising_edges) :
					(report->poll_href_rising_edges
							- report->href_active_edge_callbacks);
	poll_href_falling_delta =
			(report->href_inactive_edge_callbacks
					> report->poll_href_falling_edges) ?
					(report->href_inactive_edge_callbacks
							- report->poll_href_falling_edges) :
					(report->poll_href_falling_edges
							- report->href_inactive_edge_callbacks);
	poll_vsync_delta =
			(report->vsync_rising_callbacks > report->poll_vsync_rising_edges) ?
					(report->vsync_rising_callbacks
							- report->poll_vsync_rising_edges) :
					(report->poll_vsync_rising_edges
							- report->vsync_rising_callbacks);
	report->exti_poll_href_edges_match = (uint8_t) ((poll_href_rising_delta
			<= 1U) && (poll_href_falling_delta <= 1U));
	report->exti_poll_vsync_edges_match = (uint8_t) (poll_vsync_delta <= 1U);

	report->exti_poll_frame_trace_mismatches = 0U;
	trace_common_count =
			(report->frame_line_trace_count < report->poll_frame_trace_count) ?
					report->frame_line_trace_count :
					report->poll_frame_trace_count;
	for (i = 0U; i < trace_common_count; ++i) {
		if (report->frame_line_trace[i].href_lines
				!= report->poll_frame_trace[i].href_lines) {
			report->exti_poll_frame_trace_mismatches++;
		}
	}
	report->exti_poll_frame_trace_mismatches =
			(uint16_t) (report->exti_poll_frame_trace_mismatches
					+ ((report->frame_line_trace_count
							> report->poll_frame_trace_count) ?
							(report->frame_line_trace_count
									- report->poll_frame_trace_count) :
							(report->poll_frame_trace_count
									- report->frame_line_trace_count)));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t now_ticks;

	/* Forward EXTI events to the DVP capture engine before diagnostic processing. */
	OV7670_DvpCapture_OnExti(GPIO_Pin);

	if (s_monitor_active == 0U) {
		return;
	}

	now_ticks = TIM2->CNT;

	if (GPIO_Pin == CAM_HREF_Pin) {
		const GPIO_PinState pin_state = HAL_GPIO_ReadPin(CAM_HREF_GPIO_Port,
		CAM_HREF_Pin);
		const uint8_t is_active = (uint8_t) (pin_state == s_href_active_level);

		if ((is_active != 0U) && (s_href_is_active == 0U)) {
			s_href_is_active = 1U;
			s_href_measure_started = 1U;
			s_href_start_tick = now_ticks;
			s_href_active_pulses++;
			s_href_active_edge_callbacks++;
			if (s_vsync_have_last_rising != 0U) {
				s_lines_since_vsync_rising++;
			} else {
				s_href_lines_before_first_vsync++;
			}
		} else if ((is_active == 0U) && (s_href_is_active != 0U)) {
			s_href_is_active = 0U;
			s_href_inactive_edge_callbacks++;
			if (s_href_measure_started != 0U) {
				const uint32_t width_ticks = now_ticks - s_href_start_tick;
				s_href_measured_pulses++;
				s_href_sum_width_ticks += width_ticks;
				if (width_ticks < s_href_min_width_ticks) {
					s_href_min_width_ticks = width_ticks;
				}
				if (width_ticks > s_href_max_width_ticks) {
					s_href_max_width_ticks = width_ticks;
				}
			}
			s_href_measure_started = 0U;
		} else {
			/* Both-edge EXTI should alternate active/inactive observations. */
			s_href_state_mismatch_callbacks++;
		}
	} else if (GPIO_Pin == CAM_VSYNC_Pin) {
		/* PC12 is configured rising-edge-only for this observation window. */
		const GPIO_PinState vsync_state = HAL_GPIO_ReadPin(CAM_VSYNC_GPIO_Port,
		CAM_VSYNC_Pin);
		const GPIO_PinState href_state = HAL_GPIO_ReadPin(CAM_HREF_GPIO_Port,
		CAM_HREF_Pin);
		const uint8_t href_active_at_boundary = (uint8_t) (href_state
				== s_href_active_level);

		s_vsync_rising_callbacks++;
		if (vsync_state != GPIO_PIN_SET) {
			s_vsync_gpio_low_callbacks++;
		}
		if (href_active_at_boundary != 0U) {
			s_vsync_boundary_while_href_active++;
		}
		if (href_active_at_boundary != s_href_is_active) {
			s_href_state_mismatch_at_vsync++;
		}

		if (s_vsync_have_last_rising != 0U) {
			const uint32_t frame_period_ticks = now_ticks
					- s_vsync_last_rising_tick;
			const uint16_t frame_lines = s_lines_since_vsync_rising;

			s_complete_frames++;
			s_vsync_period_sum_ticks += frame_period_ticks;
			s_href_lines_in_complete_intervals += frame_lines;
			if (frame_period_ticks < s_vsync_period_min_ticks) {
				s_vsync_period_min_ticks = frame_period_ticks;
			}
			if (frame_period_ticks > s_vsync_period_max_ticks) {
				s_vsync_period_max_ticks = frame_period_ticks;
			}
			if (frame_lines < s_frame_lines_min) {
				s_frame_lines_min = frame_lines;
			}
			if (frame_lines > s_frame_lines_max) {
				s_frame_lines_max = frame_lines;
			}
			if (s_frame_line_trace_count < OV7670_DVP_DIAG_FRAME_TRACE_CAPACITY) {
				const uint16_t trace_index = s_frame_line_trace_count;
				s_frame_line_trace[trace_index].href_lines = frame_lines;
				s_frame_line_trace[trace_index].vsync_level_high =
						(uint8_t) (vsync_state == GPIO_PIN_SET);
				s_frame_line_trace[trace_index].href_active_at_boundary =
						href_active_at_boundary;
				s_frame_line_trace[trace_index].period_us = timer_ticks_to_us(
						frame_period_ticks, s_tim2_counter_clock_hz);
				s_frame_line_trace_count++;
			}
		}
		s_vsync_last_rising_tick = now_ticks;
		s_vsync_have_last_rising = 1U;
		s_lines_since_vsync_rising = 0U;
	}
}

OV7670_DvpDiagStatus_t OV7670_DvpDiag_Run(uint8_t com10,
		OV7670_DvpDiagReport_t *report) {
	OV7670_DvpDiagStatus_t pclk_status;
	OV7670_DvpDiagStatus_t tim2_status;
	uint32_t old_tim2_psc;
	uint32_t old_tim2_arr;
	uint32_t old_tim2_counter;
	uint8_t tim2_was_enabled;
	uint32_t tim2_probe_start;
	uint32_t tim2_probe_delta;
	uint32_t tim2_expected_delta;
	uint32_t tim2_lower_limit;
	uint32_t tim2_upper_limit;
	uint64_t pclk_period_sum_ticks = 0ULL;
	uint32_t pclk_timer_clock = 0U;
	uint16_t pclk_period_min = 0U;
	uint16_t pclk_period_max = 0U;
	uint16_t pclk_interval_outliers = 0U;
	uint8_t pclk_overcapture = 0U;
	uint8_t href_monitor_ok;
	uint8_t vsync_monitor_ok;
	uint8_t vsync_level_ok;
	uint8_t frame_lines_ok;
	uint8_t exti_saved = 0U;
	DvpPollStats_t poll_stats;
	uint32_t primask;
	DvpVsyncExtiState_t saved_vsync_exti = { 0U, 0U, 0U };

	if (report == NULL) {
		return OV7670_DVP_DIAG_BAD_ARGUMENT;
	}

	memset(report, 0, sizeof(*report));
	report->com10 = com10;
	report->href_active_low =
			(uint8_t) ((com10 & DVP_COM10_HREF_REVERSE) != 0U);
	report->vsync_active_low = (uint8_t) ((com10 & DVP_COM10_VSYNC_NEGATIVE)
			!= 0U);
	report->pclk_gated_by_href = (uint8_t) ((com10 & DVP_COM10_PCLK_GATING)
			!= 0U);
	report->pclk_reversed = (uint8_t) ((com10 & DVP_COM10_PCLK_REVERSE) != 0U);
	report->com10_baseline_ok = (uint8_t) (com10 == 0x00U);

	report->apb1_timer_clock_hz = apb1_timer_clock_hz();
	report->gpio_href_mode = (uint8_t) ((GPIOC->MODER >> (11U * 2U)) & 0x03U);
	report->gpio_vsync_mode = (uint8_t) ((GPIOC->MODER >> (12U * 2U)) & 0x03U);
	report->gpio_href_pupd = (uint8_t) ((GPIOC->PUPDR >> (11U * 2U)) & 0x03U);
	report->gpio_vsync_pupd = (uint8_t) ((GPIOC->PUPDR >> (12U * 2U)) & 0x03U);
	calculate_xclk_report(report);
	if (report->xclk_ok == 0U) {
		return OV7670_DVP_DIAG_XCLK_CONFIG_ERROR;
	}
	if (report->com10_baseline_ok == 0U) {
		return OV7670_DVP_DIAG_BAD_COM10;
	}

	old_tim2_psc = htim2.Instance->PSC;
	old_tim2_arr = htim2.Instance->ARR;
	old_tim2_counter = htim2.Instance->CNT;
	tim2_was_enabled = (uint8_t) ((htim2.Instance->CR1 & TIM_CR1_CEN) != 0U);
	report->tim2_prescaler_used = (uint16_t) old_tim2_psc;

	tim2_status = start_tim2_timestamp(old_tim2_psc,
			&report->tim2_counter_clock_hz);
	if (tim2_status != OV7670_DVP_DIAG_OK) {
		(void) stop_tim2_timestamp(old_tim2_psc, old_tim2_arr, old_tim2_counter,
				tim2_was_enabled);
		return tim2_status;
	}

	tim2_probe_start = TIM2->CNT;
	HAL_Delay(DVP_TIM2_PROBE_DELAY_MS);
	tim2_probe_delta = TIM2->CNT - tim2_probe_start;
	report->tim2_probe_delta_ticks = tim2_probe_delta;
	tim2_expected_delta = (uint32_t) (((uint64_t) report->tim2_counter_clock_hz
			* DVP_TIM2_PROBE_DELAY_MS) / 1000ULL);
	tim2_lower_limit = (tim2_expected_delta
			* (100U - DVP_TIM2_PROBE_TOLERANCE_PCT)) / 100U;
	tim2_upper_limit = (tim2_expected_delta
			* (100U + DVP_TIM2_PROBE_TOLERANCE_PCT)) / 100U;
	report->tim2_clock_ok = (uint8_t) ((report->tim2_counter_clock_hz != 0U)
			&& (tim2_probe_delta >= tim2_lower_limit)
			&& (tim2_probe_delta <= tim2_upper_limit));
	if (report->tim2_clock_ok == 0U) {
		(void) stop_tim2_timestamp(old_tim2_psc, old_tim2_arr, old_tim2_counter,
				tim2_was_enabled);
		return OV7670_DVP_DIAG_TIM2_CLOCK_ERROR;
	}

	s_tim2_counter_clock_hz = report->tim2_counter_clock_hz;
	configure_vsync_rising_only(report, &saved_vsync_exti);
	exti_saved = saved_vsync_exti.valid;
	if ((report->exti_vsync_rising_only_verified == 0U)
			|| (report->exti_href_both_edges_before == 0U)) {
		restore_vsync_exti(report, &saved_vsync_exti);
		(void) stop_tim2_timestamp(old_tim2_psc, old_tim2_arr, old_tim2_counter,
				tim2_was_enabled);
		return OV7670_DVP_DIAG_EXTI_CONFIG_ERROR;
	}

	__HAL_TIM_SET_COUNTER(&htim2, 0U);
	s_monitor_active = 0U;

	pclk_status = capture_pclk_periods(&pclk_timer_clock,
			&pclk_period_sum_ticks, &pclk_period_min, &pclk_period_max,
			&pclk_interval_outliers, &pclk_overcapture);

	if (pclk_status == OV7670_DVP_DIAG_OK) {
		report->pclk_sample_count = DVP_PCLK_CAPTURE_COUNT;
		report->pclk_period_min_ticks = pclk_period_min;
		report->pclk_period_max_ticks = pclk_period_max;
		report->pclk_interval_outliers = pclk_interval_outliers;
		report->pclk_hz = (uint32_t) (((uint64_t) pclk_timer_clock
				* ((DVP_PCLK_CAPTURE_COUNT - 1U) * DVP_PCLK_CAPTURE_DECIMATION))
				/ pclk_period_sum_ticks);
		report->pclk_dma_overcapture = pclk_overcapture;
		report->pclk_ok = (uint8_t) ((report->pclk_hz >= DVP_PCLK_MIN_HZ)
				&& (report->pclk_hz <= DVP_PCLK_MAX_HZ)
				&& (pclk_interval_outliers == 0U));
	}

	/* Start EXTI counters and polling from a clean, nearly simultaneous boundary. */
	primask = __get_PRIMASK();
	__disable_irq();
	EXTI->PR = DVP_EXTI_CAMERA_MASK;
	reset_sync_statistics(
			report->href_active_low ? GPIO_PIN_RESET : GPIO_PIN_SET,
			HAL_GPIO_ReadPin(CAM_HREF_GPIO_Port, CAM_HREF_Pin));
	s_monitor_active = 1U;
	__set_PRIMASK(primask);
	poll_camera_gpio_edges(&poll_stats, DVP_SYNC_MONITOR_WINDOW_MS,
			report->href_active_low ? GPIO_PIN_RESET : GPIO_PIN_SET,
			report->tim2_counter_clock_hz);

	primask = __get_PRIMASK();
	__disable_irq();
	s_monitor_active = 0U;
	__set_PRIMASK(primask);

	if (exti_saved != 0U) {
		restore_vsync_exti(report, &saved_vsync_exti);
	}
	tim2_status = stop_tim2_timestamp(old_tim2_psc, old_tim2_arr,
			old_tim2_counter, tim2_was_enabled);
	calculate_report_metrics(report, pclk_period_sum_ticks, pclk_timer_clock);
	copy_poll_stats_to_report(report, &poll_stats,
			report->tim2_counter_clock_hz);

	if (tim2_status != OV7670_DVP_DIAG_OK) {
		return tim2_status;
	}
	if (report->exti_restore_ok == 0U) {
		return OV7670_DVP_DIAG_EXTI_CONFIG_ERROR;
	}

	href_monitor_ok = (uint8_t) ((report->href_active_pulses
			>= DVP_MIN_MEASURED_LINES)
			&& (report->href_measured_pulses >= DVP_MIN_MEASURED_LINES)
			&& (report->pclk_ok != 0U)
			&& (report->href_average_pclk_cycles_milli
					>= (DVP_LINE_CYCLES_MIN * 1000U))
			&& (report->href_average_pclk_cycles_milli
					<= (DVP_LINE_CYCLES_MAX * 1000U)));
	vsync_monitor_ok = (uint8_t) ((report->vsync_rising_callbacks >= 2U)
			&& (report->complete_frames >= 1U)
			&& (report->frame_rate_millihz >= 500U)
			&& (report->frame_rate_millihz <= 60000U));
	vsync_level_ok = (uint8_t) (report->vsync_gpio_low_callbacks == 0U);
	frame_lines_ok = report->poll_frame_lines_ok;

	report->href_ok = href_monitor_ok;
	report->vsync_ok = vsync_monitor_ok;
	report->vsync_level_ok = vsync_level_ok;
	report->frame_lines_ok = frame_lines_ok;

	if (pclk_status != OV7670_DVP_DIAG_OK) {
		return pclk_status;
	}
	if ((report->pclk_hz < DVP_PCLK_MIN_HZ)
			|| (report->pclk_hz > DVP_PCLK_MAX_HZ)) {
		return OV7670_DVP_DIAG_PCLK_OUT_OF_RANGE;
	}
	if (report->pclk_interval_outliers != 0U) {
		return OV7670_DVP_DIAG_PCLK_INTERVAL_ERROR;
	}
	if (href_monitor_ok == 0U) {
		return (report->href_measured_pulses == 0U) ?
				OV7670_DVP_DIAG_HREF_MISSING : OV7670_DVP_DIAG_HREF_TIMING_ERROR;
	}
	if (vsync_monitor_ok == 0U) {
		return OV7670_DVP_DIAG_VSYNC_MISSING;
	}
	if (vsync_level_ok == 0U) {
		return OV7670_DVP_DIAG_VSYNC_LEVEL_ERROR;
	}
	if (report->poll_integrity_ok == 0U) {
		return OV7670_DVP_DIAG_POLL_INTEGRITY_ERROR;
	}
	if (frame_lines_ok == 0U) {
		return OV7670_DVP_DIAG_FRAME_LINES_ERROR;
	}
	return OV7670_DVP_DIAG_OK;
}

const char* OV7670_DvpDiag_StatusString(OV7670_DvpDiagStatus_t status) {
	switch (status) {
	case OV7670_DVP_DIAG_OK:
		return "OK";
	case OV7670_DVP_DIAG_BAD_ARGUMENT:
		return "BAD_ARGUMENT";
	case OV7670_DVP_DIAG_BAD_COM10:
		return "BAD_COM10";
	case OV7670_DVP_DIAG_XCLK_CONFIG_ERROR:
		return "XCLK_CONFIG_ERROR";
	case OV7670_DVP_DIAG_TIM2_CONFIG_ERROR:
		return "TIM2_CONFIG_ERROR";
	case OV7670_DVP_DIAG_TIM2_CLOCK_ERROR:
		return "TIM2_CLOCK_ERROR";
	case OV7670_DVP_DIAG_TIM3_CONFIG_ERROR:
		return "TIM3_CONFIG_ERROR";
	case OV7670_DVP_DIAG_EXTI_CONFIG_ERROR:
		return "EXTI_CONFIG_ERROR";
	case OV7670_DVP_DIAG_DMA_START_ERROR:
		return "DMA_START_ERROR";
	case OV7670_DVP_DIAG_PCLK_TIMEOUT:
		return "PCLK_TIMEOUT";
	case OV7670_DVP_DIAG_DMA_TRANSFER_ERROR:
		return "DMA_TRANSFER_ERROR";
	case OV7670_DVP_DIAG_PCLK_OUT_OF_RANGE:
		return "PCLK_OUT_OF_RANGE";
	case OV7670_DVP_DIAG_PCLK_INTERVAL_ERROR:
		return "PCLK_INTERVAL_ERROR";
	case OV7670_DVP_DIAG_HREF_MISSING:
		return "HREF_MISSING";
	case OV7670_DVP_DIAG_HREF_TIMING_ERROR:
		return "HREF_TIMING_ERROR";
	case OV7670_DVP_DIAG_VSYNC_MISSING:
		return "VSYNC_MISSING";
	case OV7670_DVP_DIAG_VSYNC_LEVEL_ERROR:
		return "VSYNC_LEVEL_ERROR";
	case OV7670_DVP_DIAG_FRAME_LINES_ERROR:
		return "FRAME_LINES_ERROR";
	case OV7670_DVP_DIAG_POLL_INTEGRITY_ERROR:
		return "POLL_INTEGRITY_ERROR";
	default:
		return "UNKNOWN";
	}
}
