#include "drivers/ov7670_dvp_capture.h"
#include <string.h>
#include "main.h"
#include "tim.h"
#include "drivers/ov7670.h"
DMA_HandleTypeDef hdma_tim1_ch4;
#define DVP_CAPTURE_DMA_SAMPLES_PER_LINE OV7670_DVP_CAPTURE_LINE_BYTES
#define DVP_CAPTURE_VSYNC_TIMEOUT_MS 600U
#define DVP_CAPTURE_FRAME_TIMEOUT_MS 600U
#define DVP_CAPTURE_HREF_IDLE_TIMEOUT_MS 20U
#define DVP_TIM_CCMR2_CC4S_MASK (3UL << 8U)
#define DVP_TIM_CCMR2_IC4PSC_MASK (3UL << 10U)
#define DVP_TIM_CCMR2_IC4F_MASK (15UL << 12U)
#define DVP_TIM_CCMR2_CC4_CONFIG_MASK (DVP_TIM_CCMR2_CC4S_MASK | DVP_TIM_CCMR2_IC4PSC_MASK | DVP_TIM_CCMR2_IC4F_MASK)
#define DVP_TIM_CCER_CC4_CONFIG_MASK (TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP)
#define DVP_TIM_DIER_CC4DE_MASK TIM_DIER_CC4DE
#define DVP_PCLK_GPIO_PIN GPIO_PIN_11
#define DVP_PCLK_GPIO_AF GPIO_AF1_TIM1
#define DVP_PCLK_GPIO_MODER_MASK (3UL << (11U * 2U))
#define DVP_PCLK_GPIO_PUPDR_MASK (3UL << (11U * 2U))
#define DVP_PCLK_GPIO_OSPEEDR_MASK (3UL << (11U * 2U))
#define DVP_PCLK_GPIO_AFR_MASK (15UL << (12U))
#define DVP_EXTI_HREF_MASK (1UL << 11U)
#define DVP_EXTI_VSYNC_MASK (1UL << 12U)
#define DVP_EXTI_CAMERA_MASK (DVP_EXTI_HREF_MASK | DVP_EXTI_VSYNC_MASK)
/* DMA stores the complete GPIOC->IDR word for each PCLK rising edge. */
static uint32_t s_line_samples[DVP_CAPTURE_DMA_SAMPLES_PER_LINE] __attribute__((aligned(4)));
static uint8_t s_frame_buffer[OV7670_DVP_CAPTURE_FRAME_BYTES] __attribute__((aligned(4)));
static volatile uint8_t s_capture_armed;
static volatile uint8_t s_frame_active;
static volatile uint8_t s_capture_done;
static volatile uint8_t s_dma_line_complete;
static volatile uint8_t s_dma_error;
static volatile uint8_t s_line_in_progress;
static volatile uint8_t s_line_end_pending;
static volatile uint8_t s_current_line_valid;
static volatile uint16_t s_current_line_index;
static volatile uint16_t s_frame_href_lines;
static volatile uint16_t s_leading_lines;
static volatile uint16_t s_active_lines;
static volatile uint16_t s_trailing_lines;
static volatile uint16_t s_active_rows_captured;
static volatile uint16_t s_active_line_samples_min;
static volatile uint16_t s_active_line_samples_max;
static volatile uint16_t s_active_line_sample_errors;
static volatile uint16_t s_non_active_line_sample_errors;
static volatile uint32_t s_frame_start_tick;
static volatile uint32_t s_frame_period_ms;
static volatile uint32_t s_frame_bytes_written;
static volatile uint8_t s_dma_fault_kind;
static volatile uint8_t s_dma_state_at_fault;
static volatile uint8_t s_dma_start_hal_status;
static volatile uint8_t s_dma_abort_hal_status;
static volatile uint16_t s_dma_abort_race_recovered;
static volatile uint16_t s_dma_start_count;
static volatile uint16_t s_dma_complete_count;
static volatile uint16_t s_dma_error_count;
static volatile uint32_t s_dma_error_code;
static volatile uint32_t s_dma_fault_ndtr;
static volatile uint32_t s_dma_fault_cr;
static volatile uint32_t s_dma_fault_fcr;
static volatile uint32_t s_dma_fault_par;
static volatile uint32_t s_dma_fault_m0ar;
static uint32_t s_old_tim1_ccmr2_cc4;
static uint32_t s_old_tim1_ccer_cc4;
static uint32_t s_old_tim1_dier_cc4de;
static uint32_t s_old_tim1_ccr4;
static uint32_t s_old_gpioa_moder;
static uint32_t s_old_gpioa_pupdr;
static uint32_t s_old_gpioa_ospeedr;
static uint32_t s_old_gpioa_otyper;
static uint32_t s_old_gpioa_afr1;
static uint8_t s_tim1_ch4_saved;
static uint8_t s_pclk_pin_saved;
static uint8_t s_dma2_initialized;
static uint32_t s_line_dma_start_cycles;
static volatile uint32_t s_line_end_pending_cycles;
static volatile uint32_t s_href_pending_service_max_us;
static volatile uint32_t s_href_pending_overrun_us;
static volatile uint8_t s_href_width_active;
static volatile uint32_t s_href_start_cycles;
static volatile uint16_t s_href_pulse_count;
static volatile uint32_t s_href_high_min_us;
static volatile uint32_t s_href_high_max_us;
static uint32_t s_dma_route_par;
static uint32_t s_dma_route_m0ar;
static uint32_t s_dma_route_ndtr;
static uint32_t s_dma_route_cr;
static volatile uint32_t s_dma_arm_to_complete_min_us;
static volatile uint32_t s_dma_arm_to_complete_max_us;
static void (*s_old_dma_complete_callback)(DMA_HandleTypeDef*);
static void (*s_old_dma_error_callback)(DMA_HandleTypeDef*);
static uint32_t s_old_exti_rtsr;
static uint32_t s_old_exti_ftsr;
static uint32_t s_old_exti_imr;
static uint8_t s_exti_saved;

typedef enum {
	DVP_ASYNC_IDLE = 0U,
	DVP_ASYNC_WAIT_HREF_IDLE,
	DVP_ASYNC_WAIT_FRAME,
	DVP_ASYNC_COMPLETE
} DvpAsyncState_t;

static DvpAsyncState_t s_async_state;
static OV7670_DvpCaptureReport_t *s_async_report;
static OV7670_DvpCaptureStatus_t s_async_result;
static uint32_t s_async_start_tick;
static uint8_t s_async_active;
static uint8_t s_async_complete;
static uint8_t s_dma_callbacks_saved;
static uint8_t extract_dvp_byte(uint32_t gpio_idr) {
	uint32_t value = 0U;
	value |= ((gpio_idr >> 2U) & 1U) << 0U; /* PC2 -> D0 */
	value |= ((gpio_idr >> 3U) & 1U) << 1U; /* PC3 -> D1 */
	value |= ((gpio_idr >> 4U) & 1U) << 2U; /* PC4 -> D2 */
	value |= ((gpio_idr >> 5U) & 1U) << 3U; /* PC5 -> D3 */
	value |= ((gpio_idr >> 6U) & 1U) << 4U; /* PC6 -> D4 */
	value |= ((gpio_idr >> 7U) & 1U) << 5U; /* PC7 -> D5 */
	value |= ((gpio_idr >> 8U) & 1U) << 6U; /* PC8 -> D6 */
	value |= ((gpio_idr >> 10U) & 1U) << 7U; /* PC10 -> D7 */
	return (uint8_t) value;
}
static uint32_t calculate_crc32(const uint8_t *data, uint32_t length) {
	uint32_t crc = 0xFFFFFFFFUL;
	uint32_t i;
	for (i = 0U; i < length; ++i) {
		uint32_t bit;
		crc ^= data[i];
		for (bit = 0U; bit < 8U; ++bit) {
			const uint32_t mask = (uint32_t) (0U - (crc & 1U));
			crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
		}
	}
	return ~crc;
}
static OV7670_DvpCaptureStatus_t save_and_configure_exti(void) {
	uint32_t primask;
	s_old_exti_rtsr = EXTI->RTSR & DVP_EXTI_CAMERA_MASK;
	s_old_exti_ftsr = EXTI->FTSR & DVP_EXTI_CAMERA_MASK;
	s_old_exti_imr = EXTI->IMR & DVP_EXTI_CAMERA_MASK;
	s_exti_saved = 1U;
	primask = __get_PRIMASK();
	__disable_irq();
	/* HREF requires both edges; VSYNC is used only as a rising-edge frame marker.
	 */
	EXTI->IMR |= DVP_EXTI_CAMERA_MASK;
	EXTI->RTSR |= DVP_EXTI_CAMERA_MASK;
	EXTI->FTSR = (EXTI->FTSR & ~DVP_EXTI_VSYNC_MASK) | DVP_EXTI_HREF_MASK;
	EXTI->PR = DVP_EXTI_CAMERA_MASK;
	if (((EXTI->RTSR & DVP_EXTI_CAMERA_MASK) != DVP_EXTI_CAMERA_MASK)
			|| ((EXTI->FTSR & DVP_EXTI_HREF_MASK) == 0U)
			|| ((EXTI->FTSR & DVP_EXTI_VSYNC_MASK) != 0U)
			|| ((EXTI->IMR & DVP_EXTI_CAMERA_MASK) != DVP_EXTI_CAMERA_MASK)) {
		__set_PRIMASK(primask);
		return OV7670_DVP_CAPTURE_EXTI_CONFIG_ERROR;
	}
	__set_PRIMASK(primask);
	return OV7670_DVP_CAPTURE_OK;
}
static uint8_t restore_exti(void) {
	uint32_t primask;
	if (s_exti_saved == 0U) {
		return 1U;
	}
	primask = __get_PRIMASK();
	__disable_irq();
	EXTI->RTSR = (EXTI->RTSR & ~DVP_EXTI_CAMERA_MASK) | s_old_exti_rtsr;
	EXTI->FTSR = (EXTI->FTSR & ~DVP_EXTI_CAMERA_MASK) | s_old_exti_ftsr;
	EXTI->IMR = (EXTI->IMR & ~DVP_EXTI_CAMERA_MASK) | s_old_exti_imr;
	EXTI->PR = DVP_EXTI_CAMERA_MASK;
	s_exti_saved = 0U;
	__set_PRIMASK(primask);
	return (uint8_t) (((EXTI->RTSR & DVP_EXTI_CAMERA_MASK) == s_old_exti_rtsr)
			&& ((EXTI->FTSR & DVP_EXTI_CAMERA_MASK) == s_old_exti_ftsr)
			&& ((EXTI->IMR & DVP_EXTI_CAMERA_MASK) == s_old_exti_imr));
}
static uint8_t dwt_cycle_counter_init(void) {
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0U;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
	return (uint8_t) ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
}
static uint32_t cycles_to_us(uint32_t cycles) {
	uint32_t core_hz = SystemCoreClock;
	if (core_hz == 0U) {
		core_hz = 100000000UL;
	}
	return (uint32_t) (((uint64_t) cycles * 1000000ULL) / core_hz);
}
static OV7670_DvpCaptureStatus_t ensure_dma2_stream4_initialized(void) {
	if (s_dma2_initialized != 0U) {
		return OV7670_DVP_CAPTURE_OK;
	}
	__HAL_RCC_DMA2_CLK_ENABLE();
	memset(&hdma_tim1_ch4, 0, sizeof(hdma_tim1_ch4));
	hdma_tim1_ch4.Instance = DMA2_Stream4;
	hdma_tim1_ch4.Init.Channel = DMA_CHANNEL_6;
	hdma_tim1_ch4.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_tim1_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_tim1_ch4.Init.MemInc = DMA_MINC_ENABLE;
	hdma_tim1_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_tim1_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_tim1_ch4.Init.Mode = DMA_NORMAL;
	hdma_tim1_ch4.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	hdma_tim1_ch4.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_tim1_ch4) != HAL_OK) {
		return OV7670_DVP_CAPTURE_DMA_INIT_ERROR;
	}
	__HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch4,
			__HAL_DMA_GET_TC_FLAG_INDEX(&hdma_tim1_ch4));
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5U, 0U);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	s_dma2_initialized = 1U;
	return OV7670_DVP_CAPTURE_OK;
}
static OV7670_DvpCaptureStatus_t prepare_tim1_ch4_capture(void) {
	OV7670_DvpCaptureStatus_t dma_status;
	/* TIM1_CH1 must already be running: it supplies the sensor XCLK. */
	if (((TIM1->CR1 & TIM_CR1_CEN) == 0U)
			|| ((TIM1->CCER & TIM_CCER_CC1E) == 0U)
			|| ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)) {
		return OV7670_DVP_CAPTURE_TIM1_CONFIG_ERROR;
	}
	if (dwt_cycle_counter_init() == 0U) {
		return OV7670_DVP_CAPTURE_TIM1_CONFIG_ERROR;
	}
	dma_status = ensure_dma2_stream4_initialized();
	if (dma_status != OV7670_DVP_CAPTURE_OK) {
		return dma_status;
	}
	__HAL_RCC_GPIOA_CLK_ENABLE();
	s_old_gpioa_moder = GPIOA->MODER & DVP_PCLK_GPIO_MODER_MASK;
	s_old_gpioa_pupdr = GPIOA->PUPDR & DVP_PCLK_GPIO_PUPDR_MASK;
	s_old_gpioa_ospeedr = GPIOA->OSPEEDR & DVP_PCLK_GPIO_OSPEEDR_MASK;
	s_old_gpioa_otyper = GPIOA->OTYPER & DVP_PCLK_GPIO_PIN;
	s_old_gpioa_afr1 = GPIOA->AFR[1] & DVP_PCLK_GPIO_AFR_MASK;
	s_pclk_pin_saved = 1U;
	s_old_tim1_ccmr2_cc4 = TIM1->CCMR2 & DVP_TIM_CCMR2_CC4_CONFIG_MASK;
	s_old_tim1_ccer_cc4 = TIM1->CCER & DVP_TIM_CCER_CC4_CONFIG_MASK;
	s_old_tim1_dier_cc4de = TIM1->DIER & DVP_TIM_DIER_CC4DE_MASK;
	s_old_tim1_ccr4 = TIM1->CCR4;
	s_tim1_ch4_saved = 1U;
	/* PA11 AF1 routes the external camera PCLK signal to TIM1_CH4. */
	GPIOA->MODER = (GPIOA->MODER & ~DVP_PCLK_GPIO_MODER_MASK) | (2UL << 22U);
	GPIOA->PUPDR &= ~DVP_PCLK_GPIO_PUPDR_MASK;
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~DVP_PCLK_GPIO_OSPEEDR_MASK)
			| (2UL << 22U);
	GPIOA->OTYPER &= ~DVP_PCLK_GPIO_PIN;
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~DVP_PCLK_GPIO_AFR_MASK)
			| ((uint32_t) DVP_PCLK_GPIO_AF << 12U);
	/* Do not stop TIM1 or change PSC/ARR/CNT: CH1 must keep generating XCLK. */
	TIM1->DIER &= ~TIM_DIER_CC4DE;
	TIM1->CCER &= ~TIM_CCER_CC4E;
	TIM1->CCMR2 = (TIM1->CCMR2 & ~DVP_TIM_CCMR2_CC4_CONFIG_MASK) | (1UL << 8U); /* CC4S=01 direct TI4, IC4PSC=DIV1, IC4F=0. */
	TIM1->CCER &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP); /* Capture rising PCLK edge.
	 */
	TIM1->SR &= ~(TIM_SR_CC4IF | TIM_SR_CC4OF);
	return OV7670_DVP_CAPTURE_OK;
}
static uint8_t restore_tim1_ch4_capture(void) {
	uint8_t timer_ok = 1U;
	uint8_t pin_ok = 1U;
	if (s_tim1_ch4_saved != 0U) {
		TIM1->DIER &= ~TIM_DIER_CC4DE;
		TIM1->CCER &= ~TIM_CCER_CC4E;
		TIM1->CCMR2 = (TIM1->CCMR2 & ~DVP_TIM_CCMR2_CC4_CONFIG_MASK)
				| s_old_tim1_ccmr2_cc4;
		TIM1->CCER = (TIM1->CCER & ~DVP_TIM_CCER_CC4_CONFIG_MASK)
				| s_old_tim1_ccer_cc4;
		TIM1->DIER = (TIM1->DIER & ~DVP_TIM_DIER_CC4DE_MASK)
				| s_old_tim1_dier_cc4de;
		TIM1->CCR4 = s_old_tim1_ccr4;
		TIM1->SR &= ~(TIM_SR_CC4IF | TIM_SR_CC4OF);
		s_tim1_ch4_saved = 0U;
		timer_ok = (uint8_t) (((TIM1->CCMR2 & DVP_TIM_CCMR2_CC4_CONFIG_MASK)
				== s_old_tim1_ccmr2_cc4)
				&& ((TIM1->CCER & DVP_TIM_CCER_CC4_CONFIG_MASK)
						== s_old_tim1_ccer_cc4)
				&& ((TIM1->DIER & DVP_TIM_DIER_CC4DE_MASK)
						== s_old_tim1_dier_cc4de));
	}
	if (s_pclk_pin_saved != 0U) {
		GPIOA->MODER = (GPIOA->MODER & ~DVP_PCLK_GPIO_MODER_MASK)
				| s_old_gpioa_moder;
		GPIOA->PUPDR = (GPIOA->PUPDR & ~DVP_PCLK_GPIO_PUPDR_MASK)
				| s_old_gpioa_pupdr;
		GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~DVP_PCLK_GPIO_OSPEEDR_MASK)
				| s_old_gpioa_ospeedr;
		GPIOA->OTYPER = (GPIOA->OTYPER & ~DVP_PCLK_GPIO_PIN)
				| s_old_gpioa_otyper;
		GPIOA->AFR[1] = (GPIOA->AFR[1] & ~DVP_PCLK_GPIO_AFR_MASK)
				| s_old_gpioa_afr1;
		s_pclk_pin_saved = 0U;
		pin_ok =
				(uint8_t) (((GPIOA->MODER & DVP_PCLK_GPIO_MODER_MASK)
						== s_old_gpioa_moder)
						&& ((GPIOA->PUPDR & DVP_PCLK_GPIO_PUPDR_MASK)
								== s_old_gpioa_pupdr)
						&& ((GPIOA->AFR[1] & DVP_PCLK_GPIO_AFR_MASK)
								== s_old_gpioa_afr1));
	}
	return (uint8_t) (timer_ok && pin_ok);
}
static void snapshot_dma_fault(uint8_t fault_kind) {
	/* Keep the first/primary failure; do not let a later HREF overrun mask TE.
	 */
	if (s_dma_fault_kind != OV7670_DVP_DMA_FAULT_NONE) {
		s_dma_error = 1U;
		return;
	}
	s_dma_fault_kind = fault_kind;
	s_dma_state_at_fault = (uint8_t) hdma_tim1_ch4.State;
	s_dma_error_code = hdma_tim1_ch4.ErrorCode;
	s_dma_fault_ndtr = hdma_tim1_ch4.Instance->NDTR;
	s_dma_fault_cr = hdma_tim1_ch4.Instance->CR;
	s_dma_fault_fcr = hdma_tim1_ch4.Instance->FCR;
	s_dma_fault_par = hdma_tim1_ch4.Instance->PAR;
	s_dma_fault_m0ar = hdma_tim1_ch4.Instance->M0AR;
	s_dma_error = 1U;
}
/*
 * DMA2 Stream4 IRQ has higher NVIC priority than the shared EXTI IRQ in the
 * supplied CubeMX project (DMA priority 5, EXTI priority 6). Therefore the
 * stream can complete after the caller observes BUSY but before
 * HAL_DMA_Abort() executes. HAL_DMA_IRQHandler then moves the handle to READY;
 * HAL_DMA_Abort() returns HAL_ERROR / HAL_DMA_ERROR_NO_XFER in that race.
 * This is a benign completion race, not a transfer failure.
 */
static uint8_t abort_dma_if_active(uint8_t fault_kind) {
	HAL_StatusTypeDef abort_status;
	if (hdma_tim1_ch4.State == HAL_DMA_STATE_READY) {
		return 1U;
	}
	abort_status = HAL_DMA_Abort(&hdma_tim1_ch4);
	s_dma_abort_hal_status = (uint8_t) abort_status;
	if (abort_status == HAL_OK) {
		return 1U;
	}
	/* The DMA-complete IRQ may have won the race after the READY check. */
	if ((hdma_tim1_ch4.State == HAL_DMA_STATE_READY)
			&& ((hdma_tim1_ch4.ErrorCode & HAL_DMA_ERROR_NO_XFER) != 0U)) {
		if (s_dma_abort_race_recovered < 0xFFFFU) {
			s_dma_abort_race_recovered++;
		}
		hdma_tim1_ch4.ErrorCode = HAL_DMA_ERROR_NONE;
		return 1U;
	}
	snapshot_dma_fault(fault_kind);
	return 0U;
}
static void line_dma_complete(DMA_HandleTypeDef *hdma) {
	if (hdma == &hdma_tim1_ch4) {
		__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC4);
		s_dma_line_complete = 1U;
		{
			/* This interval starts when DMA is armed, often while HREF is LOW.
			 * With COM10[5] enabled, PCLK is gated during horizontal blanking,
			 * so the measurement includes the wait for the next active HREF
			 * plus the 320 PCLK transfers. It must NOT be compared with the
			 * ~408 us active HREF width. Record it for diagnostics only. */
			const uint32_t arm_to_complete_us = cycles_to_us(
			DWT->CYCCNT - s_line_dma_start_cycles);
			if ((s_dma_arm_to_complete_min_us == 0U)
					|| (arm_to_complete_us < s_dma_arm_to_complete_min_us)) {
				s_dma_arm_to_complete_min_us = arm_to_complete_us;
			}
			if (arm_to_complete_us > s_dma_arm_to_complete_max_us) {
				s_dma_arm_to_complete_max_us = arm_to_complete_us;
			}
		}
		if (s_dma_complete_count < 0xFFFFU) {
			s_dma_complete_count++;
		}
	}
}
static void line_dma_error(DMA_HandleTypeDef *hdma) {
	if (hdma == &hdma_tim1_ch4) {
		__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC4);
		if (s_dma_error_count < 0xFFFFU) {
			s_dma_error_count++;
		}
		snapshot_dma_fault(OV7670_DVP_DMA_FAULT_IRQ_ERROR);
	}
}
static uint8_t arm_line_dma(void) {
	HAL_StatusTypeDef status;
	if (s_capture_armed == 0U) {
		return 1U;
	}
	__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC4);
	htim1.Instance->CCER &= ~TIM_CCER_CC4E;
	if (abort_dma_if_active(OV7670_DVP_DMA_FAULT_ABORT_REARM) == 0U) {
		return 0U;
	}
	/*
	 * TIM1_CH4 is used as a per-PCLK DMA request source; DMA reads GPIOC->IDR,
	 * not CCR4. The input-capture event is used as the DMA request source;
	 * the DMA peripheral bus must therefore be DMA2, which can access AHB1 GPIO.
	 * Clear only channel-4 status flags at the start of each HREF line.
	 */
	htim1.Instance->SR &= ~(TIM_SR_CC4IF | TIM_SR_CC4OF);
	s_dma_line_complete = 0U;
	status = HAL_DMA_Start_IT(&hdma_tim1_ch4,
			(uint32_t) (uintptr_t) &GPIOC->IDR,
			(uint32_t) (uintptr_t) &s_line_samples[0],
			DVP_CAPTURE_DMA_SAMPLES_PER_LINE);
	if (status != HAL_OK) {
		s_dma_start_hal_status = (uint8_t) status;
		snapshot_dma_fault(OV7670_DVP_DMA_FAULT_START_REARM);
		return 0U;
	}
	if (s_dma_start_count < 0xFFFFU) {
		s_dma_start_count++;
	}
	if (s_dma_start_count == 1U) {
		/* Snapshot the configured route now, but defer UART output until
		 * capture has stopped. Blocking diagnostics here lose HREF lines. */
		s_dma_route_par = hdma_tim1_ch4.Instance->PAR;
		s_dma_route_m0ar = hdma_tim1_ch4.Instance->M0AR;
		s_dma_route_ndtr = hdma_tim1_ch4.Instance->NDTR;
		s_dma_route_cr = hdma_tim1_ch4.Instance->CR;
	}
	/* Timestamp immediately before the hardware request path is enabled.
	 * No blocking UART operation is permitted from here until capture ends. */
	s_line_dma_start_cycles = DWT->CYCCNT;
	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC4);
	htim1.Instance->CCER |= TIM_CCER_CC4E;
	return 1U;
}
static void process_pending_href_line_common(uint8_t dma_already_stopped) {
	uint32_t remaining;
	uint16_t captured_samples;
	uint16_t line_index;
	uint8_t line_valid;
	if (s_line_end_pending == 0U) {
		return;
	}
	/* HREF falling ISR has disabled CC4 DMA requests. Measure the delay from
	 * the edge to line finalization. The normal path calls this from EXTI;
	 * the foreground call remains a defensive fallback only. */
	{
		const uint32_t service_us = cycles_to_us(
		DWT->CYCCNT - s_line_end_pending_cycles);
		if (service_us > s_href_pending_service_max_us) {
			s_href_pending_service_max_us = service_us;
		}
	}
	s_line_end_pending = 0U;
	/* DMA2 IRQ has higher priority than HREF EXTI. In the ISR path, READY
	 * therefore proves that the 320-sample normal-mode transfer completed.
	 * If it did not complete, mark the frame failed and let the foreground
	 * cleanup abort the stream (HAL_DMA_Abort must not wait inside EXTI). */
	if (dma_already_stopped != 0U) {
		if (hdma_tim1_ch4.State != HAL_DMA_STATE_READY) {
			snapshot_dma_fault(OV7670_DVP_DMA_FAULT_ABORT_LINE_END);
			/* Stop further requests immediately without a blocking HAL abort in
			 * interrupt context. Foreground DVP_Process performs full HAL cleanup. */
			TIM1->DIER &= ~TIM_DMA_CC4;
			TIM1->CCER &= ~TIM_CCER_CC4E;
			__HAL_DMA_DISABLE(&hdma_tim1_ch4);
			s_capture_armed = 0U;
			s_frame_active = 0U;
			s_line_in_progress = 0U;
			s_line_end_pending = 0U;
			s_current_line_valid = 0U;
			return;
		}
	} else {
		(void) abort_dma_if_active(OV7670_DVP_DMA_FAULT_ABORT_LINE_END);
	}
	remaining = hdma_tim1_ch4.Instance->NDTR;
	if (remaining > DVP_CAPTURE_DMA_SAMPLES_PER_LINE) {
		remaining = DVP_CAPTURE_DMA_SAMPLES_PER_LINE;
		snapshot_dma_fault(OV7670_DVP_DMA_FAULT_NDTR_RANGE);
	}
	captured_samples =
			(uint16_t) (DVP_CAPTURE_DMA_SAMPLES_PER_LINE - remaining);
	/* DMA has stopped writing this line; order subsequent CPU reads after DMA.
	 */
	__DMB();
	line_index = s_current_line_index;
	line_valid = s_current_line_valid;
	if ((s_frame_active != 0U) && (line_valid != 0U)) {
		if (line_index < OV7670_DVP_CAPTURE_LEADING_HREF_LINES) {
			s_leading_lines++;
			if (captured_samples != DVP_CAPTURE_DMA_SAMPLES_PER_LINE) {
				s_non_active_line_sample_errors++;
			}
		} else if (line_index < (OV7670_DVP_CAPTURE_LEADING_HREF_LINES +
		OV7670_DVP_CAPTURE_HEIGHT)) {
			uint16_t sample;
			const uint16_t row = (uint16_t) (line_index -
			OV7670_DVP_CAPTURE_LEADING_HREF_LINES);
			s_active_lines++;
			if (captured_samples < s_active_line_samples_min) {
				s_active_line_samples_min = captured_samples;
			}
			if (captured_samples > s_active_line_samples_max) {
				s_active_line_samples_max = captured_samples;
			}
			if (captured_samples != DVP_CAPTURE_DMA_SAMPLES_PER_LINE) {
				s_active_line_sample_errors++;
			} else {
				const uint32_t dst_offset = (uint32_t) row *
				OV7670_DVP_CAPTURE_LINE_BYTES;
				for (sample = 0U; sample < DVP_CAPTURE_DMA_SAMPLES_PER_LINE;
						++sample) {
					s_frame_buffer[dst_offset + sample] = extract_dvp_byte(
							s_line_samples[sample]);
				}
				s_active_rows_captured++;
				s_frame_bytes_written += DVP_CAPTURE_DMA_SAMPLES_PER_LINE;
			}
		} else {
			s_trailing_lines++;
			if (captured_samples != DVP_CAPTURE_DMA_SAMPLES_PER_LINE) {
				s_non_active_line_sample_errors++;
			}
		}
	}
	s_line_in_progress = 0U;
	s_line_end_pending = 0U;
	s_current_line_valid = 0U;
	/* Re-arm during horizontal blanking, ready for the next HREF pulse. */
	if ((s_capture_armed != 0U) && (s_capture_done == 0U)
			&& (s_dma_error == 0U)) {
		(void) arm_line_dma();
	}
}
static void process_pending_href_line(void) {
	process_pending_href_line_common(0U);
}

static void process_pending_href_line_from_isr(void) {
	process_pending_href_line_common(1U);
}

void OV7670_DvpCapture_DmaIrqHandler(void) {
	HAL_DMA_IRQHandler(&hdma_tim1_ch4);
}
void OV7670_DvpCapture_OnExti(uint16_t gpio_pin) {
	if (s_capture_armed == 0U) {
		return;
	}
	if (gpio_pin == CAM_VSYNC_Pin) {
		const GPIO_PinState level = HAL_GPIO_ReadPin(CAM_VSYNC_GPIO_Port,
		CAM_VSYNC_Pin);
		if (level == GPIO_PIN_SET) {
			if (s_frame_active == 0U) {
				s_frame_active = 1U;
				s_frame_start_tick = HAL_GetTick();
				s_frame_href_lines = 0U;
				s_leading_lines = 0U;
				s_active_lines = 0U;
				s_trailing_lines = 0U;
				s_active_rows_captured = 0U;
				s_active_line_samples_min = 0xFFFFU;
				s_active_line_samples_max = 0U;
				s_active_line_sample_errors = 0U;
				s_non_active_line_sample_errors = 0U;
				s_frame_bytes_written = 0U;
			} else {
				s_frame_period_ms = HAL_GetTick() - s_frame_start_tick;
				s_frame_active = 0U;
				s_capture_done = 1U;
			}
		}
	} else if (gpio_pin == CAM_HREF_Pin) {
		const GPIO_PinState level = HAL_GPIO_ReadPin(CAM_HREF_GPIO_Port,
		CAM_HREF_Pin);
		if (level == GPIO_PIN_SET) {
			s_href_start_cycles = DWT->CYCCNT;
			s_href_width_active = 1U;
			if ((s_line_in_progress == 0U) && (s_line_end_pending == 0U)) {
				s_line_in_progress = 1U;
				s_current_line_valid = s_frame_active;
				if (s_frame_active != 0U) {
					s_current_line_index = s_frame_href_lines;
					s_frame_href_lines++;
				}
			} else if (s_line_end_pending != 0U) {
				/* A new HREF arrived before the previous line was finalized. */
				s_href_pending_overrun_us = cycles_to_us(
				DWT->CYCCNT - s_line_end_pending_cycles);
				snapshot_dma_fault(OV7670_DVP_DMA_FAULT_HREF_PENDING_OVERRUN);
			}
		} else {
			if (s_href_width_active != 0U) {
				const uint32_t href_high_us = cycles_to_us(
				DWT->CYCCNT - s_href_start_cycles);
				s_href_width_active = 0U;
				if (s_href_pulse_count < 0xFFFFU) {
					s_href_pulse_count++;
				}
				if ((s_href_high_min_us == 0U)
						|| (href_high_us < s_href_high_min_us)) {
					s_href_high_min_us = href_high_us;
				}
				if (href_high_us > s_href_high_max_us) {
					s_href_high_max_us = href_high_us;
				}
			}
			if ((s_line_in_progress != 0U) && (s_line_end_pending == 0U)) {
				/* Stop further TIM1_CH4 DMA requests at the HREF falling edge. */
				__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC4);
				TIM1->CCER &= ~TIM_CCER_CC4E;
				s_line_in_progress = 0U;
				s_line_end_pending_cycles = DWT->CYCCNT;
				s_line_end_pending = 1U;
				/* The next HREF rising edge arrives roughly 1.6 ms later.
				 * Finalize this line immediately in the HREF EXTI context so
				 * unrelated super-loop work (I2C, CLI, journal/Flash tasks)
				 * cannot starve the DVP line service deadline. The DMA IRQ has
				 * higher NVIC priority (5) than this EXTI (6), so a completed
				 * transfer is settled before the line is copied/re-armed. */
				process_pending_href_line_from_isr();
			}
		}
	}
}
static void reset_capture_state(void) {
	s_capture_armed = 0U;
	s_frame_active = 0U;
	s_capture_done = 0U;
	s_dma_line_complete = 0U;
	s_dma_error = 0U;
	s_line_in_progress = 0U;
	s_current_line_valid = 0U;
	s_current_line_index = 0U;
	s_frame_href_lines = 0U;
	s_leading_lines = 0U;
	s_active_lines = 0U;
	s_trailing_lines = 0U;
	s_active_rows_captured = 0U;
	s_active_line_samples_min = 0xFFFFU;
	s_active_line_samples_max = 0U;
	s_active_line_sample_errors = 0U;
	s_non_active_line_sample_errors = 0U;
	s_frame_start_tick = 0U;
	s_frame_period_ms = 0U;
	s_frame_bytes_written = 0U;
	s_dma_fault_kind = OV7670_DVP_DMA_FAULT_NONE;
	s_dma_state_at_fault = 0U;
	s_dma_start_hal_status = (uint8_t) HAL_OK;
	s_dma_abort_hal_status = (uint8_t) HAL_OK;
	s_dma_abort_race_recovered = 0U;
	s_dma_start_count = 0U;
	s_dma_complete_count = 0U;
	s_dma_error_count = 0U;
	s_dma_error_code = HAL_DMA_ERROR_NONE;
	s_dma_fault_ndtr = 0U;
	s_dma_fault_cr = 0U;
	s_dma_fault_fcr = 0U;
	s_dma_fault_par = 0U;
	s_dma_fault_m0ar = 0U;
	s_line_dma_start_cycles = 0U;
	s_line_end_pending_cycles = 0U;
	s_href_pending_service_max_us = 0U;
	s_href_pending_overrun_us = 0U;
	s_href_width_active = 0U;
	s_href_start_cycles = 0U;
	s_href_pulse_count = 0U;
	s_href_high_min_us = 0U;
	s_href_high_max_us = 0U;
	s_dma_route_par = 0U;
	s_dma_route_m0ar = 0U;
	s_dma_route_ndtr = 0U;
	s_dma_route_cr = 0U;
	s_dma_arm_to_complete_min_us = 0U;
	s_dma_arm_to_complete_max_us = 0U;
	/* A successful capture overwrites every byte; avoid a 38.4 KiB clear here. */
}
static OV7670_DvpCaptureStatus_t evaluate_frame(void) {
	if (s_dma_error != 0U) {
		return OV7670_DVP_CAPTURE_DMA_ERROR;
	}
	if ((s_frame_href_lines != OV7670_DVP_CAPTURE_TOTAL_HREF_LINES)
			|| (s_leading_lines != OV7670_DVP_CAPTURE_LEADING_HREF_LINES)
			|| (s_active_lines != OV7670_DVP_CAPTURE_HEIGHT)
			|| (s_trailing_lines != OV7670_DVP_CAPTURE_TRAILING_HREF_LINES)) {
		return OV7670_DVP_CAPTURE_HREF_COUNT_ERROR;
	}
	if ((s_active_rows_captured != OV7670_DVP_CAPTURE_HEIGHT)
			|| (s_frame_bytes_written != OV7670_DVP_CAPTURE_FRAME_BYTES)) {
		return (s_active_line_sample_errors != 0U) ?
				OV7670_DVP_CAPTURE_LINE_SAMPLE_ERROR :
				OV7670_DVP_CAPTURE_ACTIVE_ROW_COUNT_ERROR;
	}
	return OV7670_DVP_CAPTURE_OK;
}

static void fill_capture_report(OV7670_DvpCaptureReport_t *report,
		OV7670_DvpCaptureStatus_t *status) {
	uint8_t timer_restore_ok;
	uint8_t exti_restore_ok;
	uint8_t primask;

	if ((report == NULL) || (status == NULL)) {
		return;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	s_capture_armed = 0U;
	__set_PRIMASK(primask);

	__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC4);
	if ((abort_dma_if_active(OV7670_DVP_DMA_FAULT_ABORT_CLEANUP) == 0U)
			&& (*status == OV7670_DVP_CAPTURE_OK)) {
		*status = OV7670_DVP_CAPTURE_DMA_ERROR;
	}

	if (s_dma_callbacks_saved != 0U) {
		hdma_tim1_ch4.XferCpltCallback = s_old_dma_complete_callback;
		hdma_tim1_ch4.XferErrorCallback = s_old_dma_error_callback;
		s_dma_callbacks_saved = 0U;
	}

	timer_restore_ok = restore_tim1_ch4_capture();
	exti_restore_ok = restore_exti();

	report->total_href_lines = s_frame_href_lines;
	report->leading_blank_lines = s_leading_lines;
	report->active_href_lines = s_active_lines;
	report->trailing_blank_lines = s_trailing_lines;
	report->active_rows_captured = s_active_rows_captured;
	report->active_line_samples_min =
			(s_active_line_samples_min == 0xFFFFU) ?
					0U : s_active_line_samples_min;
	report->active_line_samples_max = s_active_line_samples_max;
	report->active_line_sample_errors = s_active_line_sample_errors;
	report->non_active_line_sample_errors = s_non_active_line_sample_errors;
	report->frame_period_ms = s_frame_period_ms;
	report->frame_bytes = (*status == OV7670_DVP_CAPTURE_OK) ?
	OV7670_DVP_CAPTURE_FRAME_BYTES :
																0U;
	report->dma_error = s_dma_error;
	report->dma_fault_kind = s_dma_fault_kind;
	report->dma_state_at_fault = s_dma_state_at_fault;
	report->pending_line_at_exit = s_line_end_pending;
	report->dma_start_hal_status = s_dma_start_hal_status;
	report->dma_abort_hal_status = s_dma_abort_hal_status;
	report->dma_abort_race_recovered = s_dma_abort_race_recovered;
	report->dma_start_count = s_dma_start_count;
	report->dma_complete_count = s_dma_complete_count;
	report->dma_error_count = s_dma_error_count;
	report->dma_error_code = s_dma_error_code;
	report->dma_fault_ndtr = s_dma_fault_ndtr;
	report->dma_fault_cr = s_dma_fault_cr;
	report->dma_fault_fcr = s_dma_fault_fcr;
	report->dma_fault_par = s_dma_fault_par;
	report->dma_fault_m0ar = s_dma_fault_m0ar;
	report->dma_route_par = s_dma_route_par;
	report->dma_route_m0ar = s_dma_route_m0ar;
	report->dma_route_ndtr = s_dma_route_ndtr;
	report->dma_route_cr = s_dma_route_cr;
	report->href_pending_service_max_us = s_href_pending_service_max_us;
	report->href_pending_overrun_us = s_href_pending_overrun_us;
	report->dma_arm_to_complete_min_us = s_dma_arm_to_complete_min_us;
	report->dma_arm_to_complete_max_us = s_dma_arm_to_complete_max_us;
	report->href_pulse_count = s_href_pulse_count;
	report->href_high_min_us = s_href_high_min_us;
	report->href_high_max_us = s_href_high_max_us;
	report->exti_restored = exti_restore_ok;
	report->tim1_ch4_restored = timer_restore_ok;
	report->pclk_pin_restored = (uint8_t) (s_pclk_pin_saved == 0U);

	if ((*status == OV7670_DVP_CAPTURE_OK) && (timer_restore_ok != 0U)
			&& (exti_restore_ok != 0U)) {
		report->frame_crc32 = calculate_crc32(s_frame_buffer,
		OV7670_DVP_CAPTURE_FRAME_BYTES);
	} else if (*status == OV7670_DVP_CAPTURE_OK) {
		*status = OV7670_DVP_CAPTURE_RESTORE_ERROR;
		report->frame_bytes = 0U;
	}
}

OV7670_DvpCaptureStatus_t OV7670_DvpCapture_Start(
		OV7670_DvpCaptureReport_t *report) {
	OV7670_DvpCaptureStatus_t status;

	if (report == NULL) {
		return OV7670_DVP_CAPTURE_BAD_ARGUMENT;
	}
	if (s_async_active != 0U) {
		return OV7670_DVP_CAPTURE_BUSY;
	}
	if (OV7670_IsInitialized() == 0U) {
		s_async_result = OV7670_DVP_CAPTURE_CAMERA_NOT_READY;
		s_async_complete = 1U;
		s_async_state = DVP_ASYNC_COMPLETE;
		return OV7670_DVP_CAPTURE_CAMERA_NOT_READY;
	}

	memset(report, 0, sizeof(*report));
	reset_capture_state();
	s_async_report = report;
	s_async_result = OV7670_DVP_CAPTURE_IN_PROGRESS;
	s_async_start_tick = HAL_GetTick();
	s_async_active = 1U;
	s_async_complete = 0U;
	s_async_state = DVP_ASYNC_WAIT_HREF_IDLE;
	s_dma_callbacks_saved = 0U;

	status = save_and_configure_exti();
	if (status != OV7670_DVP_CAPTURE_OK) {
		(void) restore_exti();
		s_async_active = 0U;
		s_async_complete = 1U;
		s_async_state = DVP_ASYNC_COMPLETE;
		s_async_result = status;
		s_async_report = NULL;
		return status;
	}

	status = prepare_tim1_ch4_capture();
	if (status != OV7670_DVP_CAPTURE_OK) {
		report->tim1_ch4_restored = restore_tim1_ch4_capture();
		report->pclk_pin_restored = (uint8_t) (s_pclk_pin_saved == 0U);
		report->exti_restored = restore_exti();
		s_async_active = 0U;
		s_async_complete = 1U;
		s_async_state = DVP_ASYNC_COMPLETE;
		s_async_result = status;
		s_async_report = NULL;
		return status;
	}

	s_old_dma_complete_callback = hdma_tim1_ch4.XferCpltCallback;
	s_old_dma_error_callback = hdma_tim1_ch4.XferErrorCallback;
	s_dma_callbacks_saved = 1U;
	hdma_tim1_ch4.XferCpltCallback = line_dma_complete;
	hdma_tim1_ch4.XferErrorCallback = line_dma_error;
	return OV7670_DVP_CAPTURE_OK;
}

void OV7670_DvpCapture_Process(void) {
	uint32_t now;
	OV7670_DvpCaptureStatus_t result;

	if (s_async_active == 0U) {
		return;
	}

	now = HAL_GetTick();

	if (s_async_state == DVP_ASYNC_WAIT_HREF_IDLE) {
		if (HAL_GPIO_ReadPin(CAM_HREF_GPIO_Port, CAM_HREF_Pin)
				== GPIO_PIN_RESET) {
			uint8_t primask = __get_PRIMASK();
			__disable_irq();
			s_capture_armed = 1U;
			if (arm_line_dma() == 0U) {
				s_capture_armed = 0U;
				__set_PRIMASK(primask);
				result = OV7670_DVP_CAPTURE_DMA_START_ERROR;
				fill_capture_report(s_async_report, &result);
				s_async_result = result;
				s_async_state = DVP_ASYNC_COMPLETE;
				s_async_active = 0U;
				s_async_complete = 1U;
				s_async_report = NULL;
				return;
			}
			__set_PRIMASK(primask);
			s_async_state = DVP_ASYNC_WAIT_FRAME;
			s_async_start_tick = now;
		} else if ((now - s_async_start_tick)
				>= DVP_CAPTURE_HREF_IDLE_TIMEOUT_MS) {
			result = OV7670_DVP_CAPTURE_HREF_IDLE_TIMEOUT;
			fill_capture_report(s_async_report, &result);
			s_async_result = result;
			s_async_state = DVP_ASYNC_COMPLETE;
			s_async_active = 0U;
			s_async_complete = 1U;
			s_async_report = NULL;
		}
		return;
	}

	if (s_async_state != DVP_ASYNC_WAIT_FRAME) {
		return;
	}

	/* HREF falling-edge line conversion is performed in the HREF EXTI handler.
	 * This foreground path is only a defensive fallback for a pending line. */
	if (s_line_end_pending != 0U) {
		process_pending_href_line();
	}

	if (s_dma_error != 0U) {
		result = OV7670_DVP_CAPTURE_DMA_ERROR;
	} else if (s_capture_done != 0U) {
		if (s_line_end_pending != 0U) {
			process_pending_href_line();
		}
		result = evaluate_frame();
	} else if ((s_frame_active == 0U)
			&& ((now - s_async_start_tick) >= DVP_CAPTURE_VSYNC_TIMEOUT_MS)) {
		result = OV7670_DVP_CAPTURE_VSYNC_TIMEOUT;
	} else if ((s_frame_active != 0U)
			&& ((now - s_frame_start_tick) >= DVP_CAPTURE_FRAME_TIMEOUT_MS)) {
		result = OV7670_DVP_CAPTURE_FRAME_TIMEOUT;
	} else {
		return;
	}

	fill_capture_report(s_async_report, &result);
	s_async_result = result;
	s_async_state = DVP_ASYNC_COMPLETE;
	s_async_active = 0U;
	s_async_complete = 1U;
	s_async_report = NULL;
}

uint8_t OV7670_DvpCapture_IsBusy(void) {
	return s_async_active;
}

uint8_t OV7670_DvpCapture_IsComplete(void) {
	return s_async_complete;
}

OV7670_DvpCaptureStatus_t OV7670_DvpCapture_GetResult(void) {
	return s_async_result;
}

const uint8_t* OV7670_DvpCapture_GetFrameData(uint32_t *length) {
	if (length != NULL) {
		*length = OV7670_DVP_CAPTURE_FRAME_BYTES;
	}
	return &s_frame_buffer[0];
}
const char* OV7670_DvpCapture_StatusString(OV7670_DvpCaptureStatus_t status) {
	switch (status) {
	case OV7670_DVP_CAPTURE_OK:
		return "OK";
	case OV7670_DVP_CAPTURE_BAD_ARGUMENT:
		return "BAD_ARGUMENT";
	case OV7670_DVP_CAPTURE_CAMERA_NOT_READY:
		return "CAMERA_NOT_READY";
	case OV7670_DVP_CAPTURE_EXTI_CONFIG_ERROR:
		return "EXTI_CONFIG_ERROR";
	case OV7670_DVP_CAPTURE_TIM1_CONFIG_ERROR:
		return "TIM1_CH4_CONFIG_ERROR";
	case OV7670_DVP_CAPTURE_DMA_INIT_ERROR:
		return "DMA2_INIT_ERROR";
	case OV7670_DVP_CAPTURE_DMA_START_ERROR:
		return "DMA_START_ERROR";
	case OV7670_DVP_CAPTURE_VSYNC_TIMEOUT:
		return "VSYNC_TIMEOUT";
	case OV7670_DVP_CAPTURE_FRAME_TIMEOUT:
		return "FRAME_TIMEOUT";
	case OV7670_DVP_CAPTURE_HREF_IDLE_TIMEOUT:
		return "HREF_IDLE_TIMEOUT";
	case OV7670_DVP_CAPTURE_DMA_ERROR:
		return "DMA_ERROR";
	case OV7670_DVP_CAPTURE_HREF_COUNT_ERROR:
		return "HREF_COUNT_ERROR";
	case OV7670_DVP_CAPTURE_LINE_SAMPLE_ERROR:
		return "LINE_SAMPLE_ERROR";
	case OV7670_DVP_CAPTURE_ACTIVE_ROW_COUNT_ERROR:
		return "ACTIVE_ROW_COUNT_ERROR";
	case OV7670_DVP_CAPTURE_RESTORE_ERROR:
		return "RESTORE_ERROR";
	case OV7670_DVP_CAPTURE_BUSY:
		return "BUSY";
	case OV7670_DVP_CAPTURE_IN_PROGRESS:
		return "IN_PROGRESS";
	default:
		return "UNKNOWN";
	}
}
