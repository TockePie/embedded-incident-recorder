#ifndef OV7670_DVP_CAPTURE_H
#define OV7670_DVP_CAPTURE_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OV7670_DVP_CAPTURE_WIDTH                 160U
#define OV7670_DVP_CAPTURE_HEIGHT                120U
#define OV7670_DVP_CAPTURE_BYTES_PER_PIXEL       2U
#define OV7670_DVP_CAPTURE_LINE_BYTES            320U
#define OV7670_DVP_CAPTURE_FRAME_BYTES           38400UL
#define OV7670_DVP_CAPTURE_LEADING_HREF_LINES    5U
#define OV7670_DVP_CAPTURE_TRAILING_HREF_LINES   2U
#define OV7670_DVP_CAPTURE_TOTAL_HREF_LINES      127U

typedef enum {
	OV7670_DVP_CAPTURE_OK = 0,
	OV7670_DVP_CAPTURE_BAD_ARGUMENT,
	OV7670_DVP_CAPTURE_CAMERA_NOT_READY,
	OV7670_DVP_CAPTURE_EXTI_CONFIG_ERROR,
	OV7670_DVP_CAPTURE_TIM1_CONFIG_ERROR,
	OV7670_DVP_CAPTURE_DMA_INIT_ERROR,
	OV7670_DVP_CAPTURE_DMA_START_ERROR,
	OV7670_DVP_CAPTURE_VSYNC_TIMEOUT,
	OV7670_DVP_CAPTURE_FRAME_TIMEOUT,
	OV7670_DVP_CAPTURE_HREF_IDLE_TIMEOUT,
	OV7670_DVP_CAPTURE_DMA_ERROR,
	OV7670_DVP_CAPTURE_HREF_COUNT_ERROR,
	OV7670_DVP_CAPTURE_LINE_SAMPLE_ERROR,
	OV7670_DVP_CAPTURE_ACTIVE_ROW_COUNT_ERROR,
	OV7670_DVP_CAPTURE_RESTORE_ERROR,
	OV7670_DVP_CAPTURE_BUSY,
	OV7670_DVP_CAPTURE_IN_PROGRESS
} OV7670_DvpCaptureStatus_t;

typedef struct {
	uint16_t total_href_lines;
	uint16_t leading_blank_lines;
	uint16_t active_href_lines;
	uint16_t trailing_blank_lines;
	uint16_t active_rows_captured;
	uint16_t active_line_samples_min;
	uint16_t active_line_samples_max;
	uint16_t active_line_sample_errors;
	uint16_t non_active_line_sample_errors;
	uint32_t frame_period_ms;
	uint32_t frame_bytes;
	uint32_t frame_crc32;
	uint8_t dma_error;
	uint8_t dma_fault_kind;
	uint8_t dma_state_at_fault;
	uint8_t pending_line_at_exit;
	uint8_t dma_start_hal_status;
	uint8_t dma_abort_hal_status;
	uint16_t dma_abort_race_recovered;
	uint16_t dma_start_count;
	uint16_t dma_complete_count;
	uint16_t dma_error_count;
	uint32_t dma_error_code;
	uint32_t dma_fault_ndtr;
	uint32_t dma_fault_cr;
	uint32_t dma_fault_fcr;
	uint32_t dma_fault_par;
	uint32_t dma_fault_m0ar;
	uint32_t dma_route_par;
	uint32_t dma_route_m0ar;
	uint32_t dma_route_ndtr;
	uint32_t dma_route_cr;
	uint32_t href_pending_service_max_us;
	uint32_t href_pending_overrun_us;
	/* Includes any remaining HREF-low blanking wait; it is not the active
	 * HREF pulse width and is intended for diagnostics only. */
	uint32_t dma_arm_to_complete_min_us;
	uint32_t dma_arm_to_complete_max_us;
	uint16_t href_pulse_count;
	uint32_t href_high_min_us;
	uint32_t href_high_max_us;
	uint8_t exti_restored;
	uint8_t tim1_ch4_restored;
	uint8_t pclk_pin_restored;
} OV7670_DvpCaptureReport_t;

#define OV7670_DVP_DMA_FAULT_NONE                 0U
#define OV7670_DVP_DMA_FAULT_IRQ_ERROR            1U
#define OV7670_DVP_DMA_FAULT_ABORT_REARM          2U
#define OV7670_DVP_DMA_FAULT_START_REARM          3U
#define OV7670_DVP_DMA_FAULT_NDTR_RANGE           4U
#define OV7670_DVP_DMA_FAULT_ABORT_LINE_END       5U
#define OV7670_DVP_DMA_FAULT_ABORT_CLEANUP        6U
#define OV7670_DVP_DMA_FAULT_HREF_PENDING_OVERRUN 7U

/* Forward GPIO EXTI events to the frame-capture state machine. */
void OV7670_DvpCapture_OnExti(uint16_t gpio_pin);

/* Dispatch the DMA2 Stream4 interrupt to the capture state machine. */
void OV7670_DvpCapture_DmaIrqHandler(void);

/* Start and service a frame capture without a blocking wait loop. */
OV7670_DvpCaptureStatus_t OV7670_DvpCapture_Start(
		OV7670_DvpCaptureReport_t *report);
void OV7670_DvpCapture_Process(void);
uint8_t OV7670_DvpCapture_IsBusy(void);
uint8_t OV7670_DvpCapture_IsComplete(void);
OV7670_DvpCaptureStatus_t OV7670_DvpCapture_GetResult(void);
const uint8_t *OV7670_DvpCapture_GetFrameData(uint32_t *length);
const char *OV7670_DvpCapture_StatusString(OV7670_DvpCaptureStatus_t status);

#ifdef __cplusplus
}
#endif

#endif /* OV7670_DVP_CAPTURE_H */
