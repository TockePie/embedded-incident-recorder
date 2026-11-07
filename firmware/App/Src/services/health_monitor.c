#include "services/health_monitor.h"

#include <string.h>

#include "app/app.h"
#include "bsp/diag_uart.h"
#include "drivers/ov7670.h"
#include "drivers/w25q64.h"
#include "services/health_policy.h"
#include "services/cli_service.h"
#include "services/imu_service.h"
#include "services/incident_detector.h"
#include "spi.h"
#include "usart.h"

#define HEALTH_FAULT_MPU_STALE       0x00050001UL
#define HEALTH_FAULT_FLASH_IO        0x00050002UL
#define HEALTH_MPU_GRACE_MS          HEALTH_MONITOR_STARTUP_GRACE_MS
#define HEALTH_CAMERA_PID_REGISTER  0x0AU
#define HEALTH_CAMERA_VER_REGISTER  0x0BU
#define HEALTH_CAMERA_PID_EXPECTED  0x76U

static HealthMonitorSnapshot_t s_snapshot;
static uint32_t s_start_tick;
static uint32_t s_last_flash_check_tick;
static uint32_t s_last_camera_check_tick;
static uint32_t s_last_queue_drop_count;
static uint32_t s_last_uart_rx_overflow;
static uint32_t s_last_uart_tx_drop;
static uint32_t s_last_uart_hal_error_code;
static uint8_t s_flash_error_streak;
static uint8_t s_camera_error_streak;
static uint8_t s_fault_latched;
static uint8_t s_camera_not_init_reported;

static void HealthMonitor_IncrementSaturating(uint8_t *value) {
	if ((value != NULL) && (*value < 255U)) {
		++(*value);
	}
}

static void HealthMonitor_CheckFlash(uint32_t now_ms) {
	bool busy = false;
	uint8_t jedec[3] = { 0U, 0U, 0U };
	W25Q64_StatusRegisters_t registers;
	W25Q64_Status_t status;

	if ((uint32_t) (now_ms - s_last_flash_check_tick) <
	HEALTH_MONITOR_FLASH_PERIOD_MS) {
		return;
	}
	s_last_flash_check_tick = now_ms;

	status = W25Q64_IsBusy(&hspi2, &busy);
	if (status != W25Q64_STATUS_OK) {
		HealthMonitor_IncrementSaturating(&s_flash_error_streak);
		++s_snapshot.flash_error_count;
		s_snapshot.flash_state = HEALTH_DEVICE_DEGRADED;
		DiagUART_Printf(
				"[HEALTH] W25Q64 BUSY/status poll error=%u streak=%u\r\n",
				(unsigned int) status, (unsigned int) s_flash_error_streak);
		return;
	}

	/* A program/erase cycle is normal. Do not issue ID/status transactions while
	 the flash is internally busy; defer this health sample to the next period. */
	if (busy) {
		s_snapshot.flash_state = HEALTH_DEVICE_BUSY;
		return;
	}

	status = W25Q64_ReadJedecId(&hspi2, jedec);
	if ((status == W25Q64_STATUS_OK)
			&& ((jedec[0] != W25Q64_JEDEC_MANUFACTURER_ID)
					|| (jedec[1] != W25Q64_JEDEC_MEMORY_TYPE)
					|| (jedec[2] != W25Q64_JEDEC_CAPACITY))) {
		status = W25Q64_STATUS_UNEXPECTED_ID;
	}

	if (status == W25Q64_STATUS_OK) {
		status = W25Q64_ReadStatusRegisters(&hspi2, &registers);
	}

	if ((status == W25Q64_STATUS_OK) && (!registers.busy)) {
		if ((s_flash_error_streak != 0U)
				|| (s_snapshot.flash_state != HEALTH_DEVICE_OK)) {
			DiagUART_Printf(
					"[HEALTH] W25Q64 runtime check PASS JEDEC=%02X %02X %02X\r\n",
					jedec[0], jedec[1], jedec[2]);
		}
		s_flash_error_streak = 0U;
		s_snapshot.flash_state = HEALTH_DEVICE_OK;
	} else {
		HealthMonitor_IncrementSaturating(&s_flash_error_streak);
		++s_snapshot.flash_error_count;
		s_snapshot.flash_state = HEALTH_DEVICE_DEGRADED;
		DiagUART_Printf(
				"[HEALTH] W25Q64 runtime check failed status=%u streak=%u\r\n",
				(unsigned int) status, (unsigned int) s_flash_error_streak);
	}
}

static void HealthMonitor_CheckCamera(uint32_t now_ms) {
	uint8_t pid = 0U;
	uint8_t version = 0U;
	OV7670_Status_t status;

	if ((uint32_t) (now_ms - s_last_camera_check_tick) <
	HEALTH_MONITOR_CAMERA_PERIOD_MS) {
		return;
	}
	s_last_camera_check_tick = now_ms;

	if (OV7670_IsInitialized() == 0U) {
		s_snapshot.camera_state = HEALTH_DEVICE_NOT_INITIALIZED;
		s_snapshot.photos_available = false;
		if (s_camera_not_init_reported == 0U) {
			DiagUART_Printf(
					"[HEALTH] OV7670 not initialized by application; photo capture remains unavailable (not a critical fault)\r\n");
			s_camera_not_init_reported = 1U;
		}
		return;
	}

	status = OV7670_ReadRegister(HEALTH_CAMERA_PID_REGISTER, &pid);
	if (status == OV7670_STATUS_OK) {
		status = OV7670_ReadRegister(HEALTH_CAMERA_VER_REGISTER, &version);
	}
	if ((status == OV7670_STATUS_OK)
			&& ((pid != HEALTH_CAMERA_PID_EXPECTED)
					|| ((version != 0x73U) && (version != 0x70U)))) {
		status = OV7670_STATUS_ID_MISMATCH;
	}

	if (status == OV7670_STATUS_OK) {
		if (s_camera_error_streak != 0U) {
			DiagUART_Printf(
					"[HEALTH] OV7670 SCCB communication recovered PID=0x%02X VER=0x%02X\r\n",
					(unsigned int) pid, (unsigned int) version);
		}
		if (s_snapshot.camera_state != HEALTH_DEVICE_OK) {
			DiagUART_Printf(
					"[HEALTH] OV7670 runtime SCCB check PASS PID=0x%02X VER=0x%02X; photos available\r\n",
					(unsigned int) pid, (unsigned int) version);
		}
		s_camera_error_streak = 0U;
		s_snapshot.camera_state = HEALTH_DEVICE_OK;
		s_snapshot.photos_available = true;
	} else {
		HealthMonitor_IncrementSaturating(&s_camera_error_streak);
		++s_snapshot.camera_error_count;
		s_snapshot.camera_state = HEALTH_DEVICE_DEGRADED;
		s_snapshot.photos_available = false;
		if (s_camera_error_streak == HEALTH_POLICY_CAMERA_ERROR_LIMIT) {
			DiagUART_Printf(
					"[HEALTH] OV7670 runtime failure; photos disabled, recorder continues (status=%u)\r\n",
					(unsigned int) status);
		}
	}
}

static bool HealthMonitor_CheckMpu(uint32_t now_ms) {
	uint32_t sample_tick;

	if (!ImuService_IsReady()) {
		s_snapshot.mpu_state = HEALTH_DEVICE_FAILED;
		s_snapshot.mpu_sample_age_ms = UINT32_MAX;
		return false;
	}

	sample_tick = ImuService_GetLatestSampleTick();
	if ((sample_tick == 0U)
			&& ((uint32_t) (now_ms - s_start_tick) < HEALTH_MPU_GRACE_MS)) {
		s_snapshot.mpu_state = HEALTH_DEVICE_UNKNOWN;
		s_snapshot.mpu_sample_age_ms = 0U;
		return true;
	}

	s_snapshot.mpu_sample_age_ms = (uint32_t) (now_ms - sample_tick);
	if ((sample_tick == 0U)
			|| (s_snapshot.mpu_sample_age_ms > HEALTH_MONITOR_MPU_STALE_LIMIT_MS)) {
		s_snapshot.mpu_state = HEALTH_DEVICE_FAILED;
		return false;
	}

	if (s_snapshot.mpu_state != HEALTH_DEVICE_OK) {
		DiagUART_Printf("[HEALTH] MPU-6050 sample stream OK age=%lums\r\n",
				(unsigned long) s_snapshot.mpu_sample_age_ms);
	}
	s_snapshot.mpu_state = HEALTH_DEVICE_OK;
	return true;
}

static void HealthMonitor_CheckQueueAndUart(bool *queue_drop_delta,
bool *uart_error_delta) {
	IncidentDetectorStats_t detector_stats;
	uint32_t rx_overflow;
	uint32_t tx_drop;

	if (queue_drop_delta != NULL) {
		*queue_drop_delta = false;
	}
	if (uart_error_delta != NULL) {
		*uart_error_delta = false;
	}

	IncidentDetector_GetStats(&detector_stats);
	s_snapshot.queue_drop_count = detector_stats.queue_dropped;
	if (detector_stats.queue_dropped != s_last_queue_drop_count) {
		uint32_t delta = detector_stats.queue_dropped - s_last_queue_drop_count;
		s_last_queue_drop_count = detector_stats.queue_dropped;
		if (queue_drop_delta != NULL) {
			*queue_drop_delta = true;
		}
		DiagUART_Printf(
				"[HEALTH] incident queue overflow: dropped_delta=%lu total=%lu; newest events are discarded, recorder remains operational\r\n",
				(unsigned long) delta,
				(unsigned long) detector_stats.queue_dropped);
	}

	rx_overflow = CliService_GetRxOverflowCount();
	tx_drop = CliService_GetTxDropCount();
	s_snapshot.uart_rx_overflow_count = rx_overflow;
	s_snapshot.uart_tx_drop_count = tx_drop;

	s_snapshot.uart_hal_error_code = huart1.ErrorCode;
	if ((rx_overflow != s_last_uart_rx_overflow)
			|| (tx_drop != s_last_uart_tx_drop)
			|| (huart1.ErrorCode != s_last_uart_hal_error_code)) {
		if (uart_error_delta != NULL) {
			*uart_error_delta = true;
		}
		DiagUART_Printf(
				"[HEALTH] USART1 CLI transport warning rx_overflow=%lu tx_drop=%lu HAL_ErrorCode=0x%08lX; autonomous monitoring continues\r\n",
				(unsigned long) rx_overflow, (unsigned long) tx_drop,
				(unsigned long) huart1.ErrorCode);
		s_last_uart_rx_overflow = rx_overflow;
		s_last_uart_tx_drop = tx_drop;
		if (huart1.ErrorCode != s_last_uart_hal_error_code) {
			++s_snapshot.uart_hal_error_count;
		}
		s_last_uart_hal_error_code = huart1.ErrorCode;
	}
}

void HealthMonitor_Init(uint32_t now_ms) {
	memset(&s_snapshot, 0, sizeof(s_snapshot));
	s_snapshot.mpu_state = HEALTH_DEVICE_UNKNOWN;
	s_snapshot.flash_state = HEALTH_DEVICE_UNKNOWN;
	s_snapshot.camera_state =
			(OV7670_IsInitialized() != 0U) ?
					HEALTH_DEVICE_UNKNOWN : HEALTH_DEVICE_NOT_INITIALIZED;
	s_snapshot.photos_available = (OV7670_IsInitialized() != 0U);

	s_start_tick = now_ms;
	s_last_flash_check_tick = now_ms;
	s_last_camera_check_tick = now_ms;
	s_last_queue_drop_count = 0U;
	s_last_uart_rx_overflow = CliService_GetRxOverflowCount();
	s_last_uart_tx_drop = CliService_GetTxDropCount();
	s_last_uart_hal_error_code = huart1.ErrorCode;
	s_snapshot.uart_hal_error_code = huart1.ErrorCode;
	s_flash_error_streak = 0U;
	s_camera_error_streak = 0U;
	s_fault_latched = 0U;
	s_camera_not_init_reported = 0U;

	DiagUART_Printf(
			"[HEALTH] runtime monitor initialized (MPU stale>%ums, flash/camera poll=%ums)\r\n",
			(unsigned int) HEALTH_MONITOR_MPU_STALE_LIMIT_MS,
			(unsigned int) HEALTH_MONITOR_FLASH_PERIOD_MS);
}

void HealthMonitor_Process(uint32_t now_ms) {
	HealthPolicyInput_t policy_input;
	uint32_t actions;
	bool mpu_ok;
	bool queue_drop_delta = false;
	bool uart_error_delta = false;

	mpu_ok = HealthMonitor_CheckMpu(now_ms);
	HealthMonitor_CheckFlash(now_ms);
	HealthMonitor_CheckCamera(now_ms);
	HealthMonitor_CheckQueueAndUart(&queue_drop_delta, &uart_error_delta);

	policy_input.mpu_sample_fresh = mpu_ok;
	policy_input.flash_error_streak = s_flash_error_streak;
	policy_input.camera_error_streak = s_camera_error_streak;
	policy_input.queue_drop_delta = queue_drop_delta;
	policy_input.uart_error_delta = uart_error_delta;
	actions = HealthPolicy_Evaluate(&policy_input);

	/* Preserve the original latched cause if startup or another subsystem has
	 already placed the application in FAULT. */
	if (App_GetState() == APP_STATE_FAULT) {
		s_fault_latched = 1U;
	}

	if (((actions & HEALTH_POLICY_ACTION_MPU_FAULT) != 0U)
			&& (s_fault_latched == 0U)) {
		s_fault_latched = 1U;
		s_snapshot.fault_code = HEALTH_FAULT_MPU_STALE;
		DiagUART_Printf(
				"[HEALTH] CRITICAL: MPU-6050 sample stream lost (age=%lums); entering latched FAULT\r\n",
				(unsigned long) s_snapshot.mpu_sample_age_ms);
		App_EnterFault(HEALTH_FAULT_MPU_STALE);
	}

	if (((actions & HEALTH_POLICY_ACTION_FLASH_FAULT) != 0U)
			&& (s_fault_latched == 0U)) {
		s_fault_latched = 1U;
		s_snapshot.fault_code = HEALTH_FAULT_FLASH_IO;
		DiagUART_Printf(
				"[HEALTH] CRITICAL: W25Q64 communication failed repeatedly; entering latched FAULT\r\n");
		App_EnterFault(HEALTH_FAULT_FLASH_IO);
	}
}

void HealthMonitor_GetSnapshot(HealthMonitorSnapshot_t *snapshot) {
	if (snapshot != NULL) {
		*snapshot = s_snapshot;
	}
}

const char* HealthMonitor_DeviceStateName(HealthDeviceState_t state) {
	switch (state) {
	case HEALTH_DEVICE_UNKNOWN:
		return "UNKNOWN";
	case HEALTH_DEVICE_OK:
		return "OK";
	case HEALTH_DEVICE_NOT_INITIALIZED:
		return "NOT_INITIALIZED";
	case HEALTH_DEVICE_DEGRADED:
		return "DEGRADED";
	case HEALTH_DEVICE_FAILED:
		return "FAILED";
	case HEALTH_DEVICE_BUSY:
		return "BUSY";
	default:
		return "INVALID";
	}
}
