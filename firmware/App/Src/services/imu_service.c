#include "services/imu_service.h"

#include <stddef.h>
#include <string.h>

#include "bsp/diag_uart.h"
#include "drivers/mpu6050_robust_calibration.h"

#define IMU_SERVICE_RUNTIME_ERROR_LIMIT 3U
#define IMU_SERVICE_MONITOR_HARD_LIMIT 30U

static MPU6050_Handle_t s_device;
static MPU6050_Sample_t s_latest_sample;
static uint8_t s_ready;
static uint8_t s_sample_valid;
static uint8_t s_runtime_error_reported;
static uint8_t s_consecutive_errors;

#if (IMU_SERVICE_MONITOR_ENABLE != 0U)
static uint8_t s_monitor_complete;
static uint16_t s_monitor_lines;
static uint32_t s_last_log_tick;
#endif

static uint32_t s_last_sample_tick;
static uint32_t s_latest_sample_tick;
static uint32_t s_sample_sequence;

MPU6050_Status_t ImuService_Init(I2C_HandleTypeDef *hi2c, uint8_t address_7bit) {
	MPU6050_Config_t config;
	MPU6050_Status_t status;

	s_ready = 0U;
	s_sample_valid = 0U;
	s_runtime_error_reported = 0U;
	s_consecutive_errors = 0U;
	s_sample_sequence = 0U;
	s_latest_sample_tick = 0U;
#if (IMU_SERVICE_MONITOR_ENABLE != 0U)
	s_monitor_complete = 0U;
	s_monitor_lines = 0U;
#endif

	MPU6050_GetDefaultConfig(&config);

	/* Keep 200 Hz and 94 Hz DLPF to preserve impact time resolution.
	 The IIR stage smooths display/slow logic; shock detection can use raw fields. */
	config.dlpf = MPU6050_DLPF_94HZ;
	config.sample_rate_hz = 200U;
	config.accel_filter_alpha = 0.12f;
	config.gyro_filter_alpha = 0.10f;

	status = MPU6050_Init(hi2c, &s_device, address_7bit, &config);
	if (status != MPU6050_STATUS_OK) {
		return status;
	}

	s_ready = 1U;
	s_last_sample_tick = HAL_GetTick();
#if (IMU_SERVICE_MONITOR_ENABLE != 0U)
	s_last_log_tick = s_last_sample_tick;
#endif
	return MPU6050_STATUS_OK;
}

MPU6050_Status_t ImuService_CalibrateStationary(uint16_t sample_count,
		uint32_t sample_period_ms, MPU6050_CalibrationInfo_t *info) {
	if (s_ready == 0U) {
		return MPU6050_STATUS_NOT_INITIALIZED;
	}

	/* Match the configured 200 Hz sensor cadence (5 ms minimum). */
	if (sample_period_ms < 5U) {
		sample_period_ms = 5U;
	}
	{
		MPU6050_Status_t status;
		status = MPU6050_CalibrateStationaryRobust(&s_device, sample_count,
				sample_period_ms, info);
		/* One complete fresh window tolerates a transient startup disturbance. */
		if (status == MPU6050_STATUS_NOT_STATIONARY) {
			DiagUART_Printf(
					"[IMU] Calibration window unstable; retrying once after settling\r\n");
			status = MPU6050_CalibrateStationaryRobust(&s_device, sample_count,
					sample_period_ms, info);
		}
		return status;
	}
}

MPU6050_Status_t ImuService_CalibrateGyro(uint16_t sample_count,
		uint32_t sample_period_ms) {
	return ImuService_CalibrateStationary(sample_count, sample_period_ms, NULL);
}

void ImuService_RunOnce(void) {
	const uint32_t now = HAL_GetTick();
	const uint16_t sample_rate_hz = MPU6050_GetSampleRateHz(&s_device);
	uint32_t sample_period_ms;

	if ((s_ready == 0U) || (sample_rate_hz == 0U)) {
		return;
	}

	sample_period_ms = 1000U / (uint32_t) sample_rate_hz;
	if ((uint32_t) (now - s_last_sample_tick) >= sample_period_ms) {
		MPU6050_Status_t status;
		/* Do not perform catch-up bursts after a delayed foreground iteration. */
		s_last_sample_tick = now;
		status = MPU6050_ReadSample(&s_device, &s_latest_sample);
		if (status == MPU6050_STATUS_OK) {
			s_sample_valid = 1U;
			s_latest_sample_tick = now;
			++s_sample_sequence;
			s_consecutive_errors = 0U;
			s_runtime_error_reported = 0U;
		} else {
			s_sample_valid = 0U;
			if (s_consecutive_errors < 255U) {
				++s_consecutive_errors;
			}
		}

		if ((s_consecutive_errors >= IMU_SERVICE_RUNTIME_ERROR_LIMIT)
				&& (s_runtime_error_reported == 0U)) {
			DiagUART_Printf(
					"[IMU] Repeated I2C read failures; live sensor data unavailable\r\n");
			s_runtime_error_reported = 1U;
		}
	}

#if (IMU_SERVICE_MONITOR_ENABLE != 0U)
	if ((s_monitor_complete == 0U) && (s_sample_valid != 0U) &&
		((uint32_t)(now - s_last_log_tick) >= IMU_SERVICE_MONITOR_PERIOD_MS))
	{
		const long ax_mg = (long)(s_latest_sample.accel_filtered_g[0] * 1000.0f);
		const long ay_mg = (long)(s_latest_sample.accel_filtered_g[1] * 1000.0f);
		const long az_mg = (long)(s_latest_sample.accel_filtered_g[2] * 1000.0f);
		const long gx_mdps = (long)(s_latest_sample.gyro_filtered_dps[0] * 1000.0f);
		const long gy_mdps = (long)(s_latest_sample.gyro_filtered_dps[1] * 1000.0f);
		const long gz_mdps = (long)(s_latest_sample.gyro_filtered_dps[2] * 1000.0f);
		const long temp_mC = (long)(s_latest_sample.temperature_c * 1000.0f);
		const long accel_norm_mg = (long)(s_latest_sample.accel_magnitude_g * 1000.0f);

		s_last_log_tick = now;
		DiagUART_Printf("[IMU] A[mg]=%+ld,%+ld,%+ld |A|=%ld G[mdps]=%+ld,%+ld,%+ld Tdie[mC]=%ld\r\n",
						ax_mg, ay_mg, az_mg, accel_norm_mg,
						gx_mdps, gy_mdps, gz_mdps, temp_mC);
		if (s_monitor_lines < IMU_SERVICE_MONITOR_HARD_LIMIT)
		{
			++s_monitor_lines;
		}
		if (s_monitor_lines >= IMU_SERVICE_MONITOR_HARD_LIMIT)
		{
			s_monitor_complete = 1U;
			DiagUART_Printf("[IMU] Monitor finished (%u samples); acquisition remains active.\r\n",
							(unsigned int)s_monitor_lines);
		}
	}
#endif
}

bool ImuService_IsReady(void) {
	return (s_ready != 0U);
}

bool ImuService_IsGyroCalibrated(void) {
	return (s_device.gyro_calibrated != 0U);
}

bool ImuService_IsAccelScaleCalibrated(void) {
	return (s_device.accel_scale_calibrated != 0U);
}

const MPU6050_Handle_t* ImuService_GetDevice(void) {
	return (s_ready != 0U) ? &s_device : NULL;
}

const MPU6050_Sample_t* ImuService_GetLatestSample(void) {
	return (s_sample_valid != 0U) ? &s_latest_sample : NULL;
}

uint32_t ImuService_GetSampleSequence(void) {
	return s_sample_sequence;
}

uint32_t ImuService_GetLatestSampleTick(void) {
	return s_latest_sample_tick;
}
