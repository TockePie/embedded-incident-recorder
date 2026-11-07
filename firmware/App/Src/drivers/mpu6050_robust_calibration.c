#include "drivers/mpu6050_robust_calibration.h"

#include <stddef.h>
#include <string.h>

/* Robust dispersion limits, evaluated as 1.4826 * median absolute deviation. */
#define ROBUST_GYRO_SIGMA_LIMIT_DPS       0.75f
#define ROBUST_ACCEL_SIGMA_LIMIT_G        0.040f
#define ROBUST_MEAN_GYRO_LIMIT_DPS        20.0f
#define ROBUST_MIN_GRAVITY_NORM_G         0.50f
#define ROBUST_MAX_GRAVITY_NORM_G         1.50f
#define ROBUST_MIN_ACCEL_GAIN             0.65f
#define ROBUST_MAX_ACCEL_GAIN             1.35f
#define MAD_TO_SIGMA                      1.4826f

/* Static storage avoids putting several kilobytes on the STM32 stack. */
static float s_values[6][MPU6050_ROBUST_CAL_MAX_SAMPLES];
static float s_scratch[MPU6050_ROBUST_CAL_MAX_SAMPLES];

static void sort_float(float *values, uint16_t count) {
	uint16_t i;
	for (i = 1U; i < count; ++i) {
		const float key = values[i];
		uint16_t j = i;
		while ((j > 0U) && (values[j - 1U] > key)) {
			values[j] = values[j - 1U];
			--j;
		}
		values[j] = key;
	}
}

static float median_in_place(float *values, uint16_t count) {
	sort_float(values, count);
	if ((count & 1U) != 0U) {
		return values[count / 2U];
	}
	return 0.5f * (values[(count / 2U) - 1U] + values[count / 2U]);
}

static float median_absolute_deviation(const float *values, uint16_t count,
		float median) {
	uint16_t i;
	for (i = 0U; i < count; ++i) {
		const float delta = values[i] - median;
		s_scratch[i] = (delta < 0.0f) ? -delta : delta;
	}
	return median_in_place(s_scratch, count);
}

static float sqrt_local(float value) {
	float x;
	uint32_t i;
	if (value <= 0.0f) {
		return 0.0f;
	}
	x = (value >= 1.0f) ? value : 1.0f;
	for (i = 0U; i < 12U; ++i) {
		x = 0.5f * (x + value / x);
	}
	return x;
}

MPU6050_Status_t MPU6050_CalibrateStationaryRobust(MPU6050_Handle_t *device,
		uint16_t sample_count, uint32_t sample_period_ms,
		MPU6050_CalibrationInfo_t *info) {
	MPU6050_CalibrationInfo_t result;
	float medians[6];
	float robust_sigma[6];
	float gyro_norm_sq;
	float gravity_norm_sq;
	float gravity_norm;
	float accel_gain;
	uint16_t n;
	uint32_t axis;
	MPU6050_Sample_t sample;

	memset(&result, 0, sizeof(result));
	result.accel_scale_correction = 1.0f;

	if ((device == NULL) || (device->initialized == 0U)) {
		return MPU6050_STATUS_NOT_INITIALIZED;
	}
	if ((sample_count < 100U) || (sample_count > MPU6050_ROBUST_CAL_MAX_SAMPLES)
			|| (sample_period_ms < 5U)) {
		return MPU6050_STATUS_INVALID_PARAM;
	}

	/* Never leave an old calibration active if this attempt fails. */
	device->gyro_calibrated = 0U;
	device->accel_scale_calibrated = 0U;
	device->accel_scale_correction = 1.0f;
	memset(device->gyro_bias_dps, 0, sizeof(device->gyro_bias_dps));
	MPU6050_ResetFilters(device);

	/* Extra settling period: discard complete samples after sensor startup. */
	for (n = 0U; n < MPU6050_ROBUST_CAL_DISCARD_SAMPLES; ++n) {
		if (MPU6050_ReadSample(device, &sample) != MPU6050_STATUS_OK) {
			if (info != NULL) {
				*info = result;
			}
			return MPU6050_STATUS_CALIBRATION_ERROR;
		}
		HAL_Delay(sample_period_ms);
	}

	/* Store unfiltered physical values. ReadSample's *_dps and *_g fields
	 are deliberately used, not its display-oriented IIR outputs. */
	for (n = 0U; n < sample_count; ++n) {
		if (MPU6050_ReadSample(device, &sample) != MPU6050_STATUS_OK) {
			if (info != NULL) {
				*info = result;
			}
			return MPU6050_STATUS_CALIBRATION_ERROR;
		}
		for (axis = 0U; axis < 3U; ++axis) {
			s_values[axis][n] = sample.accel_g[axis];
			s_values[axis + 3U][n] = sample.gyro_dps[axis];
		}
		if ((n + 1U) < sample_count) {
			HAL_Delay(sample_period_ms);
		}
	}

	for (axis = 0U; axis < 6U; ++axis) {
		float mad_sigma;
		medians[axis] = median_in_place(s_values[axis], sample_count);
		/* median_in_place sorts the source array; no later code relies on order. */
		mad_sigma = median_absolute_deviation(s_values[axis], sample_count,
				medians[axis]) * MAD_TO_SIGMA;
		robust_sigma[axis] = mad_sigma;
	}

	result.max_accel_stddev_g = robust_sigma[0];
	result.max_gyro_stddev_dps = robust_sigma[3];
	for (axis = 1U; axis < 3U; ++axis) {
		if (robust_sigma[axis] > result.max_accel_stddev_g) {
			result.max_accel_stddev_g = robust_sigma[axis];
		}
		if (robust_sigma[axis + 3U] > result.max_gyro_stddev_dps) {
			result.max_gyro_stddev_dps = robust_sigma[axis + 3U];
		}
	}

	gyro_norm_sq = medians[3] * medians[3] + medians[4] * medians[4]
			+ medians[5] * medians[5];
	gravity_norm_sq = medians[0] * medians[0] + medians[1] * medians[1]
			+ medians[2] * medians[2];
	gravity_norm = sqrt_local(gravity_norm_sq);

	for (axis = 0U; axis < 3U; ++axis) {
		result.gyro_bias_dps[axis] = medians[axis + 3U];
	}
	result.accel_gravity_norm_before_g = gravity_norm;

	if (info != NULL) {
		*info = result;
	}

	if ((result.max_gyro_stddev_dps > ROBUST_GYRO_SIGMA_LIMIT_DPS)
			|| (result.max_accel_stddev_g > ROBUST_ACCEL_SIGMA_LIMIT_G)
			|| (sqrt_local(gyro_norm_sq) > ROBUST_MEAN_GYRO_LIMIT_DPS)) {
		return MPU6050_STATUS_NOT_STATIONARY;
	}
	if ((gravity_norm < ROBUST_MIN_GRAVITY_NORM_G)
			|| (gravity_norm > ROBUST_MAX_GRAVITY_NORM_G)) {
		return MPU6050_STATUS_GRAVITY_OUT_OF_RANGE;
	}

	accel_gain = 1.0f / gravity_norm;
	if ((accel_gain < ROBUST_MIN_ACCEL_GAIN)
			|| (accel_gain > ROBUST_MAX_ACCEL_GAIN)) {
		return MPU6050_STATUS_GRAVITY_OUT_OF_RANGE;
	}

	for (axis = 0U; axis < 3U; ++axis) {
		device->gyro_bias_dps[axis] = medians[axis + 3U];
	}
	device->accel_scale_correction = accel_gain;
	device->accel_gravity_norm_before_g = gravity_norm;
	device->gyro_calibrated = 1U;
	device->accel_scale_calibrated = 1U;

	result.accel_scale_correction = accel_gain;
	result.gyro_calibrated = 1U;
	result.accel_scale_calibrated = 1U;
	if (info != NULL) {
		*info = result;
	}
	MPU6050_ResetFilters(device);
	return MPU6050_STATUS_OK;
}
