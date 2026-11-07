#include "drivers/mpu6050.h"

#include <stddef.h>
#include <string.h>

#define REG_SMPLRT_DIV        0x19U
#define REG_CONFIG            0x1AU
#define REG_GYRO_CONFIG       0x1BU
#define REG_ACCEL_CONFIG      0x1CU
#define REG_ACCEL_XOUT_H      0x3BU
#define REG_SIGNAL_PATH_RESET 0x68U
#define REG_PWR_MGMT_1        0x6BU
#define REG_PWR_MGMT_2        0x6CU
#define I2C_TIMEOUT_MS        25U

/* Startup stationarity limits. Variance is used, not absolute gyro mean. */
#define CAL_MIN_SAMPLES             50U
#define CAL_MAX_GYRO_STDDEV_DPS     0.50f
#define CAL_MAX_ACCEL_STDDEV_G      0.040f
#define CAL_MAX_MEAN_GYRO_NORM_DPS  20.0f
#define CAL_MIN_GRAVITY_NORM_G      0.50f
#define CAL_MAX_GRAVITY_NORM_G      1.50f

/* Plausibility limits for single-pose common gravity-scale calibration. */
#define CAL_MIN_ACCEL_GAIN          0.65f
#define CAL_MAX_ACCEL_GAIN          1.35f

static const float s_accel_lsb_per_g[4] =
		{ 16384.0f, 8192.0f, 4096.0f, 2048.0f };

static const float s_gyro_lsb_per_dps[4] = { 131.0f, 65.5f, 32.8f, 16.4f };

static HAL_StatusTypeDef read_regs(I2C_HandleTypeDef *hi2c, uint16_t address,
		uint8_t reg, uint8_t *data, uint16_t length) {
	return HAL_I2C_Mem_Read(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, data,
			length, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint16_t address,
		uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, &value,
			1U, I2C_TIMEOUT_MS);
}

static int16_t parse_i16(const uint8_t *data) {
	const uint16_t value = (uint16_t) (((uint16_t) data[0] << 8U)
			| (uint16_t) data[1]);
	return (int16_t) value;
}

/* Small, bounded Newton iteration avoids requiring a separate libm sqrt link. */
static float sqrt_f32_local(float value) {
	float estimate;
	uint32_t i;

	if (value <= 0.0f) {
		return 0.0f;
	}

	estimate = (value >= 1.0f) ? value : 1.0f;
	for (i = 0U; i < 12U; ++i) {
		estimate = 0.5f * (estimate + (value / estimate));
	}
	return estimate;
}

static MPU6050_Status_t read_raw(MPU6050_Handle_t *device, int16_t accel[3],
		int16_t *temperature, int16_t gyro[3]) {
	uint8_t data[14];

	if ((device == NULL) || (device->initialized == 0U)
			|| (device->hi2c == NULL) || (accel == NULL)
			|| (temperature == NULL) || (gyro == NULL)) {
		return MPU6050_STATUS_NOT_INITIALIZED;
	}

	if (read_regs(device->hi2c, device->i2c_address_8bit,
	REG_ACCEL_XOUT_H, data, (uint16_t) sizeof(data)) != HAL_OK) {
		return MPU6050_STATUS_BUS_ERROR;
	}

	accel[0] = parse_i16(&data[0]);
	accel[1] = parse_i16(&data[2]);
	accel[2] = parse_i16(&data[4]);
	*temperature = parse_i16(&data[6]);
	gyro[0] = parse_i16(&data[8]);
	gyro[1] = parse_i16(&data[10]);
	gyro[2] = parse_i16(&data[12]);

	return MPU6050_STATUS_OK;
}

void MPU6050_GetDefaultConfig(MPU6050_Config_t *config) {
	if (config == NULL) {
		return;
	}

	config->accel_range = MPU6050_ACCEL_RANGE_16G;
	config->gyro_range = MPU6050_GYRO_RANGE_500DPS;
	config->dlpf = MPU6050_DLPF_94HZ;
	config->sample_rate_hz = MPU6050_DEFAULT_SAMPLE_RATE_HZ;
	config->accel_filter_alpha = 0.20f;
	config->gyro_filter_alpha = 0.20f;
	config->temperature_offset_c = MPU6050_TEMPERATURE_OFFSET_C;
}

MPU6050_Status_t MPU6050_Probe(I2C_HandleTypeDef *hi2c,
		MPU6050_Identity_t *identity) {
	static const uint8_t addresses[2] = {
	MPU6050_I2C_ADDR_AD0_LOW,
	MPU6050_I2C_ADDR_AD0_HIGH };
	uint8_t responded = 0U;
	uint8_t mismatch = 0U;
	uint32_t index;

	if (hi2c == NULL) {
		return MPU6050_STATUS_INVALID_PARAM;
	}

	if (identity != NULL) {
		identity->address_7bit = 0U;
		identity->who_am_i = 0U;
	}

	for (index = 0U; index < 2U; ++index) {
		const uint8_t address_7bit = addresses[index];
		const uint16_t address_8bit = (uint16_t) address_7bit << 1U;
		uint8_t who_am_i = 0U;

		if (HAL_I2C_IsDeviceReady(hi2c, address_8bit, 2U, 30U) != HAL_OK) {
			continue;
		}

		responded = 1U;
		if (read_regs(hi2c, address_8bit, MPU6050_REG_WHO_AM_I, &who_am_i, 1U)
				!= HAL_OK) {
			return MPU6050_STATUS_BUS_ERROR;
		}

		if (identity != NULL) {
			identity->address_7bit = address_7bit;
			identity->who_am_i = who_am_i;
		}

		if (who_am_i == MPU6050_WHO_AM_I_VALUE) {
			return MPU6050_STATUS_OK;
		}
		mismatch = 1U;
	}

	if (responded == 0U) {
		return MPU6050_STATUS_NOT_FOUND;
	}
	return (mismatch != 0U) ?
			MPU6050_STATUS_ID_MISMATCH : MPU6050_STATUS_BUS_ERROR;
}

MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_Handle_t *device,
		uint8_t address_7bit, const MPU6050_Config_t *config) {
	uint8_t value;
	uint8_t divider;
	uint8_t gyro_config;
	uint8_t accel_config;
	uint8_t who_am_i;

	if ((hi2c == NULL) || (device == NULL) || (config == NULL)
			|| ((address_7bit != MPU6050_I2C_ADDR_AD0_LOW)
					&& (address_7bit != MPU6050_I2C_ADDR_AD0_HIGH))
			|| ((uint32_t) config->accel_range > 3U)
			|| ((uint32_t) config->gyro_range > 3U)
			|| ((uint32_t) config->dlpf < 1U) || ((uint32_t) config->dlpf > 6U)
			|| (config->sample_rate_hz == 0U)
			|| (config->sample_rate_hz > 1000U)
			|| ((1000U % config->sample_rate_hz) != 0U)
			|| (config->accel_filter_alpha <= 0.0f)
			|| (config->accel_filter_alpha > 1.0f)
			|| (config->gyro_filter_alpha <= 0.0f)
			|| (config->gyro_filter_alpha > 1.0f)) {
		return MPU6050_STATUS_INVALID_PARAM;
	}

	memset(device, 0, sizeof(*device));
	device->hi2c = hi2c;
	device->address_7bit = address_7bit;
	device->i2c_address_8bit = (uint16_t) address_7bit << 1U;
	device->config = *config;
	device->accel_scale_correction = 1.0f;

	if (HAL_I2C_IsDeviceReady(hi2c, device->i2c_address_8bit, 2U, 30U)
			!= HAL_OK) {
		return MPU6050_STATUS_NOT_FOUND;
	}
	if (read_regs(hi2c, device->i2c_address_8bit,
	MPU6050_REG_WHO_AM_I, &who_am_i, 1U) != HAL_OK) {
		return MPU6050_STATUS_BUS_ERROR;
	}
	if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
		return MPU6050_STATUS_ID_MISMATCH;
	}
	device->who_am_i = who_am_i;

	/* Reset all sensor registers and wait for the reset to complete. */
	if (write_reg(hi2c, device->i2c_address_8bit, REG_PWR_MGMT_1, 0x80U)
			!= HAL_OK) {
		return MPU6050_STATUS_BUS_ERROR;
	}
	HAL_Delay(100U);

	/* Wake up and select the X-gyro PLL as clock source. */
	if ((write_reg(hi2c, device->i2c_address_8bit, REG_PWR_MGMT_1, 0x01U)
			!= HAL_OK)
			|| (write_reg(hi2c, device->i2c_address_8bit, REG_PWR_MGMT_2, 0x00U)
					!= HAL_OK)) {
		return MPU6050_STATUS_BUS_ERROR;
	}
	HAL_Delay(20U);

	divider = (uint8_t) ((1000U / config->sample_rate_hz) - 1U);
	gyro_config = (uint8_t) ((uint8_t) config->gyro_range << 3U);
	accel_config = (uint8_t) ((uint8_t) config->accel_range << 3U);

	if ((write_reg(hi2c, device->i2c_address_8bit, REG_CONFIG,
			(uint8_t) config->dlpf) != HAL_OK)
			|| (write_reg(hi2c, device->i2c_address_8bit, REG_SMPLRT_DIV,
					divider) != HAL_OK)
			|| (write_reg(hi2c, device->i2c_address_8bit, REG_GYRO_CONFIG,
					gyro_config) != HAL_OK)
			|| (write_reg(hi2c, device->i2c_address_8bit, REG_ACCEL_CONFIG,
					accel_config) != HAL_OK)) {
		return MPU6050_STATUS_BUS_ERROR;
	}

	/* Verify the effective register configuration, not just write ACKs. */
	if (read_regs(hi2c, device->i2c_address_8bit, REG_PWR_MGMT_1, &value, 1U)
			!= HAL_OK) {
		return MPU6050_STATUS_BUS_ERROR;
	}
	if ((value & 0x40U) != 0U) {
		return MPU6050_STATUS_CONFIG_MISMATCH;
	}

	if (read_regs(hi2c, device->i2c_address_8bit, REG_CONFIG, &value, 1U)
			!= HAL_OK) {
		return MPU6050_STATUS_BUS_ERROR;
	}
	if ((value & 0x07U) != (uint8_t) config->dlpf) {
		return MPU6050_STATUS_CONFIG_MISMATCH;
	}

	if (read_regs(hi2c, device->i2c_address_8bit, REG_SMPLRT_DIV, &value, 1U)
			!= HAL_OK) {
		return MPU6050_STATUS_BUS_ERROR;
	}
	if (value != divider) {
		return MPU6050_STATUS_CONFIG_MISMATCH;
	}

	if (read_regs(hi2c, device->i2c_address_8bit, REG_GYRO_CONFIG, &value, 1U)
			!= HAL_OK) {
		return MPU6050_STATUS_BUS_ERROR;
	}
	if ((value & 0x18U) != gyro_config) {
		return MPU6050_STATUS_CONFIG_MISMATCH;
	}

	if (read_regs(hi2c, device->i2c_address_8bit, REG_ACCEL_CONFIG, &value, 1U)
			!= HAL_OK) {
		return MPU6050_STATUS_BUS_ERROR;
	}
	if ((value & 0x18U) != accel_config) {
		return MPU6050_STATUS_CONFIG_MISMATCH;
	}

	device->accel_lsb_per_g = s_accel_lsb_per_g[(uint32_t) config->accel_range];
	device->gyro_lsb_per_dps =
			s_gyro_lsb_per_dps[(uint32_t) config->gyro_range];
	device->actual_sample_rate_hz = config->sample_rate_hz;
	device->initialized = 1U;

	/* Reset signal paths and allow the gyro to settle before calibration. */
	if (write_reg(hi2c, device->i2c_address_8bit,
	REG_SIGNAL_PATH_RESET, 0x07U) != HAL_OK) {
		device->initialized = 0U;
		return MPU6050_STATUS_BUS_ERROR;
	}
	HAL_Delay(100U);
	MPU6050_ResetFilters(device);

	return MPU6050_STATUS_OK;
}

MPU6050_Status_t MPU6050_ReadSample(MPU6050_Handle_t *device,
		MPU6050_Sample_t *sample) {
	int16_t accel_raw[3];
	int16_t gyro_raw[3];
	int16_t temperature_raw;
	MPU6050_Status_t status;
	float magnitude_squared = 0.0f;
	uint32_t axis;

	if ((device == NULL) || (sample == NULL) || (device->initialized == 0U)) {
		return MPU6050_STATUS_NOT_INITIALIZED;
	}

	status = read_raw(device, accel_raw, &temperature_raw, gyro_raw);
	if (status != MPU6050_STATUS_OK) {
		return status;
	}

	for (axis = 0U; axis < 3U; ++axis) {
		const float accel_nominal_g = ((float) accel_raw[axis]
				/ device->accel_lsb_per_g) - device->accel_bias_g[axis];
		const float accel_g = accel_nominal_g * device->accel_scale_correction;
		const float gyro_dps = ((float) gyro_raw[axis]
				/ device->gyro_lsb_per_dps) - device->gyro_bias_dps[axis];

		sample->accel_raw[axis] = accel_raw[axis];
		sample->gyro_raw[axis] = gyro_raw[axis];
		sample->accel_g[axis] = accel_g;
		sample->gyro_dps[axis] = gyro_dps;
		magnitude_squared += accel_g * accel_g;

		if (device->filter_initialized == 0U) {
			device->accel_filter_state[axis] = accel_g;
			device->gyro_filter_state[axis] = gyro_dps;
		} else {
			device->accel_filter_state[axis] +=
					device->config.accel_filter_alpha
							* (accel_g - device->accel_filter_state[axis]);
			device->gyro_filter_state[axis] += device->config.gyro_filter_alpha
					* (gyro_dps - device->gyro_filter_state[axis]);
		}

		sample->accel_filtered_g[axis] = device->accel_filter_state[axis];
		sample->gyro_filtered_dps[axis] = device->gyro_filter_state[axis];
	}

	device->filter_initialized = 1U;
	sample->temperature_raw = temperature_raw;
	sample->temperature_c = ((float) temperature_raw / 340.0f) + 36.53f
			+ device->config.temperature_offset_c;
	sample->accel_magnitude_g = sqrt_f32_local(magnitude_squared);

	return MPU6050_STATUS_OK;
}

MPU6050_Status_t MPU6050_CalibrateStationary(MPU6050_Handle_t *device,
		uint16_t sample_count, uint32_t sample_period_ms,
		MPU6050_CalibrationInfo_t *info) {
	float accel_mean[3] = { 0.0f, 0.0f, 0.0f };
	float accel_m2[3] = { 0.0f, 0.0f, 0.0f };
	float gyro_mean[3] = { 0.0f, 0.0f, 0.0f };
	float gyro_m2[3] = { 0.0f, 0.0f, 0.0f };
	float max_gyro_variance = 0.0f;
	float max_accel_variance = 0.0f;
	float gyro_mean_norm_squared;
	float accel_mean_norm_squared;
	float gravity_norm;
	float accel_gain;
	MPU6050_CalibrationInfo_t local_info;
	uint32_t n;
	uint32_t axis;

	memset(&local_info, 0, sizeof(local_info));
	local_info.accel_scale_correction = 1.0f;

	if ((device == NULL) || (device->initialized == 0U)) {
		return MPU6050_STATUS_NOT_INITIALIZED;
	}
	if ((sample_count < CAL_MIN_SAMPLES)
			|| (sample_period_ms < (1000U / device->actual_sample_rate_hz))) {
		return MPU6050_STATUS_INVALID_PARAM;
	}

	device->gyro_calibrated = 0U;
	device->accel_scale_calibrated = 0U;
	device->accel_scale_correction = 1.0f;
	memset(device->gyro_bias_dps, 0, sizeof(device->gyro_bias_dps));
	MPU6050_ResetFilters(device);

	/* Welford statistics are calculated from unfiltered physical samples. */
	for (n = 0U; n < sample_count; ++n) {
		int16_t accel_raw[3];
		int16_t gyro_raw[3];
		int16_t temperature_raw;
		MPU6050_Status_t status;
		const float divisor = (float) (n + 1U);

		status = read_raw(device, accel_raw, &temperature_raw, gyro_raw);
		(void) temperature_raw;
		if (status != MPU6050_STATUS_OK) {
			if (info != NULL)
				*info = local_info;
			return MPU6050_STATUS_CALIBRATION_ERROR;
		}

		for (axis = 0U; axis < 3U; ++axis) {
			const float accel_value = (float) accel_raw[axis]
					/ device->accel_lsb_per_g;
			const float gyro_value = (float) gyro_raw[axis]
					/ device->gyro_lsb_per_dps;
			float delta;

			delta = accel_value - accel_mean[axis];
			accel_mean[axis] += delta / divisor;
			accel_m2[axis] += delta * (accel_value - accel_mean[axis]);

			delta = gyro_value - gyro_mean[axis];
			gyro_mean[axis] += delta / divisor;
			gyro_m2[axis] += delta * (gyro_value - gyro_mean[axis]);
		}

		if ((n + 1U) < sample_count) {
			HAL_Delay(sample_period_ms);
		}
	}

	{
		const float inverse_count = 1.0f / (float) sample_count;
		for (axis = 0U; axis < 3U; ++axis) {
			const float accel_variance = accel_m2[axis] * inverse_count;
			const float gyro_variance = gyro_m2[axis] * inverse_count;
			if (accel_variance > max_accel_variance) {
				max_accel_variance = accel_variance;
			}
			if (gyro_variance > max_gyro_variance) {
				max_gyro_variance = gyro_variance;
			}
		}
	}

	gyro_mean_norm_squared = (gyro_mean[0] * gyro_mean[0])
			+ (gyro_mean[1] * gyro_mean[1]) + (gyro_mean[2] * gyro_mean[2]);
	accel_mean_norm_squared = (accel_mean[0] * accel_mean[0])
			+ (accel_mean[1] * accel_mean[1]) + (accel_mean[2] * accel_mean[2]);
	gravity_norm = sqrt_f32_local(accel_mean_norm_squared);

	local_info.max_gyro_stddev_dps = sqrt_f32_local(max_gyro_variance);
	local_info.max_accel_stddev_g = sqrt_f32_local(max_accel_variance);
	local_info.accel_gravity_norm_before_g = gravity_norm;
	for (axis = 0U; axis < 3U; ++axis) {
		local_info.gyro_bias_dps[axis] = gyro_mean[axis];
	}

	if (info != NULL) {
		*info = local_info;
	}

	/*
	 * Movement is determined from variation, not from the absolute gyro
	 * mean or a narrow 1 g norm window. A stationary sensor may legitimately
	 * have several deg/s of gyro zero-rate offset and an imperfect accel
	 * scale, both of which this calibration is meant to estimate.
	 */
	if ((local_info.max_gyro_stddev_dps > CAL_MAX_GYRO_STDDEV_DPS)
			|| (local_info.max_accel_stddev_g > CAL_MAX_ACCEL_STDDEV_G)
			|| (sqrt_f32_local(gyro_mean_norm_squared)
					> CAL_MAX_MEAN_GYRO_NORM_DPS)) {
		return MPU6050_STATUS_NOT_STATIONARY;
	}

	/* A grossly invalid magnitude suggests bad data / wrong scale, not rest. */
	if ((gravity_norm < CAL_MIN_GRAVITY_NORM_G)
			|| (gravity_norm > CAL_MAX_GRAVITY_NORM_G)) {
		return MPU6050_STATUS_GRAVITY_OUT_OF_RANGE;
	}

	accel_gain = 1.0f / gravity_norm;
	if ((accel_gain < CAL_MIN_ACCEL_GAIN)
			|| (accel_gain > CAL_MAX_ACCEL_GAIN)) {
		return MPU6050_STATUS_GRAVITY_OUT_OF_RANGE;
	}

	for (axis = 0U; axis < 3U; ++axis) {
		device->gyro_bias_dps[axis] = gyro_mean[axis];
		local_info.gyro_bias_dps[axis] = gyro_mean[axis];
	}

	device->accel_scale_correction = accel_gain;
	device->accel_gravity_norm_before_g = gravity_norm;
	device->gyro_calibrated = 1U;
	device->accel_scale_calibrated = 1U;
	local_info.accel_scale_correction = accel_gain;
	local_info.gyro_calibrated = 1U;
	local_info.accel_scale_calibrated = 1U;

	if (info != NULL) {
		*info = local_info;
	}

	MPU6050_ResetFilters(device);
	return MPU6050_STATUS_OK;
}

MPU6050_Status_t MPU6050_CalibrateGyro(MPU6050_Handle_t *device,
		uint16_t sample_count, uint32_t sample_period_ms) {
	return MPU6050_CalibrateStationary(device, sample_count, sample_period_ms,
	NULL);
}

void MPU6050_ResetFilters(MPU6050_Handle_t *device) {
	if (device == NULL) {
		return;
	}

	device->filter_initialized = 0U;
	memset(device->accel_filter_state, 0, sizeof(device->accel_filter_state));
	memset(device->gyro_filter_state, 0, sizeof(device->gyro_filter_state));
}

uint16_t MPU6050_GetSampleRateHz(const MPU6050_Handle_t *device) {
	return (device != NULL) ? device->actual_sample_rate_hz : 0U;
}

void MPU6050_GetGyroBias(const MPU6050_Handle_t *device, float bias_dps[3]) {
	if ((device == NULL) || (bias_dps == NULL)) {
		return;
	}
	memcpy(bias_dps, device->gyro_bias_dps, sizeof(device->gyro_bias_dps));
}

void MPU6050_GetCalibrationInfo(const MPU6050_Handle_t *device,
		MPU6050_CalibrationInfo_t *info) {
	if ((device == NULL) || (info == NULL)) {
		return;
	}

	memset(info, 0, sizeof(*info));
	memcpy(info->gyro_bias_dps, device->gyro_bias_dps,
			sizeof(device->gyro_bias_dps));
	info->accel_gravity_norm_before_g = device->accel_gravity_norm_before_g;
	info->accel_scale_correction = device->accel_scale_correction;
	info->gyro_calibrated = device->gyro_calibrated;
	info->accel_scale_calibrated = device->accel_scale_calibrated;
}

MPU6050_Status_t MPU6050_SetAccelBias(MPU6050_Handle_t *device,
		const float bias_g[3]) {
	uint32_t axis;

	if ((device == NULL) || (bias_g == NULL) || (device->initialized == 0U)) {
		return MPU6050_STATUS_INVALID_PARAM;
	}

	for (axis = 0U; axis < 3U; ++axis) {
		if ((bias_g[axis] < -1.0f) || (bias_g[axis] > 1.0f)) {
			return MPU6050_STATUS_INVALID_PARAM;
		}
	}

	memcpy(device->accel_bias_g, bias_g, sizeof(device->accel_bias_g));
	MPU6050_ResetFilters(device);
	return MPU6050_STATUS_OK;
}
