#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define MPU6050_I2C_ADDR_AD0_LOW       0x68U
#define MPU6050_I2C_ADDR_AD0_HIGH      0x69U
#define MPU6050_REG_WHO_AM_I           0x75U
#define MPU6050_WHO_AM_I_VALUE         0x68U
#define MPU6050_DEFAULT_SAMPLE_RATE_HZ 200U
#define MPU6050_DEFAULT_CAL_SAMPLES    300U

/* The offset compensates the sensor die-temperature reading, not ambient
 * or cargo temperature. Zero disables the compensation offset. */
#ifndef MPU6050_TEMPERATURE_OFFSET_C
#define MPU6050_TEMPERATURE_OFFSET_C   (0.0f)
#endif

typedef enum {
	MPU6050_ACCEL_RANGE_2G = 0U,
	MPU6050_ACCEL_RANGE_4G = 1U,
	MPU6050_ACCEL_RANGE_8G = 2U,
	MPU6050_ACCEL_RANGE_16G = 3U
} MPU6050_AccelRange_t;

typedef enum {
	MPU6050_GYRO_RANGE_250DPS = 0U,
	MPU6050_GYRO_RANGE_500DPS = 1U,
	MPU6050_GYRO_RANGE_1000DPS = 2U,
	MPU6050_GYRO_RANGE_2000DPS = 3U
} MPU6050_GyroRange_t;

/* DLPF_CFG=2: approximately 94 Hz accel / 98 Hz gyro bandwidth. */
typedef enum {
	MPU6050_DLPF_184HZ = 1U,
	MPU6050_DLPF_94HZ = 2U,
	MPU6050_DLPF_44HZ = 3U,
	MPU6050_DLPF_21HZ = 4U,
	MPU6050_DLPF_10HZ = 5U,
	MPU6050_DLPF_5HZ = 6U
} MPU6050_Dlpf_t;

typedef enum {
	MPU6050_STATUS_OK = 0,
	MPU6050_STATUS_NOT_FOUND,
	MPU6050_STATUS_BUS_ERROR,
	MPU6050_STATUS_ID_MISMATCH,
	MPU6050_STATUS_INVALID_PARAM,
	MPU6050_STATUS_CONFIG_MISMATCH,
	MPU6050_STATUS_NOT_INITIALIZED,
	MPU6050_STATUS_NOT_STATIONARY,
	MPU6050_STATUS_CALIBRATION_ERROR,
	MPU6050_STATUS_GRAVITY_OUT_OF_RANGE
} MPU6050_Status_t;

typedef struct {
	uint8_t address_7bit;
	uint8_t who_am_i;
} MPU6050_Identity_t;

typedef struct {
	MPU6050_AccelRange_t accel_range;
	MPU6050_GyroRange_t gyro_range;
	MPU6050_Dlpf_t dlpf;
	uint16_t sample_rate_hz;
	float accel_filter_alpha;
	float gyro_filter_alpha;
	float temperature_offset_c;
} MPU6050_Config_t;

typedef struct {
	int16_t accel_raw[3];
	int16_t temperature_raw;
	int16_t gyro_raw[3];

	/* Calibrated, non-software-filtered physical values. */
	float accel_g[3];
	float gyro_dps[3];

	/* Temperature of the MPU-6050 die, in degrees Celsius. */
	float temperature_c;

	/* Vector magnitude after startup accelerometer scale calibration. */
	float accel_magnitude_g;

	/* Additional IIR-filtered values for display / slow-changing logic. */
	float accel_filtered_g[3];
	float gyro_filtered_dps[3];
} MPU6050_Sample_t;

typedef struct {
	float gyro_bias_dps[3];
	float accel_gravity_norm_before_g;
	float accel_scale_correction;
	float max_gyro_stddev_dps;
	float max_accel_stddev_g;
	uint8_t gyro_calibrated;
	uint8_t accel_scale_calibrated;
} MPU6050_CalibrationInfo_t;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint16_t i2c_address_8bit;
	uint8_t address_7bit;
	uint8_t who_am_i;
	uint8_t initialized;
	uint8_t filter_initialized;
	uint8_t gyro_calibrated;
	uint8_t accel_scale_calibrated;

	MPU6050_Config_t config;
	float accel_lsb_per_g;
	float gyro_lsb_per_dps;

	float gyro_bias_dps[3];
	float accel_bias_g[3];
	float accel_scale_correction;
	float accel_gravity_norm_before_g;

	float accel_filter_state[3];
	float gyro_filter_state[3];
	uint16_t actual_sample_rate_hz;
} MPU6050_Handle_t;

void MPU6050_GetDefaultConfig(MPU6050_Config_t *config);
MPU6050_Status_t MPU6050_Probe(I2C_HandleTypeDef *hi2c,
		MPU6050_Identity_t *identity);
MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_Handle_t *device,
		uint8_t address_7bit, const MPU6050_Config_t *config);
MPU6050_Status_t MPU6050_ReadSample(MPU6050_Handle_t *device,
		MPU6050_Sample_t *sample);

/*
 * Calibrate a stationary device by estimating gyro zero-rate bias and one
 * common accelerometer scale correction from the measured gravity magnitude.
 * This is not a full multi-position, per-axis accelerometer calibration.
 */
MPU6050_Status_t MPU6050_CalibrateStationary(MPU6050_Handle_t *device,
		uint16_t sample_count, uint32_t sample_period_ms,
		MPU6050_CalibrationInfo_t *info);

/* Compatibility wrapper: performs the complete stationary calibration. */
MPU6050_Status_t MPU6050_CalibrateGyro(MPU6050_Handle_t *device,
		uint16_t sample_count, uint32_t sample_period_ms);

void MPU6050_ResetFilters(MPU6050_Handle_t *device);
uint16_t MPU6050_GetSampleRateHz(const MPU6050_Handle_t *device);
void MPU6050_GetGyroBias(const MPU6050_Handle_t *device, float bias_dps[3]);
void MPU6050_GetCalibrationInfo(const MPU6050_Handle_t *device,
		MPU6050_CalibrationInfo_t *info);
MPU6050_Status_t MPU6050_SetAccelBias(MPU6050_Handle_t *device,
		const float bias_g[3]);

#ifdef __cplusplus
}
#endif
#endif /* MPU6050_H */
