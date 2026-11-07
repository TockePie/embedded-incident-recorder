#ifndef MPU6050_ROBUST_CALIBRATION_H
#define MPU6050_ROBUST_CALIBRATION_H

#include <stdint.h>

#include "drivers/mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ROBUST_CAL_MAX_SAMPLES 500U
#define MPU6050_ROBUST_CAL_DISCARD_SAMPLES 40U

/* Median/MAD statistics reduce outlier influence; sample_count must not
 * exceed MPU6050_ROBUST_CAL_MAX_SAMPLES. */
MPU6050_Status_t MPU6050_CalibrateStationaryRobust(
		MPU6050_Handle_t *device,
		uint16_t sample_count,
		uint32_t sample_period_ms,
		MPU6050_CalibrationInfo_t *info);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_ROBUST_CALIBRATION_H */
