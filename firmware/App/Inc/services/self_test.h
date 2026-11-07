#ifndef SELF_TEST_H
#define SELF_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "drivers/mpu6050.h"

typedef enum {
	SELF_TEST_OVERALL_PASS = 0,
	SELF_TEST_OVERALL_DEGRADED,
	SELF_TEST_OVERALL_FAIL
} SelfTestOverallStatus_t;

typedef struct {
	bool mpu6050_ok;
	bool mpu6050_gyro_calibrated;
	bool mpu6050_accel_scale_calibrated;
	bool ov7670_ok;
	bool w25q64_ok;

	uint8_t mpu6050_address_7bit;
	uint8_t mpu6050_who_am_i;
	uint8_t ov7670_pid;
	uint8_t ov7670_ver;
	uint8_t w25q64_jedec_id[3];
	uint8_t w25q64_status1;
	uint8_t w25q64_status2;

	float mpu6050_gyro_bias_dps[3];
	float mpu6050_gravity_norm_before_g;
	float mpu6050_accel_scale_correction;
	float mpu6050_max_gyro_stddev_dps;
	float mpu6050_max_accel_stddev_g;

	SelfTestOverallStatus_t overall;
} SelfTestReport_t;

/** Probe the connected devices and run startup calibration checks. */
SelfTestOverallStatus_t SelfTest_Run(void);

/** Return the report produced by the most recent SelfTest_Run() call. */
const SelfTestReport_t *SelfTest_GetReport(void);

#ifdef __cplusplus
}
#endif
#endif /* SELF_TEST_H */
