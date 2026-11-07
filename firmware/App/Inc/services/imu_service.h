#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers/mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_SERVICE_MONITOR_ENABLE
#define IMU_SERVICE_MONITOR_ENABLE 0U
#endif

#ifndef IMU_SERVICE_MONITOR_PERIOD_MS
#define IMU_SERVICE_MONITOR_PERIOD_MS 200U
#endif

MPU6050_Status_t ImuService_Init(I2C_HandleTypeDef *hi2c,
		uint8_t address_7bit);
MPU6050_Status_t ImuService_CalibrateStationary(uint16_t sample_count,
		uint32_t sample_period_ms, MPU6050_CalibrationInfo_t *info);
MPU6050_Status_t ImuService_CalibrateGyro(uint16_t sample_count,
		uint32_t sample_period_ms);
void ImuService_RunOnce(void);
bool ImuService_IsReady(void);
bool ImuService_IsGyroCalibrated(void);
bool ImuService_IsAccelScaleCalibrated(void);
const MPU6050_Handle_t *ImuService_GetDevice(void);
const MPU6050_Sample_t *ImuService_GetLatestSample(void);
uint32_t ImuService_GetSampleSequence(void);
uint32_t ImuService_GetLatestSampleTick(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SERVICE_H */
