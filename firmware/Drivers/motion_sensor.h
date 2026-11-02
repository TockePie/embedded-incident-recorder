#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

void MotionSensor_Init(void);
bool MotionSensor_ReadData(int16_t *accel_data);

#endif // MOTION_SENSOR_H
