#include "motion_sensor.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void MotionSensor_Init(void) {
    
}

bool MotionSensor_ReadData(int16_t *accel_data) {
    return true;
}
