#ifndef DRIVERS_STUB_H
#define DRIVERS_STUB_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float temp;
    uint32_t timestamp_ms;
} SensorData_t;

void Drivers_InitAll(void);
bool Drivers_Sensors_ReadData(SensorData_t *out_data);
void Drivers_UART_ProcessRxChar(uint8_t rx_char);
bool Drivers_Flash_WriteChunk(const uint8_t *data, uint16_t size);

#endif
