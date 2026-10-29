#include "drivers_stub.h"
#include "main.h"
#include <stddef.h>

static uint8_t rx_ring_buffer[64];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

void Drivers_InitAll(void) {
}

bool Drivers_Sensors_ReadData(SensorData_t *out_data) {
    if (out_data == NULL) return false;
    out_data->accel_x = 0.01f;
    out_data->accel_y = 0.02f;
    out_data->accel_z = 0.98f;
    out_data->temp = 24.5f;
    out_data->timestamp_ms = HAL_GetTick();
    return true;
}

void Drivers_UART_ProcessRxChar(uint8_t rx_char) {
    uint8_t next_head = (rx_head + 1) % 64;
    if (next_head != rx_tail) {
        rx_ring_buffer[rx_head] = rx_char;
        rx_head = next_head;
    }
}

bool Drivers_Flash_WriteChunk(const uint8_t *data, uint16_t size) {
    (void)data;
    (void)size;
    return true;
}
