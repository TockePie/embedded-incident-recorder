#include "drivers_stub.h"
#include "main.h"
#include <stddef.h>

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

static uint8_t uart_rx_ring[128];
static volatile uint16_t uart_head = 0;
static volatile uint16_t uart_tail = 0;

void Drivers_InitAll(void) {
    HAL_GPIO_WritePin(W25Q64_SPI_CS_PORT, W25Q64_SPI_CS_PIN, GPIO_PIN_SET);
}

bool Drivers_BMP180_Read(Bmp180Data_t *out_data) {
    if (out_data == NULL) return false;

    uint8_t raw_data[2] = {0};
    uint8_t reg = 0xF6;

    if (HAL_I2C_Master_Transmit(&hi2c1, BMP180_I2C_ADDR, &reg, 1, 10) == HAL_OK) {
        if (HAL_I2C_Master_Receive(&hi2c1, BMP180_I2C_ADDR, raw_data, 2, 20) == HAL_OK) {
            int16_t raw_temp = (int16_t)((raw_data[0] << 8) | raw_data[1]);
            out_data->temperature = (float)raw_temp / 10.0f;
            out_data->pressure = 1013.25f;
            out_data->timestamp_ms = HAL_GetTick();
            
            if (out_data->temperature > 30.0f) {
                out_data->threshold_exceeded = true;
            } else {
                out_data->threshold_exceeded = false;
            }
            return true;
        }
    }

    out_data->temperature = 22.0f;
    out_data->pressure = 1013.0f;
    out_data->timestamp_ms = HAL_GetTick();
    out_data->threshold_exceeded = false;
    return false;
}

bool Drivers_W25Q64_WritePage(uint32_t address, const uint8_t *data, uint16_t length) {
    if (data == NULL || length == 0) return false;

    uint8_t cmd[4];
    cmd[0] = 0x02;
    cmd[1] = (uint8_t)(address >> 16);
    cmd[2] = (uint8_t)(address >> 8);
    cmd[3] = (uint8_t)(address);

    HAL_GPIO_WritePin(W25Q64_SPI_CS_PORT, W25Q64_SPI_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, cmd, 4, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)data, length, 50);
    HAL_GPIO_WritePin(W25Q64_SPI_CS_PORT, W25Q64_SPI_CS_PIN, GPIO_PIN_SET);

    return true;
}

void Drivers_UART_EnqueueRx(uint8_t rx_char) {
    uint16_t next = (uart_head + 1) % sizeof(uart_rx_ring);
    if (next != uart_tail) {
        uart_rx_ring[uart_head] = rx_char;
        uart_head = next;
    }
}

bool Drivers_UART_DequeueRx(uint8_t *out_char) {
    if (uart_head == uart_tail || out_char == NULL) return false;
    *out_char = uart_rx_ring[uart_tail];
    uart_tail = (uart_tail + 1) % sizeof(uart_rx_ring);
    return true;
}

void Drivers_Camera_TriggerCapture(void) {
}
