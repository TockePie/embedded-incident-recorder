#ifndef DRIVERS_STUB_H
#define DRIVERS_STUB_H

#include <stdint.h>
#include <stdbool.h>

#define BMP180_I2C_ADDR     (0x77 << 1)
#define W25Q64_SPI_CS_PORT  GPIOB
#define W25Q64_SPI_CS_PIN   GPIO_PIN_6

typedef struct {
    float temperature;
    float pressure;
    bool  threshold_exceeded;
    uint32_t timestamp_ms;
} Bmp180Data_t;

void Drivers_InitAll(void);
bool Drivers_BMP180_Read(Bmp180Data_t *out_data);
bool Drivers_W25Q64_WritePage(uint32_t address, const uint8_t *data, uint16_t length);
void Drivers_UART_EnqueueRx(uint8_t rx_char);
bool Drivers_UART_DequeueRx(uint8_t *out_char);
void Drivers_Camera_TriggerCapture(void);

#endif
