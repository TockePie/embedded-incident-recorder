#ifndef FLASH_SPI_H
#define FLASH_SPI_H

#include <stdint.h>
#include <stdbool.h>

void Flash_Init(void);
bool Flash_WriteEventData(uint32_t timestamp, uint8_t event_type);
bool Flash_ReadEventData(uint32_t address, uint8_t *buffer, uint16_t size);

#endif // FLASH_SPI_H
