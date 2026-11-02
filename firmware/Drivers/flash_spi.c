#include "flash_spi.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

void Flash_Init(void) {
    // Ініціалізація CS (Chip Select) для SPI Flash
}

bool Flash_WriteEventData(uint32_t timestamp, uint8_t event_type) {
    // Імітація/реалізація запису кадру та метаданих події у Flash
    return true;
}

bool Flash_ReadEventData(uint32_t address, uint8_t *buffer, uint16_t size) {
    return true;
}
