#include "bus_interface.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

bus_status_t hal_i2c_read(uint16_t dev_addr, uint16_t reg_addr, uint8_t reg_len, uint8_t *data, size_t len) {
    uint16_t mem_size = (reg_len == 2) ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr, mem_size, data, (uint16_t)len, 100);
    return (status == HAL_OK) ? BUS_OK : BUS_ERROR;
}

bus_status_t hal_i2c_write(uint16_t dev_addr, uint16_t reg_addr, uint8_t reg_len, const uint8_t *data, size_t len) {
    uint16_t mem_size = (reg_len == 2) ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(dev_addr << 1), reg_addr, mem_size, (uint8_t *)data, (uint16_t)len, 100);
    return (status == HAL_OK) ? BUS_OK : BUS_ERROR;
}

bus_status_t hal_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len) {
    HAL_StatusTypeDef status;
    if (tx_data != NULL && rx_data != NULL) {
        status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_data, rx_data, (uint16_t)len, 100);
    } else if (tx_data != NULL) {
        status = HAL_SPI_Transmit(&hspi1, (uint8_t *)tx_data, (uint16_t)len, 100);
    } else if (rx_data != NULL) {
        status = HAL_SPI_Receive(&hspi1, rx_data, (uint16_t)len, 100);
    } else {
        return BUS_INVALID_PARAM;
    }
    return (status == HAL_OK) ? BUS_OK : BUS_ERROR;
}

void hal_spi_cs_control(bool state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

const i2c_bus_t g_i2c_platform_bus = {
    .read = hal_i2c_read,
    .write = hal_i2c_write,
    .user_ctx = &hi2c1
};

const spi_bus_t g_spi_platform_bus = {
    .transfer = hal_spi_transfer,
    .cs_control = hal_spi_cs_control,
    .user_ctx = &hspi1
};
