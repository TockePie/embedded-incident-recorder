#include "stm32f4xx_hal.h"
#include "bus_interface.h"
#include "bus_adapters.h"

#define GY89_I2C_ADDR_L3GD20  0x6B
#define GY89_I2C_ADDR_LSM303D 0x1D
#define REG_WHO_AM_I          0x0F

#define CMD_JEDEC_ID          0x9F

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();

    const i2c_bus_t *i2c = &g_i2c_platform_bus;
    const spi_bus_t *spi = &g_spi_platform_bus;

    uint8_t i2c_who_am_i = 0;
    uint8_t spi_tx[4] = {CMD_JEDEC_ID, 0x00, 0x00, 0x00};
    uint8_t spi_rx[4] = {0};

    while (1) {
        /* 1. I2C Ping (GY-89) */
        i2c->read(GY89_I2C_ADDR_L3GD20, REG_WHO_AM_I, 1, &i2c_who_am_i, 1);
        HAL_Delay(10);
        i2c->read(GY89_I2C_ADDR_LSM303D, REG_WHO_AM_I, 1, &i2c_who_am_i, 1);
        HAL_Delay(10);

        /* 2. SPI Ping (Flash JEDEC ID) */
        spi->cs_control(true);
        spi->transfer(spi_tx, spi_rx, 4);
        spi->cs_control(false);

        HAL_Delay(100);
    }
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SPI1 CS Pin (PA4) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_SPI1_Init(void) {
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}

static void SystemClock_Config(void) {
}
