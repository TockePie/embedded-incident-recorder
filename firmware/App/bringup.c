#include "bringup.h"
#include "bus_interface.h"
#include "bus_adapters.h"

#define GY89_I2C_ADDR_L3GD20  0x6B
#define GY89_I2C_ADDR_LSM303D 0x1D
#define REG_WHO_AM_I          0x0F
#define CMD_JEDEC_ID          0x9F

void app_bringup_run(void) {
    const i2c_bus_t *i2c = &g_i2c_platform_bus;
    const spi_bus_t *spi = &g_spi_platform_bus;

    uint8_t i2c_who_am_i = 0;
    uint8_t spi_tx[4] = {CMD_JEDEC_ID, 0x00, 0x00, 0x00};
    uint8_t spi_rx[4] = {0};

    /* 1. I2C Ping (GY-89 / SCCB Camera) */
    i2c->read(GY89_I2C_ADDR_L3GD20, REG_WHO_AM_I, 1, &i2c_who_am_i, 1);
    i2c->read(GY89_I2C_ADDR_LSM303D, REG_WHO_AM_I, 1, &i2c_who_am_i, 1);

    /* 2. SPI Ping (SPI2 Flash) */
    spi->cs_control(true);
    spi->transfer(spi_tx, spi_rx, 4);
    spi->cs_control(false);
}
