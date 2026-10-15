#include "bringup.h"
#include "bus_interface.h"
#include "bus_adapters.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;

#define GY89_I2C_ADDR_L3GD20  0x6B
#define GY89_I2C_ADDR_LSM303D 0x1D
#define OV7670_SCCB_ADDR      0x21
#define REG_WHO_AM_I          0x0F
#define CMD_JEDEC_ID          0x9F
#define CMD_READ_STATUS       0x05

typedef struct {
    uint8_t l3gd20_id;
    uint8_t lsm303d_id;
    uint8_t ov7670_pid;
    uint8_t spi_flash_id[4];
    uint8_t spi_flash_status;
} bringup_data_t;

static bringup_data_t g_bringup_data;

static bus_status_t sccb_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *val) {
    const i2c_bus_t *i2c = &g_i2c_platform_bus;
    if (i2c->write(dev_addr, reg_addr, 1, NULL, 0) != BUS_OK) {
        return BUS_ERROR;
    }
    return i2c->read(dev_addr, 0, 0, val, 1);
}

void app_bringup_init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void app_bringup_run(void) {
    const i2c_bus_t *i2c = &g_i2c_platform_bus;
    const spi_bus_t *spi = &g_spi_platform_bus;

    i2c->read(GY89_I2C_ADDR_L3GD20, REG_WHO_AM_I, 1, &g_bringup_data.l3gd20_id, 1);
    i2c->read(GY89_I2C_ADDR_LSM303D, REG_WHO_AM_I, 1, &g_bringup_data.lsm303d_id, 1);

    sccb_read_reg(OV7670_SCCB_ADDR, 0x0A, &g_bringup_data.ov7670_pid);

    uint8_t spi_tx[4] = {CMD_JEDEC_ID, 0x00, 0x00, 0x00};
    spi->cs_control(true);
    spi->transfer(spi_tx, g_bringup_data.spi_flash_id, 4);
    spi->cs_control(false);

    uint8_t cmd_status = CMD_READ_STATUS;
    spi->cs_control(true);
    spi->transfer(&cmd_status, NULL, 1);
    spi->transfer(NULL, &g_bringup_data.spi_flash_status, 1);
    spi->cs_control(false);

    uint8_t uart_msg[] = "BRINGUP: OK\r\n";
    HAL_UART_Transmit(&huart2, uart_msg, sizeof(uart_msg) - 1, 50);

    static uint16_t pwm_val = 0;
    pwm_val = (pwm_val + 100) % 1000;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_val);
}
