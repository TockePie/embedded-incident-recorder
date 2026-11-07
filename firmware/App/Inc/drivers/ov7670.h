#ifndef OV7670_H
#define OV7670_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32f4xx_hal.h"

/* The supported camera profile is QQVGA with RGB565 pixel encoding. */
typedef enum {
	OV7670_RESOLUTION_QQVGA_160X120 = 0
} OV7670_Resolution_t;

typedef enum {
	OV7670_PIXEL_FORMAT_RGB565 = 0
} OV7670_PixelFormat_t;

typedef struct {
	OV7670_Resolution_t resolution;
	OV7670_PixelFormat_t pixel_format;
} OV7670_Config_t;

typedef enum {
	OV7670_STATUS_OK = 0,
	OV7670_STATUS_XCLK_ERROR,
	OV7670_STATUS_BUS_STUCK,
	OV7670_STATUS_ID_READ_ERROR,
	OV7670_STATUS_ID_MISMATCH,
	OV7670_STATUS_BAD_ARGUMENT,
	OV7670_STATUS_UNSUPPORTED_CONFIG,
	OV7670_STATUS_NOT_INITIALIZED,
	OV7670_STATUS_REGISTER_READ_ERROR,
	OV7670_STATUS_REGISTER_WRITE_ERROR,
	OV7670_STATUS_CONFIG_VERIFY_ERROR
} OV7670_Status_t;

typedef struct {
	uint8_t xclk_pin_configured;
	uint8_t xclk_started;
	uint8_t sioc_idle_high;
	uint8_t siod_idle_high;
	uint8_t dc_write_address;
	uint8_t dc_register_address;
	uint8_t dc_read_address;
	uint8_t pid;
	uint8_t ver;
	uint8_t midh;
	uint8_t midl;
	uint8_t pid_read_ok;
	uint8_t ver_read_ok;
	uint8_t midh_read_ok;
	uint8_t midl_read_ok;
} OV7670_Diagnostics_t;

typedef struct {
	uint8_t pid;
	uint8_t ver;
	uint8_t midh;
	uint8_t midl;
	uint16_t width;
	uint16_t height;
	uint8_t bytes_per_pixel;
	uint32_t frame_size_bytes;
	uint32_t xclk_hz;
} OV7670_Info_t;

typedef struct {
	uint8_t reg;
	uint8_t expected;
	uint8_t actual;
	uint8_t read_ok;
	uint8_t match;
} OV7670_RegisterCheck_t;

#define OV7670_CONFIG_CHECK_COUNT       13U
#define OV7670_QQVGA_WIDTH              160U
#define OV7670_QQVGA_HEIGHT             120U
#define OV7670_RGB565_BYTES_PER_PIXEL   2U
#define OV7670_QQVGA_RGB565_FRAME_BYTES \
	((uint32_t)OV7670_QQVGA_WIDTH * (uint32_t)OV7670_QQVGA_HEIGHT * \
	 (uint32_t)OV7670_RGB565_BYTES_PER_PIXEL)

/* Read the sensor identity, then return it to RESET/PWDN and stop XCLK. */
OV7670_Status_t OV7670_ProbeEx(TIM_HandleTypeDef *xclk_timer,
		uint32_t xclk_channel, uint8_t *pid, uint8_t *ver,
		OV7670_Diagnostics_t *diagnostics);
OV7670_Status_t OV7670_Probe(TIM_HandleTypeDef *xclk_timer,
		uint32_t xclk_channel, uint8_t *pid, uint8_t *ver);

/* Fill the currently supported baseline profile: QQVGA RGB565. */
void OV7670_GetDefaultConfig(OV7670_Config_t *config);

/*
 * Full camera bring-up. Starts XCLK, powers/releases the sensor, checks its
 * identity, soft-resets it, programs QQVGA RGB565 and verifies key registers.
 * On success XCLK remains running and the sensor remains active. On failure
 * the camera is returned to RESET/PWDN and XCLK is stopped.
 */
OV7670_Status_t OV7670_Init(TIM_HandleTypeDef *xclk_timer,
		uint32_t xclk_channel, const OV7670_Config_t *config,
		OV7670_Info_t *info);

/* Return the camera to its safe low-power state and stop XCLK. */
OV7670_Status_t OV7670_DeInit(void);

/* Register access is available only after successful OV7670_Init(). */
OV7670_Status_t OV7670_ReadRegister(uint8_t reg, uint8_t *value);
OV7670_Status_t OV7670_WriteRegister(uint8_t reg, uint8_t value);

/* Read back all key profile registers. checks must have 13 entries. */
OV7670_Status_t OV7670_VerifyConfiguration(OV7670_RegisterCheck_t *checks,
		uint8_t checks_capacity, uint8_t *failed_count);

/* Retrieve the checks made by the last OV7670_Init(), including when init
 * failed and the driver has already returned the sensor to its safe state. */
OV7670_Status_t OV7670_GetLastConfigurationChecks(
		OV7670_RegisterCheck_t *checks, uint8_t checks_capacity,
		uint8_t *failed_count);

const char *OV7670_StatusString(OV7670_Status_t status);
uint8_t OV7670_IsInitialized(void);
OV7670_Status_t OV7670_GetInfo(OV7670_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* OV7670_H */
