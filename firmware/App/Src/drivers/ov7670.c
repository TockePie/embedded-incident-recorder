#include "drivers/ov7670.h"

#include <stddef.h>
#include <string.h>

#include "main.h"

#define OV7670_SCCB_WRITE_ADDRESS       0x42U
#define OV7670_SCCB_READ_ADDRESS        0x43U
#define OV7670_REG_PID                  0x0AU
#define OV7670_REG_VER                  0x0BU
#define OV7670_REG_MIDH                 0x1CU
#define OV7670_REG_MIDL                 0x1DU
#define OV7670_REG_COM3                 0x0CU
#define OV7670_REG_CLKRC                0x11U
#define OV7670_REG_COM7                 0x12U
#define OV7670_REG_COM8                 0x13U
#define OV7670_REG_COM10                0x15U
#define OV7670_REG_HSTART               0x17U
#define OV7670_REG_HSTOP                0x18U
#define OV7670_REG_VSTART               0x19U
#define OV7670_REG_VSTOP                0x1AU
#define OV7670_REG_HREF                 0x32U
#define OV7670_REG_TSLB                 0x3AU
#define OV7670_REG_COM14                0x3EU
#define OV7670_REG_COM15                0x40U
#define OV7670_REG_SCALING_XSC          0x70U
#define OV7670_REG_SCALING_YSC          0x71U
#define OV7670_REG_SCALING_DCWCTR       0x72U
#define OV7670_REG_SCALING_PCLK_DIV     0x73U
#define OV7670_REG_RGB444               0x8CU
#define OV7670_REG_SCALING_PCLK_DELAY   0xA2U

#define OV7670_PID_VALUE                0x76U
#define OV7670_VER_VALUE_A              0x73U
#define OV7670_VER_VALUE_B              0x70U

#define OV7670_SCCB_HALF_PERIOD_US       5U
#define OV7670_XCLK_HZ                   12500000UL
#define OV7670_XCLK_STARTUP_MS           5U
#define OV7670_POWERUP_SETTLE_MS         2U
#define OV7670_RESET_SETTLE_MS           10U
#define OV7670_SOFT_RESET_SETTLE_MS      20U
#define OV7670_CONFIG_SETTLE_MS          30U

#define OV7670_ARRAY_COUNT(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

typedef struct {
	uint8_t reg;
	uint8_t value;
} OV7670_RegisterValue_t;

/*
 * This QQVGA scaler profile follows the OmniVision implementation guide and
 * is adapted for RGB565 output. The guide specifies its reference timing at
 * 24 MHz XCLK; this design uses 12.5 MHz, so its frame rate differs accordingly.
 */
static const OV7670_RegisterValue_t s_qqvga_rgb565_registers[] = {
	/* ---------- 1. Output format, scaling, and clock configuration ---------- */
	{ OV7670_REG_CLKRC, 0x01U },
	{ OV7670_REG_COM7, 0x04U },              /* RGB, VGA base + scaler */
	{ OV7670_REG_COM3, 0x04U },              /* down-sampling enabled */
	{ OV7670_REG_COM14, 0x1AU },             /* manual scaler, PCLK / 4 */
	{ OV7670_REG_SCALING_XSC, 0x3AU },
	{ OV7670_REG_SCALING_YSC, 0x35U },
	{ OV7670_REG_SCALING_DCWCTR, 0x22U },    /* QQVGA decimation */
	{ OV7670_REG_SCALING_PCLK_DIV, 0xF2U },
	{ OV7670_REG_SCALING_PCLK_DELAY, 0x02U },
	{ OV7670_REG_COM15, 0xD0U },             /* full range + RGB565 */
	{ OV7670_REG_RGB444, 0x00U },            /* RGB444 disabled */
	{ OV7670_REG_TSLB, 0x04U },
	{ OV7670_REG_COM10, 0x00U },

	/* ---------- 2. Output window configuration ---------- */
	{ OV7670_REG_HSTART, 0x18U },
	{ OV7670_REG_HSTOP,  0x06U },
	{ OV7670_REG_HREF,   0x24U },
	{ OV7670_REG_VSTART, 0x02U },
	{ OV7670_REG_VSTOP,  0x7AU },
	{ 0x03U, 0x0AU },                        /* VREF */

	/* ---------- 3. Gamma correction ---------- */
	{ 0x7AU, 0x20U }, { 0x7BU, 0x10U }, { 0x7CU, 0x1EU }, { 0x7DU, 0x35U },
	{ 0x7EU, 0x5AU }, { 0x7FU, 0x69U }, { 0x80U, 0x76U }, { 0x81U, 0x80U },
	{ 0x82U, 0x88U }, { 0x83U, 0x8FU }, { 0x84U, 0x96U }, { 0x85U, 0xA3U },
	{ 0x86U, 0xAFU }, { 0x87U, 0xC4U }, { 0x88U, 0xD7U }, { 0x89U, 0xE8U },

	/* ---------- 4. AGC / AEC ---------- */
	{ 0x00U, 0x00U },                        /* GAIN */
	{ 0x10U, 0x00U },                        /* AECH */
	{ 0x0DU, 0x40U },                        /* COM4 */
	{ 0x14U, 0x38U },                        /* COM9: AGC ceiling 16x */
	{ 0xA5U, 0x05U }, { 0xABU, 0x07U },      /* BD50MAX, BD60MAX */
	{ 0x24U, 0x95U }, { 0x25U, 0x33U }, { 0x26U, 0xE3U },
	{ 0x9FU, 0x78U }, { 0xA0U, 0x68U }, { 0xA1U, 0x03U },
	{ 0xA6U, 0xD8U }, { 0xA7U, 0xD8U }, { 0xA8U, 0xF0U },
	{ 0xA9U, 0x90U }, { 0xAAU, 0x94U },

	/* ---------- 5. Color-related control registers ---------- */
	{ 0xB0U, 0x84U },
	{ 0xB1U, 0x0CU }, { 0xB2U, 0x0EU }, { 0xB3U, 0x82U }, { 0xB8U, 0x0AU },
	{ 0x4DU, 0x40U }, { 0x4EU, 0x20U }, { 0x74U, 0x10U },

	/* ---------- 6. Automatic white balance (AWB) ---------- */
	{ 0x43U, 0x0AU }, { 0x44U, 0xF0U }, { 0x45U, 0x34U },
	{ 0x46U, 0x58U }, { 0x47U, 0x28U }, { 0x48U, 0x3AU },
	{ 0x59U, 0x88U }, { 0x5AU, 0x88U }, { 0x5BU, 0x44U },
	{ 0x5CU, 0x67U }, { 0x5DU, 0x49U }, { 0x5EU, 0x0EU },
	{ 0x6CU, 0x0AU }, { 0x6DU, 0x55U }, { 0x6EU, 0x11U },
	{ 0x6FU, 0x9FU }, { 0x6AU, 0x40U },
	{ 0x01U, 0x40U },                        /* Initial blue gain */
	{ 0x02U, 0x60U },                        /* Initial red gain */

	/* ---------- 7. RGB color matrix and saturation ---------- */
	{ 0x4FU, 0xB3U }, { 0x50U, 0xB3U }, { 0x51U, 0x00U },
	{ 0x52U, 0x3DU }, { 0x53U, 0xA7U }, { 0x54U, 0xE4U },
	{ 0x58U, 0x9EU },                        /* MTXS */
	{ 0x41U, 0x38U },                        /* COM16: AWB gain + edge/denoise */
	{ 0x3FU, 0x00U },                        /* EDGE */
	{ 0x75U, 0x05U }, { 0x76U, 0xE1U }, { 0x4CU, 0x00U }, { 0x77U, 0x01U },
	{ 0x3DU, 0xC0U },                        /* COM13: gamma ON + UV sat auto */
	{ 0x4BU, 0x09U },                        /* UV averaging */
	{ 0xC9U, 0x60U },                        /* saturation control */
	{ 0x56U, 0x40U },                        /* contrast */

	/* ---------- 8. Enable AEC, AGC, and AWB after profile configuration ---------- */
	{ OV7670_REG_COM8, 0xE7U }
};

static const OV7670_RegisterValue_t s_verify_registers[OV7670_CONFIG_CHECK_COUNT] =
		{ { OV7670_REG_CLKRC, 0x01U }, { OV7670_REG_COM7, 0x04U }, {
		OV7670_REG_COM3, 0x04U }, { OV7670_REG_COM14, 0x1AU }, {
		OV7670_REG_SCALING_XSC, 0x3AU }, { OV7670_REG_SCALING_YSC, 0x35U }, {
		OV7670_REG_SCALING_DCWCTR, 0x22U }, {
		OV7670_REG_SCALING_PCLK_DIV, 0xF2U }, {
		OV7670_REG_SCALING_PCLK_DELAY, 0x02U }, {
		OV7670_REG_COM15, 0xD0U }, { OV7670_REG_RGB444, 0x00U }, {
		OV7670_REG_TSLB, 0x04U }, { OV7670_REG_COM8, 0xE7U } };

static uint8_t s_dwt_initialized;
static uint8_t s_siod_is_input;
static uint8_t s_camera_initialized;
static TIM_HandleTypeDef *s_xclk_timer;
static uint32_t s_xclk_channel;
static OV7670_Info_t s_camera_info;
static OV7670_RegisterCheck_t s_last_config_checks[OV7670_CONFIG_CHECK_COUNT];
static uint8_t s_last_config_failed_count;

static void OV7670_DWT_Init(void) {
	if (s_dwt_initialized != 0U) {
		return;
	}

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	s_dwt_initialized = 1U;
}

static void OV7670_DelayUs(uint32_t delay_us) {
	uint32_t start;
	uint32_t ticks;
	uint32_t cycles_per_us = SystemCoreClock / 1000000U;

	if (cycles_per_us == 0U) {
		cycles_per_us = 1U;
	}

	ticks = delay_us * cycles_per_us;
	start = DWT->CYCCNT;
	while ((uint32_t) (DWT->CYCCNT - start) < ticks) {
		/* DWT cycle-counter delay for the software SCCB bus. */
	}
}

static void OV7670_SIOD_SetInput(void) {
	GPIO_InitTypeDef gpio = { 0 };

	if (s_siod_is_input != 0U) {
		return;
	}

	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_SET);
	gpio.Pin = CAM_SIOD_Pin;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAM_SIOD_GPIO_Port, &gpio);
	s_siod_is_input = 1U;
}

static void OV7670_SIOD_SetOpenDrain(void) {
	GPIO_InitTypeDef gpio = { 0 };

	if (s_siod_is_input == 0U) {
		return;
	}

	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_SET);
	gpio.Pin = CAM_SIOD_Pin;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAM_SIOD_GPIO_Port, &gpio);
	s_siod_is_input = 0U;
}

static void OV7670_ConfigureSCCBPins(void) {
	GPIO_InitTypeDef gpio = { 0 };

	HAL_GPIO_WritePin(CAM_SIOC_GPIO_Port, CAM_SIOC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_SET);

	gpio.Pin = CAM_SIOC_Pin | CAM_SIOD_Pin;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAM_SIOC_GPIO_Port, &gpio);
	s_siod_is_input = 0U;
}

static HAL_StatusTypeDef OV7670_ConfigureXCLKPin(void) {
	GPIO_InitTypeDef gpio = { 0 };

	if (CAM_XCLK_Pin == 0U) {
		return HAL_ERROR;
	}

	gpio.Pin = CAM_XCLK_Pin;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(CAM_XCLK_GPIO_Port, &gpio);
	return HAL_OK;
}

static void OV7670_SCCB_ClockLow(void) {
	HAL_GPIO_WritePin(CAM_SIOC_GPIO_Port, CAM_SIOC_Pin, GPIO_PIN_RESET);
}

static void OV7670_SCCB_ClockRelease(void) {
	HAL_GPIO_WritePin(CAM_SIOC_GPIO_Port, CAM_SIOC_Pin, GPIO_PIN_SET);
}

static void OV7670_SCCB_DataLow(void) {
	OV7670_SIOD_SetOpenDrain();
	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_RESET);
}

static void OV7670_SCCB_DataRelease(void) {
	OV7670_SIOD_SetOpenDrain();
	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_SET);
}

static GPIO_PinState OV7670_SCCB_ReadData(void) {
	return HAL_GPIO_ReadPin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin);
}

/* Verify that both open-drain bus lines are released HIGH before/after a
 * public register operation. The camera does not provide a conventional
 * I2C ACK, so line-idle and register read-back checks are important. */
static uint8_t OV7670_SCCB_BusIsIdle(void) {
	uint8_t sioc_high;
	uint8_t siod_high;

	OV7670_SCCB_ClockRelease();
	OV7670_SCCB_DataRelease();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	sioc_high =
			(HAL_GPIO_ReadPin(CAM_SIOC_GPIO_Port, CAM_SIOC_Pin) == GPIO_PIN_SET) ?
					1U : 0U;
	OV7670_SIOD_SetInput();
	siod_high =
			(HAL_GPIO_ReadPin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin) == GPIO_PIN_SET) ?
					1U : 0U;
	OV7670_SIOD_SetOpenDrain();
	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_SET);
	return ((sioc_high != 0U) && (siod_high != 0U)) ? 1U : 0U;
}

static void OV7670_SCCB_Start(void) {
	OV7670_SIOD_SetOpenDrain();
	OV7670_SCCB_DataRelease();
	OV7670_SCCB_ClockRelease();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SCCB_DataLow();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SCCB_ClockLow();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
}

static void OV7670_SCCB_Stop(void) {
	OV7670_SIOD_SetOpenDrain();
	OV7670_SCCB_ClockLow();
	OV7670_SCCB_DataLow();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SCCB_ClockRelease();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SCCB_DataRelease();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
}

/* The ninth SCCB clock is a don't-care/turnaround clock, not a mandatory
 * conventional-I2C ACK. Keep it sampled for diagnostics but do not require
 * either level: modules/controllers vary in how this bit is driven. */
static uint8_t OV7670_SCCB_WriteByte(uint8_t value) {
	uint8_t dc_level;

	OV7670_SIOD_SetOpenDrain();
	for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
		OV7670_SCCB_ClockLow();
		if ((value & mask) != 0U) {
			OV7670_SCCB_DataRelease();
		} else {
			OV7670_SCCB_DataLow();
		}
		OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
		OV7670_SCCB_ClockRelease();
		OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	}

	/* Lower SIOC before releasing SIOD, otherwise a final zero bit can look
	 * like an unintended STOP condition. */
	OV7670_SCCB_ClockLow();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SIOD_SetInput();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SCCB_ClockRelease();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	dc_level = (OV7670_SCCB_ReadData() == GPIO_PIN_SET) ? 1U : 0U;
	OV7670_SCCB_ClockLow();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SIOD_SetOpenDrain();
	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_SET);
	return dc_level;
}

static uint8_t OV7670_SCCB_ReadByte(void) {
	uint8_t value = 0U;

	OV7670_SIOD_SetInput();
	for (uint8_t bit = 0U; bit < 8U; ++bit) {
		OV7670_SCCB_ClockLow();
		OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
		OV7670_SCCB_ClockRelease();
		OV7670_DelayUs(2U);
		value <<= 1U;
		if (OV7670_SCCB_ReadData() == GPIO_PIN_SET) {
			value |= 1U;
		}
		OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US - 2U);
	}

	OV7670_SCCB_ClockLow();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SIOD_SetOpenDrain();
	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_SET);
	OV7670_SCCB_ClockRelease();
	OV7670_DelayUs(OV7670_SCCB_HALF_PERIOD_US);
	OV7670_SCCB_ClockLow();
	return value;
}

static uint8_t OV7670_SCCB_ReadRegisterInternal(uint8_t reg, uint8_t *value,
		uint8_t *dc_write_address, uint8_t *dc_register_address,
		uint8_t *dc_read_address) {
	if (value == NULL) {
		return 0U;
	}

	/* Phase 1: address + register index, STOP. Phase 2: read address + data. */
	OV7670_SCCB_Start();
	if (dc_write_address != NULL) {
		*dc_write_address = OV7670_SCCB_WriteByte(OV7670_SCCB_WRITE_ADDRESS);
	} else {
		(void) OV7670_SCCB_WriteByte(OV7670_SCCB_WRITE_ADDRESS);
	}
	if (dc_register_address != NULL) {
		*dc_register_address = OV7670_SCCB_WriteByte(reg);
	} else {
		(void) OV7670_SCCB_WriteByte(reg);
	}
	OV7670_SCCB_Stop();
	OV7670_DelayUs(2U);

	OV7670_SCCB_Start();
	if (dc_read_address != NULL) {
		*dc_read_address = OV7670_SCCB_WriteByte(OV7670_SCCB_READ_ADDRESS);
	} else {
		(void) OV7670_SCCB_WriteByte(OV7670_SCCB_READ_ADDRESS);
	}
	*value = OV7670_SCCB_ReadByte();
	OV7670_SCCB_Stop();
	return 1U;
}

static uint8_t OV7670_SCCB_WriteRegisterInternal(uint8_t reg, uint8_t value) {
	OV7670_SCCB_Start();
	(void) OV7670_SCCB_WriteByte(OV7670_SCCB_WRITE_ADDRESS);
	(void) OV7670_SCCB_WriteByte(reg);
	(void) OV7670_SCCB_WriteByte(value);
	OV7670_SCCB_Stop();
	return 1U;
}

static void OV7670_EnterSafeState(TIM_HandleTypeDef *timer, uint32_t channel) {
	HAL_GPIO_WritePin(CAM_RESET_N_GPIO_Port, CAM_RESET_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_SET);

	if (timer != NULL) {
		(void) HAL_TIM_PWM_Stop(timer, channel);
	}

	OV7670_SCCB_ClockRelease();
	OV7670_SCCB_DataRelease();
}

static OV7670_Status_t OV7670_StartAndIdentify(TIM_HandleTypeDef *xclk_timer,
		uint32_t xclk_channel, OV7670_Info_t *info,
		OV7670_Diagnostics_t *diagnostics) {
	uint8_t sioc_idle;
	uint8_t siod_idle;

	if (diagnostics != NULL) {
		memset(diagnostics, 0, sizeof(*diagnostics));
	}

	if ((xclk_timer == NULL) || (xclk_timer->Instance != TIM1)
			|| (xclk_channel != TIM_CHANNEL_1)) {
		return OV7670_STATUS_XCLK_ERROR;
	}

	OV7670_DWT_Init();
	OV7670_ConfigureSCCBPins();
	(void) HAL_TIM_PWM_Stop(xclk_timer, xclk_channel);

	HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CAM_RESET_N_GPIO_Port, CAM_RESET_N_Pin, GPIO_PIN_RESET);
	HAL_Delay(2U);

	if (OV7670_ConfigureXCLKPin() != HAL_OK) {
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
		return OV7670_STATUS_XCLK_ERROR;
	}
	if (diagnostics != NULL) {
		diagnostics->xclk_pin_configured = 1U;
	}
	if (HAL_TIM_PWM_Start(xclk_timer, xclk_channel) != HAL_OK) {
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
		return OV7670_STATUS_XCLK_ERROR;
	}
	if (diagnostics != NULL) {
		diagnostics->xclk_started = 1U;
	}

	HAL_Delay(OV7670_XCLK_STARTUP_MS);
	HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);
	HAL_Delay(OV7670_POWERUP_SETTLE_MS);
	HAL_GPIO_WritePin(CAM_RESET_N_GPIO_Port, CAM_RESET_N_Pin, GPIO_PIN_SET);
	HAL_Delay(OV7670_RESET_SETTLE_MS);

	OV7670_SCCB_ClockRelease();
	OV7670_SCCB_DataRelease();
	HAL_Delay(1U);
	sioc_idle =
			(HAL_GPIO_ReadPin(CAM_SIOC_GPIO_Port, CAM_SIOC_Pin) == GPIO_PIN_SET) ?
					1U : 0U;
	OV7670_SIOD_SetInput();
	siod_idle =
			(HAL_GPIO_ReadPin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin) == GPIO_PIN_SET) ?
					1U : 0U;
	OV7670_SIOD_SetOpenDrain();
	HAL_GPIO_WritePin(CAM_SIOD_GPIO_Port, CAM_SIOD_Pin, GPIO_PIN_SET);
	if (diagnostics != NULL) {
		diagnostics->sioc_idle_high = sioc_idle;
		diagnostics->siod_idle_high = siod_idle;
	}

	if ((sioc_idle == 0U) || (siod_idle == 0U)) {
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
		return OV7670_STATUS_BUS_STUCK;
	}

	if ((OV7670_SCCB_ReadRegisterInternal(OV7670_REG_PID, &info->pid,
			(diagnostics != NULL) ? &diagnostics->dc_write_address : NULL,
			NULL, (diagnostics != NULL) ? &diagnostics->dc_read_address : NULL)
			== 0U)
			|| (OV7670_SCCB_ReadRegisterInternal(OV7670_REG_VER, &info->ver,
			NULL,
					(diagnostics != NULL) ?
							&diagnostics->dc_register_address : NULL,
					NULL) == 0U)) {
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
		return OV7670_STATUS_ID_READ_ERROR;
	}

	if (OV7670_SCCB_ReadRegisterInternal(OV7670_REG_MIDH, &info->midh,
	NULL, NULL, NULL) != 0U) {
		if (diagnostics != NULL) {
			diagnostics->midh_read_ok = 1U;
		}
	}
	if (OV7670_SCCB_ReadRegisterInternal(OV7670_REG_MIDL, &info->midl,
	NULL, NULL, NULL) != 0U) {
		if (diagnostics != NULL) {
			diagnostics->midl_read_ok = 1U;
		}
	}
	if (diagnostics != NULL) {
		diagnostics->pid = info->pid;
		diagnostics->ver = info->ver;
		diagnostics->midh = info->midh;
		diagnostics->midl = info->midl;
		diagnostics->pid_read_ok = 1U;
		diagnostics->ver_read_ok = 1U;
	}

	if ((info->pid == 0x00U) || (info->pid == 0xFFU) || (info->ver == 0x00U)
			|| (info->ver == 0xFFU)) {
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
		return OV7670_STATUS_ID_READ_ERROR;
	}
	if ((info->pid != OV7670_PID_VALUE)
			|| ((info->ver != OV7670_VER_VALUE_A)
					&& (info->ver != OV7670_VER_VALUE_B))) {
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
		return OV7670_STATUS_ID_MISMATCH;
	}

	return OV7670_STATUS_OK;
}

static OV7670_Status_t OV7670_ApplyProfile(void) {
	for (uint8_t i = 0U; i < OV7670_ARRAY_COUNT(s_qqvga_rgb565_registers);
			++i) {
		if (OV7670_SCCB_WriteRegisterInternal(s_qqvga_rgb565_registers[i].reg,
				s_qqvga_rgb565_registers[i].value) == 0U) {
			return OV7670_STATUS_REGISTER_WRITE_ERROR;
		}
	}

	HAL_Delay(OV7670_CONFIG_SETTLE_MS);
	return OV7670_STATUS_OK;
}

void OV7670_GetDefaultConfig(OV7670_Config_t *config) {
	if (config == NULL) {
		return;
	}

	config->resolution = OV7670_RESOLUTION_QQVGA_160X120;
	config->pixel_format = OV7670_PIXEL_FORMAT_RGB565;
}

OV7670_Status_t OV7670_Init(TIM_HandleTypeDef *xclk_timer,
		uint32_t xclk_channel, const OV7670_Config_t *config,
		OV7670_Info_t *info) {
	OV7670_Config_t selected_config;
	OV7670_Info_t local_info;
	OV7670_Status_t status;
	uint8_t failed_count = 0U;

	if (info != NULL) {
		memset(info, 0, sizeof(*info));
	}

	if ((xclk_timer == NULL) || (xclk_timer->Instance != TIM1)
			|| (xclk_channel != TIM_CHANNEL_1)) {
		return OV7670_STATUS_XCLK_ERROR;
	}

	if (config == NULL) {
		OV7670_GetDefaultConfig(&selected_config);
	} else {
		selected_config = *config;
	}

	if ((selected_config.resolution != OV7670_RESOLUTION_QQVGA_160X120)
			|| (selected_config.pixel_format != OV7670_PIXEL_FORMAT_RGB565)) {
		return OV7670_STATUS_UNSUPPORTED_CONFIG;
	}

	memset(&local_info, 0, sizeof(local_info));
	memset(&s_camera_info, 0, sizeof(s_camera_info));
	memset(s_last_config_checks, 0, sizeof(s_last_config_checks));
	s_last_config_failed_count = 0U;
	s_camera_initialized = 0U;
	s_xclk_timer = NULL;

	status = OV7670_StartAndIdentify(xclk_timer, xclk_channel, &local_info,
	NULL);
	if (info != NULL) {
		*info = local_info;
	}
	if (status != OV7670_STATUS_OK) {
		return status;
	}

	/* Soft reset precedes the configuration profile. XCLK remains active. */
	(void) OV7670_SCCB_WriteRegisterInternal(OV7670_REG_COM7, 0x80U);
	HAL_Delay(OV7670_SOFT_RESET_SETTLE_MS);

	status = OV7670_ApplyProfile();
	if (status != OV7670_STATUS_OK) {
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
		return status;
	}

	/* Enable the guarded register-read path used by configuration verification. */
	s_camera_initialized = 1U;
	s_xclk_timer = xclk_timer;
	s_xclk_channel = xclk_channel;

	status = OV7670_VerifyConfiguration(s_last_config_checks,
	OV7670_CONFIG_CHECK_COUNT, &failed_count);
	s_last_config_failed_count = failed_count;
	if ((status != OV7670_STATUS_OK) || (failed_count != 0U)) {
		s_camera_initialized = 0U;
		s_xclk_timer = NULL;
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
		return (status == OV7670_STATUS_OK) ?
				OV7670_STATUS_CONFIG_VERIFY_ERROR : status;
	}

	local_info.width = OV7670_QQVGA_WIDTH;
	local_info.height = OV7670_QQVGA_HEIGHT;
	local_info.bytes_per_pixel = OV7670_RGB565_BYTES_PER_PIXEL;
	local_info.frame_size_bytes = OV7670_QQVGA_RGB565_FRAME_BYTES;
	local_info.xclk_hz = OV7670_XCLK_HZ;
	s_camera_info = local_info;

	if (info != NULL) {
		*info = local_info;
	}

	/* Success: keep PWDN low, RESET_N high, and TIM1_CH1 running. */
	return OV7670_STATUS_OK;
}

OV7670_Status_t OV7670_DeInit(void) {
	if (s_xclk_timer == NULL) {
		HAL_GPIO_WritePin(CAM_RESET_N_GPIO_Port, CAM_RESET_N_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_SET);
		return OV7670_STATUS_OK;
	}

	OV7670_EnterSafeState(s_xclk_timer, s_xclk_channel);
	s_xclk_timer = NULL;
	s_camera_initialized = 0U;
	memset(&s_camera_info, 0, sizeof(s_camera_info));
	return OV7670_STATUS_OK;
}

OV7670_Status_t OV7670_ReadRegister(uint8_t reg, uint8_t *value) {
	if (value == NULL) {
		return OV7670_STATUS_BAD_ARGUMENT;
	}
	if (s_camera_initialized == 0U) {
		return OV7670_STATUS_NOT_INITIALIZED;
	}
	if (OV7670_SCCB_BusIsIdle() == 0U) {
		return OV7670_STATUS_BUS_STUCK;
	}
	if (OV7670_SCCB_ReadRegisterInternal(reg, value, NULL, NULL, NULL) == 0U) {
		return OV7670_STATUS_REGISTER_READ_ERROR;
	}
	return (OV7670_SCCB_BusIsIdle() != 0U) ?
			OV7670_STATUS_OK : OV7670_STATUS_BUS_STUCK;
}

OV7670_Status_t OV7670_WriteRegister(uint8_t reg, uint8_t value) {
	if (s_camera_initialized == 0U) {
		return OV7670_STATUS_NOT_INITIALIZED;
	}
	if (OV7670_SCCB_BusIsIdle() == 0U) {
		return OV7670_STATUS_BUS_STUCK;
	}
	if (OV7670_SCCB_WriteRegisterInternal(reg, value) == 0U) {
		return OV7670_STATUS_REGISTER_WRITE_ERROR;
	}
	return (OV7670_SCCB_BusIsIdle() != 0U) ?
			OV7670_STATUS_OK : OV7670_STATUS_BUS_STUCK;
}

OV7670_Status_t OV7670_VerifyConfiguration(OV7670_RegisterCheck_t *checks,
		uint8_t checks_capacity, uint8_t *failed_count) {
	uint8_t failures = 0U;

	if ((checks == NULL) || (failed_count == NULL)
			|| (checks_capacity < OV7670_CONFIG_CHECK_COUNT)) {
		return OV7670_STATUS_BAD_ARGUMENT;
	}
	if (s_camera_initialized == 0U) {
		return OV7670_STATUS_NOT_INITIALIZED;
	}

	memset(checks, 0,
			sizeof(OV7670_RegisterCheck_t) * OV7670_CONFIG_CHECK_COUNT);
	for (uint8_t i = 0U; i < OV7670_CONFIG_CHECK_COUNT; ++i) {
		checks[i].reg = s_verify_registers[i].reg;
		checks[i].expected = s_verify_registers[i].value;
	}

	if (OV7670_SCCB_BusIsIdle() == 0U) {
		*failed_count = OV7670_CONFIG_CHECK_COUNT;
		return OV7670_STATUS_BUS_STUCK;
	}

	for (uint8_t i = 0U; i < OV7670_CONFIG_CHECK_COUNT; ++i) {
		if (OV7670_SCCB_ReadRegisterInternal(checks[i].reg, &checks[i].actual,
		NULL, NULL, NULL) != 0U) {
			checks[i].read_ok = 1U;
			checks[i].match =
					(checks[i].actual == checks[i].expected) ? 1U : 0U;
		}
		if ((checks[i].read_ok == 0U) || (checks[i].match == 0U)) {
			++failures;
		}
	}

	*failed_count = failures;
	if (OV7670_SCCB_BusIsIdle() == 0U) {
		return OV7670_STATUS_BUS_STUCK;
	}
	return (failures == 0U) ?
			OV7670_STATUS_OK : OV7670_STATUS_CONFIG_VERIFY_ERROR;
}

OV7670_Status_t OV7670_GetLastConfigurationChecks(
		OV7670_RegisterCheck_t *checks, uint8_t checks_capacity,
		uint8_t *failed_count) {
	if ((checks == NULL) || (failed_count == NULL)
			|| (checks_capacity < OV7670_CONFIG_CHECK_COUNT)) {
		return OV7670_STATUS_BAD_ARGUMENT;
	}

	memcpy(checks, s_last_config_checks, sizeof(s_last_config_checks));
	*failed_count = s_last_config_failed_count;
	return OV7670_STATUS_OK;
}

const char* OV7670_StatusString(OV7670_Status_t status) {
	switch (status) {
	case OV7670_STATUS_OK:
		return "OK";
	case OV7670_STATUS_XCLK_ERROR:
		return "XCLK_ERROR";
	case OV7670_STATUS_BUS_STUCK:
		return "BUS_STUCK";
	case OV7670_STATUS_ID_READ_ERROR:
		return "ID_READ_ERROR";
	case OV7670_STATUS_ID_MISMATCH:
		return "ID_MISMATCH";
	case OV7670_STATUS_BAD_ARGUMENT:
		return "BAD_ARGUMENT";
	case OV7670_STATUS_UNSUPPORTED_CONFIG:
		return "UNSUPPORTED_CONFIG";
	case OV7670_STATUS_NOT_INITIALIZED:
		return "NOT_INITIALIZED";
	case OV7670_STATUS_REGISTER_READ_ERROR:
		return "REGISTER_READ_ERROR";
	case OV7670_STATUS_REGISTER_WRITE_ERROR:
		return "REGISTER_WRITE_ERROR";
	case OV7670_STATUS_CONFIG_VERIFY_ERROR:
		return "CONFIG_VERIFY_ERROR";
	default:
		return "UNKNOWN";
	}
}

uint8_t OV7670_IsInitialized(void) {
	return s_camera_initialized;
}

OV7670_Status_t OV7670_GetInfo(OV7670_Info_t *info) {
	if (info == NULL) {
		return OV7670_STATUS_BAD_ARGUMENT;
	}
	if (s_camera_initialized == 0U) {
		return OV7670_STATUS_NOT_INITIALIZED;
	}
	*info = s_camera_info;
	return OV7670_STATUS_OK;
}

OV7670_Status_t OV7670_ProbeEx(TIM_HandleTypeDef *xclk_timer,
		uint32_t xclk_channel, uint8_t *pid, uint8_t *ver,
		OV7670_Diagnostics_t *diagnostics) {
	OV7670_Info_t info;
	OV7670_Status_t status;

	if ((pid == NULL) || (ver == NULL)) {
		return OV7670_STATUS_BAD_ARGUMENT;
	}

	memset(&info, 0, sizeof(info));
	status = OV7670_StartAndIdentify(xclk_timer, xclk_channel, &info,
			diagnostics);
	*pid = info.pid;
	*ver = info.ver;
	if ((xclk_timer != NULL) && (xclk_timer->Instance == TIM1)
			&& (xclk_channel == TIM_CHANNEL_1)) {
		OV7670_EnterSafeState(xclk_timer, xclk_channel);
	} else {
		HAL_GPIO_WritePin(CAM_RESET_N_GPIO_Port, CAM_RESET_N_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_SET);
	}
	s_camera_initialized = 0U;
	s_xclk_timer = NULL;
	return status;
}

OV7670_Status_t OV7670_Probe(TIM_HandleTypeDef *xclk_timer,
		uint32_t xclk_channel, uint8_t *pid, uint8_t *ver) {
	return OV7670_ProbeEx(xclk_timer, xclk_channel, pid, ver, NULL);
}
