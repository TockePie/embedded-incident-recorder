#ifndef CONFIG_SERVICE_H
#define CONFIG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	CONFIG_MODE_CARGO = 0,
	CONFIG_MODE_SITE = 1
} ConfigMode_t;

/* This structure is stored in external Flash. Preserve its field order and
 * size, and update the storage schema when changing its layout. */
typedef struct {
	/* Cargo shock peak or Site vibration RMS threshold, depending on mode. */
	uint16_t impact_threshold_mg;
	uint16_t tilt_threshold_deg;
	int16_t temp_min_c;
	int16_t temp_max_c;
} ConfigProfile_t;

typedef struct {
	uint32_t mode;
	ConfigProfile_t cargo;
	ConfigProfile_t site;
} ConfigData_t;

typedef enum {
	CONFIG_SAVE_IDLE = 0,
	CONFIG_SAVE_IN_PROGRESS,
	CONFIG_SAVE_OK,
	CONFIG_SAVE_ERROR
} ConfigSaveStatus_t;

typedef enum {
	CONFIG_RESULT_OK = 0,
	CONFIG_RESULT_BUSY,
	CONFIG_RESULT_LOCKED,
	CONFIG_RESULT_INVALID,
	CONFIG_RESULT_IO_ERROR,
	CONFIG_RESULT_BAD_PASSWORD,
	CONFIG_RESULT_NOT_SEALED
} ConfigResult_t;

bool ConfigService_Init(void);
void ConfigService_Process(void);
void ConfigService_Get(const ConfigData_t **config, bool *sealed,
		uint32_t *generation);
ConfigSaveStatus_t ConfigService_GetSaveStatus(void);
ConfigResult_t ConfigService_SetMode(ConfigMode_t mode);

/* Shock is available in Cargo; vibration RMS is available in Site. */
ConfigResult_t ConfigService_SetShockThreshold(uint16_t mg);
ConfigResult_t ConfigService_SetVibrationThreshold(uint16_t mg);
ConfigResult_t ConfigService_SetTilt(ConfigMode_t mode, uint16_t degrees);
ConfigResult_t ConfigService_SetTemperature(ConfigMode_t mode,
		int16_t min_c, int16_t max_c);
ConfigResult_t ConfigService_Seal(const char *password);
ConfigResult_t ConfigService_Unseal(const char *password);
ConfigResult_t ConfigService_VerifyPassword(const char *password);
ConfigResult_t ConfigService_FactoryReset(void);
uint32_t ConfigService_GetWrongPasswordCount(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SERVICE_H */
