#include "services/config_service.h"

#include "drivers/w25q64.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "utils/crypto_sha256.h"

#include <stddef.h>
#include <string.h>

#define CFG_SLOT_A_ADDR          0x000000UL
#define CFG_SLOT_B_ADDR          0x001000UL
#define CFG_MAGIC                0x43464733UL
#define CFG_SCHEMA               1U
#define CFG_COMMIT_MARKER        0x51A7C0DEUL
#define CFG_PBKDF2_ROUNDS        2048UL
#define CFG_PASSWORD_MIN         8U
#define CFG_PASSWORD_MAX         32U
#define CFG_SALT_SIZE            16U
#define CFG_HASH_SIZE            32U
#define CFG_TEMP_MIN_LIMIT       (-40)
#define CFG_TEMP_MAX_LIMIT       (85)

#define CFG_UID_BASE             0x1FFF7A10UL

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t schema;
	uint16_t record_size;
	uint32_t generation;
	uint8_t sealed;
	uint8_t reserved[3];
	uint8_t salt[CFG_SALT_SIZE];
	uint8_t password_hash[CFG_HASH_SIZE];
	ConfigData_t config;
	uint32_t crc32;
	uint32_t commit;
} ConfigRecord_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(ConfigData_t) == 20U,
		"Persisted ConfigData_t layout changed; increment schema version");
_Static_assert(sizeof(ConfigRecord_t) <= W25Q64_PAGE_SIZE_BYTES,
		"Configuration record must fit in one W25Q64 page");
#endif

typedef enum {
	SAVE_IDLE = 0,
	SAVE_ERASE_WAIT,
	SAVE_PROGRAM_WAIT,
	SAVE_VERIFY,
	SAVE_COMMIT_WAIT,
	SAVE_FINAL_VERIFY,
	SAVE_CLEANUP_ERASE_WAIT
} SaveState_t;

static ConfigData_t s_config;
static ConfigData_t s_candidate;
static ConfigRecord_t s_record;
static ConfigRecord_t s_verify;
static uint8_t s_salt[CFG_SALT_SIZE];
static uint8_t s_hash[CFG_HASH_SIZE];
static uint8_t s_sealed;
static uint8_t s_active_slot = 0xFFU;
static uint8_t s_target_slot;
static uint32_t s_generation;
static uint32_t s_wrong_password_count;
static SaveState_t s_save_state;
static ConfigSaveStatus_t s_save_status;
static uint8_t s_candidate_sealed;
static uint8_t s_candidate_salt[CFG_SALT_SIZE];
static uint8_t s_candidate_hash[CFG_HASH_SIZE];
static uint32_t s_candidate_generation;
static uint8_t s_cleanup_old_slot;
static uint8_t s_cleanup_slot;

static uint32_t Config_Crc32(const uint8_t *data, size_t length) {
	uint32_t crc = 0xFFFFFFFFUL;
	size_t index;
	unsigned bit;

	for (index = 0U; index < length; index++) {
		crc ^= data[index];
		for (bit = 0U; bit < 8U; bit++) {
			crc = (crc >> 1U) ^ (((crc & 1U) != 0U) ? 0xEDB88320UL : 0UL);
		}
	}

	return ~crc;
}

static uint32_t Config_SlotAddress(uint8_t slot) {
	return (slot == 0U) ? CFG_SLOT_A_ADDR : CFG_SLOT_B_ADDR;
}

static bool Config_IsValid(const ConfigData_t *config) {
	const ConfigProfile_t *profiles[2];
	unsigned index;

	if ((config == NULL)
			|| ((config->mode != CONFIG_MODE_CARGO)
					&& (config->mode != CONFIG_MODE_SITE))) {
		return false;
	}

	profiles[0] = &config->cargo;
	profiles[1] = &config->site;

	for (index = 0U; index < 2U; index++) {
		const ConfigProfile_t *profile = profiles[index];

		if ((profile->impact_threshold_mg < 100U)
				|| (profile->impact_threshold_mg > 16000U)
				|| (profile->tilt_threshold_deg < 1U)
				|| (profile->tilt_threshold_deg > 180U)
				|| (profile->temp_min_c < CFG_TEMP_MIN_LIMIT)
				|| (profile->temp_max_c > CFG_TEMP_MAX_LIMIT)
				|| (profile->temp_min_c >= profile->temp_max_c)) {
			return false;
		}
	}

	return true;
}

static bool Config_RecordIsValid(const ConfigRecord_t *record) {
	ConfigData_t config_copy;

	if ((record->magic != CFG_MAGIC) || (record->schema != CFG_SCHEMA)
			|| (record->record_size != sizeof(*record)) || (record->sealed > 1U)
			|| (record->commit != CFG_COMMIT_MARKER)) {
		return false;
	}

	if (record->crc32
			!= Config_Crc32((const uint8_t*) record,
					offsetof(ConfigRecord_t, crc32))) {
		return false;
	}

	/* Copy out of the packed record before accessing naturally aligned fields. */
	memcpy(&config_copy, &record->config, sizeof(config_copy));
	return Config_IsValid(&config_copy);
}

static void Config_SetDefaults(void) {
	s_config.mode = CONFIG_MODE_CARGO;

	s_config.cargo.impact_threshold_mg = 2500U;
	s_config.cargo.tilt_threshold_deg = 45U;
	s_config.cargo.temp_min_c = 15;
	s_config.cargo.temp_max_c = 26;

	s_config.site.impact_threshold_mg = 100U;
	s_config.site.tilt_threshold_deg = 30U;
	s_config.site.temp_min_c = 15;
	s_config.site.temp_max_c = 26;

	s_sealed = 0U;
	memset(s_salt, 0, sizeof(s_salt));
	memset(s_hash, 0, sizeof(s_hash));
	s_generation = 0U;
	s_active_slot = 0xFFU;
}

static bool Config_IsPasswordValid(const char *password, size_t *length) {
	size_t password_length;

	if ((password == NULL) || (length == NULL)) {
		return false;
	}

	password_length = strlen(password);
	if ((password_length < CFG_PASSWORD_MIN)
			|| (password_length > CFG_PASSWORD_MAX)) {
		return false;
	}

	*length = password_length;
	return true;
}

static void Config_MakeSalt(uint8_t salt[CFG_SALT_SIZE], uint32_t generation) {
	const volatile uint32_t *uid = (const volatile uint32_t*) CFG_UID_BASE;
	const uint32_t uid0 = uid[0];
	const uint32_t uid1 = uid[1];
	const uint32_t uid2 = uid[2];
	uint8_t material[20];
	uint8_t digest[32];
	uint32_t values[5];
	unsigned index;

	values[0] = uid0;
	values[1] = uid1;
	values[2] = uid2;
	values[3] = HAL_GetTick();
	values[4] = generation;

	for (index = 0U; index < 5U; index++) {
		material[index * 4U] = (uint8_t) values[index];
		material[index * 4U + 1U] = (uint8_t) (values[index] >> 8U);
		material[index * 4U + 2U] = (uint8_t) (values[index] >> 16U);
		material[index * 4U + 3U] = (uint8_t) (values[index] >> 24U);
	}

	Crypto_Sha256(material, sizeof(material), digest);
	memcpy(salt, digest, CFG_SALT_SIZE);
	memset(material, 0, sizeof(material));
	memset(digest, 0, sizeof(digest));
}

static ConfigResult_t Config_BeginCommitGeneration(const ConfigData_t *config,
		uint8_t sealed, const uint8_t *salt, const uint8_t *password_hash,
		uint32_t generation) {
	W25Q64_Status_t flash_status;

	if (s_save_state != SAVE_IDLE) {
		return CONFIG_RESULT_BUSY;
	}

	if (!Config_IsValid(config)) {
		return CONFIG_RESULT_INVALID;
	}

	s_candidate = *config;
	s_candidate_sealed = sealed;
	memcpy(s_candidate_salt, salt, CFG_SALT_SIZE);
	memcpy(s_candidate_hash, password_hash, CFG_HASH_SIZE);
	s_candidate_generation = generation;

	memset(&s_record, 0xFF, sizeof(s_record));
	s_record.magic = CFG_MAGIC;
	s_record.schema = CFG_SCHEMA;
	s_record.record_size = (uint16_t) sizeof(s_record);
	s_record.generation = s_candidate_generation;
	s_record.sealed = sealed;
	memset(s_record.reserved, 0, sizeof(s_record.reserved));
	memcpy(s_record.salt, salt, CFG_SALT_SIZE);
	memcpy(s_record.password_hash, password_hash, CFG_HASH_SIZE);
	s_record.config = *config;
	s_record.crc32 = Config_Crc32((const uint8_t*) &s_record,
			offsetof(ConfigRecord_t, crc32));
	s_record.commit = CFG_COMMIT_MARKER;

	s_target_slot = (s_active_slot == 0U) ? 1U : 0U;
	s_cleanup_old_slot = 0U;
	s_save_status = CONFIG_SAVE_IN_PROGRESS;

	flash_status = W25Q64_StartSectorErase(&hspi2,
			Config_SlotAddress(s_target_slot));
	if (flash_status != W25Q64_STATUS_OK) {
		s_save_status = CONFIG_SAVE_ERROR;
		return CONFIG_RESULT_IO_ERROR;
	}

	s_save_state = SAVE_ERASE_WAIT;
	return CONFIG_RESULT_OK;
}

static ConfigResult_t Config_BeginCommit(const ConfigData_t *config,
		uint8_t sealed, const uint8_t *salt, const uint8_t *password_hash) {
	return Config_BeginCommitGeneration(config, sealed, salt, password_hash,
			s_generation + 1U);
}

bool ConfigService_Init(void) {
	ConfigRecord_t slot_a;
	ConfigRecord_t slot_b;
	W25Q64_Status_t read_a;
	W25Q64_Status_t read_b;
	bool valid_a;
	bool valid_b;

	s_save_state = SAVE_IDLE;
	s_save_status = CONFIG_SAVE_IDLE;
	s_wrong_password_count = 0U;
	s_cleanup_old_slot = 0U;

	read_a = W25Q64_Read(&hspi2, CFG_SLOT_A_ADDR, (uint8_t*) &slot_a,
			sizeof(slot_a));
	read_b = W25Q64_Read(&hspi2, CFG_SLOT_B_ADDR, (uint8_t*) &slot_b,
			sizeof(slot_b));

	if ((read_a != W25Q64_STATUS_OK) || (read_b != W25Q64_STATUS_OK)) {
		return false;
	}

	valid_a = Config_RecordIsValid(&slot_a);
	valid_b = Config_RecordIsValid(&slot_b);

	if (valid_a && valid_b) {
		const ConfigRecord_t *newest;

		newest =
				((int32_t) (slot_a.generation - slot_b.generation) > 0) ?
						&slot_a : &slot_b;
		s_active_slot = (newest == &slot_a) ? 0U : 1U;
		s_record = *newest;
	} else if (valid_a) {
		s_active_slot = 0U;
		s_record = slot_a;
	} else if (valid_b) {
		s_active_slot = 1U;
		s_record = slot_b;
	} else {
		Config_SetDefaults();
		return true;
	}

	memcpy(&s_config, &s_record.config, sizeof(s_config));
	s_sealed = s_record.sealed;
	s_generation = s_record.generation;
	memcpy(s_salt, s_record.salt, CFG_SALT_SIZE);
	memcpy(s_hash, s_record.password_hash, CFG_HASH_SIZE);

	if ((s_config.cargo.temp_min_c < CFG_TEMP_MIN_LIMIT)
			|| (s_config.cargo.temp_max_c > CFG_TEMP_MAX_LIMIT)
			|| (s_config.site.temp_min_c < CFG_TEMP_MIN_LIMIT)
			|| (s_config.site.temp_max_c > CFG_TEMP_MAX_LIMIT)) {
		ConfigData_t migrated = s_config;

		if ((migrated.cargo.temp_min_c < CFG_TEMP_MIN_LIMIT)
				|| (migrated.cargo.temp_max_c > CFG_TEMP_MAX_LIMIT)) {
			migrated.cargo.temp_min_c = 15;
			migrated.cargo.temp_max_c = 26;
		}

		if ((migrated.site.temp_min_c < CFG_TEMP_MIN_LIMIT)
				|| (migrated.site.temp_max_c > CFG_TEMP_MAX_LIMIT)) {
			migrated.site.temp_min_c = 15;
			migrated.site.temp_max_c = 26;
		}

		if (Config_BeginCommit(&migrated, s_sealed, s_salt, s_hash)
				!= CONFIG_RESULT_OK) {
			return false;
		}
	}

	return true;
}

void ConfigService_Process(void) {
	W25Q64_Status_t flash_status;
	bool flash_busy = false;

	switch (s_save_state) {
	case SAVE_IDLE:
		return;

	case SAVE_ERASE_WAIT:
		flash_status = W25Q64_IsBusy(&hspi2, &flash_busy);
		if (flash_status != W25Q64_STATUS_OK)
			goto save_failed;
		if (flash_busy)
			return;

		flash_status = W25Q64_StartPageProgram(&hspi2,
				Config_SlotAddress(s_target_slot), (const uint8_t*) &s_record,
				(uint16_t) offsetof(ConfigRecord_t, commit));
		if (flash_status != W25Q64_STATUS_OK)
			goto save_failed;
		s_save_state = SAVE_PROGRAM_WAIT;
		return;

	case SAVE_PROGRAM_WAIT:
		flash_status = W25Q64_IsBusy(&hspi2, &flash_busy);
		if (flash_status != W25Q64_STATUS_OK)
			goto save_failed;
		if (flash_busy)
			return;
		s_save_state = SAVE_VERIFY;
		return;

	case SAVE_VERIFY:
		flash_status = W25Q64_Read(&hspi2, Config_SlotAddress(s_target_slot),
				(uint8_t*) &s_verify,
				(uint32_t) offsetof(ConfigRecord_t, commit));
		if (flash_status != W25Q64_STATUS_OK)
			goto save_failed;

		if (memcmp(&s_record, &s_verify, offsetof(ConfigRecord_t, commit))
				!= 0) {
			goto save_failed;
		}

		flash_status = W25Q64_StartPageProgram(&hspi2,
				Config_SlotAddress(s_target_slot) +
				(uint32_t)offsetof(ConfigRecord_t, commit),
				(const uint8_t*) &s_record.commit,
				(uint16_t) sizeof(s_record.commit));
		if (flash_status != W25Q64_STATUS_OK)
			goto save_failed;
		s_save_state = SAVE_COMMIT_WAIT;
		return;

	case SAVE_COMMIT_WAIT:
		flash_status = W25Q64_IsBusy(&hspi2, &flash_busy);
		if (flash_status != W25Q64_STATUS_OK)
			goto save_failed;
		if (flash_busy)
			return;
		s_save_state = SAVE_FINAL_VERIFY;
		return;

	case SAVE_FINAL_VERIFY:
		flash_status = W25Q64_Read(&hspi2, Config_SlotAddress(s_target_slot),
				(uint8_t*) &s_verify, sizeof(s_verify));
		if ((flash_status != W25Q64_STATUS_OK)
				|| !Config_RecordIsValid(&s_verify)) {
			goto save_failed;
		}

		{
			uint8_t previous_slot = s_active_slot;
			s_config = s_candidate;
			s_sealed = s_candidate_sealed;
			s_generation = s_candidate_generation;
			s_active_slot = s_target_slot;
			memcpy(s_salt, s_candidate_salt, CFG_SALT_SIZE);
			memcpy(s_hash, s_candidate_hash, CFG_HASH_SIZE);

			if ((s_cleanup_old_slot != 0U) && (previous_slot <= 1U)) {
				s_cleanup_slot = previous_slot;
				flash_status = W25Q64_StartSectorErase(&hspi2,
						Config_SlotAddress(previous_slot));
				if (flash_status != W25Q64_STATUS_OK) {
					goto save_failed;
				}
				s_save_state = SAVE_CLEANUP_ERASE_WAIT;
				return;
			}
		}
		s_cleanup_old_slot = 0U;
		s_save_status = CONFIG_SAVE_OK;
		s_save_state = SAVE_IDLE;
		return;

	case SAVE_CLEANUP_ERASE_WAIT:
		flash_status = W25Q64_IsBusy(&hspi2, &flash_busy);
		if (flash_status != W25Q64_STATUS_OK)
			goto save_failed;
		if (flash_busy)
			return;
		flash_status = W25Q64_VerifyErased(&hspi2,
				Config_SlotAddress(s_cleanup_slot),
				W25Q64_SECTOR_SIZE_BYTES);
		if (flash_status != W25Q64_STATUS_OK)
			goto save_failed;
		s_cleanup_old_slot = 0U;
		s_save_status = CONFIG_SAVE_OK;
		s_save_state = SAVE_IDLE;
		return;

	default:
		break;
	}

	save_failed: s_cleanup_old_slot = 0U;
	s_save_status = CONFIG_SAVE_ERROR;
	s_save_state = SAVE_IDLE;
}

void ConfigService_Get(const ConfigData_t **config,
bool *sealed, uint32_t *generation) {
	if (config != NULL)
		*config = &s_config;
	if (sealed != NULL)
		*sealed = (s_sealed != 0U);
	if (generation != NULL)
		*generation = s_generation;
}

ConfigSaveStatus_t ConfigService_GetSaveStatus(void) {
	return s_save_status;
}

static ConfigResult_t Config_CommitCurrent(const ConfigData_t *config) {
	return Config_BeginCommit(config, s_sealed, s_salt, s_hash);
}

ConfigResult_t ConfigService_SetMode(ConfigMode_t mode) {
	ConfigData_t candidate = s_config;

	if (s_sealed != 0U)
		return CONFIG_RESULT_LOCKED;
	if ((mode != CONFIG_MODE_CARGO) && (mode != CONFIG_MODE_SITE))
		return CONFIG_RESULT_INVALID;

	candidate.mode = mode;
	return Config_CommitCurrent(&candidate);
}

static ConfigResult_t Config_SetMechanicalThreshold(ConfigMode_t mode,
		uint16_t mg) {
	ConfigData_t candidate = s_config;
	ConfigProfile_t *profile;

	if (s_sealed != 0U)
		return CONFIG_RESULT_LOCKED;
	if (((mode != CONFIG_MODE_CARGO) && (mode != CONFIG_MODE_SITE))
			|| (mg < 100U) || (mg > 16000U)) {
		return CONFIG_RESULT_INVALID;
	}
	profile = (mode == CONFIG_MODE_CARGO) ? &candidate.cargo : &candidate.site;
	profile->impact_threshold_mg = mg;
	return Config_CommitCurrent(&candidate);
}

ConfigResult_t ConfigService_SetShockThreshold(uint16_t mg) {
	return Config_SetMechanicalThreshold(CONFIG_MODE_CARGO, mg);
}

ConfigResult_t ConfigService_SetVibrationThreshold(uint16_t mg) {
	return Config_SetMechanicalThreshold(CONFIG_MODE_SITE, mg);
}

ConfigResult_t ConfigService_SetTilt(ConfigMode_t mode, uint16_t degrees) {
	ConfigData_t candidate = s_config;
	ConfigProfile_t *profile;

	if (s_sealed != 0U)
		return CONFIG_RESULT_LOCKED;
	if ((mode != CONFIG_MODE_CARGO) || (degrees < 1U) || (degrees > 180U)) {
		return CONFIG_RESULT_INVALID;
	}

	profile = (mode == CONFIG_MODE_CARGO) ? &candidate.cargo : &candidate.site;
	profile->tilt_threshold_deg = degrees;
	return Config_CommitCurrent(&candidate);
}

ConfigResult_t ConfigService_SetTemperature(ConfigMode_t mode, int16_t min_c,
		int16_t max_c) {
	ConfigData_t candidate = s_config;
	ConfigProfile_t *profile;

	if (s_sealed != 0U)
		return CONFIG_RESULT_LOCKED;
	if (((mode != CONFIG_MODE_CARGO) && (mode != CONFIG_MODE_SITE))
			|| (min_c < CFG_TEMP_MIN_LIMIT) || (max_c > CFG_TEMP_MAX_LIMIT)
			|| (min_c >= max_c)) {
		return CONFIG_RESULT_INVALID;
	}

	profile = (mode == CONFIG_MODE_CARGO) ? &candidate.cargo : &candidate.site;
	profile->temp_min_c = min_c;
	profile->temp_max_c = max_c;
	return Config_CommitCurrent(&candidate);
}

ConfigResult_t ConfigService_Seal(const char *password) {
	size_t password_length;
	uint8_t salt[CFG_SALT_SIZE];
	uint8_t password_hash[CFG_HASH_SIZE];

	if (s_sealed != 0U)
		return CONFIG_RESULT_LOCKED;
	if (s_save_state != SAVE_IDLE)
		return CONFIG_RESULT_BUSY;
	if (!Config_IsPasswordValid(password, &password_length)) {
		return CONFIG_RESULT_INVALID;
	}

	Config_MakeSalt(salt, s_generation + 1U);
	Crypto_Pbkdf2HmacSha256((const uint8_t*) password, password_length, salt,
			sizeof(salt),
			CFG_PBKDF2_ROUNDS, password_hash);

	{
		ConfigResult_t result = Config_BeginCommit(&s_config, 1U, salt,
				password_hash);
		memset(password_hash, 0, sizeof(password_hash));
		return result;
	}
}

ConfigResult_t ConfigService_Unseal(const char *password) {
	size_t password_length;
	uint8_t calculated_hash[CFG_HASH_SIZE];

	if (s_sealed == 0U)
		return CONFIG_RESULT_NOT_SEALED;
	if (s_save_state != SAVE_IDLE)
		return CONFIG_RESULT_BUSY;
	if (!Config_IsPasswordValid(password, &password_length)) {
		return CONFIG_RESULT_INVALID;
	}

	Crypto_Pbkdf2HmacSha256((const uint8_t*) password, password_length, s_salt,
			sizeof(s_salt),
			CFG_PBKDF2_ROUNDS, calculated_hash);

	if (!Crypto_ConstantTimeEqual(calculated_hash, s_hash,
			sizeof(calculated_hash))) {
		s_wrong_password_count++;
		memset(calculated_hash, 0, sizeof(calculated_hash));
		return CONFIG_RESULT_BAD_PASSWORD;
	}

	memset(calculated_hash, 0, sizeof(calculated_hash));
	return Config_BeginCommit(&s_config, 0U, s_salt, s_hash);
}

ConfigResult_t ConfigService_VerifyPassword(const char *password) {
	size_t password_length;
	uint8_t calculated_hash[CFG_HASH_SIZE];

	if (s_save_state != SAVE_IDLE) {
		return CONFIG_RESULT_BUSY;
	}
	if (s_sealed == 0U) {
		return CONFIG_RESULT_NOT_SEALED;
	}
	if (!Config_IsPasswordValid(password, &password_length)) {
		return CONFIG_RESULT_INVALID;
	}

	Crypto_Pbkdf2HmacSha256((const uint8_t*) password, password_length, s_salt,
			sizeof(s_salt),
			CFG_PBKDF2_ROUNDS, calculated_hash);

	if (!Crypto_ConstantTimeEqual(calculated_hash, s_hash,
			sizeof(calculated_hash))) {
		s_wrong_password_count++;
		memset(calculated_hash, 0, sizeof(calculated_hash));
		return CONFIG_RESULT_BAD_PASSWORD;
	}

	memset(calculated_hash, 0, sizeof(calculated_hash));
	return CONFIG_RESULT_OK;
}

ConfigResult_t ConfigService_FactoryReset(void) {
	ConfigData_t defaults;
	uint8_t empty_salt[CFG_SALT_SIZE] = { 0U };
	uint8_t empty_hash[CFG_HASH_SIZE] = { 0U };
	ConfigResult_t result;

	if (s_sealed == 0U) {
		return CONFIG_RESULT_NOT_SEALED;
	}
	if (s_save_state != SAVE_IDLE) {
		return CONFIG_RESULT_BUSY;
	}

	defaults.mode = CONFIG_MODE_CARGO;
	defaults.cargo.impact_threshold_mg = 2500U;
	defaults.cargo.tilt_threshold_deg = 45U;
	defaults.cargo.temp_min_c = 15;
	defaults.cargo.temp_max_c = 26;
	defaults.site.impact_threshold_mg = 100U;
	defaults.site.tilt_threshold_deg = 30U;
	defaults.site.temp_min_c = 15;
	defaults.site.temp_max_c = 26;

	/* Factory reset deliberately resets generation to zero. The old active
	 * slot is erased only after the generation-0 defaults record is committed
	 * and verified, so power loss before cleanup safely rolls back to old data. */
	result = Config_BeginCommitGeneration(&defaults, 0U, empty_salt, empty_hash,
			0U);
	if (result == CONFIG_RESULT_OK) {
		/* The new defaults are committed first; only then erase the old sealed slot. */
		s_cleanup_old_slot = 1U;
	}
	memset(empty_salt, 0, sizeof(empty_salt));
	memset(empty_hash, 0, sizeof(empty_hash));
	return result;
}

uint32_t ConfigService_GetWrongPasswordCount(void) {
	return s_wrong_password_count;
}
