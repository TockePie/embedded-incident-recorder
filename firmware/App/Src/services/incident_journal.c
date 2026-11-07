#include "services/incident_journal.h"

#include <stddef.h>
#include <string.h>

#include "drivers/w25q64.h"
#include "spi.h"
#include "services/config_service.h"
#include "services/storage_maintenance.h"
#include "services/photo_store.h"
#include "services/cli_service.h"
#include "app/app.h"
#include "bsp/diag_uart.h"
#include "stm32f4xx_hal.h"

#define JOURNAL_BASE_ADDRESS       0x002000UL /* sectors 2..127; config owns 0..1 */
#define JOURNAL_END_ADDRESS        0x080000UL /* sector 128 starts photo partition */
#define JOURNAL_SECTOR_SIZE        W25Q64_SECTOR_SIZE_BYTES
#define JOURNAL_HEADER_SIZE        64U
#define JOURNAL_RECORD_SIZE        64U
#define JOURNAL_RECORD_OFFSET      JOURNAL_HEADER_SIZE
#define JOURNAL_HEADER_MAGIC       0x4A534543UL /* JSEC */
#define JOURNAL_RECORD_MAGIC       0x4A524543UL /* JREC */
#define JOURNAL_HEADER_SCHEMA       1U
#define JOURNAL_RECORD_SCHEMA       2U
#define JOURNAL_COMMIT_MARKER       0xC01117EDUL
#define JOURNAL_INVALID_SECTOR      0xFFFFU
#define JOURNAL_FAULT_IO            0x00060001UL

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t schema;
	uint16_t size;
	uint32_t generation;
	uint32_t first_sequence;
	uint8_t reserved[8];
	uint32_t crc32;
	uint32_t commit;
} JournalSectorHeader_t;

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t schema;
	uint16_t size;
	uint32_t sequence;
	uint32_t timestamp_ms;
	uint8_t mode;
	uint8_t type;
	uint16_t reserved0;
	int32_t value;
	int32_t threshold;
	uint8_t photo_outcome;
	uint8_t photo_reason;
	uint8_t reserved1[26];
	uint32_t crc32;
	uint32_t commit;
} JournalFlashRecord_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(JournalSectorHeader_t) == 32U,
		"Journal sector header metadata must remain 32 bytes");
_Static_assert(sizeof(JournalFlashRecord_t) == JOURNAL_RECORD_SIZE,
		"Journal record layout must remain 64 bytes");
#endif

typedef enum {
	JSTATE_IDLE = 0,
	JSTATE_TARGET_SCAN,
	JSTATE_PHOTO_RELEASE_START,
	JSTATE_PHOTO_RELEASE_WAIT,
	JSTATE_ERASE_WAIT,
	JSTATE_HEADER_DATA_WAIT,
	JSTATE_HEADER_COMMIT_WAIT,
	JSTATE_HEADER_VERIFY,
	JSTATE_RECORD_DATA_WAIT,
	JSTATE_RECORD_COMMIT_WAIT,
	JSTATE_RECORD_VERIFY,
	JSTATE_PHOTO_OUTCOME_FIND,
	JSTATE_PHOTO_OUTCOME_READ,
	JSTATE_PHOTO_OUTCOME_WAIT,
	JSTATE_PHOTO_OUTCOME_VERIFY,
	JSTATE_ERROR
} JournalState_t;

static IncidentJournalStats_t s_stats;
static JournalState_t s_state;
static uint16_t s_active_sector;
static uint16_t s_target_sector;
static uint8_t s_next_slot;
static uint32_t s_sector_generation;
static uint32_t s_next_sequence;
static JournalSectorHeader_t s_header;
static JournalFlashRecord_t s_record;
static JournalFlashRecord_t s_verify_record;
static JournalSectorHeader_t s_verify_header;
static IncidentEvent_t s_pending_event;
static bool s_have_pending_event;
static bool s_target_has_header;
static uint8_t s_target_scan_slot;
static uint16_t s_target_valid_records;
static uint32_t s_target_first_sequence;
static IncidentJournalRecord_t s_last_committed_record;
static uint32_t s_commit_generation;
static bool s_photo_release_started;
static bool s_photo_outcome_pending;
static uint32_t s_photo_outcome_sequence;
static uint8_t s_photo_outcome_value;
static uint8_t s_photo_reason_value;
static uint16_t s_photo_outcome_scan_sector;
static uint16_t s_photo_outcome_sector;
static uint8_t s_photo_outcome_slot;
static uint8_t s_photo_outcome_bytes[2];
static JournalFlashRecord_t s_photo_outcome_record;

static uint32_t Journal_SectorAddress(uint16_t index) {
	return JOURNAL_BASE_ADDRESS + ((uint32_t) index * JOURNAL_SECTOR_SIZE);
}

static uint32_t Journal_RecordAddress(uint16_t sector, uint8_t slot) {
	return Journal_SectorAddress(sector) + JOURNAL_RECORD_OFFSET
			+ ((uint32_t) slot * JOURNAL_RECORD_SIZE);
}

static bool Journal_IsErased(const uint8_t *data, uint32_t length) {
	uint32_t i;
	for (i = 0U; i < length; ++i) {
		if (data[i] != 0xFFU)
			return false;
	}
	return true;
}

static uint32_t Journal_RecordCRC(const JournalFlashRecord_t *record) {
	if (record->schema == JOURNAL_RECORD_SCHEMA) {
		JournalFlashRecord_t copy = *record;
		/* These bytes are programmed after the immutable event COMMIT. */
		copy.photo_outcome = INCIDENT_PHOTO_OUTCOME_PENDING;
		copy.photo_reason = (uint8_t) INCIDENT_PHOTO_REASON_NONE;
		return W25Q64_CalculateCRC32((const uint8_t*) &copy,
				(uint32_t) offsetof(JournalFlashRecord_t, crc32));
	}
	return W25Q64_CalculateCRC32((const uint8_t*) record,
			(uint32_t) offsetof(JournalFlashRecord_t, crc32));
}

static bool Journal_HeaderValid(const JournalSectorHeader_t *header) {
	if ((header->magic != JOURNAL_HEADER_MAGIC)
			|| (header->schema != JOURNAL_HEADER_SCHEMA)
			|| (header->size != sizeof(*header))
			|| (header->first_sequence == 0U)
			|| (header->commit != JOURNAL_COMMIT_MARKER)) {
		return false;
	}
	return header->crc32
			== W25Q64_CalculateCRC32((const uint8_t*) header,
					(uint32_t) offsetof(JournalSectorHeader_t, crc32));
}

static bool Journal_RecordValid(const JournalFlashRecord_t *record) {
	if ((record->magic != JOURNAL_RECORD_MAGIC)
			|| ((record->schema != 1U)
					&& (record->schema != JOURNAL_RECORD_SCHEMA))
			|| (record->size != sizeof(*record))
			|| (record->commit != JOURNAL_COMMIT_MARKER)
			|| ((record->mode != CONFIG_MODE_CARGO)
					&& (record->mode != CONFIG_MODE_SITE))
			|| (record->type < INCIDENT_TYPE_SHOCK)
			|| (record->type > INCIDENT_TYPE_VIBRATION)) {
		return false;
	}
	return record->crc32 == Journal_RecordCRC(record);
}

static bool Journal_GenerationNewer(uint32_t a, uint32_t b) {
	return (int32_t) (a - b) > 0;
}

static bool Journal_FlashAvailable(void) {
	return (ConfigService_GetSaveStatus() != CONFIG_SAVE_IN_PROGRESS)
			&& (StorageMaintenance_GetStatus() == STORAGE_MAINTENANCE_IDLE)
			&& (CliService_IsMaintenanceLocked() == false);
}

static void Journal_Fail(W25Q64_Status_t status, uint32_t address) {
	s_state = JSTATE_ERROR;
	s_stats.ready = false;
	s_stats.io_errors++;
	DiagUART_Printf(
			"[JOURNAL] CRITICAL flash operation failed status=%u address=0x%06lX; journal disabled\r\n",
			(unsigned int) status, (unsigned long) address);
	App_EnterFault(JOURNAL_FAULT_IO);
}

static bool Journal_ReadHeader(uint16_t sector, JournalSectorHeader_t *header) {
	return W25Q64_Read(&hspi2, Journal_SectorAddress(sector), (uint8_t*) header,
			sizeof(*header)) == W25Q64_STATUS_OK;
}

static bool Journal_ReadRecord(uint16_t sector, uint8_t slot,
		JournalFlashRecord_t *record) {
	return W25Q64_Read(&hspi2, Journal_RecordAddress(sector, slot),
			(uint8_t*) record, sizeof(*record)) == W25Q64_STATUS_OK;
}

IncidentJournalStatus_t IncidentJournal_Init(void) {
	uint16_t sector;
	uint16_t newest = JOURNAL_INVALID_SECTOR;
	uint32_t newest_generation = 0U;
	JournalSectorHeader_t header;
	uint8_t slot;
	bool found_erased = false;

	memset(&s_stats, 0, sizeof(s_stats));
	s_state = JSTATE_IDLE;
	s_active_sector = JOURNAL_INVALID_SECTOR;
	s_stats.active_sector_index = JOURNAL_INVALID_SECTOR;
	s_target_sector = 0U;
	s_next_slot = 0U;
	s_sector_generation = 0U;
	s_next_sequence = 1U;
	s_have_pending_event = false;
	s_target_has_header = false;
	s_target_scan_slot = 0U;
	s_target_valid_records = 0U;
	s_target_first_sequence = 0U;
	s_commit_generation = 0U;
	memset(&s_last_committed_record, 0, sizeof(s_last_committed_record));
	s_photo_release_started = false;
	s_photo_outcome_pending = false;
	s_photo_outcome_sequence = 0U;
	s_photo_outcome_value = INCIDENT_PHOTO_OUTCOME_PENDING;
	s_photo_reason_value = (uint8_t) INCIDENT_PHOTO_REASON_NONE;
	s_photo_outcome_scan_sector = 0U;
	s_photo_outcome_sector = JOURNAL_INVALID_SECTOR;
	s_photo_outcome_slot = 0U;

	if ((JOURNAL_BASE_ADDRESS
			+ ((uint32_t) INCIDENT_JOURNAL_SECTOR_COUNT * JOURNAL_SECTOR_SIZE))
			!=
			JOURNAL_END_ADDRESS) {
		return INCIDENT_JOURNAL_CORRUPT;
	}

	/* Only 126 small sector headers are read during boot. The record tail of
	 the newest sector is then scanned to recover the exact append position. */
	for (sector = 0U; sector < INCIDENT_JOURNAL_SECTOR_COUNT; ++sector) {
		if (!Journal_ReadHeader(sector, &header)) {
			s_stats.io_errors++;
			return INCIDENT_JOURNAL_IO_ERROR;
		}
		if (Journal_HeaderValid(&header)) {
			s_stats.valid_sector_count++;
			if ((newest == JOURNAL_INVALID_SECTOR)
					|| Journal_GenerationNewer(header.generation,
							newest_generation)) {
				newest = sector;
				newest_generation = header.generation;
			}
		}
	}

	if (newest == JOURNAL_INVALID_SECTOR) {
		s_active_sector = JOURNAL_INVALID_SECTOR;
		s_next_slot = INCIDENT_JOURNAL_RECORDS_PER_SECTOR;
		s_stats.ready = true;
		s_stats.available_records = (uint32_t) INCIDENT_JOURNAL_SECTOR_COUNT *
		INCIDENT_JOURNAL_RECORDS_PER_SECTOR;
		DiagUART_Printf(
				"[JOURNAL] empty journal; first sector will be initialized on demand\r\n");
		return INCIDENT_JOURNAL_OK;
	}

	s_active_sector = newest;
	s_sector_generation = newest_generation;
	if (!Journal_ReadHeader(newest, &s_header))
		return INCIDENT_JOURNAL_IO_ERROR;
	s_next_sequence = s_header.first_sequence;

	for (slot = 0U; slot < INCIDENT_JOURNAL_RECORDS_PER_SECTOR; ++slot) {
		if (!Journal_ReadRecord(newest, slot, &s_verify_record))
			return INCIDENT_JOURNAL_IO_ERROR;
		if (Journal_IsErased((const uint8_t*) &s_verify_record,
				sizeof(s_verify_record))) {
			s_next_slot = slot;
			found_erased = true;
			break;
		}
		if (!Journal_RecordValid(&s_verify_record)) {
			/* A torn/corrupt slot is never re-used: flash can only program 1->0.
			 The remainder of this sector is abandoned and the next sector rotated. */
			s_stats.crc_rejected_records++;
			s_next_slot = INCIDENT_JOURNAL_RECORDS_PER_SECTOR;
			break;
		}
		if ((s_verify_record.sequence >= s_next_sequence)
				|| (s_next_sequence == 0U)) {
			s_next_sequence =
					(s_verify_record.sequence == UINT32_MAX) ?
							0U : (s_verify_record.sequence + 1U);
		}
		s_stats.active_sector_records++;
	}
	if (!found_erased && (s_next_slot < INCIDENT_JOURNAL_RECORDS_PER_SECTOR))
		s_next_slot = INCIDENT_JOURNAL_RECORDS_PER_SECTOR;

	s_stats.active_sector_index = s_active_sector;
	s_stats.sector_generation = s_sector_generation;
	s_stats.next_sequence = s_next_sequence;
	s_stats.ready = true;
	s_stats.full_cycle_started = (s_stats.valid_sector_count
			>= INCIDENT_JOURNAL_SECTOR_COUNT);
	s_stats.available_records = ((uint32_t) (INCIDENT_JOURNAL_SECTOR_COUNT
			- s_stats.valid_sector_count) *
	INCIDENT_JOURNAL_RECORDS_PER_SECTOR)
			+ (uint32_t) (INCIDENT_JOURNAL_RECORDS_PER_SECTOR
					- ((s_next_slot < INCIDENT_JOURNAL_RECORDS_PER_SECTOR) ?
							s_next_slot : INCIDENT_JOURNAL_RECORDS_PER_SECTOR));
	DiagUART_Printf(
			"[JOURNAL] recovery PASS sector=%u generation=%lu active_records=%u sectors=%u next_sequence=%lu\r\n",
			(unsigned int) s_active_sector, (unsigned long) s_sector_generation,
			(unsigned int) s_stats.active_sector_records,
			(unsigned int) s_stats.valid_sector_count,
			(unsigned long) s_next_sequence);
	return INCIDENT_JOURNAL_OK;
}

static void Journal_PrepareHeader(void) {
	memset(&s_header, 0xFF, sizeof(s_header));
	s_header.magic = JOURNAL_HEADER_MAGIC;
	s_header.schema = JOURNAL_HEADER_SCHEMA;
	s_header.size = sizeof(s_header);
	s_header.generation = s_sector_generation + 1U;
	s_header.first_sequence = s_next_sequence;
	memset(s_header.reserved, 0, sizeof(s_header.reserved));
	s_header.crc32 = W25Q64_CalculateCRC32((const uint8_t*) &s_header,
			(uint32_t) offsetof(JournalSectorHeader_t, crc32));
	s_header.commit = JOURNAL_COMMIT_MARKER;
}

static void Journal_PrepareRecord(void) {
	memset(&s_record, 0xFF, sizeof(s_record));
	s_record.magic = JOURNAL_RECORD_MAGIC;
	s_record.schema = JOURNAL_RECORD_SCHEMA;
	s_record.size = sizeof(s_record);
	s_record.sequence = s_next_sequence;
	s_record.timestamp_ms = s_pending_event.timestamp_ms;
	s_record.mode = (uint8_t) s_pending_event.mode;
	s_record.type = (uint8_t) s_pending_event.type;
	s_record.reserved0 = 0U;
	s_record.value = s_pending_event.value;
	s_record.threshold = s_pending_event.threshold;
	memset(s_record.reserved1, 0, sizeof(s_record.reserved1));
	s_record.photo_outcome = INCIDENT_PHOTO_OUTCOME_PENDING;
	s_record.photo_reason = (uint8_t) INCIDENT_PHOTO_REASON_NONE;
	s_record.crc32 = Journal_RecordCRC(&s_record);
	s_record.commit = JOURNAL_COMMIT_MARKER;
}

void IncidentJournal_Process(void) {
	W25Q64_Status_t status;
	bool busy = false;
	uint32_t address;

	if ((!s_stats.ready) || (s_state == JSTATE_ERROR))
		return;

	if (s_state == JSTATE_IDLE) {
		if (!Journal_FlashAvailable())
			return;
		if (s_photo_outcome_pending) {
			s_photo_outcome_scan_sector = 0U;
			s_photo_outcome_sector = JOURNAL_INVALID_SECTOR;
			s_state = JSTATE_PHOTO_OUTCOME_FIND;
			return;
		}
		if (s_have_pending_event == false) {
			if (!IncidentDetector_Peek(0U, &s_pending_event))
				return;
			s_have_pending_event = true;
		}
		/* Sequence zero is reserved as the exhausted sentinel. Never wrap and
		 reuse an event ID: photos will use this value as their owner ID. */
		if (s_next_sequence == 0U) {
			s_stats.ready = false;
			DiagUART_Printf(
					"[JOURNAL] CRITICAL: persistent event ID space exhausted; refusing ID reuse\r\n");
			App_EnterFault(0x00060002UL);
			return;
		}
		if ((s_active_sector == JOURNAL_INVALID_SECTOR)
				|| (s_next_slot >= INCIDENT_JOURNAL_RECORDS_PER_SECTOR)) {
			JournalSectorHeader_t target_header;
			s_target_sector =
					(s_active_sector == JOURNAL_INVALID_SECTOR) ?
							0U :
							(uint16_t) ((s_active_sector + 1U)
									% INCIDENT_JOURNAL_SECTOR_COUNT);
			status = W25Q64_Read(&hspi2, Journal_SectorAddress(s_target_sector),
					(uint8_t*) &target_header, sizeof(target_header));
			if (status == W25Q64_STATUS_BUSY)
				return;
			if (status != W25Q64_STATUS_OK) {
				Journal_Fail(status, Journal_SectorAddress(s_target_sector));
				return;
			}
			s_target_has_header = Journal_HeaderValid(&target_header);
			s_target_first_sequence =
					s_target_has_header ? target_header.first_sequence : 0U;
			s_target_scan_slot = 0U;
			s_target_valid_records = 0U;
			if (s_target_has_header)
				s_state = JSTATE_TARGET_SCAN;
			else {
				status = W25Q64_StartSectorErase(&hspi2,
						Journal_SectorAddress(s_target_sector));
				if (status == W25Q64_STATUS_OK)
					s_state = JSTATE_ERASE_WAIT;
				else if (status != W25Q64_STATUS_BUSY)
					Journal_Fail(status,
							Journal_SectorAddress(s_target_sector));
			}
			return;
		}
		Journal_PrepareRecord();
		address = Journal_RecordAddress(s_active_sector, s_next_slot);
		status = W25Q64_StartPageProgram(&hspi2, address,
				(const uint8_t*) &s_record,
				(uint16_t) offsetof(JournalFlashRecord_t, commit));
		if (status == W25Q64_STATUS_OK)
			s_state = JSTATE_RECORD_DATA_WAIT;
		else if (status != W25Q64_STATUS_BUSY)
			Journal_Fail(status, address);
		return;
	}

	status = W25Q64_IsBusy(&hspi2, &busy);
	if (status != W25Q64_STATUS_OK) {
		Journal_Fail(status,
				(s_active_sector == JOURNAL_INVALID_SECTOR) ?
						Journal_SectorAddress(s_target_sector) :
						Journal_RecordAddress(s_active_sector, s_next_slot));
		return;
	}
	if (busy)
		return;

	switch (s_state) {
	case JSTATE_TARGET_SCAN: {
		JournalFlashRecord_t old_record;
		if (!Journal_ReadRecord(s_target_sector, s_target_scan_slot,
				&old_record)) {
			Journal_Fail(W25Q64_STATUS_IO_ERROR,
					Journal_RecordAddress(s_target_sector, s_target_scan_slot));
			break;
		}
		if (Journal_RecordValid(&old_record))
			s_target_valid_records++;
		s_target_scan_slot++;
		if (s_target_scan_slot >= INCIDENT_JOURNAL_RECORDS_PER_SECTOR) {
			/* Photos are children of persistent journal IDs. Reclaim their extents
			 * before erasing the parent journal sector. PhotoStore performs the
			 * tombstone + reverse-order erase cooperatively. */
			if (s_target_has_header) {
				s_photo_release_started = false;
				s_state = JSTATE_PHOTO_RELEASE_START;
			} else {
				status = W25Q64_StartSectorErase(&hspi2,
						Journal_SectorAddress(s_target_sector));
				if (status == W25Q64_STATUS_OK) {
					s_stats.dropped_records += s_target_valid_records;
					s_stats.overwritten_sectors++;
					s_stats.full_cycle_started = true;
					s_state = JSTATE_ERASE_WAIT;
				} else if (status != W25Q64_STATUS_BUSY) {
					Journal_Fail(status,
							Journal_SectorAddress(s_target_sector));
				}
			}
		}
		break;
	}

	case JSTATE_PHOTO_RELEASE_START:
		if (!s_photo_release_started) {
			uint32_t last_sequence = s_target_first_sequence
					+ INCIDENT_JOURNAL_RECORDS_PER_SECTOR - 1U;
			PhotoStore_Status_t photo_status;
			if (last_sequence < s_target_first_sequence)
				last_sequence = UINT32_MAX;
			photo_status = PhotoStore_RequestReleaseRange(
					s_target_first_sequence, last_sequence);
			if (photo_status == PHOTO_STORE_STATUS_OK) {
				s_photo_release_started = true;
			} else if (photo_status != PHOTO_STORE_STATUS_BUSY
					&& photo_status != PHOTO_STORE_STATUS_NOT_READY) {
				DiagUART_Printf(
						"[JOURNAL] waiting for photo reclamation service (status=%u)\r\n",
						(unsigned int) photo_status);
			}
		}
		if (s_photo_release_started)
			s_state = JSTATE_PHOTO_RELEASE_WAIT;
		break;

	case JSTATE_PHOTO_RELEASE_WAIT:
		if (PhotoStore_IsReleaseComplete()) {
			status = W25Q64_StartSectorErase(&hspi2,
					Journal_SectorAddress(s_target_sector));
			if (status == W25Q64_STATUS_OK) {
				s_stats.dropped_records += s_target_valid_records;
				s_stats.overwritten_sectors++;
				s_stats.full_cycle_started = true;
				s_photo_release_started = false;
				s_state = JSTATE_ERASE_WAIT;
			} else if (status != W25Q64_STATUS_BUSY) {
				Journal_Fail(status, Journal_SectorAddress(s_target_sector));
			}
		}
		break;

	case JSTATE_ERASE_WAIT:
		Journal_PrepareHeader();
		address = Journal_SectorAddress(s_target_sector);
		status = W25Q64_StartPageProgram(&hspi2, address,
				(const uint8_t*) &s_header,
				(uint16_t) offsetof(JournalSectorHeader_t, commit));
		if (status == W25Q64_STATUS_OK)
			s_state = JSTATE_HEADER_DATA_WAIT;
		else if (status != W25Q64_STATUS_BUSY)
			Journal_Fail(status, address);
		break;

	case JSTATE_HEADER_DATA_WAIT:
		address = Journal_SectorAddress(s_target_sector) +
		(uint32_t)offsetof(JournalSectorHeader_t, commit);
		status = W25Q64_StartPageProgram(&hspi2, address,
				(const uint8_t*) &s_header.commit, sizeof(s_header.commit));
		if (status == W25Q64_STATUS_OK)
			s_state = JSTATE_HEADER_COMMIT_WAIT;
		else if (status != W25Q64_STATUS_BUSY)
			Journal_Fail(status, address);
		break;

	case JSTATE_HEADER_COMMIT_WAIT:
		address = Journal_SectorAddress(s_target_sector);
		status = W25Q64_Read(&hspi2, address, (uint8_t*) &s_verify_header,
				sizeof(s_verify_header));
		if (status != W25Q64_STATUS_OK) {
			Journal_Fail(status, address);
			break;
		}
		if (!Journal_HeaderValid(&s_verify_header)) {
			Journal_Fail(W25Q64_STATUS_VERIFY_FAILED, address);
			break;
		}
		s_active_sector = s_target_sector;
		if (!s_target_has_header
				&& (s_stats.valid_sector_count < INCIDENT_JOURNAL_SECTOR_COUNT))
			s_stats.valid_sector_count++;
		s_target_has_header = true;
		if (s_stats.valid_sector_count >= INCIDENT_JOURNAL_SECTOR_COUNT)
			s_stats.full_cycle_started = true;
		s_sector_generation = s_header.generation;
		s_next_slot = 0U;
		s_stats.active_sector_index = s_active_sector;
		s_stats.active_sector_records = 0U;
		s_stats.sector_generation = s_sector_generation;
		s_stats.available_records = ((uint32_t) (INCIDENT_JOURNAL_SECTOR_COUNT
				- s_stats.valid_sector_count) *
		INCIDENT_JOURNAL_RECORDS_PER_SECTOR)
				+ INCIDENT_JOURNAL_RECORDS_PER_SECTOR;
		s_state = JSTATE_IDLE;
		break;

	case JSTATE_RECORD_DATA_WAIT:
		address = Journal_RecordAddress(s_active_sector, s_next_slot) +
		(uint32_t)offsetof(JournalFlashRecord_t, commit);
		status = W25Q64_StartPageProgram(&hspi2, address,
				(const uint8_t*) &s_record.commit, sizeof(s_record.commit));
		if (status == W25Q64_STATUS_OK)
			s_state = JSTATE_RECORD_COMMIT_WAIT;
		else if (status != W25Q64_STATUS_BUSY)
			Journal_Fail(status, address);
		break;

	case JSTATE_RECORD_COMMIT_WAIT:
		address = Journal_RecordAddress(s_active_sector, s_next_slot);
		status = W25Q64_Read(&hspi2, address, (uint8_t*) &s_verify_record,
				sizeof(s_verify_record));
		if (status != W25Q64_STATUS_OK) {
			Journal_Fail(status, address);
			break;
		}
		if (!Journal_RecordValid(&s_verify_record)) {
			Journal_Fail(W25Q64_STATUS_VERIFY_FAILED, address);
			break;
		}
		/* Remove the RAM event only after the persistent commit marker and CRC
		 have both been read back successfully. */
		{
			IncidentEvent_t popped;
			if (IncidentDetector_Pop(&popped) == false) {
				Journal_Fail(W25Q64_STATUS_VERIFY_FAILED, address);
				break;
			}
			if (popped.sequence != s_pending_event.sequence) {
				Journal_Fail(W25Q64_STATUS_VERIFY_FAILED, address);
				break;
			}
		}
		s_last_committed_record.sequence = s_verify_record.sequence;
		s_last_committed_record.timestamp_ms = s_verify_record.timestamp_ms;
		s_last_committed_record.mode = s_verify_record.mode;
		s_last_committed_record.type = s_verify_record.type;
		s_last_committed_record.value = s_verify_record.value;
		s_last_committed_record.threshold = s_verify_record.threshold;
		s_last_committed_record.photo_outcome = s_verify_record.photo_outcome;
		s_last_committed_record.photo_reason = s_verify_record.photo_reason;
		++s_commit_generation;
		s_have_pending_event = false;
		s_next_sequence++;
		s_next_slot++;
		s_stats.active_sector_records++;
		s_stats.committed_records_this_boot++;
		s_stats.next_sequence = s_next_sequence;
		if (s_stats.available_records > 0U)
			s_stats.available_records--;
		s_state = JSTATE_IDLE;
		break;

	case JSTATE_PHOTO_OUTCOME_FIND: {
		JournalSectorHeader_t candidate;
		W25Q64_Status_t read_status = W25Q64_Read(&hspi2,
				Journal_SectorAddress(s_photo_outcome_scan_sector),
				(uint8_t*) &candidate, sizeof(candidate));
		if (read_status == W25Q64_STATUS_BUSY)
			return;
		if (read_status != W25Q64_STATUS_OK) {
			Journal_Fail(read_status,
					Journal_SectorAddress(s_photo_outcome_scan_sector));
			break;
		}
		if (Journal_HeaderValid(&candidate)
				&& (s_photo_outcome_sequence >= candidate.first_sequence)
				&& ((s_photo_outcome_sequence - candidate.first_sequence)
						< INCIDENT_JOURNAL_RECORDS_PER_SECTOR)) {
			s_photo_outcome_sector = s_photo_outcome_scan_sector;
			s_photo_outcome_slot = (uint8_t) (s_photo_outcome_sequence
					- candidate.first_sequence);
			s_state = JSTATE_PHOTO_OUTCOME_READ;
		} else {
			++s_photo_outcome_scan_sector;
			if (s_photo_outcome_scan_sector >= INCIDENT_JOURNAL_SECTOR_COUNT) {
				s_photo_outcome_pending = false;
				s_state = JSTATE_IDLE;
			}
		}
		break;
	}

	case JSTATE_PHOTO_OUTCOME_READ: {
		W25Q64_Status_t read_status = W25Q64_Read(&hspi2,
				Journal_RecordAddress(s_photo_outcome_sector,
						s_photo_outcome_slot),
				(uint8_t*) &s_photo_outcome_record,
				sizeof(s_photo_outcome_record));
		if (read_status == W25Q64_STATUS_BUSY)
			return;
		if (read_status != W25Q64_STATUS_OK) {
			Journal_Fail(read_status,
					Journal_RecordAddress(s_photo_outcome_sector,
							s_photo_outcome_slot));
			break;
		}
		if (!Journal_RecordValid(&s_photo_outcome_record)
				|| (s_photo_outcome_record.sequence != s_photo_outcome_sequence)) {
			Journal_Fail(W25Q64_STATUS_VERIFY_FAILED,
					Journal_RecordAddress(s_photo_outcome_sector,
							s_photo_outcome_slot));
			break;
		}
		/* Schema-1 reserved bytes participate in their legacy CRC and must never
		 * be modified in place. Only new schema-2 records have mutable outcome. */
		if (s_photo_outcome_record.schema != JOURNAL_RECORD_SCHEMA) {
			s_photo_outcome_pending = false;
			s_state = JSTATE_IDLE;
			break;
		}
		if (s_photo_outcome_record.photo_outcome
				!= INCIDENT_PHOTO_OUTCOME_PENDING) {
			s_photo_outcome_pending = false;
			s_state = JSTATE_IDLE;
			break;
		}
		s_photo_outcome_bytes[0] = s_photo_outcome_value;
		s_photo_outcome_bytes[1] = s_photo_reason_value;
		status =
				W25Q64_StartPageProgram(&hspi2,
						Journal_RecordAddress(s_photo_outcome_sector,
								s_photo_outcome_slot) + (uint32_t) offsetof(JournalFlashRecord_t, photo_outcome),
						s_photo_outcome_bytes, sizeof(s_photo_outcome_bytes));
		if (status == W25Q64_STATUS_OK)
			s_state = JSTATE_PHOTO_OUTCOME_WAIT;
		else if (status != W25Q64_STATUS_BUSY)
			Journal_Fail(status,
					Journal_RecordAddress(s_photo_outcome_sector,
							s_photo_outcome_slot));
		break;
	}

	case JSTATE_PHOTO_OUTCOME_WAIT:
		s_state = JSTATE_PHOTO_OUTCOME_VERIFY;
		break;

	case JSTATE_PHOTO_OUTCOME_VERIFY: {
		W25Q64_Status_t read_status = W25Q64_Read(&hspi2,
				Journal_RecordAddress(s_photo_outcome_sector,
						s_photo_outcome_slot),
				(uint8_t*) &s_photo_outcome_record,
				sizeof(s_photo_outcome_record));
		if (read_status == W25Q64_STATUS_BUSY)
			return;
		if ((read_status != W25Q64_STATUS_OK)
				|| !Journal_RecordValid(&s_photo_outcome_record)
				|| (s_photo_outcome_record.sequence != s_photo_outcome_sequence)
				|| (s_photo_outcome_record.photo_outcome
						!= s_photo_outcome_value)
				|| (s_photo_outcome_record.photo_reason != s_photo_reason_value)) {
			Journal_Fail(
					(read_status == W25Q64_STATUS_OK) ?
							W25Q64_STATUS_VERIFY_FAILED : read_status,
					Journal_RecordAddress(s_photo_outcome_sector,
							s_photo_outcome_slot));
			break;
		}
		s_photo_outcome_pending = false;
		s_photo_outcome_sequence = 0U;
		s_state = JSTATE_IDLE;
		break;
	}

	default:
		break;
	}
}

void IncidentJournal_GetStats(IncidentJournalStats_t *stats) {
	if (stats != NULL) {
		IncidentDetectorStats_t detector_stats;
		*stats = s_stats;
		IncidentDetector_GetStats(&detector_stats);
		stats->write_pending = s_have_pending_event || (s_state != JSTATE_IDLE)
				|| s_photo_outcome_pending
				|| (detector_stats.queue_count != 0U);
	}
}

IncidentJournalStatus_t IncidentJournal_ReadRecent(
		IncidentJournalRecord_t *records, uint8_t count, uint8_t *read_count) {
	uint16_t offset;
	uint8_t found = 0U;
	if ((records == NULL) || (read_count == NULL) || (count == 0U)
			|| (count > INCIDENT_JOURNAL_RECENT_MAX))
		return INCIDENT_JOURNAL_CORRUPT;
	*read_count = 0U;
	if (!s_stats.ready)
		return INCIDENT_JOURNAL_NOT_READY;
	if (s_active_sector == JOURNAL_INVALID_SECTOR)
		return INCIDENT_JOURNAL_OK;

	for (offset = 0U; offset < INCIDENT_JOURNAL_SECTOR_COUNT && found < count;
			++offset) {
		uint16_t sector = (uint16_t) ((s_active_sector
				+ INCIDENT_JOURNAL_SECTOR_COUNT - offset) %
		INCIDENT_JOURNAL_SECTOR_COUNT);
		JournalSectorHeader_t header;
		int slot;
		{
			W25Q64_Status_t read_status = W25Q64_Read(&hspi2,
					Journal_SectorAddress(sector), (uint8_t*) &header,
					sizeof(header));
			if (read_status == W25Q64_STATUS_BUSY)
				return INCIDENT_JOURNAL_BUSY;
			if (read_status != W25Q64_STATUS_OK)
				return INCIDENT_JOURNAL_IO_ERROR;
		}
		if (!Journal_HeaderValid(&header))
			continue;
		for (slot = (int) INCIDENT_JOURNAL_RECORDS_PER_SECTOR - 1;
				slot >= 0 && found < count; --slot) {
			JournalFlashRecord_t record;
			{
				W25Q64_Status_t read_status = W25Q64_Read(&hspi2,
						Journal_RecordAddress(sector, (uint8_t) slot),
						(uint8_t*) &record, sizeof(record));
				if (read_status == W25Q64_STATUS_BUSY)
					return INCIDENT_JOURNAL_BUSY;
				if (read_status != W25Q64_STATUS_OK)
					return INCIDENT_JOURNAL_IO_ERROR;
			}
			if (Journal_RecordValid(&record)) {
				records[found].sequence = record.sequence;
				records[found].timestamp_ms = record.timestamp_ms;
				records[found].mode = record.mode;
				records[found].type = record.type;
				records[found].value = record.value;
				records[found].threshold = record.threshold;
				records[found].photo_outcome =
						(record.schema >= JOURNAL_RECORD_SCHEMA) ?
								record.photo_outcome :
								INCIDENT_PHOTO_OUTCOME_LEGACY;
				records[found].photo_reason =
						(record.schema >= JOURNAL_RECORD_SCHEMA) ?
								record.photo_reason :
								(uint8_t) INCIDENT_PHOTO_REASON_NONE;
				++found;
			}
		}
	}
	*read_count = found;
	return INCIDENT_JOURNAL_OK;
}

IncidentJournalStatus_t IncidentJournal_BeginReadAll(
		IncidentJournalReadAllCursor_t *cursor) {
	if (cursor == NULL)
		return INCIDENT_JOURNAL_CORRUPT;
	if (!s_stats.ready)
		return INCIDENT_JOURNAL_NOT_READY;
	{
		PhotoStore_State_t photo_state = PhotoStore_GetState();
		if (IncidentJournal_IsBusy() || !Journal_FlashAvailable()
				|| (photo_state == PHOTO_STORE_STATE_RECOVERING)
				|| (photo_state == PHOTO_STORE_STATE_WRITING)
				|| (photo_state == PHOTO_STORE_STATE_RELEASING))
			return INCIDENT_JOURNAL_BUSY;
	}

	memset(cursor, 0, sizeof(*cursor));
	cursor->sector = 0U;
	cursor->active_sector_snapshot = s_active_sector;
	cursor->active_slot_limit = (s_next_slot < INCIDENT_JOURNAL_RECORDS_PER_SECTOR) ?
			s_next_slot : INCIDENT_JOURNAL_RECORDS_PER_SECTOR;
	cursor->active = 1U;
	return INCIDENT_JOURNAL_OK;
}

IncidentJournalStatus_t IncidentJournal_RestartReadAll(
		IncidentJournalReadAllCursor_t *cursor) {
	if ((cursor == NULL) || (cursor->active == 0U))
		return INCIDENT_JOURNAL_CORRUPT;
	cursor->sector = 0U;
	cursor->sectors_scanned = 0U;
	cursor->slot = 0U;
	cursor->sector_slot_limit = 0U;
	cursor->header_loaded = 0U;
	cursor->records_returned = 0U;
	return INCIDENT_JOURNAL_OK;
}

IncidentJournalStatus_t IncidentJournal_ReadAllNext(
		IncidentJournalReadAllCursor_t *cursor, IncidentJournalRecord_t *record_out) {
	JournalSectorHeader_t header;
	JournalFlashRecord_t record;
	W25Q64_Status_t read_status;

	if ((cursor == NULL) || (record_out == NULL) || (cursor->active == 0U))
		return INCIDENT_JOURNAL_CORRUPT;
	if (!s_stats.ready)
		return INCIDENT_JOURNAL_NOT_READY;
	if (cursor->sectors_scanned >= INCIDENT_JOURNAL_SECTOR_COUNT)
		return INCIDENT_JOURNAL_NOT_FOUND;

	if (cursor->header_loaded == 0U) {
		cursor->sector = cursor->sectors_scanned;
		read_status = W25Q64_Read(&hspi2, Journal_SectorAddress(cursor->sector),
				(uint8_t *) &header, sizeof(header));
		if (read_status == W25Q64_STATUS_BUSY)
			return INCIDENT_JOURNAL_BUSY;
		if (read_status != W25Q64_STATUS_OK)
			return INCIDENT_JOURNAL_IO_ERROR;
		if (!Journal_HeaderValid(&header)) {
			cursor->sectors_scanned++;
			return INCIDENT_JOURNAL_BUSY;
		}
		cursor->sector_slot_limit = (cursor->sector == cursor->active_sector_snapshot) ?
				cursor->active_slot_limit : INCIDENT_JOURNAL_RECORDS_PER_SECTOR;
		cursor->sector_first_sequence = header.first_sequence;
		cursor->slot = 0U;
		cursor->header_loaded = 1U;
		if (cursor->sector_slot_limit == 0U) {
			cursor->header_loaded = 0U;
			cursor->sectors_scanned++;
		}
		return INCIDENT_JOURNAL_BUSY;
	}

	if (cursor->slot >= cursor->sector_slot_limit) {
		cursor->header_loaded = 0U;
		cursor->sectors_scanned++;
		return INCIDENT_JOURNAL_BUSY;
	}

	read_status = W25Q64_Read(&hspi2,
			Journal_RecordAddress(cursor->sector, cursor->slot),
			(uint8_t *) &record, sizeof(record));
	if (read_status == W25Q64_STATUS_BUSY)
		return INCIDENT_JOURNAL_BUSY;
	if (read_status != W25Q64_STATUS_OK)
		return INCIDENT_JOURNAL_IO_ERROR;
	cursor->slot++;
	if (!Journal_RecordValid(&record)
			|| (record.sequence != (cursor->sector_first_sequence
					+ (uint32_t) (cursor->slot - 1U))))
		return INCIDENT_JOURNAL_BUSY;

	record_out->sequence = record.sequence;
	record_out->timestamp_ms = record.timestamp_ms;
	record_out->mode = record.mode;
	record_out->type = record.type;
	record_out->value = record.value;
	record_out->threshold = record.threshold;
	record_out->photo_outcome = (record.schema >= JOURNAL_RECORD_SCHEMA) ?
			record.photo_outcome : INCIDENT_PHOTO_OUTCOME_LEGACY;
	record_out->photo_reason = (record.schema >= JOURNAL_RECORD_SCHEMA) ?
			record.photo_reason : (uint8_t) INCIDENT_PHOTO_REASON_NONE;
	cursor->records_returned++;
	return INCIDENT_JOURNAL_OK;
}

IncidentJournalStatus_t IncidentJournal_ReadBySequence(uint32_t sequence,
		IncidentJournalRecord_t *record_out) {
	uint16_t sector;

	if ((sequence == 0U) || (record_out == NULL))
		return INCIDENT_JOURNAL_CORRUPT;
	if (!s_stats.ready)
		return INCIDENT_JOURNAL_NOT_READY;

	for (sector = 0U; sector < INCIDENT_JOURNAL_SECTOR_COUNT; ++sector) {
		JournalSectorHeader_t header;
		uint32_t slot_offset;
		JournalFlashRecord_t record;
		W25Q64_Status_t read_status;

		read_status = W25Q64_Read(&hspi2, Journal_SectorAddress(sector),
				(uint8_t*) &header, sizeof(header));
		if (read_status == W25Q64_STATUS_BUSY)
			return INCIDENT_JOURNAL_BUSY;
		if (read_status != W25Q64_STATUS_OK)
			return INCIDENT_JOURNAL_IO_ERROR;
		if (!Journal_HeaderValid(&header))
			continue;

		/* Each sector contains at most 63 monotonically assigned IDs. This
		 range check avoids scanning all 7,938 record slots for a lookup. */
		if ((sequence < header.first_sequence)
				|| ((sequence - header.first_sequence)
						>= INCIDENT_JOURNAL_RECORDS_PER_SECTOR))
			continue;

		slot_offset = sequence - header.first_sequence;
		read_status = W25Q64_Read(&hspi2,
				Journal_RecordAddress(sector, (uint8_t) slot_offset),
				(uint8_t*) &record, sizeof(record));
		if (read_status == W25Q64_STATUS_BUSY)
			return INCIDENT_JOURNAL_BUSY;
		if (read_status != W25Q64_STATUS_OK)
			return INCIDENT_JOURNAL_IO_ERROR;
		if (!Journal_RecordValid(&record) || (record.sequence != sequence))
			continue;

		record_out->sequence = record.sequence;
		record_out->timestamp_ms = record.timestamp_ms;
		record_out->mode = record.mode;
		record_out->type = record.type;
		record_out->value = record.value;
		record_out->threshold = record.threshold;
		record_out->photo_outcome =
				(record.schema >= JOURNAL_RECORD_SCHEMA) ?
						record.photo_outcome : INCIDENT_PHOTO_OUTCOME_LEGACY;
		record_out->photo_reason =
				(record.schema >= JOURNAL_RECORD_SCHEMA) ?
						record.photo_reason :
						(uint8_t) INCIDENT_PHOTO_REASON_NONE;
		return INCIDENT_JOURNAL_OK;
	}
	return INCIDENT_JOURNAL_NOT_FOUND;
}

bool IncidentJournal_IsBusy(void) {
	IncidentDetectorStats_t detector_stats;
	IncidentDetector_GetStats(&detector_stats);
	return (s_state != JSTATE_IDLE) || s_have_pending_event
			|| s_photo_outcome_pending || (detector_stats.queue_count != 0U);
}

bool IncidentJournal_GetLastCommittedRecord(IncidentJournalRecord_t *record,
		uint32_t *generation) {
	if ((record == NULL) || (generation == NULL) || (s_commit_generation == 0U))
		return false;
	*record = s_last_committed_record;
	*generation = s_commit_generation;
	return true;
}

IncidentJournalStatus_t IncidentJournal_RequestPhotoOutcome(uint32_t sequence,
		uint8_t outcome, uint8_t reason) {
	if ((sequence == 0U)
			|| ((outcome != INCIDENT_PHOTO_OUTCOME_STORED)
					&& (outcome != INCIDENT_PHOTO_OUTCOME_NO_PHOTO))
			|| ((outcome == INCIDENT_PHOTO_OUTCOME_STORED)
					&& (reason != (uint8_t) INCIDENT_PHOTO_REASON_NONE))
			|| ((outcome == INCIDENT_PHOTO_OUTCOME_NO_PHOTO)
					&& (reason == (uint8_t) INCIDENT_PHOTO_REASON_NONE))) {
		return INCIDENT_JOURNAL_CORRUPT;
	}
	if (!s_stats.ready)
		return INCIDENT_JOURNAL_NOT_READY;
	if (s_photo_outcome_pending
			|| ((s_state >= JSTATE_PHOTO_OUTCOME_FIND)
					&& (s_state <= JSTATE_PHOTO_OUTCOME_VERIFY))) {
		return INCIDENT_JOURNAL_BUSY;
	}
	s_photo_outcome_sequence = sequence;
	s_photo_outcome_value = outcome;
	s_photo_reason_value = reason;
	s_photo_outcome_pending = true;
	return INCIDENT_JOURNAL_OK;
}

bool IncidentJournal_IsPhotoOutcomePending(uint32_t sequence) {
	return s_photo_outcome_pending && (s_photo_outcome_sequence == sequence);
}

bool IncidentJournal_RunSelfTest(void) {
	JournalSectorHeader_t header;
	JournalFlashRecord_t record;
	JournalFlashRecord_t legacy_record;
	uint32_t expected_crc;
	static const uint8_t crc_vector[] = "123456789";
	if (W25Q64_CalculateCRC32(crc_vector, 9U) != 0xCBF43926UL)
		return false;
	if (((JOURNAL_RECORD_OFFSET % JOURNAL_RECORD_SIZE) != 0U)
			|| ((W25Q64_PAGE_SIZE_BYTES % JOURNAL_RECORD_SIZE) != 0U)
			|| ((JOURNAL_HEADER_SIZE
					+ (INCIDENT_JOURNAL_RECORDS_PER_SECTOR * JOURNAL_RECORD_SIZE))
					!=
					JOURNAL_SECTOR_SIZE))
		return false;
	memset(&header, 0xFF, sizeof(header));
	header.magic = JOURNAL_HEADER_MAGIC;
	header.schema = JOURNAL_HEADER_SCHEMA;
	header.size = sizeof(header);
	header.generation = 7U;
	header.first_sequence = 19U;
	memset(header.reserved, 0, sizeof(header.reserved));
	header.crc32 = W25Q64_CalculateCRC32((const uint8_t*) &header,
			(uint32_t) offsetof(JournalSectorHeader_t, crc32));
	header.commit = JOURNAL_COMMIT_MARKER;
	if (!Journal_HeaderValid(&header))
		return false;

	/* Schema-1 records must remain readable after the schema-2 extension. */
	memset(&legacy_record, 0xFF, sizeof(legacy_record));
	legacy_record.magic = JOURNAL_RECORD_MAGIC;
	legacy_record.schema = 1U;
	legacy_record.size = sizeof(legacy_record);
	legacy_record.sequence = 18U;
	legacy_record.timestamp_ms = 123455U;
	legacy_record.mode = CONFIG_MODE_CARGO;
	legacy_record.type = INCIDENT_TYPE_SHOCK;
	legacy_record.reserved0 = 0U;
	legacy_record.value = 2900;
	legacy_record.threshold = 2500;
	legacy_record.photo_outcome = 0U;
	legacy_record.photo_reason = 0U;
	memset(legacy_record.reserved1, 0, sizeof(legacy_record.reserved1));
	legacy_record.crc32 = W25Q64_CalculateCRC32((const uint8_t*) &legacy_record,
			(uint32_t) offsetof(JournalFlashRecord_t, crc32));
	legacy_record.commit = JOURNAL_COMMIT_MARKER;
	if (!Journal_RecordValid(&legacy_record))
		return false;

	memset(&record, 0xFF, sizeof(record));
	record.magic = JOURNAL_RECORD_MAGIC;
	record.schema = JOURNAL_RECORD_SCHEMA;
	record.size = sizeof(record);
	record.sequence = 19U;
	record.timestamp_ms = 123456U;
	record.mode = CONFIG_MODE_CARGO;
	record.type = INCIDENT_TYPE_SHOCK;
	record.reserved0 = 0U;
	record.value = 3000;
	record.threshold = 2500;
	memset(record.reserved1, 0, sizeof(record.reserved1));
	record.photo_outcome = INCIDENT_PHOTO_OUTCOME_PENDING;
	record.photo_reason = (uint8_t) INCIDENT_PHOTO_REASON_NONE;
	record.crc32 = Journal_RecordCRC(&record);
	record.commit = JOURNAL_COMMIT_MARKER;
	if (!Journal_RecordValid(&record))
		return false;
	expected_crc = record.crc32;
	record.photo_outcome = INCIDENT_PHOTO_OUTCOME_NO_PHOTO;
	record.photo_reason = (uint8_t) INCIDENT_PHOTO_REASON_COOLDOWN;
	if (!Journal_RecordValid(&record))
		return false;
	record.value++;
	if (Journal_RecordValid(&record))
		return false;
	return (expected_crc != 0U) && (JOURNAL_BASE_ADDRESS == 0x2000UL)
			&& (JOURNAL_END_ADDRESS == 0x80000UL)
			&& ((JOURNAL_END_ADDRESS - JOURNAL_BASE_ADDRESS)
					== (INCIDENT_JOURNAL_SECTOR_COUNT * JOURNAL_SECTOR_SIZE));
}
