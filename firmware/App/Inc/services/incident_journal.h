#ifndef INCIDENT_JOURNAL_H
#define INCIDENT_JOURNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "services/incident_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INCIDENT_JOURNAL_SECTOR_COUNT       126U
#define INCIDENT_JOURNAL_RECORDS_PER_SECTOR  63U
#define INCIDENT_JOURNAL_RECENT_MAX          30U

/* Persistent photo outcome; 0xFF means pending/unknown after interruption. */
#define INCIDENT_PHOTO_OUTCOME_LEGACY        0x00U
#define INCIDENT_PHOTO_OUTCOME_PENDING       0xFFU
#define INCIDENT_PHOTO_OUTCOME_STORED        0xFEU
#define INCIDENT_PHOTO_OUTCOME_NO_PHOTO      0xFCU

typedef enum {
	INCIDENT_PHOTO_REASON_NONE = 0xFFU,
	INCIDENT_PHOTO_REASON_COOLDOWN = 0xFEU,
	INCIDENT_PHOTO_REASON_CAMERA_UNAVAILABLE = 0xFCU,
	INCIDENT_PHOTO_REASON_CAPTURE_FAILED = 0xF8U,
	INCIDENT_PHOTO_REASON_STORAGE_FULL = 0xF0U,
	INCIDENT_PHOTO_REASON_STORAGE_ERROR = 0xE0U,
	INCIDENT_PHOTO_REASON_QUEUE_FULL = 0xC0U,
	INCIDENT_PHOTO_REASON_POLICY = 0x80U
} IncidentPhotoReason_t;

typedef enum {
	INCIDENT_JOURNAL_OK = 0,
	INCIDENT_JOURNAL_BUSY,
	INCIDENT_JOURNAL_NOT_READY,
	INCIDENT_JOURNAL_IO_ERROR,
	INCIDENT_JOURNAL_CORRUPT,
	INCIDENT_JOURNAL_NOT_FOUND
} IncidentJournalStatus_t;

typedef struct {
	uint32_t sequence;
	uint32_t timestamp_ms;
	uint8_t mode;
	uint8_t type;
	int32_t value;
	int32_t threshold;
	uint8_t photo_outcome;
	uint8_t photo_reason;
} IncidentJournalRecord_t;

/* Cooperative read cursor for exporting the complete retained journal.
 * One ReadAllNext() call performs at most one SPI-flash read. The cursor is a
 * snapshot of the sector range at BeginReadAll(); call only while DISARMED. */
typedef struct {
	uint16_t sector;
	uint16_t sectors_scanned;
	uint16_t active_sector_snapshot;
	uint8_t slot;
	uint8_t active_slot_limit;
	uint8_t sector_slot_limit;
	uint8_t header_loaded;
	uint8_t active;
	uint32_t sector_first_sequence;
	uint32_t records_returned;
} IncidentJournalReadAllCursor_t;

typedef struct {
	bool ready;
	bool write_pending;
	bool full_cycle_started;
	uint16_t active_sector_index;
	uint16_t active_sector_records;
	uint16_t valid_sector_count;
	uint32_t available_records;
	uint32_t overwritten_sectors;
	uint32_t sector_generation;
	uint32_t next_sequence;
	uint32_t committed_records_this_boot;
	uint32_t dropped_records;
	uint32_t crc_rejected_records;
	uint32_t io_errors;
} IncidentJournalStats_t;

/* Scans the reserved journal partition. No erase/program is performed here. */
IncidentJournalStatus_t IncidentJournal_Init(void);
/* One bounded state-machine step. Call frequently from the super loop. */
void IncidentJournal_Process(void);
void IncidentJournal_GetStats(IncidentJournalStats_t *stats);
/* Returns up to count newest valid records, newest first. count <= MAX. */
IncidentJournalStatus_t IncidentJournal_ReadRecent(
		IncidentJournalRecord_t *records, uint8_t count, uint8_t *read_count);
/* Starts/restarts a bounded full-journal scan. BUSY means a Flash/journal
 * transaction is active; no blocking wait is performed. */
IncidentJournalStatus_t IncidentJournal_BeginReadAll(
		IncidentJournalReadAllCursor_t *cursor);
IncidentJournalStatus_t IncidentJournal_RestartReadAll(
		IncidentJournalReadAllCursor_t *cursor);
/* Scans physical sector order; persistent IDs are authoritative for sorting.
 * OK returns one record; BUSY means call again later; NOT_FOUND is end. */
IncidentJournalStatus_t IncidentJournal_ReadAllNext(
		IncidentJournalReadAllCursor_t *cursor, IncidentJournalRecord_t *record);
/* Finds one retained record by its persistent, non-reused sequence ID. */
IncidentJournalStatus_t IncidentJournal_ReadBySequence(uint32_t sequence,
		IncidentJournalRecord_t *record);
/* True while an erase/program/verify transaction is in progress. */
bool IncidentJournal_IsBusy(void);
/* Snapshot of the newest record committed during this boot. The generation
 * increments exactly once after persistent CRC/commit read-back succeeds. */
bool IncidentJournal_GetLastCommittedRecord(IncidentJournalRecord_t *record,
		uint32_t *generation);
/* Queues a persistent one-way photo outcome update for a committed event. */
IncidentJournalStatus_t IncidentJournal_RequestPhotoOutcome(uint32_t sequence,
		uint8_t outcome, uint8_t reason);
bool IncidentJournal_IsPhotoOutcomePending(uint32_t sequence);
/* Validate record layout and CRC handling without accessing external Flash. */
bool IncidentJournal_RunSelfTest(void);

#ifdef __cplusplus
}
#endif
#endif /* INCIDENT_JOURNAL_H */
