#include "services/photo_store.h"

#include <stddef.h>
#include <string.h>

#include "drivers/w25q64.h"
#include "spi.h"
#include "services/config_service.h"
#include "services/storage_maintenance.h"
#include "services/cli_service.h"
#include "bsp/diag_uart.h"

#define PHOTO_IO_CHUNK 256U
#define PHOTO_BITMAP_BYTES (PHOTO_STORE_PARTITION_SECTORS / 8U)
#define PHOTO_INDEX_INVALID 0xFFFFU

typedef struct {
	uint32_t event_id;
	uint16_t start_sector;
	uint16_t sector_count;
} PhotoIndexEntry_t;

typedef enum {
	PS_UNINITIALIZED = 0,
	PS_RECOVERING,
	PS_READY,
	PS_WRITING,
	PS_RELEASING,
	PS_CLEANUP,
	PS_ERROR
} PhotoStoreInternalState_t;

typedef enum {
	PW_IDLE = 0,
	PW_CRC,
	PW_RESERVED_WAIT,
	PW_PAYLOAD_START,
	PW_PAYLOAD_WAIT,
	PW_VERIFY_PAYLOAD,
	PW_COMMIT_START,
	PW_COMMIT_WAIT,
	PW_VERIFY
} PhotoWriteState_t;

typedef enum {
	PC_MARK_START = 0, PC_MARK_WAIT, PC_ERASE_START, PC_ERASE_WAIT
} PhotoCleanupPhase_t;

static PhotoStore_Stats_t s_stats;
static PhotoStoreInternalState_t s_state;
static PhotoWriteState_t s_write_state;
static PhotoCleanupPhase_t s_cleanup_phase;
static uint8_t s_used_bitmap[PHOTO_BITMAP_BYTES];
static PhotoIndexEntry_t s_index[PHOTO_STORE_MAX_ENTRIES];
static uint16_t s_index_count;
static uint32_t s_scan_sector;
static uint32_t s_crc;
static uint32_t s_crc_offset;
static uint32_t s_write_offset;
static uint32_t s_payload_address;
static uint32_t s_extent_sectors;
static uint16_t s_write_start_sector;
static const uint8_t *s_frame;
static uint32_t s_frame_bytes;
static PhotoStore_Metadata_t s_metadata;
static PhotoStore_Metadata_t s_verify_metadata;
static uint8_t s_scan_header[PHOTO_STORE_METADATA_SIZE];
static uint32_t s_pending_crc;
static uint32_t s_verify_crc;
static uint32_t s_verify_offset;
static uint8_t s_verify_buffer[PHOTO_IO_CHUNK];
static bool s_flash_pending;

static PhotoStore_Metadata_t s_cleanup_metadata;
static uint16_t s_cleanup_start_sector;
static uint16_t s_cleanup_remaining;
static uint16_t s_cleanup_index;
static PhotoStoreInternalState_t s_cleanup_resume_state;
static uint32_t s_cleanup_resume_sector;
static uint32_t s_release_first_id;
static uint32_t s_release_last_id;
static uint16_t s_release_cursor;
static bool s_release_complete;
static bool s_release_requested;
static bool s_recovery_reported;
static bool s_read_session_active;

static uint32_t Ps_CrcUpdate(uint32_t crc, const uint8_t *data, uint32_t length) {
	uint32_t i;
	uint8_t bit;
	for (i = 0U; i < length; ++i) {
		crc ^= data[i];
		for (bit = 0U; bit < 8U; ++bit) {
			crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
		}
	}
	return crc;
}

uint32_t PhotoStore_CalculateCRC32(const uint8_t *data, uint32_t length) {
	if ((data == NULL) && (length != 0U)) {
		return 0U;
	}
	return Ps_CrcUpdate(0xFFFFFFFFUL, data, length) ^ 0xFFFFFFFFUL;
}

void PhotoStore_MetadataPrepare(PhotoStore_Metadata_t *metadata) {
	if (metadata == NULL) {
		return;
	}
	metadata->magic = PHOTO_STORE_METADATA_MAGIC;
	metadata->schema = PHOTO_STORE_METADATA_SCHEMA;
	metadata->size = (uint16_t) sizeof(*metadata);
	metadata->reserved0 = 0U;
	memset(metadata->reserved1, 0, sizeof(metadata->reserved1));
	metadata->metadata_crc32 = PhotoStore_CalculateCRC32(
			(const uint8_t*) metadata,
			(uint32_t) offsetof(PhotoStore_Metadata_t, metadata_crc32));
	metadata->commit = PHOTO_STORE_COMMIT_MARKER;
}

PhotoStore_FormatStatus_t PhotoStore_MetadataValidate(
		const PhotoStore_Metadata_t *metadata) {
	uint32_t crc;
	uint32_t sector_start;
	if (metadata == NULL)
		return PHOTO_STORE_FORMAT_BAD_ARGUMENT;
	if (metadata->magic != PHOTO_STORE_METADATA_MAGIC)
		return PHOTO_STORE_FORMAT_BAD_MAGIC;
	if (metadata->schema != PHOTO_STORE_METADATA_SCHEMA)
		return PHOTO_STORE_FORMAT_BAD_SCHEMA;
	if (metadata->size != sizeof(*metadata))
		return PHOTO_STORE_FORMAT_BAD_SIZE;
	if (metadata->commit != PHOTO_STORE_COMMIT_MARKER)
		return PHOTO_STORE_FORMAT_BAD_COMMIT;
	crc = PhotoStore_CalculateCRC32((const uint8_t*) metadata,
			(uint32_t) offsetof(PhotoStore_Metadata_t, metadata_crc32));
	if (metadata->metadata_crc32 != crc)
		return PHOTO_STORE_FORMAT_BAD_METADATA_CRC;
	if ((metadata->event_id == 0U) || (metadata->width == 0U)
			|| (metadata->height == 0U) || (metadata->frame_bytes == 0U)
			|| (metadata->payload_sector_count == 0U)
			|| (metadata->bytes_per_pixel == 0U)) {
		return PHOTO_STORE_FORMAT_BAD_GEOMETRY;
	}
	if ((metadata->pixel_format < PHOTO_STORE_PIXEL_FORMAT_RGB565)
			|| (metadata->pixel_format > PHOTO_STORE_PIXEL_FORMAT_GRAY8)) {
		return PHOTO_STORE_FORMAT_BAD_GEOMETRY;
	}
	if ((metadata->payload_address
			< PHOTO_STORE_PARTITION_BASE + PHOTO_STORE_PAYLOAD_OFFSET)
			|| (metadata->payload_address >= PHOTO_STORE_PARTITION_END)
			|| ((metadata->payload_address - PHOTO_STORE_PAYLOAD_OFFSET
					- PHOTO_STORE_PARTITION_BASE) % PHOTO_STORE_SECTOR_SIZE
					!= 0U)) {
		return PHOTO_STORE_FORMAT_BAD_PAYLOAD_RANGE;
	}
	sector_start = metadata->payload_address - PHOTO_STORE_PAYLOAD_OFFSET;
	if ((metadata->payload_sector_count
			> (PHOTO_STORE_PARTITION_END - sector_start)
					/ PHOTO_STORE_SECTOR_SIZE)
			|| (metadata->frame_bytes
					> ((uint32_t) metadata->payload_sector_count
							* PHOTO_STORE_SECTOR_SIZE
							- PHOTO_STORE_PAYLOAD_OFFSET))) {
		return PHOTO_STORE_FORMAT_BAD_PAYLOAD_RANGE;
	}
	return PHOTO_STORE_FORMAT_OK;
}

bool PhotoStore_RunFormatSelfTest(void) {
	static const uint8_t vector[] = "123456789";
	PhotoStore_Metadata_t metadata;
	memset(&metadata, 0, sizeof(metadata));
	metadata.event_id = 1U;
	metadata.width = 160U;
	metadata.height = 120U;
	metadata.pixel_format = PHOTO_STORE_PIXEL_FORMAT_RGB565;
	metadata.bytes_per_pixel = 2U;
	metadata.frame_bytes = 38400U;
	metadata.payload_address = PHOTO_STORE_PARTITION_BASE
			+ PHOTO_STORE_PAYLOAD_OFFSET;
	metadata.payload_sector_count = 10U;
	PhotoStore_MetadataPrepare(&metadata);
	return (sizeof(metadata) == PHOTO_STORE_METADATA_SIZE)
			&& (PhotoStore_CalculateCRC32(vector, 9U) == 0xCBF43926UL)
			&& (PhotoStore_MetadataValidate(&metadata) == PHOTO_STORE_FORMAT_OK);
}

static uint32_t Ps_SectorAddress(uint32_t sector) {
	return PHOTO_STORE_PARTITION_BASE + (sector * PHOTO_STORE_SECTOR_SIZE);
}

static bool Ps_BitGet(uint16_t sector) {
	return (s_used_bitmap[sector >> 3U] & (uint8_t) (1U << (sector & 7U))) != 0U;
}

static void Ps_BitSet(uint16_t sector, bool used) {
	uint8_t mask = (uint8_t) (1U << (sector & 7U));
	if (used)
		s_used_bitmap[sector >> 3U] |= mask;
	else
		s_used_bitmap[sector >> 3U] &= (uint8_t) ~mask;
}

static void Ps_MarkExtent(uint16_t start, uint16_t count, bool used) {
	uint16_t i;
	for (i = 0U;
			i < count && ((uint32_t) start + i) < PHOTO_STORE_PARTITION_SECTORS;
			++i) {
		Ps_BitSet((uint16_t) (start + i), used);
	}
}

static void Ps_UpdateStats(void) {
	uint16_t i;
	uint32_t used = 0U;
	uint32_t next = PHOTO_STORE_PARTITION_END;
	for (i = 0U; i < PHOTO_STORE_PARTITION_SECTORS; ++i) {
		if (Ps_BitGet(i))
			++used;
		else if (next == PHOTO_STORE_PARTITION_END)
			next = Ps_SectorAddress(i);
	}
	s_stats.occupied_sectors = used;
	s_stats.free_sectors = PHOTO_STORE_PARTITION_SECTORS - used;
	s_stats.index_entries = s_index_count;
	s_stats.next_free_address = next;
	s_stats.release_pending = !s_release_complete;
}

static bool Ps_AllFF(const uint8_t *data, uint32_t length) {
	uint32_t i;
	for (i = 0U; i < length; ++i)
		if (data[i] != 0xFFU)
			return false;
	return true;
}

static bool Ps_ReservationHeaderValid(const PhotoStore_Metadata_t *metadata,
		uint32_t sector_address) {
	uint32_t crc;
	uint32_t sector_start;
	if ((metadata->magic != PHOTO_STORE_METADATA_MAGIC)
			|| (metadata->schema != PHOTO_STORE_METADATA_SCHEMA)
			|| (metadata->size != sizeof(*metadata))
			|| (metadata->event_id == 0U) || (metadata->width == 0U)
			|| (metadata->height == 0U) || (metadata->frame_bytes == 0U)
			|| (metadata->bytes_per_pixel == 0U)
			|| (metadata->payload_sector_count == 0U)
			|| (metadata->payload_address
					!= sector_address + PHOTO_STORE_PAYLOAD_OFFSET)) {
		return false;
	}
	/* commit is deliberately not constrained here: a power cut while the
	 * 4-byte commit/tombstone word is being programmed may leave a partial
	 * value. The CRC excludes this word, so a valid reservation header still
	 * identifies the complete extent and recovery can safely erase it. */
	crc = PhotoStore_CalculateCRC32((const uint8_t*) metadata,
			(uint32_t) offsetof(PhotoStore_Metadata_t, metadata_crc32));
	if (metadata->metadata_crc32 != crc)
		return false;
	sector_start = sector_address;
	if ((metadata->payload_sector_count
			> (PHOTO_STORE_PARTITION_END - sector_start)
					/ PHOTO_STORE_SECTOR_SIZE)
			|| (metadata->frame_bytes
					> ((uint32_t) metadata->payload_sector_count
							* PHOTO_STORE_SECTOR_SIZE
							- PHOTO_STORE_PAYLOAD_OFFSET))) {
		return false;
	}
	return true;
}

static int Ps_FindIndexByEvent(uint32_t event_id) {
	uint16_t i;
	for (i = 0U; i < s_index_count; ++i) {
		if (s_index[i].event_id == event_id)
			return (int) i;
	}
	return -1;
}

static bool Ps_AddIndex(uint32_t event_id, uint16_t start, uint16_t count) {
	if (s_index_count >= PHOTO_STORE_MAX_ENTRIES)
		return false;
	s_index[s_index_count].event_id = event_id;
	s_index[s_index_count].start_sector = start;
	s_index[s_index_count].sector_count = count;
	++s_index_count;
	return true;
}

static void Ps_RemoveIndexAt(uint16_t index) {
	if (index >= s_index_count)
		return;
	if (index + 1U < s_index_count) {
		memmove(&s_index[index], &s_index[index + 1U],
				(size_t) (s_index_count - index - 1U) * sizeof(s_index[0]));
	}
	--s_index_count;
}

static void Ps_Fail(void) {
	++s_stats.io_errors;
	DiagUART_Printf(
			"[PHOTO] storage state ERROR io_errors=%lu; photo writes disabled, journal rotation will wait rather than orphan event-photo links\r\n",
			(unsigned long) s_stats.io_errors);
	s_state = PS_ERROR;
	s_stats.state = PHOTO_STORE_STATE_ERROR;
	s_flash_pending = false;
	s_release_complete = false;
	Ps_UpdateStats();
}

static bool Ps_PollFlash(void) {
	uint8_t sr1 = 0U;
	W25Q64_Status_t status = W25Q64_ReadStatusRegister1(&hspi2, &sr1);
	if (status != W25Q64_STATUS_OK) {
		Ps_Fail();
		return false;
	}
	if ((sr1 & W25Q64_STATUS1_BUSY_MASK) != 0U)
		return false;
	s_flash_pending = false;
	return true;
}

static bool Ps_StartProgram(uint32_t address, const uint8_t *data,
		uint16_t length) {
	W25Q64_Status_t status = W25Q64_StartPageProgram(&hspi2, address, data,
			length);
	if (status == W25Q64_STATUS_BUSY)
		return false;
	if (status != W25Q64_STATUS_OK) {
		Ps_Fail();
		return false;
	}
	s_flash_pending = true;
	return true;
}

static bool Ps_StartErase(uint32_t address) {
	W25Q64_Status_t status = W25Q64_StartSectorErase(&hspi2, address);
	if (status == W25Q64_STATUS_BUSY)
		return false;
	if (status != W25Q64_STATUS_OK) {
		Ps_Fail();
		return false;
	}
	s_flash_pending = true;
	return true;
}

static bool Ps_FlashAllowed(void) {
	return (ConfigService_GetSaveStatus() != CONFIG_SAVE_IN_PROGRESS)
			&& (StorageMaintenance_GetStatus() == STORAGE_MAINTENANCE_IDLE)
			&& (CliService_IsMaintenanceLocked() == false);
}

static bool Ps_FindFreeExtent(uint16_t needed, uint16_t *start_out) {
	uint16_t start;
	uint16_t run = 0U;
	if (start_out == NULL || needed == 0U)
		return false;
	for (start = 0U; start < PHOTO_STORE_PARTITION_SECTORS; ++start) {
		if (!Ps_BitGet(start)) {
			++run;
			if (run >= needed) {
				*start_out = (uint16_t) (start + 1U - needed);
				return true;
			}
		} else {
			run = 0U;
		}
	}
	return false;
}

static void Ps_BeginCleanup(const PhotoStore_Metadata_t *metadata,
		uint16_t start_sector, PhotoStoreInternalState_t resume_state,
		uint32_t resume_sector, uint16_t index_to_remove) {
	s_cleanup_metadata = *metadata;
	s_cleanup_start_sector = start_sector;
	s_cleanup_remaining = metadata->payload_sector_count;
	s_cleanup_index = index_to_remove;
	s_cleanup_resume_state = resume_state;
	s_cleanup_resume_sector = resume_sector;
	s_cleanup_phase = PC_MARK_START;
	s_state = PS_CLEANUP;
	s_stats.state = PHOTO_STORE_STATE_RELEASING;
}

static void Ps_ProcessCleanup(void) {
	uint32_t metadata_address = Ps_SectorAddress(s_cleanup_start_sector);
	uint32_t erase_address;

	if (s_flash_pending && !Ps_PollFlash())
		return;
	if (s_state == PS_ERROR)
		return;

	switch (s_cleanup_phase) {
	case PC_MARK_START:
		if (s_cleanup_metadata.commit != PHOTO_STORE_TOMBSTONE_MARKER) {
			s_cleanup_metadata.commit = PHOTO_STORE_TOMBSTONE_MARKER;
			if (!Ps_StartProgram(
					metadata_address
							+ (uint32_t) offsetof(PhotoStore_Metadata_t, commit),
					(const uint8_t*) &s_cleanup_metadata.commit,
					(uint16_t) sizeof(s_cleanup_metadata.commit)))
				return;
			s_cleanup_phase = PC_MARK_WAIT;
		} else {
			s_cleanup_phase = PC_ERASE_START;
		}
		break;

	case PC_MARK_WAIT:
		if (!s_flash_pending)
			s_cleanup_phase = PC_ERASE_START;
		break;

	case PC_ERASE_START:
		if (s_cleanup_remaining == 0U) {
			if (s_cleanup_index != PHOTO_INDEX_INVALID) {
				Ps_RemoveIndexAt(s_cleanup_index);
				s_stats.committed_photos =
						(s_stats.committed_photos > 0U) ?
								s_stats.committed_photos - 1U : 0U;
			}
			if (s_cleanup_resume_state == PS_RECOVERING) {
				s_scan_sector = s_cleanup_resume_sector;
				s_state = PS_RECOVERING;
				s_stats.state = PHOTO_STORE_STATE_RECOVERING;
			} else {
				s_release_cursor = 0U;
				s_state = PS_RELEASING;
				s_stats.state = PHOTO_STORE_STATE_RELEASING;
			}
			Ps_UpdateStats();
			return;
		}
		erase_address = Ps_SectorAddress(
				s_cleanup_start_sector + (uint32_t) s_cleanup_remaining - 1U);
		if (!Ps_StartErase(erase_address))
			return;
		s_cleanup_phase = PC_ERASE_WAIT;
		break;

	case PC_ERASE_WAIT:
		if (!s_flash_pending) {
			Ps_BitSet(
					(uint16_t) (s_cleanup_start_sector
							+ (uint32_t) s_cleanup_remaining - 1U), false);
			--s_cleanup_remaining;
			s_cleanup_phase = PC_ERASE_START;
		}
		break;

	default:
		Ps_Fail();
		break;
	}
}

PhotoStore_Status_t PhotoStore_Init(void) {
	memset(&s_stats, 0, sizeof(s_stats));
	memset(s_used_bitmap, 0, sizeof(s_used_bitmap));
	memset(s_index, 0, sizeof(s_index));
	s_index_count = 0U;
	s_state = PS_RECOVERING;
	s_write_state = PW_IDLE;
	s_scan_sector = 0U;
	s_flash_pending = false;
	s_release_complete = true;
	s_release_requested = false;
	s_recovery_reported = false;
	s_read_session_active = false;
	s_stats.state = PHOTO_STORE_STATE_RECOVERING;
	s_stats.next_free_address = PHOTO_STORE_PARTITION_BASE;
	return PHOTO_STORE_STATUS_OK;
}

void PhotoStore_Process(void) {
	if (!Ps_FlashAllowed())
		return;

	if (s_state == PS_CLEANUP) {
		Ps_ProcessCleanup();
		return;
	}

	if (s_state == PS_RECOVERING) {
		PhotoStore_Metadata_t metadata;
		uint32_t address;
		uint16_t start_sector;
		if (s_scan_sector >= PHOTO_STORE_PARTITION_SECTORS) {
			s_state = PS_READY;
			s_stats.state = PHOTO_STORE_STATE_READY;
			Ps_UpdateStats();
			if (!s_recovery_reported) {
				DiagUART_Printf(
						"[PHOTO] recovery PASS scanned=%lu committed=%lu occupied=%lu free=%lu next=0x%06lX\r\n",
						(unsigned long) s_stats.scanned_sectors,
						(unsigned long) s_stats.committed_photos,
						(unsigned long) s_stats.occupied_sectors,
						(unsigned long) s_stats.free_sectors,
						(unsigned long) s_stats.next_free_address);
				s_recovery_reported = true;
			}
			return;
		}
		address = Ps_SectorAddress(s_scan_sector);
		{
			W25Q64_Status_t status = W25Q64_Read(&hspi2, address, s_scan_header,
					sizeof(s_scan_header));
			if (status == W25Q64_STATUS_BUSY)
				return;
			if (status != W25Q64_STATUS_OK) {
				Ps_Fail();
				return;
			}
		}
		++s_stats.scanned_sectors;
		if (Ps_AllFF(s_scan_header, sizeof(s_scan_header))) {
			++s_scan_sector;
			return;
		}
		memcpy(&metadata, s_scan_header, sizeof(metadata));
		if (!Ps_ReservationHeaderValid(&metadata, address)) {
			/* A torn header with unknown extent is quarantined as one sector. */
			Ps_BitSet((uint16_t) s_scan_sector, true);
			++s_scan_sector;
			return;
		}
		start_sector = (uint16_t) s_scan_sector;
		Ps_MarkExtent(start_sector, metadata.payload_sector_count, true);
		if ((metadata.commit == PHOTO_STORE_COMMIT_MARKER)
				&& (PhotoStore_MetadataValidate(&metadata)
						== PHOTO_STORE_FORMAT_OK)) {
			if (!Ps_AddIndex(metadata.event_id, start_sector,
					metadata.payload_sector_count)) {
				Ps_Fail();
				return;
			}
			++s_stats.committed_photos;
			if (metadata.event_id > s_stats.last_event_id)
				s_stats.last_event_id = metadata.event_id;
			s_scan_sector += metadata.payload_sector_count;
		} else {
			Ps_BeginCleanup(&metadata, start_sector, PS_RECOVERING,
					s_scan_sector + metadata.payload_sector_count,
					PHOTO_INDEX_INVALID);
		}
		Ps_UpdateStats();
		return;
	}

	if (s_state == PS_RELEASING) {
		uint16_t i;
		if (s_flash_pending && !Ps_PollFlash())
			return;
		for (i = s_release_cursor; i < s_index_count; ++i) {
			if ((s_index[i].event_id >= s_release_first_id)
					&& (s_index[i].event_id <= s_release_last_id)) {
				PhotoStore_Metadata_t metadata;
				uint32_t address = Ps_SectorAddress(s_index[i].start_sector);
				W25Q64_Status_t status = W25Q64_Read(&hspi2, address,
						(uint8_t*) &metadata, sizeof(metadata));
				if (status == W25Q64_STATUS_BUSY)
					return;
				if (status != W25Q64_STATUS_OK) {
					Ps_Fail();
					return;
				}
				if (PhotoStore_MetadataValidate(&metadata)
						!= PHOTO_STORE_FORMAT_OK) {
					Ps_Fail();
					return;
				}
				Ps_BeginCleanup(&metadata, s_index[i].start_sector,
						PS_RELEASING, 0U, i);
				return;
			}
		}
		s_release_requested = false;
		s_release_complete = true;
		s_state = PS_READY;
		s_stats.state = PHOTO_STORE_STATE_READY;
		Ps_UpdateStats();
		DiagUART_Printf("[PHOTO] reclaim complete event_id_range=%lu..%lu\r\n",
				(unsigned long) s_release_first_id,
				(unsigned long) s_release_last_id);
		return;
	}

	if (s_state != PS_WRITING)
		return;
	if (s_flash_pending && !Ps_PollFlash())
		return;
	if (s_state == PS_ERROR)
		return;

	switch (s_write_state) {
	case PW_CRC: {
		uint32_t count = s_frame_bytes - s_crc_offset;
		if (count > PHOTO_IO_CHUNK)
			count = PHOTO_IO_CHUNK;
		s_crc = Ps_CrcUpdate(s_crc, &s_frame[s_crc_offset], count);
		s_crc_offset += count;
		if (s_crc_offset >= s_frame_bytes) {
			s_pending_crc = s_crc ^ 0xFFFFFFFFUL;
			s_metadata.payload_crc32 = s_pending_crc;
			PhotoStore_MetadataPrepare(&s_metadata);
			s_metadata.commit = PHOTO_STORE_RESERVED_MARKER;
			if (!Ps_StartProgram(Ps_SectorAddress(s_write_start_sector),
					(const uint8_t*) &s_metadata, PHOTO_STORE_METADATA_SIZE))
				return;
			s_write_state = PW_RESERVED_WAIT;
		}
		break;
	}
	case PW_RESERVED_WAIT:
		if (!s_flash_pending)
			s_write_state = PW_PAYLOAD_START;
		break;
	case PW_PAYLOAD_START: {
		uint32_t count = s_frame_bytes - s_write_offset;
		uint32_t address = s_payload_address + s_write_offset;
		uint32_t page_left = W25Q64_PAGE_SIZE_BYTES
				- (address % W25Q64_PAGE_SIZE_BYTES);
		if (count > PHOTO_IO_CHUNK)
			count = PHOTO_IO_CHUNK;
		if (count > page_left)
			count = page_left;
		if (!Ps_StartProgram(address, &s_frame[s_write_offset],
				(uint16_t) count))
			return;
		s_write_offset += count;
		s_write_state = PW_PAYLOAD_WAIT;
		break;
	}
	case PW_PAYLOAD_WAIT:
		if (!s_flash_pending) {
			if (s_write_offset >= s_frame_bytes) {
				s_verify_crc = 0xFFFFFFFFUL;
				s_verify_offset = 0U;
				s_write_state = PW_VERIFY_PAYLOAD;
			} else
				s_write_state = PW_PAYLOAD_START;
		}
		break;
	case PW_VERIFY_PAYLOAD: {
		uint32_t count = s_frame_bytes - s_verify_offset;
		W25Q64_Status_t status;
		if (count > PHOTO_IO_CHUNK)
			count = PHOTO_IO_CHUNK;
		status = W25Q64_Read(&hspi2, s_payload_address + s_verify_offset,
				s_verify_buffer, count);
		if (status == W25Q64_STATUS_BUSY)
			return;
		if (status != W25Q64_STATUS_OK) {
			Ps_Fail();
			return;
		}
		s_verify_crc = Ps_CrcUpdate(s_verify_crc, s_verify_buffer, count);
		s_verify_offset += count;
		if (s_verify_offset >= s_frame_bytes) {
			if ((s_verify_crc ^ 0xFFFFFFFFUL) != s_pending_crc) {
				Ps_Fail();
				return;
			}
			s_write_state = PW_COMMIT_START;
		}
		break;
	}
	case PW_COMMIT_START:
		s_metadata.commit = PHOTO_STORE_COMMIT_MARKER;
		if (!Ps_StartProgram(
				Ps_SectorAddress(
						s_write_start_sector) + (uint32_t) offsetof(PhotoStore_Metadata_t, commit),
				(const uint8_t*) &s_metadata.commit,
				(uint16_t) sizeof(s_metadata.commit)))
			return;
		s_write_state = PW_COMMIT_WAIT;
		break;
	case PW_COMMIT_WAIT:
		if (!s_flash_pending)
			s_write_state = PW_VERIFY;
		break;
	case PW_VERIFY: {
		W25Q64_Status_t status = W25Q64_Read(&hspi2,
				Ps_SectorAddress(s_write_start_sector),
				(uint8_t*) &s_verify_metadata, sizeof(s_verify_metadata));
		if (status == W25Q64_STATUS_BUSY)
			return;
		if ((status != W25Q64_STATUS_OK)
				|| (PhotoStore_MetadataValidate(&s_verify_metadata)
						!= PHOTO_STORE_FORMAT_OK)
				|| (s_verify_metadata.payload_crc32 != s_pending_crc)) {
			Ps_Fail();
			return;
		}
		if (!Ps_AddIndex(s_metadata.event_id, s_write_start_sector,
				(uint16_t) s_extent_sectors)) {
			Ps_Fail();
			return;
		}
		++s_stats.committed_photos;
		s_stats.last_event_id = s_metadata.event_id;
		s_state = PS_READY;
		s_stats.state = PHOTO_STORE_STATE_READY;
		s_write_state = PW_IDLE;
		s_frame = NULL;
		s_frame_bytes = 0U;
		Ps_UpdateStats();
		if (s_release_requested) {
			s_state = PS_RELEASING;
			s_stats.state = PHOTO_STORE_STATE_RELEASING;
		}
		break;
	}
	default:
		Ps_Fail();
		break;
	}
}

PhotoStore_Status_t PhotoStore_BeginWrite(uint32_t event_id,
		uint32_t capture_tick_ms, uint16_t width, uint16_t height,
		uint8_t pixel_format, uint8_t bytes_per_pixel,
		const uint8_t *frame_data, uint32_t frame_bytes) {
	uint32_t needed;
	uint16_t start_sector;
	if (s_state != PS_READY) {
		return (s_state == PS_WRITING || s_state == PS_RELEASING) ?
				PHOTO_STORE_STATUS_BUSY : PHOTO_STORE_STATUS_NOT_READY;
	}
	if (s_release_requested || s_read_session_active)
		return PHOTO_STORE_STATUS_BUSY;
	if ((event_id == 0U) || (width == 0U) || (height == 0U)
			|| (bytes_per_pixel == 0U) || (frame_data == NULL)
			|| (frame_bytes == 0U)) {
		return PHOTO_STORE_STATUS_INVALID_ARGUMENT;
	}
	if ((pixel_format < PHOTO_STORE_PIXEL_FORMAT_RGB565)
			|| (pixel_format > PHOTO_STORE_PIXEL_FORMAT_GRAY8)
			|| ((pixel_format == PHOTO_STORE_PIXEL_FORMAT_GRAY8)
					&& bytes_per_pixel != 1U)
			|| ((pixel_format != PHOTO_STORE_PIXEL_FORMAT_GRAY8)
					&& bytes_per_pixel != 2U)
			|| (((uint64_t) width * height * bytes_per_pixel) > frame_bytes)) {
		return PHOTO_STORE_STATUS_INVALID_ARGUMENT;
	}
	if (Ps_FindIndexByEvent(event_id) >= 0)
		return PHOTO_STORE_STATUS_ALREADY_EXISTS;
	needed = (PHOTO_STORE_PAYLOAD_OFFSET + frame_bytes + PHOTO_STORE_SECTOR_SIZE
			- 1U) / PHOTO_STORE_SECTOR_SIZE;
	if ((needed == 0U) || (needed > PHOTO_STORE_PARTITION_SECTORS)
			|| (needed > UINT16_MAX)
			|| (s_index_count >= PHOTO_STORE_MAX_ENTRIES)) {
		return PHOTO_STORE_STATUS_NO_SPACE;
	}
	if (!Ps_FindFreeExtent((uint16_t) needed, &start_sector))
		return PHOTO_STORE_STATUS_NO_SPACE;
	Ps_MarkExtent(start_sector, (uint16_t) needed, true);

	memset(&s_metadata, 0, sizeof(s_metadata));
	s_metadata.event_id = event_id;
	s_metadata.capture_tick_ms = capture_tick_ms;
	s_metadata.width = width;
	s_metadata.height = height;
	s_metadata.pixel_format = pixel_format;
	s_metadata.bytes_per_pixel = bytes_per_pixel;
	s_metadata.frame_bytes = frame_bytes;
	s_metadata.payload_address = Ps_SectorAddress(
			start_sector) + PHOTO_STORE_PAYLOAD_OFFSET;
	s_metadata.payload_sector_count = (uint16_t) needed;

	s_write_start_sector = start_sector;
	s_payload_address = s_metadata.payload_address;
	s_extent_sectors = needed;
	s_frame = frame_data;
	s_frame_bytes = frame_bytes;
	s_crc = 0xFFFFFFFFUL;
	s_crc_offset = 0U;
	s_write_offset = 0U;
	s_verify_crc = 0xFFFFFFFFUL;
	s_verify_offset = 0U;
	s_write_state = PW_CRC;
	s_state = PS_WRITING;
	s_stats.state = PHOTO_STORE_STATE_WRITING;
	Ps_UpdateStats();
	return PHOTO_STORE_STATUS_OK;
}

PhotoStore_Status_t PhotoStore_Find(uint32_t event_id,
		PhotoStore_Metadata_t *metadata) {
	int index;
	uint32_t address;
	W25Q64_Status_t status;
	if ((event_id == 0U) || (metadata == NULL))
		return PHOTO_STORE_STATUS_INVALID_ARGUMENT;
	if (s_state != PS_READY)
		return PHOTO_STORE_STATUS_NOT_READY;
	index = Ps_FindIndexByEvent(event_id);
	if (index < 0)
		return PHOTO_STORE_STATUS_NOT_FOUND;
	address = Ps_SectorAddress(s_index[index].start_sector);
	status = W25Q64_Read(&hspi2, address, (uint8_t*) metadata,
			sizeof(*metadata));
	if (status == W25Q64_STATUS_BUSY)
		return PHOTO_STORE_STATUS_BUSY;
	if (status != W25Q64_STATUS_OK)
		return PHOTO_STORE_STATUS_IO_ERROR;
	if (PhotoStore_MetadataValidate(metadata) != PHOTO_STORE_FORMAT_OK
			|| metadata->event_id != event_id)
		return PHOTO_STORE_STATUS_VERIFY_ERROR;
	return PHOTO_STORE_STATUS_OK;
}

PhotoStore_Status_t PhotoStore_ReadPayload(
		const PhotoStore_Metadata_t *metadata, uint32_t offset,
		uint8_t *destination, uint32_t length) {
	W25Q64_Status_t status;
	if ((metadata == NULL) || (destination == NULL) || (length == 0U)
			|| (PhotoStore_MetadataValidate(metadata) != PHOTO_STORE_FORMAT_OK)
			|| (offset > metadata->frame_bytes)
			|| (length > metadata->frame_bytes - offset)) {
		return PHOTO_STORE_STATUS_INVALID_ARGUMENT;
	}
	if (s_state != PS_READY)
		return PHOTO_STORE_STATUS_NOT_READY;
	status = W25Q64_Read(&hspi2, metadata->payload_address + offset,
			destination, length);
	if (status == W25Q64_STATUS_BUSY)
		return PHOTO_STORE_STATUS_BUSY;
	return (status == W25Q64_STATUS_OK) ?
			PHOTO_STORE_STATUS_OK : PHOTO_STORE_STATUS_IO_ERROR;
}

PhotoStore_Status_t PhotoStore_BeginReadSession(void) {
	if (s_state != PS_READY)
		return PHOTO_STORE_STATUS_NOT_READY;
	if (s_read_session_active || s_release_requested)
		return PHOTO_STORE_STATUS_BUSY;
	s_read_session_active = true;
	return PHOTO_STORE_STATUS_OK;
}

void PhotoStore_EndReadSession(void) {
	s_read_session_active = false;
}

PhotoStore_Status_t PhotoStore_GetEventIdAt(uint16_t index, uint32_t *event_id) {
	if (event_id == NULL)
		return PHOTO_STORE_STATUS_INVALID_ARGUMENT;
	if (s_state != PS_READY)
		return PHOTO_STORE_STATUS_NOT_READY;
	if (index >= s_index_count)
		return PHOTO_STORE_STATUS_NOT_FOUND;
	*event_id = s_index[index].event_id;
	return PHOTO_STORE_STATUS_OK;
}

PhotoStore_Status_t PhotoStore_RequestReleaseRange(uint32_t first_event_id,
		uint32_t last_event_id) {
	if ((first_event_id == 0U) || (last_event_id < first_event_id))
		return PHOTO_STORE_STATUS_INVALID_ARGUMENT;
	if (s_state == PS_ERROR || s_state == PS_UNINITIALIZED)
		return PHOTO_STORE_STATUS_NOT_READY;
	if (s_release_requested || s_state == PS_RELEASING || s_state == PS_CLEANUP
			|| s_read_session_active)
		return PHOTO_STORE_STATUS_BUSY;
	if (s_state != PS_READY && s_state != PS_WRITING)
		return PHOTO_STORE_STATUS_NOT_READY;
	s_release_first_id = first_event_id;
	s_release_last_id = last_event_id;
	s_release_cursor = 0U;
	s_release_requested = true;
	s_release_complete = false;
	s_stats.release_pending = true;
	DiagUART_Printf("[PHOTO] reclaim started event_id_range=%lu..%lu\r\n",
			(unsigned long) first_event_id, (unsigned long) last_event_id);
	if (s_state == PS_READY) {
		s_state = PS_RELEASING;
		s_stats.state = PHOTO_STORE_STATE_RELEASING;
	}
	return PHOTO_STORE_STATUS_OK;
}

bool PhotoStore_IsReleaseComplete(void) {
	return s_release_complete;
}

PhotoStore_State_t PhotoStore_GetState(void) {
	switch (s_state) {
	case PS_RECOVERING:
		return PHOTO_STORE_STATE_RECOVERING;
	case PS_READY:
		return PHOTO_STORE_STATE_READY;
	case PS_WRITING:
		return PHOTO_STORE_STATE_WRITING;
	case PS_RELEASING:
	case PS_CLEANUP:
		return PHOTO_STORE_STATE_RELEASING;
	case PS_ERROR:
		return PHOTO_STORE_STATE_ERROR;
	default:
		return PHOTO_STORE_STATE_UNINITIALIZED;
	}
}

void PhotoStore_GetStats(PhotoStore_Stats_t *stats) {
	if (stats != NULL) {
		Ps_UpdateStats();
		*stats = s_stats;
	}
}
