#ifndef PHOTO_STORE_H
#define PHOTO_STORE_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers/w25q64.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PHOTO_STORE_PARTITION_BASE       0x00080000UL
#define PHOTO_STORE_PARTITION_END        0x00800000UL
#define PHOTO_STORE_SECTOR_SIZE          4096UL
#define PHOTO_STORE_PARTITION_SECTORS    1920UL
#define PHOTO_STORE_METADATA_MAGIC       0x50484F54UL
#define PHOTO_STORE_METADATA_SCHEMA      1U
#define PHOTO_STORE_METADATA_SIZE        64U
#define PHOTO_STORE_COMMIT_MARKER        0x50484F4CUL
#define PHOTO_STORE_RESERVED_MARKER      0xFFFFFFFFUL
#define PHOTO_STORE_TOMBSTONE_MARKER     0x00000000UL
#define PHOTO_STORE_PAYLOAD_OFFSET       256UL
#define PHOTO_STORE_PIXEL_FORMAT_RGB565  1U
#define PHOTO_STORE_PIXEL_FORMAT_YUV422  2U
#define PHOTO_STORE_PIXEL_FORMAT_GRAY8   3U
#define PHOTO_STORE_MAX_ENTRIES          256U

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t schema;
	uint16_t size;
	uint32_t event_id;
	uint32_t capture_tick_ms;
	uint16_t width;
	uint16_t height;
	uint8_t pixel_format;
	uint8_t bytes_per_pixel;
	uint16_t flags;
	uint32_t frame_bytes;
	uint32_t payload_address;
	uint16_t payload_sector_count;
	uint16_t reserved0;
	uint32_t payload_crc32;
	uint32_t metadata_crc32;
	uint32_t commit;
	uint8_t reserved1[16];
} PhotoStore_Metadata_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(PhotoStore_Metadata_t) == PHOTO_STORE_METADATA_SIZE,
		"PhotoStore metadata on-flash layout must remain 64 bytes");
#endif

typedef enum {
	PHOTO_STORE_FORMAT_OK = 0,
	PHOTO_STORE_FORMAT_BAD_ARGUMENT,
	PHOTO_STORE_FORMAT_BAD_MAGIC,
	PHOTO_STORE_FORMAT_BAD_SCHEMA,
	PHOTO_STORE_FORMAT_BAD_SIZE,
	PHOTO_STORE_FORMAT_BAD_COMMIT,
	PHOTO_STORE_FORMAT_BAD_METADATA_CRC,
	PHOTO_STORE_FORMAT_BAD_GEOMETRY,
	PHOTO_STORE_FORMAT_BAD_PAYLOAD_RANGE
} PhotoStore_FormatStatus_t;

typedef enum {
	PHOTO_STORE_STATUS_OK = 0,
	PHOTO_STORE_STATUS_BUSY,
	PHOTO_STORE_STATUS_NOT_READY,
	PHOTO_STORE_STATUS_INVALID_ARGUMENT,
	PHOTO_STORE_STATUS_NO_SPACE,
	PHOTO_STORE_STATUS_IO_ERROR,
	PHOTO_STORE_STATUS_VERIFY_ERROR,
	PHOTO_STORE_STATUS_NOT_FOUND,
	PHOTO_STORE_STATUS_ALREADY_EXISTS
} PhotoStore_Status_t;

typedef enum {
	PHOTO_STORE_STATE_UNINITIALIZED = 0,
	PHOTO_STORE_STATE_RECOVERING,
	PHOTO_STORE_STATE_READY,
	PHOTO_STORE_STATE_WRITING,
	PHOTO_STORE_STATE_RELEASING,
	PHOTO_STORE_STATE_ERROR
} PhotoStore_State_t;

typedef struct {
	PhotoStore_State_t state;
	uint32_t scanned_sectors;
	uint32_t occupied_sectors;
	uint32_t free_sectors;
	uint32_t committed_photos;
	uint32_t next_free_address;
	uint32_t last_event_id;
	uint32_t io_errors;
	uint16_t index_entries;
	bool release_pending;
} PhotoStore_Stats_t;

uint32_t PhotoStore_CalculateCRC32(const uint8_t *data, uint32_t length);
void PhotoStore_MetadataPrepare(PhotoStore_Metadata_t *metadata);
PhotoStore_FormatStatus_t PhotoStore_MetadataValidate(
		const PhotoStore_Metadata_t *metadata);
bool PhotoStore_RunFormatSelfTest(void);

PhotoStore_Status_t PhotoStore_Init(void);
void PhotoStore_Process(void);
PhotoStore_State_t PhotoStore_GetState(void);
void PhotoStore_GetStats(PhotoStore_Stats_t *stats);

PhotoStore_Status_t PhotoStore_BeginWrite(uint32_t event_id,
		uint32_t capture_tick_ms, uint16_t width, uint16_t height,
		uint8_t pixel_format, uint8_t bytes_per_pixel,
		const uint8_t *frame_data, uint32_t frame_bytes);
PhotoStore_Status_t PhotoStore_Find(uint32_t event_id,
		PhotoStore_Metadata_t *metadata);
PhotoStore_Status_t PhotoStore_ReadPayload(
		const PhotoStore_Metadata_t *metadata, uint32_t offset,
		uint8_t *destination, uint32_t length);
PhotoStore_Status_t PhotoStore_BeginReadSession(void);
void PhotoStore_EndReadSession(void);
/* Return an event ID from the recovered index of committed photos. */
PhotoStore_Status_t PhotoStore_GetEventIdAt(uint16_t index, uint32_t *event_id);

/* Journal calls this before erasing a sector containing old event IDs.
 * The photo extents are tombstoned and erased asynchronously, metadata sector
 * last, so a power loss cannot leave an unidentifiable partial allocation. */
PhotoStore_Status_t PhotoStore_RequestReleaseRange(uint32_t first_event_id,
		uint32_t last_event_id);
bool PhotoStore_IsReleaseComplete(void);

#ifdef __cplusplus
}
#endif
#endif /* PHOTO_STORE_H */
