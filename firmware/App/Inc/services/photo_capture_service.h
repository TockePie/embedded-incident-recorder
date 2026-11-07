#ifndef PHOTO_CAPTURE_SERVICE_H
#define PHOTO_CAPTURE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "services/photo_store.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PHOTO_CAPTURE_COOLDOWN_MS 3000U
#define PHOTO_CAPTURE_QUEUE_CAPACITY 32U

typedef struct {
	bool camera_available;
	uint32_t events_observed;
	uint32_t photos_stored;
	uint32_t skipped_cooldown;
	uint32_t skipped_camera_unavailable;
	uint32_t skipped_store_error;
	uint32_t queue_dropped;
	uint32_t capture_failures;
	uint32_t queue_depth;
	uint32_t active_event_id;
	uint16_t camera_width;
	uint16_t camera_height;
	uint32_t frame_bytes;
} PhotoCaptureService_Stats_t;

/* Initialize the OV7670. A camera failure disables photos but does not by
 * itself place the application in FAULT. */
bool PhotoCaptureService_Init(void);
/* Check for newly committed events and advance one capture/write transaction. */
void PhotoCaptureService_Process(void);
void PhotoCaptureService_GetStats(PhotoCaptureService_Stats_t *stats);
bool PhotoCaptureService_IsBusy(void);
/* Called after a confirmed media erase/reinitialization. */
void PhotoCaptureService_ResetAfterMediaErase(void);

#ifdef __cplusplus
}
#endif
#endif /* PHOTO_CAPTURE_SERVICE_H */
