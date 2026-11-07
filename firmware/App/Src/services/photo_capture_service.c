#include "services/photo_capture_service.h"

#include <string.h>

#include "bsp/diag_uart.h"
#include "drivers/ov7670.h"
#include "drivers/ov7670_dvp_capture.h"
#include "services/incident_journal.h"
#include "services/config_service.h"
#include "services/incident_detector.h"
#include "tim.h"

#define PHOTO_CAMERA_REG_COM10 0x15U
#define PHOTO_CAMERA_REG_COM12 0x3CU
#define PHOTO_CAMERA_COM10_PCLK_HBLANK 0x20U
#define PHOTO_CAMERA_COM12_HREF_ALWAYS 0x80U

#define PHOTO_CAPTURE_TYPE_COUNT 5U
#define PHOTO_OUTCOME_BACKLOG_CAPACITY 32U

typedef enum {
	PCS_IDLE = 0, PCS_CAPTURING, PCS_CAPTURE_DONE, PCS_WRITING
} PhotoCaptureState_t;

static IncidentJournalRecord_t s_queue[PHOTO_CAPTURE_QUEUE_CAPACITY];
static uint8_t s_queue_head;
static uint8_t s_queue_count;
static uint32_t s_last_commit_generation;
static uint32_t s_last_photo_event_tick[PHOTO_CAPTURE_TYPE_COUNT];
static uint32_t s_last_photo_commit_tick[PHOTO_CAPTURE_TYPE_COUNT];
static bool s_last_photo_tick_valid[PHOTO_CAPTURE_TYPE_COUNT];
static PhotoCaptureService_Stats_t s_stats;
static PhotoCaptureState_t s_state;
static OV7670_Info_t s_camera_info;
static OV7670_DvpCaptureReport_t s_capture_report;
static uint32_t s_capture_frame_bytes;
static uint32_t s_capture_tick_ms;
static const uint8_t *s_capture_frame;
static bool s_initialized;
static bool s_outcome_waiting;
static bool s_outcome_submitted;
static uint32_t s_outcome_event_id;
static IncidentPhotoReason_t s_outcome_reason;
static const char *s_outcome_reason_text;
static bool s_capture_failure_reported;
typedef struct {
	uint32_t event_id;
	IncidentPhotoReason_t reason;
} PhotoOutcomeBacklogEntry_t;
static PhotoOutcomeBacklogEntry_t s_outcome_backlog[PHOTO_OUTCOME_BACKLOG_CAPACITY];
static uint8_t s_outcome_backlog_head;
static uint8_t s_outcome_backlog_count;
static bool s_outcome_backlog_submitted;

static IncidentJournalRecord_t* PhotoCapture_HeadEvent(void) {
	if (s_queue_count == 0U)
		return NULL;
	return &s_queue[s_queue_head];
}

static void PhotoCapture_PopEvent(void) {
	if (s_queue_count != 0U) {
		s_queue_head = (uint8_t) ((s_queue_head + 1U)
				% PHOTO_CAPTURE_QUEUE_CAPACITY);
		--s_queue_count;
	}
	s_state = PCS_IDLE;
	s_capture_frame = NULL;
	s_capture_frame_bytes = 0U;
	s_capture_tick_ms = 0U;
	s_stats.active_event_id = 0U;
	s_stats.queue_depth = s_queue_count;
	s_capture_failure_reported = false;
}

static bool PhotoCapture_CameraResponds(void) {
	uint8_t pid = 0U;
	OV7670_Status_t status = OV7670_ReadRegister(0x0AU, &pid);
	if ((status == OV7670_STATUS_OK) && (pid == 0x76U))
		return true;
	DiagUART_Printf(
			"[PHOTO] camera probe after capture error failed status=%s PID=0x%02X\r\n",
			OV7670_StatusString(status), (unsigned int) pid);
	return false;
}

static bool PhotoCapture_ConfigureCaptureTiming(void) {
	uint8_t com12 = 0U;
	uint8_t com10 = 0U;
	uint8_t verify = 0U;

	if (OV7670_ReadRegister(PHOTO_CAMERA_REG_COM12, &com12) != OV7670_STATUS_OK)
		return false;
	com12 |= PHOTO_CAMERA_COM12_HREF_ALWAYS;
	if (OV7670_WriteRegister(PHOTO_CAMERA_REG_COM12, com12) != OV7670_STATUS_OK)
		return false;
	if (OV7670_ReadRegister(PHOTO_CAMERA_REG_COM12, &verify) != OV7670_STATUS_OK
			|| verify != com12)
		return false;

	if (OV7670_ReadRegister(PHOTO_CAMERA_REG_COM10, &com10) != OV7670_STATUS_OK)
		return false;
	com10 |= PHOTO_CAMERA_COM10_PCLK_HBLANK;
	if (OV7670_WriteRegister(PHOTO_CAMERA_REG_COM10, com10) != OV7670_STATUS_OK)
		return false;
	if (OV7670_ReadRegister(PHOTO_CAMERA_REG_COM10, &verify) != OV7670_STATUS_OK
			|| verify != com10)
		return false;
	return true;
}

static bool PhotoCapture_RequestOutcome(uint32_t event_id, uint8_t outcome,
		uint8_t reason) {
	IncidentJournalStatus_t status = IncidentJournal_RequestPhotoOutcome(
			event_id, outcome, reason);
	if (status == INCIDENT_JOURNAL_OK)
		return true;
	if (status == INCIDENT_JOURNAL_NOT_FOUND)
		return true;
	if (status == INCIDENT_JOURNAL_BUSY || status == INCIDENT_JOURNAL_NOT_READY)
		return false;
	DiagUART_Printf(
			"[PHOTO] could not persist STORED outcome event_id=%lu journal_status=%u; photo data remains committed\r\n",
			(unsigned long) event_id, (unsigned int) status);
	return true;
}

static bool PhotoCapture_CommitPendingOutcome(void) {
	IncidentJournalStatus_t status;
	if (!s_outcome_waiting)
		return true;
	if (!s_outcome_submitted) {
		status = IncidentJournal_RequestPhotoOutcome(s_outcome_event_id,
		INCIDENT_PHOTO_OUTCOME_NO_PHOTO, (uint8_t) s_outcome_reason);
		if (status == INCIDENT_JOURNAL_BUSY
				|| status == INCIDENT_JOURNAL_NOT_READY)
			return false;
		if (status == INCIDENT_JOURNAL_NOT_FOUND) {
			/* The parent event was already evicted; no outcome needs to survive. */
		} else if (status != INCIDENT_JOURNAL_OK) {
			DiagUART_Printf(
					"[PHOTO] could not persist no-photo outcome event_id=%lu journal_status=%u; event remains pending\r\n",
					(unsigned long) s_outcome_event_id, (unsigned int) status);
			s_outcome_waiting = false;
			s_outcome_submitted = false;
			s_outcome_event_id = 0U;
			s_outcome_reason_text = NULL;
			PhotoCapture_PopEvent();
			return true;
		} else {
			s_outcome_submitted = true;
			return false;
		}
	} else if (IncidentJournal_IsPhotoOutcomePending(s_outcome_event_id)) {
		return false;
	}

	switch (s_outcome_reason) {
	case INCIDENT_PHOTO_REASON_COOLDOWN:
		++s_stats.skipped_cooldown;
		break;
	case INCIDENT_PHOTO_REASON_CAMERA_UNAVAILABLE:
		++s_stats.skipped_camera_unavailable;
		break;
	case INCIDENT_PHOTO_REASON_CAPTURE_FAILED:
		++s_stats.capture_failures;
		break;
	case INCIDENT_PHOTO_REASON_STORAGE_FULL:
	case INCIDENT_PHOTO_REASON_STORAGE_ERROR:
		++s_stats.skipped_store_error;
		break;
	case INCIDENT_PHOTO_REASON_QUEUE_FULL:
		++s_stats.queue_dropped;
		break;
	default:
		break;
	}
	DiagUART_Printf("[PHOTO] event_id=%lu has no photo: %s\r\n",
			(unsigned long) s_outcome_event_id,
			(s_outcome_reason_text != NULL) ?
					s_outcome_reason_text : "reason unavailable");
	s_outcome_waiting = false;
	s_outcome_submitted = false;
	s_outcome_event_id = 0U;
	s_outcome_reason_text = NULL;
	PhotoCapture_PopEvent();
	return true;
}

static void PhotoCapture_FinishWithoutPhoto(uint32_t event_id,
		IncidentPhotoReason_t reason, const char *reason_text) {
	if (!s_outcome_waiting) {
		s_outcome_waiting = true;
		s_outcome_submitted = false;
		s_outcome_event_id = event_id;
		s_outcome_reason = reason;
		s_outcome_reason_text = reason_text;
	}
	(void) PhotoCapture_CommitPendingOutcome();
}

static void PhotoCapture_QueueDeferredOutcome(uint32_t event_id,
		IncidentPhotoReason_t reason) {
	uint8_t tail;
	if (s_outcome_backlog_count >= PHOTO_OUTCOME_BACKLOG_CAPACITY) {
		DiagUART_Printf(
				"[PHOTO] outcome backlog full; event_id=%lu remains pending in journal\r\n",
				(unsigned long) event_id);
		return;
	}
	tail = (uint8_t) ((s_outcome_backlog_head + s_outcome_backlog_count)
			% PHOTO_OUTCOME_BACKLOG_CAPACITY);
	s_outcome_backlog[tail].event_id = event_id;
	s_outcome_backlog[tail].reason = reason;
	++s_outcome_backlog_count;
}

static bool PhotoCapture_ProcessDeferredOutcome(void) {
	PhotoOutcomeBacklogEntry_t *entry;
	IncidentJournalStatus_t status;
	if (s_outcome_backlog_count == 0U)
		return true;
	entry = &s_outcome_backlog[s_outcome_backlog_head];
	if (!s_outcome_backlog_submitted) {
		status = IncidentJournal_RequestPhotoOutcome(entry->event_id,
		INCIDENT_PHOTO_OUTCOME_NO_PHOTO, (uint8_t) entry->reason);
		if (status == INCIDENT_JOURNAL_BUSY
				|| status == INCIDENT_JOURNAL_NOT_READY)
			return false;
		if (status == INCIDENT_JOURNAL_NOT_FOUND) {
			/* Parent event was evicted before the outcome could be persisted. */
		} else if (status != INCIDENT_JOURNAL_OK) {
			DiagUART_Printf(
					"[PHOTO] could not persist deferred outcome for event_id=%lu status=%u; event remains pending\r\n",
					(unsigned long) entry->event_id, (unsigned int) status);
			s_outcome_backlog_head = (uint8_t) ((s_outcome_backlog_head + 1U)
					% PHOTO_OUTCOME_BACKLOG_CAPACITY);
			--s_outcome_backlog_count;
			s_outcome_backlog_submitted = false;
			return true;
		} else {
			s_outcome_backlog_submitted = true;
			return false;
		}
	} else if (IncidentJournal_IsPhotoOutcomePending(entry->event_id)) {
		return false;
	}
	DiagUART_Printf("[PHOTO] event_id=%lu has no photo: %s\r\n",
			(unsigned long) entry->event_id,
			(entry->reason == INCIDENT_PHOTO_REASON_POLICY) ?
					"event type is outside the active mode photo policy" :
			(entry->reason == INCIDENT_PHOTO_REASON_QUEUE_FULL) ?
					"photo request queue is full" : "photo was not created");
	s_outcome_backlog_head = (uint8_t) ((s_outcome_backlog_head + 1U)
			% PHOTO_OUTCOME_BACKLOG_CAPACITY);
	--s_outcome_backlog_count;
	s_outcome_backlog_submitted = false;
	return true;
}

static void PhotoCapture_ObserveJournalCommit(void) {
	IncidentJournalRecord_t record;
	uint32_t generation = 0U;
	if (!IncidentJournal_GetLastCommittedRecord(&record, &generation))
		return;
	if (generation == s_last_commit_generation)
		return;
	s_last_commit_generation = generation;
	++s_stats.events_observed;

	/* The photo policy covers every incident type generated by either mode. */
	if (((record.mode == CONFIG_MODE_CARGO)
			&& (record.type != INCIDENT_TYPE_SHOCK)
			&& (record.type != INCIDENT_TYPE_TILT)
			&& (record.type != INCIDENT_TYPE_TEMPERATURE))
			|| ((record.mode == CONFIG_MODE_SITE)
					&& (record.type != INCIDENT_TYPE_VIBRATION)
					&& (record.type != INCIDENT_TYPE_TEMPERATURE))) {
		PhotoCapture_QueueDeferredOutcome(record.sequence,
				INCIDENT_PHOTO_REASON_POLICY);
		return;
	}

	if (s_queue_count >= PHOTO_CAPTURE_QUEUE_CAPACITY) {
		++s_stats.queue_dropped;
		PhotoCapture_QueueDeferredOutcome(record.sequence,
				INCIDENT_PHOTO_REASON_QUEUE_FULL);
		return;
	}

	s_queue[(s_queue_head + s_queue_count) % PHOTO_CAPTURE_QUEUE_CAPACITY] =
			record;
	++s_queue_count;
	s_stats.queue_depth = s_queue_count;
}

static bool PhotoCapture_CooldownActive(const IncidentJournalRecord_t *event) {
	uint8_t type;
	if (event == NULL)
		return false;
	type = event->type;
	if (type >= PHOTO_CAPTURE_TYPE_COUNT || !s_last_photo_tick_valid[type])
		return false;
	return ((uint32_t) (event->timestamp_ms - s_last_photo_event_tick[type])
			< PHOTO_CAPTURE_COOLDOWN_MS)
			|| ((uint32_t) (HAL_GetTick() - s_last_photo_commit_tick[type])
					< PHOTO_CAPTURE_COOLDOWN_MS);
}

bool PhotoCaptureService_Init(void) {
	OV7670_Config_t config;
	OV7670_Status_t camera_status;

	memset(&s_stats, 0, sizeof(s_stats));
	memset(&s_camera_info, 0, sizeof(s_camera_info));
	memset(&s_capture_report, 0, sizeof(s_capture_report));
	memset(s_last_photo_event_tick, 0, sizeof(s_last_photo_event_tick));
	memset(s_last_photo_commit_tick, 0, sizeof(s_last_photo_commit_tick));
	memset(s_last_photo_tick_valid, 0, sizeof(s_last_photo_tick_valid));
	memset(s_queue, 0, sizeof(s_queue));
	s_queue_head = 0U;
	s_queue_count = 0U;
	s_last_commit_generation = 0U;
	s_state = PCS_IDLE;
	s_capture_frame = NULL;
	s_capture_frame_bytes = 0U;
	s_capture_tick_ms = 0U;
	s_outcome_waiting = false;
	s_outcome_submitted = false;
	s_outcome_event_id = 0U;
	s_outcome_reason = INCIDENT_PHOTO_REASON_NONE;
	s_outcome_reason_text = NULL;
	s_capture_failure_reported = false;
	memset(s_outcome_backlog, 0, sizeof(s_outcome_backlog));
	s_outcome_backlog_head = 0U;
	s_outcome_backlog_count = 0U;
	s_outcome_backlog_submitted = false;
	s_initialized = true;

	OV7670_GetDefaultConfig(&config);
	camera_status = OV7670_Init(&htim1, TIM_CHANNEL_1, &config, &s_camera_info);
	if (camera_status != OV7670_STATUS_OK) {
		s_stats.camera_available = false;
		DiagUART_Printf(
				"[PHOTO] camera initialization failed (%s); events will be recorded without photos\r\n",
				OV7670_StatusString(camera_status));
		return false;
	}
	if ((s_camera_info.width == 0U) || (s_camera_info.height == 0U)
			|| (s_camera_info.bytes_per_pixel == 0U)
			|| (s_camera_info.frame_size_bytes == 0U)) {
		(void) OV7670_DeInit();
		s_stats.camera_available = false;
		DiagUART_Printf(
				"[PHOTO] camera returned invalid frame geometry; events will be recorded without photos\r\n");
		return false;
	}
	if (!PhotoCapture_ConfigureCaptureTiming()) {
		(void) OV7670_DeInit();
		s_stats.camera_available = false;
		DiagUART_Printf(
				"[PHOTO] camera DVP timing setup failed; events will be recorded without photos\r\n");
		return false;
	}
	s_stats.camera_available = true;
	DiagUART_Printf(
			"[PHOTO] camera ready mode=%ux%u RGB565 bytes=%lu; line service uses HREF interrupt\r\n",
			(unsigned int) s_camera_info.width,
			(unsigned int) s_camera_info.height,
			(unsigned long) s_camera_info.frame_size_bytes);
	return true;
}

void PhotoCaptureService_Process(void) {
	IncidentJournalRecord_t *event;
	PhotoStore_Metadata_t existing;
	PhotoStore_Status_t store_status;

	if (!s_initialized)
		return;
	PhotoCapture_ObserveJournalCommit();
	if (s_outcome_waiting) {
		(void) PhotoCapture_CommitPendingOutcome();
		return;
	}
	if (!PhotoCapture_ProcessDeferredOutcome())
		return;
	event = PhotoCapture_HeadEvent();
	if (event == NULL)
		return;
	s_stats.active_event_id = event->sequence;

	if ((PhotoStore_GetState() == PHOTO_STORE_STATE_ERROR)
			|| (PhotoStore_GetState() == PHOTO_STORE_STATE_UNINITIALIZED)) {
		if (s_state != PCS_WRITING) {
			PhotoCapture_FinishWithoutPhoto(event->sequence,
					INCIDENT_PHOTO_REASON_STORAGE_ERROR,
					"photo storage is unavailable; the event remains in the journal");
		}
		return;
	}

	if (s_state == PCS_WRITING) {
		if (PhotoStore_GetState() == PHOTO_STORE_STATE_READY) {
			if (!PhotoCapture_RequestOutcome(event->sequence,
			INCIDENT_PHOTO_OUTCOME_STORED,
					(uint8_t) INCIDENT_PHOTO_REASON_NONE))
				return;
			if (event->type < PHOTO_CAPTURE_TYPE_COUNT) {
				s_last_photo_event_tick[event->type] = event->timestamp_ms;
				s_last_photo_commit_tick[event->type] = HAL_GetTick();
				s_last_photo_tick_valid[event->type] = true;
			}
			++s_stats.photos_stored;
			DiagUART_Printf(
					"[PHOTO] stored event_id=%lu bytes=%lu CRC/readback verified\r\n",
					(unsigned long) event->sequence,
					(unsigned long) s_capture_frame_bytes);
			PhotoCapture_PopEvent();
		} else if (PhotoStore_GetState() == PHOTO_STORE_STATE_ERROR) {
			PhotoCapture_FinishWithoutPhoto(event->sequence,
					INCIDENT_PHOTO_REASON_STORAGE_ERROR,
					"Flash write or read-back verification failed; event remains in the journal");
		}
		return;
	}

	if (s_state == PCS_CAPTURING) {
		OV7670_DvpCaptureStatus_t capture_result;
		if (OV7670_DvpCapture_IsBusy() != 0U)
			return;
		capture_result = OV7670_DvpCapture_GetResult();
		if (capture_result != OV7670_DVP_CAPTURE_OK) {
			if (!s_capture_failure_reported) {
				s_capture_failure_reported = true;
				DiagUART_Printf(
						"[PHOTO] capture failed event_id=%lu status=%s href=%u active_rows=%u/%u samples=%u..%u sample_errors=%u\r\n",
						(unsigned long) event->sequence,
						OV7670_DvpCapture_StatusString(capture_result),
						(unsigned int) s_capture_report.total_href_lines,
						(unsigned int) s_capture_report.active_rows_captured,
						(unsigned int) s_camera_info.height,
						(unsigned int) s_capture_report.active_line_samples_min,
						(unsigned int) s_capture_report.active_line_samples_max,
						(unsigned int) s_capture_report.active_line_sample_errors);
				DiagUART_Printf(
						"[PHOTO] DMA fault=%u state=%u error=0x%08lX NDTR=%lu CR=0x%08lX FCR=0x%08lX\r\n",
						(unsigned int) s_capture_report.dma_fault_kind,
						(unsigned int) s_capture_report.dma_state_at_fault,
						(unsigned long) s_capture_report.dma_error_code,
						(unsigned long) s_capture_report.dma_fault_ndtr,
						(unsigned long) s_capture_report.dma_fault_cr,
						(unsigned long) s_capture_report.dma_fault_fcr);
				DiagUART_Printf(
						"[PHOTO] DMA PAR=0x%08lX M0AR=0x%08lX starts=%u complete=%u errors=%u abort_race=%u HREF_service_max_us=%lu overrun_us=%lu\r\n",
						(unsigned long) s_capture_report.dma_fault_par,
						(unsigned long) s_capture_report.dma_fault_m0ar,
						(unsigned int) s_capture_report.dma_start_count,
						(unsigned int) s_capture_report.dma_complete_count,
						(unsigned int) s_capture_report.dma_error_count,
						(unsigned int) s_capture_report.dma_abort_race_recovered,
						(unsigned long) s_capture_report.href_pending_service_max_us,
						(unsigned long) s_capture_report.href_pending_overrun_us);
			}
			if (!PhotoCapture_CameraResponds()) {
				s_stats.camera_available = false;
				PhotoCapture_FinishWithoutPhoto(event->sequence,
						INCIDENT_PHOTO_REASON_CAMERA_UNAVAILABLE,
						"OV7670 stopped responding; event logging continues in degraded mode");
			} else {
				PhotoCapture_FinishWithoutPhoto(event->sequence,
						INCIDENT_PHOTO_REASON_CAPTURE_FAILED,
						"camera frame capture failed; event is preserved without a photo");
			}
			return;
		}
		s_capture_frame = OV7670_DvpCapture_GetFrameData(
				&s_capture_frame_bytes);
		s_capture_tick_ms = HAL_GetTick();
		if ((s_capture_frame == NULL)
				|| (s_capture_frame_bytes != s_camera_info.frame_size_bytes)) {
			PhotoCapture_FinishWithoutPhoto(event->sequence,
					INCIDENT_PHOTO_REASON_CAPTURE_FAILED,
					"captured frame size does not match camera metadata");
			return;
		}
		s_state = PCS_CAPTURE_DONE;
	}

	if (s_state == PCS_CAPTURE_DONE) {
		IncidentJournalRecord_t retained_event;
		IncidentJournalStatus_t journal_status;
		if (PhotoStore_GetState() != PHOTO_STORE_STATE_READY)
			return;
		journal_status = IncidentJournal_ReadBySequence(event->sequence,
				&retained_event);
		if (journal_status == INCIDENT_JOURNAL_BUSY)
			return;
		if (journal_status != INCIDENT_JOURNAL_OK) {
			++s_stats.skipped_store_error;
			DiagUART_Printf(
					"[PHOTO] parent event_id=%lu was evicted before photo commit; captured frame discarded\r\n",
					(unsigned long) event->sequence);
			PhotoCapture_PopEvent();
			return;
		}
		store_status = PhotoStore_BeginWrite(event->sequence, s_capture_tick_ms,
				s_camera_info.width, s_camera_info.height,
				PHOTO_STORE_PIXEL_FORMAT_RGB565, s_camera_info.bytes_per_pixel,
				s_capture_frame, s_capture_frame_bytes);
		if (store_status == PHOTO_STORE_STATUS_BUSY
				|| store_status == PHOTO_STORE_STATUS_NOT_READY)
			return;
		if (store_status != PHOTO_STORE_STATUS_OK) {
			PhotoCapture_FinishWithoutPhoto(event->sequence,
					(store_status == PHOTO_STORE_STATUS_NO_SPACE) ?
							INCIDENT_PHOTO_REASON_STORAGE_FULL :
							INCIDENT_PHOTO_REASON_STORAGE_ERROR,
					(store_status == PHOTO_STORE_STATUS_NO_SPACE) ?
							"Flash photo partition has no free extent" :
							"photo storage could not start the write transaction");
			return;
		}
		s_state = PCS_WRITING;
		return;
	}

	if (s_state != PCS_IDLE)
		return;

	/* Resolve an already committed image before applying cooldown/camera
	 * checks. A duplicate request must never downgrade an existing photo to
	 * a no-photo outcome. */
	if (PhotoStore_GetState() == PHOTO_STORE_STATE_READY) {
		store_status = PhotoStore_Find(event->sequence, &existing);
		if (store_status == PHOTO_STORE_STATUS_BUSY)
			return;
		if (store_status == PHOTO_STORE_STATUS_OK) {
			if (!PhotoCapture_RequestOutcome(event->sequence,
			INCIDENT_PHOTO_OUTCOME_STORED,
					(uint8_t) INCIDENT_PHOTO_REASON_NONE))
				return;
			DiagUART_Printf(
					"[PHOTO] existing committed photo found for event_id=%lu; duplicate capture suppressed\r\n",
					(unsigned long) event->sequence);
			PhotoCapture_PopEvent();
			return;
		}
		if (store_status != PHOTO_STORE_STATUS_NOT_FOUND) {
			PhotoCapture_FinishWithoutPhoto(event->sequence,
					INCIDENT_PHOTO_REASON_STORAGE_ERROR,
					"photo index lookup failed; event remains in the journal");
			return;
		}
	}

	if (!s_stats.camera_available || OV7670_IsInitialized() == 0U) {
		PhotoCapture_FinishWithoutPhoto(event->sequence,
				INCIDENT_PHOTO_REASON_CAMERA_UNAVAILABLE,
				"OV7670 camera is unavailable; event recording continues in degraded mode");
		return;
	}

	if (PhotoCapture_CooldownActive(event)) {
		PhotoCapture_FinishWithoutPhoto(event->sequence,
				INCIDENT_PHOTO_REASON_COOLDOWN,
				"same incident type was photographed less than 3 seconds ago (cooldown)");
		return;
	}

	memset(&s_capture_report, 0, sizeof(s_capture_report));
	{
		OV7670_DvpCaptureStatus_t capture_status = OV7670_DvpCapture_Start(
				&s_capture_report);
		if (capture_status == OV7670_DVP_CAPTURE_BUSY)
			return;
		if (capture_status != OV7670_DVP_CAPTURE_OK) {
			PhotoCapture_FinishWithoutPhoto(event->sequence,
					INCIDENT_PHOTO_REASON_CAPTURE_FAILED,
					"camera capture engine could not be started");
			return;
		}
	}
	s_state = PCS_CAPTURING;
	DiagUART_Printf("[PHOTO] capture started event_id=%lu\r\n",
			(unsigned long) event->sequence);
}

void PhotoCaptureService_GetStats(PhotoCaptureService_Stats_t *stats) {
	if (stats != NULL) {
		*stats = s_stats;
		stats->queue_depth = s_queue_count;
		stats->active_event_id =
				(s_queue_count != 0U) ? s_queue[s_queue_head].sequence : 0U;
		stats->camera_width = s_camera_info.width;
		stats->camera_height = s_camera_info.height;
		stats->frame_bytes = s_camera_info.frame_size_bytes;
	}
}

bool PhotoCaptureService_IsBusy(void) {
	return (s_state != PCS_IDLE) || (s_queue_count != 0U);
}

void PhotoCaptureService_ResetAfterMediaErase(void) {
	bool camera_available = s_stats.camera_available
			&& (OV7670_IsInitialized() != 0U);
	memset(s_queue, 0, sizeof(s_queue));
	memset(s_last_photo_event_tick, 0, sizeof(s_last_photo_event_tick));
	memset(s_last_photo_commit_tick, 0, sizeof(s_last_photo_commit_tick));
	memset(s_last_photo_tick_valid, 0, sizeof(s_last_photo_tick_valid));
	s_queue_head = 0U;
	s_queue_count = 0U;
	s_last_commit_generation = 0U;
	s_state = PCS_IDLE;
	s_capture_frame = NULL;
	s_capture_frame_bytes = 0U;
	s_capture_tick_ms = 0U;
	s_outcome_waiting = false;
	s_outcome_submitted = false;
	s_outcome_event_id = 0U;
	s_outcome_reason = INCIDENT_PHOTO_REASON_NONE;
	s_outcome_reason_text = NULL;
	s_capture_failure_reported = false;
	memset(s_outcome_backlog, 0, sizeof(s_outcome_backlog));
	s_outcome_backlog_head = 0U;
	s_outcome_backlog_count = 0U;
	s_outcome_backlog_submitted = false;
	memset(&s_stats, 0, sizeof(s_stats));
	/* Media erase resets photo counters/index, not the physical camera. */
	s_stats.camera_available = camera_available;
}
