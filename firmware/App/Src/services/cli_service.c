#include "services/cli_service.h"
#include "services/config_service.h"
#include "services/storage_maintenance.h"
#include "services/incident_detector.h"
#include "services/incident_journal.h"
#include "services/photo_store.h"
#include "services/photo_capture_service.h"
#include "services/led_service.h"
#include "app/app.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#define CLI_RX_RING_SIZE 128U
#define CLI_TX_RING_SIZE 8192U
#define CLI_TX_COMMAND_RESERVE 7800U
#define CLI_JOURNAL_SCAN_BUDGET 8U
#define CLI_JOURNAL_LINE_SIZE 192U
#define CLI_PHOTO_CHUNK_SIZE 256U
#define CLI_PHOTO_CHUNK_ACK_TIMEOUT_MS 1500U
#define CLI_PHOTO_CHUNK_MAX_RETRIES 3U
#define CLI_PHOTO_ACK_BYTE 0x06U
#define CLI_PHOTO_NACK_BYTE 0x15U
#define CLI_PHOTO_LIST_LINE_SIZE 128U

#define CLI_LINE_SIZE 80U
#define CLI_PROCESS_BUDGET 64U
#define CLI_TX_FORMAT_SIZE 256U
#if ((CLI_RX_RING_SIZE & (CLI_RX_RING_SIZE - 1U)) != 0U)
#error "CLI_RX_RING_SIZE must be a power of two"
#endif
#if ((CLI_TX_RING_SIZE & (CLI_TX_RING_SIZE - 1U)) != 0U)
#error "CLI_TX_RING_SIZE must be a power of two"
#endif
typedef struct {
	uint8_t data[CLI_RX_RING_SIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
} CliRxRing_t;
typedef struct {
	uint8_t data[CLI_TX_RING_SIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint16_t active_length;
	volatile uint8_t transfer_active;
} CliTxRing_t;
static CliRxRing_t s_rx;
static CliTxRing_t s_tx;
static uint8_t s_rx_byte;
static char s_line[CLI_LINE_SIZE];
static uint16_t s_line_length;
static uint8_t s_discard_line;
static uint8_t s_last_was_cr;
static volatile uint32_t s_rx_overflow_count;
static volatile uint32_t s_tx_drop_count;
static volatile uint32_t s_tx_queue_overflow_count;
static volatile uint32_t s_tx_start_error_count;
static volatile uint32_t s_rx_restart_error_count;
static uint8_t s_initialized;
static ConfigSaveStatus_t s_last_config_save_status;
typedef enum {
	CLI_ERASE_SCOPE_NONE = 0,
	CLI_ERASE_SCOPE_CONFIG,
	CLI_ERASE_SCOPE_MEDIA,
	CLI_ERASE_SCOPE_ALL
} CliEraseScope_t;
static CliEraseScope_t s_pending_erase_scope;
static uint32_t s_erase_confirmation_deadline;
static uint8_t s_erase_operation_active;
static uint8_t s_erase_waiting_config_reset;
static uint8_t s_factory_reset_pending;
static uint8_t s_confirmation_active;
static uint8_t s_maintenance_lock;
static uint16_t s_last_erase_progress_report;

typedef enum {
	CLI_JOURNAL_EXPORT_IDLE = 0,
	CLI_JOURNAL_EXPORT_COUNT,
	CLI_JOURNAL_EXPORT_STREAM
} CliJournalExportState_t;
static CliJournalExportState_t s_journal_export_state;
static volatile uint8_t s_journal_export_active;
static IncidentJournalReadAllCursor_t s_journal_cursor;
static uint32_t s_journal_export_total;
static uint32_t s_journal_export_sent;
static uint32_t s_journal_export_first_id;
static uint32_t s_journal_export_last_id;

typedef enum {
	CLI_PHOTO_TX_IDLE = 0,
	CLI_PHOTO_TX_WAIT_HEADER,
	CLI_PHOTO_TX_PREPARE_CHUNK,
	CLI_PHOTO_TX_WAIT_CHUNK_HEADER,
	CLI_PHOTO_TX_WAIT_CHUNK_DATA,
	CLI_PHOTO_TX_WAIT_CHUNK_ACK,
	CLI_PHOTO_TX_DRAIN,
	CLI_PHOTO_TX_WAIT_END,
	CLI_PHOTO_TX_ALL_NEXT,
	CLI_PHOTO_TX_ABORT_DRAIN,
	CLI_PHOTO_TX_ABORT_END,
	CLI_PHOTO_TX_ABORT_ALL_END
} CliPhotoTransferState_t;
static CliPhotoTransferState_t s_photo_tx_state;
static PhotoStore_Metadata_t s_photo_tx_metadata;
static uint8_t s_photo_tx_buffer[CLI_PHOTO_CHUNK_SIZE];
static uint32_t s_photo_tx_offset;
static uint32_t s_photo_tx_crc;
static uint32_t s_photo_chunk_crc;
static uint16_t s_photo_chunk_length;
static volatile uint8_t s_photo_chunk_ack_received;
static volatile uint8_t s_photo_chunk_nack_received;
static uint8_t s_photo_chunk_retry_count;
static uint32_t s_photo_chunk_ack_deadline;
static volatile bool s_photo_tx_active;
static volatile uint8_t s_photo_abort_requested;
static bool s_photo_tx_internal_write;
static bool s_photo_tx_failed;
static bool s_photo_tx_abort_after_chunk;
static bool s_photo_tx_all_mode;
static bool s_photo_tx_read_lock;
static bool s_photo_tx_current_active;
static uint16_t s_photo_all_index;
static uint16_t s_photo_all_count;
static uint16_t s_photo_all_sent;
static uint32_t s_photo_tx_start_tick;
static uint32_t s_photo_tx_timeout_ms;
static char s_photo_deferred_text[512];
static uint16_t s_photo_deferred_length;

typedef enum {
	CLI_PHOTO_LIST_IDLE = 0,
	CLI_PHOTO_LIST_WAIT_BEGIN,
	CLI_PHOTO_LIST_STREAM,
	CLI_PHOTO_LIST_WAIT_END,
	CLI_PHOTO_LIST_ABORT_DRAIN,
	CLI_PHOTO_LIST_WAIT_ABORT_END
} CliPhotoListState_t;
static CliPhotoListState_t s_photo_list_state;
static volatile bool s_photo_list_active;
static volatile uint8_t s_photo_list_abort_requested;
static bool s_photo_list_read_lock;
static uint16_t s_photo_list_index;
static uint16_t s_photo_list_count;
static uint16_t s_photo_list_sent;
static uint16_t s_photo_list_skipped;
static void Cli_TxKick(void);
static void Cli_Printf(const char *format, ...);
static void Cli_SendPrompt(void);
static void Cli_ExecuteLine(char *line);
static void Cli_HandleErase(char *arguments);
static void Cli_HandleJournal(char *arguments);
static void Cli_ProcessJournalExport(void);
static void Cli_HandleLed(char *arguments);
static void Cli_HandlePhoto(char *arguments);
static void Cli_ProcessPhotoTransfer(void);
static void Cli_ProcessPhotoList(void);
static void Cli_FinishPhotoList(void);
static void Cli_StartPhotoList(void);
static void Cli_FinishPhotoTransfer(void);
static void Cli_PhotoAllNext(void);
static bool Cli_TxWriteRaw(const uint8_t *data, uint16_t length);
static bool Cli_TxIsIdle(void);
static void Cli_FlushPhotoDeferredText(void);
static void Cli_ReinitializeJournal(void);
static void Cli_HandleConfirmationAnswer(char *answer);
static void Cli_StartConfirmedErase(void);
static void Cli_FinishMaintenance(const char *message);
static void Cli_PrintProgress(const char *message);
static uint8_t Cli_IsPasswordCharacter(void);
static void Cli_AsyncNotice(const char *message);
static uint8_t Cli_IsDestructiveOperationPending(void);
static const char* Cli_ModeName(ConfigMode_t mode);
static const char* Cli_SaveName(ConfigSaveStatus_t status);
static uint16_t Cli_NextIndex(uint16_t index, uint16_t size) {

	return (uint16_t) ((index + 1U) & (size - 1U));
}

/*
 * Atomically enter a binary/machine-protocol RX-exclusive operation.
 * Bytes queued before the command was accepted (for example, a second
 * command sent immediately after the first one) must not be replayed after
 * the protocol finishes. While the active flag is set, the RX ISR discards
 * all input except Ctrl+C.
 */
static void Cli_EnterPhotoProtocolMode(bool photo_list_mode) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	s_rx.tail = s_rx.head;
	s_line_length = 0U;
	s_line[0] = '\0';
	s_discard_line = 0U;
	s_last_was_cr = 0U;
	if (photo_list_mode) {
		s_photo_list_active = true;
	} else {
		s_photo_tx_active = true;
	}
	__set_PRIMASK(primask);
}
static void Cli_TxWrite(const char *text) {
	uint16_t head;
	uint16_t tail;
	uint16_t free_space;
	uint16_t index;
	size_t length;
	size_t i;
	if (text == NULL) {
		return;
	}
	if ((s_photo_tx_active || s_photo_list_active)
			&& !s_photo_tx_internal_write) {
		length = strlen(text);
		if ((length + s_photo_deferred_length)
				< sizeof(s_photo_deferred_text)) {
			memcpy(&s_photo_deferred_text[s_photo_deferred_length], text,
					length);
			s_photo_deferred_length = (uint16_t) (s_photo_deferred_length
					+ length);
			s_photo_deferred_text[s_photo_deferred_length] = '\0';
		} else {
			s_tx_drop_count++;
			s_tx_queue_overflow_count++;
		}
		return;
	}
	length = strlen(text);
	if (length == 0U) {
		return;
	}
	if (length >= CLI_TX_RING_SIZE) {
		s_tx_drop_count++;
		s_tx_queue_overflow_count++;
		return;
	}
	head = s_tx.head;
	tail = s_tx.tail;
	free_space = (uint16_t) ((tail - head - 1U) & (CLI_TX_RING_SIZE - 1U));
	if (length > free_space) {
		s_tx_drop_count++;
		s_tx_queue_overflow_count++;
		return;
	}
	/* Publish the new head only after the complete message is copied. */
	index = head;
	for (i = 0U; i < length; i++) {
		s_tx.data[index] = (uint8_t) text[i];
		index = Cli_NextIndex(index, CLI_TX_RING_SIZE);
	}
	s_tx.head = index;
	Cli_TxKick();
}
static uint16_t Cli_TxFreeSpace(void) {
	uint16_t head = s_tx.head;
	uint16_t tail = s_tx.tail;
	return (uint16_t) ((tail - head - 1U) & (CLI_TX_RING_SIZE - 1U));
}

static bool Cli_TxWriteRaw(const uint8_t *data, uint16_t length) {
	uint16_t head;
	uint16_t index;
	uint16_t i;
	if ((data == NULL) || (length == 0U) || (length > Cli_TxFreeSpace()))
		return false;
	head = s_tx.head;
	index = head;
	for (i = 0U; i < length; ++i) {
		s_tx.data[index] = data[i];
		index = Cli_NextIndex(index, CLI_TX_RING_SIZE);
	}
	s_tx.head = index;
	return true;
}

static bool Cli_TxIsIdle(void) {
	return (s_tx.head == s_tx.tail) && (s_tx.transfer_active == 0U);
}

static void Cli_FlushPhotoDeferredText(void) {
	char deferred[sizeof(s_photo_deferred_text)];
	if (s_photo_deferred_length == 0U)
		return;
	memcpy(deferred, s_photo_deferred_text, s_photo_deferred_length);
	deferred[s_photo_deferred_length] = '\0';
	s_photo_deferred_length = 0U;
	s_photo_deferred_text[0] = '\0';
	Cli_TxWrite(deferred);
}

static uint32_t Cli_Crc32Update(uint32_t crc, const uint8_t *data,
		uint32_t length) {
	uint32_t i;
	uint8_t bit;
	for (i = 0U; i < length; ++i) {
		crc ^= data[i];
		for (bit = 0U; bit < 8U; ++bit)
			crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
	}
	return crc;
}

static void Cli_CommitPhotoChunk(void) {
	s_photo_tx_crc = Cli_Crc32Update(s_photo_tx_crc, s_photo_tx_buffer,
			s_photo_chunk_length);
	s_photo_tx_offset += s_photo_chunk_length;
	s_photo_chunk_ack_received = 0U;
	s_photo_chunk_nack_received = 0U;
	s_photo_chunk_retry_count = 0U;
}

static void Cli_FinishPhotoTransfer(void) {
	if (s_photo_tx_read_lock) {
		PhotoStore_EndReadSession();
		s_photo_tx_read_lock = false;
	}
	s_photo_tx_active = false;
	s_photo_tx_state = CLI_PHOTO_TX_IDLE;
	s_photo_tx_all_mode = false;
	s_photo_tx_current_active = false;
	s_photo_abort_requested = 0U;
	s_photo_tx_abort_after_chunk = false;
	s_photo_chunk_ack_received = 0U;
	s_photo_chunk_nack_received = 0U;
	s_photo_chunk_retry_count = 0U;
	s_photo_tx_internal_write = true;
	Cli_FlushPhotoDeferredText();
	s_photo_tx_internal_write = false;
}

static uint32_t Cli_PhotoTransferTimeoutMs(uint32_t frame_bytes) {
	/* 115200 8-N-1, conservative throughput plus scheduling margin. */
	uint32_t timeout_ms = 15000U + (frame_bytes / 8U);
	if (timeout_ms < 15000U)
		timeout_ms = 15000U;
	return timeout_ms;
}

static void Cli_PhotoQueueAbortTrailer(void) {
	s_photo_tx_internal_write = true;
	if (s_photo_tx_current_active) {
		Cli_Printf("IRPHOTO_ABORT,%lu,%lu,USER\r\n",
				(unsigned long) s_photo_tx_metadata.event_id,
				(unsigned long) s_photo_tx_offset);
		s_photo_tx_current_active = false;
		s_photo_tx_state = CLI_PHOTO_TX_ABORT_END;
	} else if (s_photo_tx_all_mode) {
		Cli_Printf("IRPHOTO_ALL_ABORT,%u,%u\r\n",
				(unsigned int) s_photo_all_sent,
				(unsigned int) s_photo_all_index);
		s_photo_tx_state = CLI_PHOTO_TX_ABORT_ALL_END;
	} else {
		s_photo_tx_state = CLI_PHOTO_TX_ABORT_END;
	}
	s_photo_tx_internal_write = false;
}

static void Cli_PhotoAllNext(void) {
	uint32_t event_id = 0U;
	PhotoStore_Status_t photo_status;
	PhotoStore_Status_t index_status;

	if (s_photo_all_index >= s_photo_all_count) {
		s_photo_tx_internal_write = true;
		Cli_Printf("IRPHOTO_ALL_END,%u\r\n", (unsigned int) s_photo_all_sent);
		s_photo_tx_internal_write = false;
		s_photo_tx_all_mode = false;
		s_photo_tx_state = CLI_PHOTO_TX_WAIT_END;
		return;
	}

	/* At most one index/journal/metadata lookup per super-loop pass. */
	index_status = PhotoStore_GetEventIdAt(s_photo_all_index, &event_id);
	if (index_status == PHOTO_STORE_STATUS_BUSY
			|| index_status == PHOTO_STORE_STATUS_NOT_READY)
		return;
	++s_photo_all_index;
	if (index_status != PHOTO_STORE_STATUS_OK)
		return;

	photo_status = PhotoStore_Find(event_id, &s_photo_tx_metadata);
	if (photo_status == PHOTO_STORE_STATUS_BUSY) {
		--s_photo_all_index;
		return;
	}
	if (photo_status != PHOTO_STORE_STATUS_OK)
		return;

	s_photo_tx_offset = 0U;
	s_photo_tx_crc = 0xFFFFFFFFUL;
	s_photo_tx_failed = false;
	s_photo_tx_abort_after_chunk = false;
	s_photo_chunk_retry_count = 0U;
	s_photo_chunk_ack_received = 0U;
	s_photo_chunk_nack_received = 0U;
	s_photo_tx_current_active = true;
	s_photo_tx_start_tick = HAL_GetTick();
	s_photo_tx_timeout_ms = Cli_PhotoTransferTimeoutMs(
			s_photo_tx_metadata.frame_bytes);
	s_photo_tx_state = CLI_PHOTO_TX_WAIT_HEADER;
	s_photo_tx_internal_write = true;
	Cli_Printf("IRPHOTO4,%lu,%u,%u,%u,%u,%lu,%08lX,%u\r\n",
			(unsigned long) s_photo_tx_metadata.event_id,
			(unsigned int) s_photo_tx_metadata.width,
			(unsigned int) s_photo_tx_metadata.height,
			(unsigned int) s_photo_tx_metadata.pixel_format,
			(unsigned int) s_photo_tx_metadata.bytes_per_pixel,
			(unsigned long) s_photo_tx_metadata.frame_bytes,
			(unsigned long) s_photo_tx_metadata.payload_crc32,
			(unsigned int) CLI_PHOTO_CHUNK_SIZE);
	s_photo_tx_internal_write = false;
}

static void Cli_ProcessPhotoTransfer(void) {
	PhotoStore_Status_t status;
	uint32_t remaining;
	uint32_t chunk_crc;
	char line[96];
	int length;

	if (!s_photo_tx_active)
		return;

	if (s_photo_abort_requested != 0U) {
		s_photo_abort_requested = 0U;
		if ((s_photo_tx_state == CLI_PHOTO_TX_WAIT_CHUNK_HEADER)
				|| (s_photo_tx_state == CLI_PHOTO_TX_WAIT_CHUNK_DATA)) {
			/* A chunk header commits the receiver to exactly chunk_length bytes.
			 * Finish that chunk before emitting the textual abort trailer. */
			s_photo_tx_abort_after_chunk = true;
		} else if (s_photo_tx_state == CLI_PHOTO_TX_WAIT_CHUNK_ACK) {
			if (s_photo_chunk_ack_received != 0U)
				Cli_CommitPhotoChunk();
			s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
		} else if (s_photo_tx_state == CLI_PHOTO_TX_WAIT_END) {
			/* The current photo is already terminal; in photo-all mode stop
			 * before starting the next retained photo. */
			if (s_photo_tx_all_mode)
				s_photo_tx_abort_after_chunk = true;
		} else if (s_photo_tx_state != CLI_PHOTO_TX_ABORT_DRAIN
				&& s_photo_tx_state != CLI_PHOTO_TX_ABORT_END
				&& s_photo_tx_state != CLI_PHOTO_TX_ABORT_ALL_END) {
			s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
		}
	}

	if ((s_photo_tx_state != CLI_PHOTO_TX_ALL_NEXT)
			&& (s_photo_tx_state != CLI_PHOTO_TX_WAIT_END)
			&& (s_photo_tx_state != CLI_PHOTO_TX_ABORT_DRAIN)
			&& (s_photo_tx_state != CLI_PHOTO_TX_ABORT_END)
			&& (s_photo_tx_state != CLI_PHOTO_TX_ABORT_ALL_END)
			&& ((uint32_t) (HAL_GetTick() - s_photo_tx_start_tick)
					> s_photo_tx_timeout_ms)) {
		s_photo_tx_failed = true;
		if ((s_photo_tx_state != CLI_PHOTO_TX_WAIT_CHUNK_HEADER)
				&& (s_photo_tx_state != CLI_PHOTO_TX_WAIT_CHUNK_DATA)) {
			s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
		}
		/* If a chunk header is already on the wire, its payload is completed;
		 * WAIT_CHUNK_DATA will then move to DRAIN because tx_failed is set. */
	}

	switch (s_photo_tx_state) {
	case CLI_PHOTO_TX_ALL_NEXT:
		if (s_photo_tx_abort_after_chunk) {
			s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
			return;
		}
		Cli_PhotoAllNext();
		return;

	case CLI_PHOTO_TX_WAIT_HEADER:
		if (Cli_TxIsIdle()) {
			if (s_photo_tx_abort_after_chunk)
				s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
			else
				s_photo_tx_state = CLI_PHOTO_TX_PREPARE_CHUNK;
		}
		return;

	case CLI_PHOTO_TX_PREPARE_CHUNK:
		s_photo_chunk_ack_received = 0U;
		s_photo_chunk_nack_received = 0U;
		if (s_photo_tx_abort_after_chunk) {
			s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
			return;
		}
		if (s_photo_tx_failed) {
			s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
			return;
		}
		remaining = s_photo_tx_metadata.frame_bytes - s_photo_tx_offset;
		if (remaining == 0U) {
			s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
			return;
		}
		s_photo_chunk_length = (uint16_t) ((remaining > CLI_PHOTO_CHUNK_SIZE) ?
				CLI_PHOTO_CHUNK_SIZE : remaining);
		status = PhotoStore_ReadPayload(&s_photo_tx_metadata,
				s_photo_tx_offset, s_photo_tx_buffer, s_photo_chunk_length);
		if (status == PHOTO_STORE_STATUS_BUSY)
			return;
		if (status != PHOTO_STORE_STATUS_OK) {
			s_photo_tx_failed = true;
			s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
			return;
		}
		chunk_crc = Cli_Crc32Update(0xFFFFFFFFUL, s_photo_tx_buffer,
				s_photo_chunk_length) ^ 0xFFFFFFFFUL;
		s_photo_chunk_crc = chunk_crc;
		length = snprintf(line, sizeof(line),
				"IRPHOTO_CHUNK,%lu,%u,%08lX\r\n",
				(unsigned long) s_photo_tx_offset,
				(unsigned int) s_photo_chunk_length,
				(unsigned long) s_photo_chunk_crc);
		if ((length <= 0) || ((size_t) length >= sizeof(line))) {
			s_photo_tx_failed = true;
			s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
			return;
		}
		if (Cli_TxFreeSpace() < (uint16_t) length)
			return;
		s_photo_tx_internal_write = true;
		Cli_TxWrite(line);
		s_photo_tx_internal_write = false;
		s_photo_tx_state = CLI_PHOTO_TX_WAIT_CHUNK_HEADER;
		return;

	case CLI_PHOTO_TX_WAIT_CHUNK_HEADER:
		if (Cli_TxIsIdle()) {
			if (Cli_TxWriteRaw(s_photo_tx_buffer, s_photo_chunk_length)) {
				s_photo_tx_state = CLI_PHOTO_TX_WAIT_CHUNK_DATA;
			}
		}
		return;

	case CLI_PHOTO_TX_WAIT_CHUNK_DATA:
		if (Cli_TxIsIdle()) {
			if (s_photo_tx_failed) {
				s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
			} else if (s_photo_chunk_ack_received != 0U) {
				/* ACK can arrive before the super loop observes TX-idle. */
				Cli_CommitPhotoChunk();
				if (s_photo_tx_abort_after_chunk)
					s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
				else if (s_photo_tx_offset >= s_photo_tx_metadata.frame_bytes)
					s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
				else
					s_photo_tx_state = CLI_PHOTO_TX_PREPARE_CHUNK;
			} else if (s_photo_tx_abort_after_chunk) {
				s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
			} else {
				s_photo_chunk_ack_deadline = HAL_GetTick()
						+ CLI_PHOTO_CHUNK_ACK_TIMEOUT_MS;
				s_photo_tx_state = CLI_PHOTO_TX_WAIT_CHUNK_ACK;
			}
		}
		return;

	case CLI_PHOTO_TX_WAIT_CHUNK_ACK:
		if (s_photo_abort_requested != 0U) {
			s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
			return;
		}
		if (s_photo_chunk_ack_received != 0U) {
			Cli_CommitPhotoChunk();
			if (s_photo_tx_offset >= s_photo_tx_metadata.frame_bytes)
				s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
			else
				s_photo_tx_state = CLI_PHOTO_TX_PREPARE_CHUNK;
			return;
		}
		if ((s_photo_chunk_nack_received != 0U)
				|| ((int32_t) (HAL_GetTick() - s_photo_chunk_ack_deadline) >= 0)) {
			s_photo_chunk_nack_received = 0U;
			if (s_photo_chunk_retry_count < CLI_PHOTO_CHUNK_MAX_RETRIES) {
				++s_photo_chunk_retry_count;
				s_photo_tx_state = CLI_PHOTO_TX_PREPARE_CHUNK;
			} else {
				s_photo_tx_failed = true;
				s_photo_tx_state = CLI_PHOTO_TX_DRAIN;
			}
		}
		return;

	case CLI_PHOTO_TX_DRAIN:
		if (Cli_TxIsIdle()) {
			uint32_t crc = s_photo_tx_crc ^ 0xFFFFFFFFUL;
			s_photo_tx_internal_write = true;
			if (s_photo_tx_failed) {
				Cli_Printf("IRPHOTO_ERROR,%lu,%lu,%08lX\r\n",
						(unsigned long) s_photo_tx_metadata.event_id,
						(unsigned long) s_photo_tx_offset, (unsigned long) crc);
			} else {
				Cli_Printf("IRPHOTO_END,%lu,%lu,%08lX\r\n",
						(unsigned long) s_photo_tx_metadata.event_id,
						(unsigned long) s_photo_tx_offset, (unsigned long) crc);
				++s_photo_all_sent;
			}
			s_photo_tx_internal_write = false;
			s_photo_tx_state = CLI_PHOTO_TX_WAIT_END;
		}
		return;

	case CLI_PHOTO_TX_WAIT_END:
		if (Cli_TxIsIdle()) {
			s_photo_tx_current_active = false;
			if (s_photo_tx_all_mode) {
				if (s_photo_tx_abort_after_chunk) {
					s_photo_tx_state = CLI_PHOTO_TX_ABORT_DRAIN;
				} else if (s_photo_tx_failed) {
					s_photo_tx_internal_write = true;
					Cli_Printf("IRPHOTO_ALL_ABORT,%u,%u\r\n",
							(unsigned int) s_photo_all_sent,
							(unsigned int) s_photo_all_index);
					s_photo_tx_internal_write = false;
					s_photo_tx_state = CLI_PHOTO_TX_ABORT_ALL_END;
				} else {
					s_photo_tx_state = CLI_PHOTO_TX_ALL_NEXT;
					s_photo_tx_start_tick = HAL_GetTick();
				}
			} else {
				Cli_FinishPhotoTransfer();
			}
		}
		return;

	case CLI_PHOTO_TX_ABORT_DRAIN:
		if (Cli_TxIsIdle())
			Cli_PhotoQueueAbortTrailer();
		return;

	case CLI_PHOTO_TX_ABORT_END:
		if (Cli_TxIsIdle()) {
			if (s_photo_tx_all_mode) {
				s_photo_tx_internal_write = true;
				Cli_Printf("IRPHOTO_ALL_ABORT,%u,%u\r\n",
						(unsigned int) s_photo_all_sent,
						(unsigned int) s_photo_all_index);
				s_photo_tx_internal_write = false;
				s_photo_tx_state = CLI_PHOTO_TX_ABORT_ALL_END;
			} else {
				Cli_FinishPhotoTransfer();
			}
		}
		return;

	case CLI_PHOTO_TX_ABORT_ALL_END:
		if (Cli_TxIsIdle())
			Cli_FinishPhotoTransfer();
		return;

	case CLI_PHOTO_TX_IDLE:
	default:
		Cli_FinishPhotoTransfer();
		return;
	}
}

static void Cli_FinishPhotoList(void) {
	if (s_photo_list_read_lock) {
		PhotoStore_EndReadSession();
		s_photo_list_read_lock = false;
	}
	s_photo_list_active = false;
	s_photo_list_state = CLI_PHOTO_LIST_IDLE;
	s_photo_list_abort_requested = 0U;
	s_photo_tx_internal_write = true;
	Cli_FlushPhotoDeferredText();
	s_photo_tx_internal_write = false;
}

static void Cli_StartPhotoList(void) {
	PhotoStore_Stats_t stats;
	PhotoStore_Status_t status;
	if (App_GetState() != APP_STATE_DISARMED) {
		Cli_TxWrite("ERR: photos list is available only in DISARMED.\r\n");
		return;
	}
	if (PhotoCaptureService_IsBusy()) {
		Cli_TxWrite("ERR: photo capture/storage is active; retry later.\r\n");
		return;
	}
	PhotoStore_GetStats(&stats);
	if (stats.state != PHOTO_STORE_STATE_READY) {
		Cli_TxWrite("ERR: photo storage recovery is not complete.\r\n");
		return;
	}
	status = PhotoStore_BeginReadSession();
	if (status != PHOTO_STORE_STATUS_OK) {
		Cli_TxWrite("ERR: photo storage is busy; retry later.\r\n");
		return;
	}
	s_photo_list_read_lock = true;
	s_photo_list_index = 0U;
	s_photo_list_count = stats.index_entries;
	s_photo_list_sent = 0U;
	s_photo_list_skipped = 0U;
	s_photo_list_abort_requested = 0U;
	s_photo_list_state = CLI_PHOTO_LIST_WAIT_BEGIN;
	Cli_EnterPhotoProtocolMode(true);
	s_photo_tx_internal_write = true;
	Cli_Printf("IRPHOTOS_BEGIN,1,%u\r\n", (unsigned int) s_photo_list_count);
	s_photo_tx_internal_write = false;
}

static void Cli_ProcessPhotoList(void) {
	uint32_t event_id;
	PhotoStore_Metadata_t metadata;
	PhotoStore_Status_t status;
	char line[CLI_PHOTO_LIST_LINE_SIZE];
	int length;
	if (!s_photo_list_active)
		return;
	if (s_photo_list_abort_requested != 0U) {
		s_photo_list_abort_requested = 0U;
		if ((s_photo_list_state != CLI_PHOTO_LIST_WAIT_END)
				&& (s_photo_list_state != CLI_PHOTO_LIST_WAIT_ABORT_END))
			s_photo_list_state = CLI_PHOTO_LIST_ABORT_DRAIN;
	}
	switch (s_photo_list_state) {
	case CLI_PHOTO_LIST_WAIT_BEGIN:
		if (Cli_TxIsIdle())
			s_photo_list_state = CLI_PHOTO_LIST_STREAM;
		return;
	case CLI_PHOTO_LIST_STREAM:
		if (s_photo_list_index >= s_photo_list_count) {
			if (s_photo_list_read_lock) {
				PhotoStore_EndReadSession();
				s_photo_list_read_lock = false;
			}
			if (Cli_TxIsIdle()) {
				s_photo_tx_internal_write = true;
				Cli_Printf("IRPHOTOS_END,%u,%u\r\n",
						(unsigned int) s_photo_list_sent,
						(unsigned int) s_photo_list_skipped);
				s_photo_tx_internal_write = false;
				s_photo_list_state = CLI_PHOTO_LIST_WAIT_END;
			}
			return;
		}
		if (Cli_TxFreeSpace() < 144U)
			return;
		status = PhotoStore_GetEventIdAt(s_photo_list_index, &event_id);
		if (status == PHOTO_STORE_STATUS_BUSY || status == PHOTO_STORE_STATUS_NOT_READY)
			return;
		++s_photo_list_index;
		if (status != PHOTO_STORE_STATUS_OK) {
			++s_photo_list_skipped;
			return;
		}
		status = PhotoStore_Find(event_id, &metadata);
		if (status == PHOTO_STORE_STATUS_BUSY) {
			--s_photo_list_index;
			return;
		}
		if (status != PHOTO_STORE_STATUS_OK) {
			++s_photo_list_skipped;
			return;
		}
		length = snprintf(line, sizeof(line),
				"IRPHOTOS_ITEM,%lu,%lu,%u,%u,%u,%u,%lu,%08lX\r\n",
				(unsigned long) metadata.event_id,
				(unsigned long) metadata.capture_tick_ms,
				(unsigned int) metadata.width,
				(unsigned int) metadata.height,
				(unsigned int) metadata.pixel_format,
				(unsigned int) metadata.bytes_per_pixel,
				(unsigned long) metadata.frame_bytes,
				(unsigned long) metadata.payload_crc32);
		if ((length <= 0) || ((size_t) length >= sizeof(line))) {
			++s_photo_list_skipped;
			return;
		}
		s_photo_tx_internal_write = true;
		Cli_TxWrite(line);
		s_photo_tx_internal_write = false;
		++s_photo_list_sent;
		return;
	case CLI_PHOTO_LIST_WAIT_END:
		if (Cli_TxIsIdle())
			Cli_FinishPhotoList();
		return;
	case CLI_PHOTO_LIST_ABORT_DRAIN:
		if (Cli_TxIsIdle()) {
			if (s_photo_list_read_lock) {
				PhotoStore_EndReadSession();
				s_photo_list_read_lock = false;
			}
			s_photo_tx_internal_write = true;
			Cli_Printf("IRPHOTOS_ABORT,%u,%u\r\n",
					(unsigned int) s_photo_list_sent,
					(unsigned int) s_photo_list_index);
			s_photo_tx_internal_write = false;
			s_photo_list_state = CLI_PHOTO_LIST_WAIT_ABORT_END;
		}
		return;
	case CLI_PHOTO_LIST_WAIT_ABORT_END:
		if (Cli_TxIsIdle())
			Cli_FinishPhotoList();
		return;
	case CLI_PHOTO_LIST_IDLE:
	default:
		Cli_FinishPhotoList();
		return;
	}
}

static void Cli_Printf(const char *format, ...) {
	char buffer[CLI_TX_FORMAT_SIZE];
	int length;
	va_list args;
	va_start(args, format);
	length = vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	if ((length > 0) && ((size_t) length < sizeof(buffer))) {
		Cli_TxWrite(buffer);
	} else {
		s_tx_drop_count++;
	}
}
static void Cli_TxKick(void) {

	uint16_t tail;
	uint16_t head;
	uint16_t length;
	if ((s_initialized == 0U) || (s_tx.transfer_active != 0U)) {
		return;
	}
	tail = s_tx.tail;
	head = s_tx.head;
	if (tail == head) {
		return;
	}
	if (head > tail) {
		length = (uint16_t) (head - tail);
	} else {
		length = (uint16_t) (CLI_TX_RING_SIZE - tail);
	}
	s_tx.active_length = length;
	s_tx.transfer_active = 1U;
	if (HAL_UART_Transmit_IT(&huart1, &s_tx.data[tail], length) != HAL_OK) {
		s_tx.transfer_active = 0U;
		s_tx.active_length = 0U;
		s_tx_start_error_count++;
	}
}
static uint8_t Cli_RxPop(uint8_t *byte) {
	uint16_t tail;
	if ((byte == NULL) || (s_rx.tail == s_rx.head)) {
		return 0U;
	}
	tail = s_rx.tail;
	*byte = s_rx.data[tail];
	s_rx.data[tail] = 0U;
	s_rx.tail = Cli_NextIndex(tail, CLI_RX_RING_SIZE);
	return 1U;
}
static const char* Cli_StateName(AppState_t state) {
	switch (state) {
	case APP_STATE_DISARMED:
		return "DISARMED";
	case APP_STATE_ARMED:
		return "ARMED";
	case APP_STATE_FAULT:
		return "FAULT";
	default:
		return "UNKNOWN";
	}
}
static const char* Cli_StartupName(SelfTestOverallStatus_t status) {
	switch (status) {
	case SELF_TEST_OVERALL_PASS:
		return "PASS";
	case SELF_TEST_OVERALL_DEGRADED:
		return "DEGRADED";
	case SELF_TEST_OVERALL_FAIL:
		return "FAIL";
	default:
		return "UNKNOWN";
	}
}

static void Cli_PrintHelp(void) {
	Cli_TxWrite(
			"Incident Recorder CLI\r\n"
					" help - show available commands\r\n"
					" status - show system state and diagnostics\r\n"
					" config - show Cargo/Site profiles and storage status\r\n"
					" config mode cargo|site\r\n"
					" config cargo shock <mg>|tilt <deg>|temp <minC> <maxC>\r\n"
					" config site vibration <mg>|temp <minC> <maxC>\r\n"
					" Cargo: SHOCK + TILT + TEMPERATURE; Site: VIBRATION + TEMPERATURE\r\n"
					" journal [1..30] - show newest records (default: 5)\r\n"
					" journal all - stream the complete retained journal (DISARMED)\r\n"
					" journal id <sequence> - show one retained event by its persistent ID\r\n"
					" led test - run the non-blocking incident LED pattern (DISARMED)\r\n"
					" photos [1..30] - list recent events and photo availability/reason\r\n"
					" photos list - machine-readable retained-photo metadata\r\n"
					" photo status - show camera, storage and capture status\r\n"
					" photo id <event_id> - download one event photo (chunked IRPHOTO4 with ACK/NACK)\r\n"
					" photo all - download all retained event photos (chunked)\r\n"
					" photo selftest - run read-only metadata/CRC format test\r\n");
	Cli_TxWrite(" seal <password> / unseal <password> (8..32 chars)\r\n"
			" erase config|media|all <password>\r\n"
			"Erase confirmation is interactive: answer YES or NO.\r\n"
			"Erase requires DISARMED, SEALED config and password.\r\n"
			"Edits require DISARMED; changes are saved to W25Q64.\r\n");
}
static void Cli_PrintStatus(void) {
	Cli_Printf("[STATUS] state=%s startup=%s uptime_ms=%lu\r\n",
			Cli_StateName(App_GetState()),
			Cli_StartupName(App_GetStartupStatus()),
			(unsigned long) HAL_GetTick());
	{
		const ConfigData_t *cfg;
		bool sealed;
		uint32_t generation;
		ConfigService_Get(&cfg, &sealed, &generation);
		Cli_Printf(
				"[STATUS] config_mode=%s sealed=%u generation=%lu save=%s bad_password=%lu\r\n",
				Cli_ModeName(cfg->mode), (unsigned) sealed, (unsigned
				long) generation, Cli_SaveName(ConfigService_GetSaveStatus()),
				(unsigned long) ConfigService_GetWrongPasswordCount());
	}
	Cli_Printf(
			"[STATUS] rx_overflow=%lu tx_drop=%lu tx_queue_overflow=%lu tx_start_error=%lu rx_restart_error=%lu\r\n",
			(unsigned long) CliService_GetRxOverflowCount(),
			(unsigned long) CliService_GetTxDropCount(),
			(unsigned long) s_tx_queue_overflow_count,
			(unsigned long) s_tx_start_error_count,
			(unsigned long) s_rx_restart_error_count);
}
static const char* Cli_ModeName(ConfigMode_t mode) {
	return (mode == CONFIG_MODE_SITE) ? "SITE" : "CARGO";
}
static const char* Cli_SaveName(ConfigSaveStatus_t status) {
	switch (status) {
	case CONFIG_SAVE_IDLE:
		return "IDLE";
	case CONFIG_SAVE_IN_PROGRESS:
		return "IN_PROGRESS";
	case CONFIG_SAVE_OK:
		return "OK";
	case CONFIG_SAVE_ERROR:
		return "ERROR";
	default:
		return "UNKNOWN";
	}
}
static void Cli_PrintProfile(const char *name, ConfigMode_t mode,
		const ConfigProfile_t *p) {
	if (mode == CONFIG_MODE_CARGO) {

		Cli_Printf(
				"[CONFIG] %s shock_mg=%u tilt_deg=%u temp_min_c=%d temp_max_c=%d\r\n",
				name, (unsigned) p->impact_threshold_mg,
				(unsigned) p->tilt_threshold_deg, (int) p->temp_min_c,
				(int) p->temp_max_c);
	} else {
		Cli_Printf(
				"[CONFIG] %s vibration_rms_mg=%u temp_min_c=%d temp_max_c=%d\r\n",
				name, (unsigned) p->impact_threshold_mg, (int) p->temp_min_c,
				(int) p->temp_max_c);
	}
}
static void Cli_PrintConfig(void) {
	const ConfigData_t *cfg;
	bool sealed;
	uint32_t generation;
	ConfigService_Get(&cfg, &sealed, &generation);
	Cli_Printf("[CONFIG] mode=%s sealed=%s generation=%lu save=%s\r\n",
			Cli_ModeName(cfg->mode), sealed ? "YES" : "NO",
			(unsigned long) generation,
			Cli_SaveName(ConfigService_GetSaveStatus()));
	Cli_PrintProfile("CARGO", CONFIG_MODE_CARGO, &cfg->cargo);
	Cli_PrintProfile("SITE", CONFIG_MODE_SITE, &cfg->site);
}
static uint8_t Cli_EqualsIgnoreCase(const char *a, const char *b) {
	while ((*a != '\0') && (*b != '\0')) {
		if (tolower((unsigned char)*a) != tolower((unsigned char )*b))
			return 0U;
		a++;
		b++;
	}
	return (uint8_t) ((*a == '\0') && (*b == '\0'));
}
static uint8_t Cli_ParseLong(const char *text, long *value) {
	char *end;
	long v;
	if ((text == NULL) || (*text == '\0'))
		return 0U;
	v = strtol(text, &end, 10);
	if ((*end != '\0') || (end == text))
		return 0U;
	*value = v;
	return 1U;
}
static char* Cli_NextToken(char **cursor) {
	char *p = *cursor;
	char *token;
	while ((*p != '\0') && (isspace((unsigned char)*p) != 0))
		p++;
	if (*p == '\0') {
		*cursor = p;
		return NULL;
	}
	token = p;
	while ((*p != '\0') && (isspace((unsigned char)*p) == 0))
		p++;
	if (*p != '\0')
		*p++ = '\0';
	*cursor = p;
	return token;
}
static void Cli_ReportConfigResult(ConfigResult_t result) {
	switch (result) {

	case CONFIG_RESULT_OK:
		Cli_TxWrite("OK: configuration save started; wait for completion.\r\n");
		break;
	case CONFIG_RESULT_BUSY:
		Cli_TxWrite("ERR: configuration save is already in progress.\r\n");
		break;
	case CONFIG_RESULT_LOCKED:
		Cli_TxWrite("ERR: configuration is SEALED; use unseal first.\r\n");
		break;
	case CONFIG_RESULT_INVALID:
		Cli_TxWrite(
				"ERR: invalid value or password (password must be 8..32 characters).\r\n");
		break;
	case CONFIG_RESULT_IO_ERROR:
		Cli_TxWrite("ERR: Flash operation could not be started.\r\n");
		break;
	case CONFIG_RESULT_BAD_PASSWORD:
		Cli_TxWrite(
				"ERR: incorrect password; configuration remains SEALED.\r\n");
		break;
	case CONFIG_RESULT_NOT_SEALED:
		Cli_TxWrite("ERR: configuration is not SEALED.\r\n");
		break;
	default:
		Cli_TxWrite("ERR: configuration operation failed.\r\n");
		break;
	}
}
static uint8_t Cli_IsDestructiveOperationPending(void) {
	return (uint8_t) ((s_pending_erase_scope != CLI_ERASE_SCOPE_NONE)
			|| (s_erase_operation_active != 0U)
			|| (s_erase_waiting_config_reset != 0U)
			|| (s_factory_reset_pending != 0U)
			|| (StorageMaintenance_GetStatus() != STORAGE_MAINTENANCE_IDLE)
			|| (IncidentJournal_IsBusy() != false));
}
static void Cli_HandleConfig(char *arguments) {
	char *cursor = arguments;
	char *a = Cli_NextToken(&cursor);
	char *b = Cli_NextToken(&cursor);
	char *c = Cli_NextToken(&cursor);
	char *d = Cli_NextToken(&cursor);
	char *e = Cli_NextToken(&cursor);
	long v1, v2;
	ConfigMode_t mode;
	ConfigResult_t result;
	if (a == NULL) {
		Cli_PrintConfig();
		return;
	}
	if (Cli_EqualsIgnoreCase(a, "mode") != 0U) {
		if ((b == NULL) || (c != NULL)) {
			Cli_TxWrite("Usage: config mode cargo|site\r\n");
			return;
		}
		if (Cli_EqualsIgnoreCase(b, "cargo") != 0U)
			mode = CONFIG_MODE_CARGO;
		else if (Cli_EqualsIgnoreCase(b, "site") != 0U)
			mode = CONFIG_MODE_SITE;
		else {
			Cli_TxWrite("ERR: mode must be cargo or site.\r\n");
			return;
		}
		if (App_GetState() != APP_STATE_DISARMED) {
			Cli_TxWrite(
					"ERR: configuration can only be changed in DISARMED.\r\n");
			return;
		}
		if (Cli_IsDestructiveOperationPending() != 0U) {
			Cli_TxWrite(
					"ERR: configuration is locked while journal/Flash maintenance is active; retry after it completes.\r\n");
			return;
		}
		result = ConfigService_SetMode(mode);
		Cli_ReportConfigResult(result);
		return;
	}
	if ((b == NULL) || (c == NULL)) {
		Cli_TxWrite(
				"Usage: config cargo shock <mg>|tilt <deg>|temp <minC> <maxC>; config site vibration <mg>|temp <minC> <maxC>\r\n");
		return;
	}
	if (Cli_EqualsIgnoreCase(a, "cargo") != 0U)
		mode = CONFIG_MODE_CARGO;
	else if (Cli_EqualsIgnoreCase(a, "site") != 0U)
		mode = CONFIG_MODE_SITE;
	else {
		Cli_TxWrite("ERR: profile must be cargo or site.\r\n");
		return;
	}
	if (App_GetState() != APP_STATE_DISARMED) {
		Cli_TxWrite("ERR: configuration can only be changed in DISARMED.\r\n");
		return;
	}
	if (Cli_IsDestructiveOperationPending() != 0U) {
		Cli_TxWrite(
				"ERR: configuration is locked while journal/Flash maintenance is active; retry after it completes.\r\n");
		return;
	}
	if ((mode == CONFIG_MODE_CARGO) &&

	(Cli_EqualsIgnoreCase(b, "shock") != 0U)) {
		if ((d != NULL) || (e != NULL) || (Cli_ParseLong(c, &v1) == 0U)
				|| (v1 < 100L) || (v1 > 16000L)) {
			Cli_TxWrite("ERR: SHOCK threshold range is 100..16000 mg.\r\n");
			return;
		}
		result = ConfigService_SetShockThreshold((uint16_t) v1);
	} else if ((mode == CONFIG_MODE_SITE)
			&& (Cli_EqualsIgnoreCase(b, "vibration") != 0U)) {
		if ((d != NULL) || (e != NULL) || (Cli_ParseLong(c, &v1) == 0U)
				|| (v1 < 100L) || (v1 > 16000L)) {
			Cli_TxWrite(
					"ERR: VIBRATION RMS threshold range is 100..16000 mg.\r\n");
			return;
		}
		result = ConfigService_SetVibrationThreshold((uint16_t) v1);
	} else if (Cli_EqualsIgnoreCase(b, "tilt") != 0U) {
		if (mode == CONFIG_MODE_SITE) {
			Cli_TxWrite(
					"ERR: TILT is available only in Cargo mode; Site supports VIBRATION and TEMPERATURE.\r\n");
			return;
		}
		if ((d != NULL) || (e != NULL) || (Cli_ParseLong(c, &v1) == 0U)
				|| (v1 < 1L) || (v1 > 180L)) {
			Cli_TxWrite("ERR: tilt threshold range is 1..180 degrees.\r\n");
			return;
		}
		result = ConfigService_SetTilt(mode, (uint16_t) v1);
	} else if (Cli_EqualsIgnoreCase(b, "temp") != 0U) {
		if ((d == NULL) || (e != NULL) || (Cli_ParseLong(c, &v1) == 0U)
				|| (Cli_ParseLong(d, &v2) == 0U) || (v1 < -40L) || (v2 > 85L)
				|| (v1 >= v2)) {
			Cli_TxWrite(
					"ERR: temperature limits must be within -40..85 C and min < max.\r\n");
			return;
		}
		result = ConfigService_SetTemperature(mode, (int16_t) v1, (int16_t) v2);
	} else if ((Cli_EqualsIgnoreCase(b, "impact") != 0U)
			|| ((mode == CONFIG_MODE_CARGO)
					&& (Cli_EqualsIgnoreCase(b, "vibration") != 0U))
			|| ((mode == CONFIG_MODE_SITE)
					&& (Cli_EqualsIgnoreCase(b, "shock") != 0U))) {
		Cli_TxWrite(
				(mode == CONFIG_MODE_CARGO) ?
						"ERR: Cargo uses 'shock'; Site uses 'vibration'.\r\n" :
						"ERR: Site uses 'vibration'; Cargo uses 'shock'.\r\n");
		return;
	} else {
		Cli_TxWrite("ERR: invalid parameter for selected mode.\r\n");
		return;
	}
	Cli_ReportConfigResult(result);
}
static void Cli_HandlePasswordCommand(const char *command, char *password) {
	ConfigResult_t result;
	if ((password == NULL) || (*password == '\0')) {
		Cli_TxWrite("Usage: seal|unseal <password> (8..32 characters)\r\n");
		return;
	}
	if (strchr(password, ' ') != NULL) {
		Cli_TxWrite("ERR: password must not contain spaces.\r\n");
		return;
	}
	if (App_GetState() != APP_STATE_DISARMED) {
		Cli_TxWrite("ERR: seal/unseal is allowed only in DISARMED.\r\n");
		return;
	}
	if (Cli_IsDestructiveOperationPending() != 0U) {
		Cli_TxWrite(
				"ERR: seal/unseal is locked while journal/Flash maintenance is active; retry after it completes.\r\n");
		return;
	}
	if (strcmp(command, "seal") == 0)
		result = ConfigService_Seal(password);

	else
		result = ConfigService_Unseal(password);
	Cli_ReportConfigResult(result);
}
static void Cli_HandleErase(char *arguments) {
	char *cursor = arguments;
	char *scope_text = Cli_NextToken(&cursor);
	char *password = Cli_NextToken(&cursor);
	char *extra = Cli_NextToken(&cursor);
	ConfigResult_t auth;
	if ((scope_text == NULL) || (password == NULL) || (extra != NULL)) {
		Cli_TxWrite("Usage: erase config|media|all <password>\r\n");
		return;
	}
	if (App_GetState() != APP_STATE_DISARMED) {
		Cli_TxWrite("ERR: erase is allowed only in DISARMED.\r\n");
		return;
	}
	if ((s_erase_operation_active != 0U) || (s_factory_reset_pending != 0U)
			|| (StorageMaintenance_GetStatus() != STORAGE_MAINTENANCE_IDLE)
			|| (s_confirmation_active != 0U) || (s_maintenance_lock != 0U)
			|| (IncidentJournal_IsBusy() != false)
			|| PhotoCaptureService_IsBusy()
			|| (PhotoStore_GetState() == PHOTO_STORE_STATE_WRITING)
			|| (PhotoStore_GetState() == PHOTO_STORE_STATE_RELEASING)
			|| (PhotoStore_GetState() == PHOTO_STORE_STATE_RECOVERING)) {
		Cli_TxWrite(
				"ERR: journal or another erase/reset Flash operation is active.\r\n");
		return;
	}
	if (Cli_EqualsIgnoreCase(scope_text, "config") != 0U)
		s_pending_erase_scope = CLI_ERASE_SCOPE_CONFIG;
	else if (Cli_EqualsIgnoreCase(scope_text, "media") != 0U)
		s_pending_erase_scope = CLI_ERASE_SCOPE_MEDIA;
	else if (Cli_EqualsIgnoreCase(scope_text, "all") != 0U)
		s_pending_erase_scope = CLI_ERASE_SCOPE_ALL;
	else {
		Cli_TxWrite("ERR: erase scope must be config, media or all.\r\n");
		return;
	}
	auth = ConfigService_VerifyPassword(password);
	memset(password, 0, strlen(password));
	if (auth != CONFIG_RESULT_OK) {
		s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
		if (auth == CONFIG_RESULT_NOT_SEALED)
			Cli_TxWrite(
					"ERR: destructive erase requires SEALED configuration; seal it first.\r\n");
		else
			Cli_ReportConfigResult(auth);
		return;
	}
	s_erase_confirmation_deadline = HAL_GetTick() + 30000U;
	s_confirmation_active = 1U;
	Cli_TxWrite("WARNING: destructive operation requested.\r\n");
	if (s_pending_erase_scope == CLI_ERASE_SCOPE_CONFIG)
		Cli_TxWrite(
				"This restores factory Cargo/Site settings and erases the old password slot.\r\n");
	else if (s_pending_erase_scope == CLI_ERASE_SCOPE_MEDIA)
		Cli_TxWrite(
				"This erases sectors 2..2047 (the complete journal/photo data partition).\r\n");
	else
		Cli_TxWrite(
				"This erases sectors 2..2047 and restores factory configuration with generation=0.\r\n");
	Cli_TxWrite("Type YES to continue or NO to cancel: ");

}
static void Cli_HandleConfirmationAnswer(char *answer) {
	char *start = answer;
	char *end;
	if (s_confirmation_active == 0U) {
		return;
	}
	while (isspace((unsigned char)*start) != 0)
		start++;
	end = start + strlen(start);
	while ((end > start) && (isspace((unsigned char)end[-1]) != 0))
		*--end = '\0';
	if ((int32_t) (HAL_GetTick() - s_erase_confirmation_deadline) >= 0) {
		s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
		s_confirmation_active = 0U;
		Cli_TxWrite(
				"Destructive operation cancelled: confirmation timed out.\r\n");
		return;
	}
	if (Cli_EqualsIgnoreCase(start, "NO") != 0U) {
		s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
		s_confirmation_active = 0U;
		Cli_TxWrite("Destructive operation cancelled.\r\n");
		return;
	}
	if (Cli_EqualsIgnoreCase(start, "YES") != 0U) {
		if (App_GetState() != APP_STATE_DISARMED) {
			s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
			s_confirmation_active = 0U;
			Cli_TxWrite(
					"Destructive operation cancelled: system is no longer DISARMED.\r\n");
			return;
		}
		s_confirmation_active = 0U;
		Cli_StartConfirmedErase();
		return;
	}
	Cli_TxWrite("Please answer YES or NO: ");
}
static void Cli_StartConfirmedErase(void) {
	ConfigResult_t result;
	s_maintenance_lock = 1U;
	s_rx.tail = s_rx.head; /* discard bytes received before maintenance lock */
	memset(s_line, 0, sizeof(s_line));
	s_line_length = 0U;
	s_discard_line = 0U;
	if (s_pending_erase_scope == CLI_ERASE_SCOPE_CONFIG) {
		result = ConfigService_FactoryReset();
		if (result != CONFIG_RESULT_OK) {
			s_maintenance_lock = 0U;
			s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
			Cli_ReportConfigResult(result);
			Cli_TxWrite("IR> ");
			return;
		}

		s_factory_reset_pending = 1U;
		Cli_TxWrite(
				"[ERASE] factory configuration reset started. CLI locked until verification completes.\r\n");
	} else {
		if (!StorageMaintenance_StartDataPartitionErase()) {
			s_maintenance_lock = 0U;
			s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
			Cli_TxWrite("ERR: storage erase service is busy.\r\nIR> ");
			return;
		}
		s_erase_operation_active = 1U;
		s_erase_waiting_config_reset =
				(s_pending_erase_scope == CLI_ERASE_SCOPE_ALL) ? 1U : 0U;
		s_last_erase_progress_report = 0U;
		Cli_TxWrite(
				"[ERASE] started: 2046 sectors. CLI locked until erase and read-back verification complete. Do not power off.\r\n");
	}
	s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
}
static void Cli_ReinitializeJournal(void) {
	if (IncidentJournal_Init() != INCIDENT_JOURNAL_OK) {
		App_EnterFault(0x00060001UL);
		Cli_TxWrite(
				"[JOURNAL] ERROR: recovery after media erase failed; system entered FAULT.\r\n");
	}
	if (PhotoStore_Init() != PHOTO_STORE_STATUS_OK) {
		Cli_TxWrite(
				"[PHOTO] ERROR: recovery after media erase could not start.\r\n");
	}
	PhotoCaptureService_ResetAfterMediaErase();
}
static void Cli_FinishMaintenance(const char *message) {
	s_maintenance_lock = 0U;
	s_confirmation_active = 0U;
	s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
	s_erase_operation_active = 0U;
	s_erase_waiting_config_reset = 0U;
	s_factory_reset_pending = 0U;
	memset(s_line, 0, sizeof(s_line));
	s_line_length = 0U;
	s_discard_line = 0U;
	Cli_TxWrite(message);
	Cli_TxWrite("\r\nIR> ");
}
static void Cli_PrintProgress(const char *message) {
	Cli_TxWrite(message);
	Cli_TxWrite("\r\n");
}
/* Redraw the active input line without leaving duplicate prompts or blank lines.
 * ANSI/VT100 EL2 clears the current terminal row before the async notification.
 */
static void Cli_AsyncNotice(const char *message) {
	Cli_TxWrite("\r\x1B[2K");
	Cli_TxWrite(message);
	Cli_TxWrite("\r\nIR> ");
	if ((s_line_length > 0U) && (s_discard_line == 0U)) {
		s_line[s_line_length] = '\0';
		Cli_TxWrite(s_line);
	}
}
static void Cli_PrintJournalRecord(const IncidentJournalRecord_t *record) {
	IncidentEvent_t event;
	event.sequence = record->sequence;
	event.timestamp_ms = record->timestamp_ms;
	event.mode = (ConfigMode_t) record->mode;
	event.type = (IncidentType_t) record->type;
	event.value = record->value;
	event.threshold = record->threshold;
	Cli_Printf(
			"[JOURNAL RECORD] id=%lu type=%s mode=%s tick_ms=%lu value=%ld threshold=%ld\r\n",
			(unsigned long) event.sequence,
			IncidentDetector_TypeName(event.type),
			(event.mode == CONFIG_MODE_SITE) ? "SITE" : "CARGO",
			(unsigned long) event.timestamp_ms, (long) event.value,
			(long) event.threshold);
}

static void Cli_StartJournalExport(void) {
	IncidentJournalStatus_t status;
	if (App_GetState() != APP_STATE_DISARMED) {
		Cli_TxWrite("ERR: journal all is available only in DISARMED.\r\n");
		return;
	}
	if (s_journal_export_active != 0U) {
		Cli_TxWrite("ERR: journal export is already active.\r\n");
		return;
	}
	if (PhotoCaptureService_IsBusy()) {
		Cli_TxWrite("ERR: photo capture/storage is still active; retry journal all later.\r\n");
		return;
	}
	status = IncidentJournal_BeginReadAll(&s_journal_cursor);
	if (status != INCIDENT_JOURNAL_OK) {
		Cli_Printf("ERR: journal export cannot start (status=%u); wait for Flash activity to finish.\r\n",
				(unsigned int) status);
		return;
	}
	s_journal_export_total = 0U;
	s_journal_export_sent = 0U;
	s_journal_export_first_id = 0xFFFFFFFFUL;
	s_journal_export_last_id = 0U;
	s_journal_export_state = CLI_JOURNAL_EXPORT_COUNT;
	s_journal_export_active = 1U;
}

static void Cli_JournalExportAbort(uint32_t status) {
	Cli_Printf("IRJOURNAL_ERROR,%lu,%lu\r\n", (unsigned long) status,
			(unsigned long) s_journal_export_sent);
	s_journal_export_state = CLI_JOURNAL_EXPORT_IDLE;
	s_journal_export_active = 0U;
	Cli_SendPrompt();
}

static void Cli_ProcessJournalExport(void) {
	IncidentJournalRecord_t record;
	IncidentJournalStatus_t status;
	uint8_t budget;
	char line[CLI_JOURNAL_LINE_SIZE];
	int length;

	if (s_journal_export_active == 0U)
		return;

	if (s_journal_export_state == CLI_JOURNAL_EXPORT_COUNT) {
		for (budget = 0U; budget < CLI_JOURNAL_SCAN_BUDGET; ++budget) {
			status = IncidentJournal_ReadAllNext(&s_journal_cursor, &record);
			if (status == INCIDENT_JOURNAL_OK) {
				s_journal_export_total++;
				if (record.sequence < s_journal_export_first_id)
					s_journal_export_first_id = record.sequence;
				if (record.sequence > s_journal_export_last_id)
					s_journal_export_last_id = record.sequence;
			} else if (status == INCIDENT_JOURNAL_BUSY) {
				continue;
			} else if (status == INCIDENT_JOURNAL_NOT_FOUND) {
				(void) IncidentJournal_RestartReadAll(&s_journal_cursor);
				if (s_journal_export_total == 0U)
					s_journal_export_first_id = 0U;
				length = snprintf(line, sizeof(line),
						"IRJOURNAL_BEGIN,%lu,%lu,%lu\r\n",
						(unsigned long) s_journal_export_total,
						(unsigned long) s_journal_export_first_id,
						(unsigned long) s_journal_export_last_id);
				if ((length <= 0) || ((size_t) length >= sizeof(line))) {
					Cli_JournalExportAbort((uint32_t) INCIDENT_JOURNAL_CORRUPT);
					return;
				}
				Cli_TxWrite(line);
				s_journal_export_state = CLI_JOURNAL_EXPORT_STREAM;
				return;
			} else {
				Cli_JournalExportAbort((uint32_t) status);
				return;
			}
		}
		return;
	}

	if (s_journal_export_state == CLI_JOURNAL_EXPORT_STREAM) {
		for (budget = 0U; budget < 4U; ++budget) {
			/* Keep room for a complete machine-readable line before advancing
			 * the cursor. This is back-pressure, not a blocking wait. */
			if (Cli_TxFreeSpace() < 224U)
				return;
			status = IncidentJournal_ReadAllNext(&s_journal_cursor, &record);
			if (status == INCIDENT_JOURNAL_OK) {
				length = snprintf(line, sizeof(line),
						"IRJOURNAL_ITEM,%lu,%s,%s,%lu,%ld,%ld,%u,%u\r\n",
						(unsigned long) record.sequence,
						IncidentDetector_TypeName((IncidentType_t) record.type),
						(record.mode == CONFIG_MODE_SITE) ? "SITE" : "CARGO",
						(unsigned long) record.timestamp_ms,
						(long) record.value, (long) record.threshold,
						(unsigned int) record.photo_outcome,
						(unsigned int) record.photo_reason);
				if ((length <= 0) || ((size_t) length >= sizeof(line))) {
					Cli_JournalExportAbort((uint32_t) INCIDENT_JOURNAL_CORRUPT);
					return;
				}
				Cli_TxWrite(line);
				s_journal_export_sent++;
			} else if (status == INCIDENT_JOURNAL_BUSY) {
				continue;
			} else if (status == INCIDENT_JOURNAL_NOT_FOUND) {
				length = snprintf(line, sizeof(line),
						"IRJOURNAL_END,%lu,%lu\r\n",
						(unsigned long) s_journal_export_sent,
						(unsigned long) (s_journal_export_total -
								((s_journal_export_sent <= s_journal_export_total) ?
										s_journal_export_sent : s_journal_export_total)));
				if ((length > 0) && ((size_t) length < sizeof(line)))
					Cli_TxWrite(line);
				s_journal_export_state = CLI_JOURNAL_EXPORT_IDLE;
				s_journal_export_active = 0U;
				Cli_SendPrompt();
				return;
			} else {
				Cli_JournalExportAbort((uint32_t) status);
				return;
			}
		}
	}
}

static void Cli_HandleLed(char *arguments) {
	if (!Cli_EqualsIgnoreCase(arguments, "test")) {
		Cli_TxWrite("Usage: led test\r\n");
		return;
	}
	if (App_GetState() != APP_STATE_DISARMED) {
		Cli_TxWrite("ERR: led test is available only in DISARMED.\r\n");
		return;
	}
	LedService_TriggerIncidentPattern(HAL_GetTick());
	Cli_TxWrite("[LED TEST] three-pulse incident indication started (non-blocking).\r\n");
}

static void Cli_HandleJournal(char *arguments) {
	IncidentJournalStats_t stats;
	IncidentJournalRecord_t records[INCIDENT_JOURNAL_RECENT_MAX];
	uint8_t requested = 5U;
	uint8_t read_count = 0U;
	uint8_t index;
	IncidentJournalStatus_t result;

	if ((arguments != NULL) && Cli_EqualsIgnoreCase(arguments, "all")) {
		Cli_StartJournalExport();
		return;
	}

	if ((arguments != NULL) && (strncmp(arguments, "id", 2U) == 0)
			&& ((arguments[2] == '\0') || (arguments[2] == ' ')
					|| (arguments[2] == '\t'))) {
		char *id_text = arguments + 2;
		char *end = NULL;
		unsigned long parsed;
		IncidentJournalRecord_t record;
		while ((*id_text == ' ') || (*id_text == '\t'))
			++id_text;
		parsed = strtoul(id_text, &end, 10);
		while ((end != NULL) && ((*end == ' ') || (*end == '\t')))
			++end;
		if ((id_text[0] == '\0') || (end == id_text)
				|| ((end != NULL) && (*end != '\0')) || (parsed == 0UL)
				|| (parsed > 0xFFFFFFFFUL)) {
			Cli_TxWrite("Usage: journal id <sequence>\r\n");
			return;
		}
		result = IncidentJournal_ReadBySequence((uint32_t) parsed, &record);
		if (result == INCIDENT_JOURNAL_NOT_FOUND) {
			Cli_Printf(
					"[JOURNAL] event ID %lu not found (overwritten, absent or invalid)\r\n",
					parsed);
			return;
		}
		if (result != INCIDENT_JOURNAL_OK) {
			Cli_Printf("[JOURNAL] read failed status=%u\r\n",
					(unsigned int) result);
			return;
		}
		Cli_PrintJournalRecord(&record);
		return;
	}

	if ((arguments != NULL) && (*arguments != '\0')) {
		char *end = NULL;
		unsigned long parsed = strtoul(arguments, &end, 10);
		while ((end != NULL) && ((*end == ' ') || (*end == '\t')))
			++end;
		if ((end == arguments) || ((end != NULL) && (*end != '\0'))
				|| (parsed == 0UL) || (parsed > INCIDENT_JOURNAL_RECENT_MAX)) {
			Cli_TxWrite("Usage: journal [1..30] | journal id <sequence>\r\n");
			return;
		}
		requested = (uint8_t) parsed;
	}

	IncidentJournal_GetStats(&stats);
	Cli_Printf("[JOURNAL] ready=%u free=%lu pending=%u\r\n",
			(unsigned int) stats.ready, (unsigned long) stats.available_records,
			(unsigned int) stats.write_pending);

	result = IncidentJournal_ReadRecent(records, requested, &read_count);
	if (result != INCIDENT_JOURNAL_OK) {
		Cli_Printf("[JOURNAL] read failed status=%u\r\n",
				(unsigned int) result);
		return;
	}
	Cli_Printf("[JOURNAL] records_returned=%u (newest first; requested=%u)\r\n",
			(unsigned int) read_count, (unsigned int) requested);
	for (index = 0U; index < read_count; ++index)
		Cli_PrintJournalRecord(&records[index]);
}

static const char* Cli_PhotoStoreStateName(PhotoStore_State_t state) {
	switch (state) {
	case PHOTO_STORE_STATE_RECOVERING:
		return "RECOVERING";
	case PHOTO_STORE_STATE_READY:
		return "READY";
	case PHOTO_STORE_STATE_WRITING:
		return "WRITING";
	case PHOTO_STORE_STATE_RELEASING:
		return "RELEASING";
	case PHOTO_STORE_STATE_ERROR:
		return "ERROR";
	default:
		return "UNINITIALIZED";
	}
}

static bool Cli_ParseEventId(char *text, uint32_t *event_id) {
	char *end = NULL;
	unsigned long parsed;
	while ((*text == ' ') || (*text == '\t'))
		++text;
	parsed = strtoul(text, &end, 10);
	while ((end != NULL) && ((*end == ' ') || (*end == '\t')))
		++end;
	if ((text[0] == '\0') || (end == text) || (end != NULL && *end != '\0')
			|| parsed == 0UL || parsed > 0xFFFFFFFFUL)
		return false;
	*event_id = (uint32_t) parsed;
	return true;
}

static const char* Cli_PhotoReasonText(uint8_t reason) {
	switch ((IncidentPhotoReason_t) reason) {
	case INCIDENT_PHOTO_REASON_COOLDOWN:
		return "cooldown (<3 s for same incident type)";
	case INCIDENT_PHOTO_REASON_CAMERA_UNAVAILABLE:
		return "camera unavailable; degraded mode";
	case INCIDENT_PHOTO_REASON_CAPTURE_FAILED:
		return "camera frame capture failed";
	case INCIDENT_PHOTO_REASON_STORAGE_FULL:
		return "photo partition full";
	case INCIDENT_PHOTO_REASON_STORAGE_ERROR:
		return "Flash write/read-back error";
	case INCIDENT_PHOTO_REASON_QUEUE_FULL:
		return "photo request queue full";
	case INCIDENT_PHOTO_REASON_POLICY:
		return "event type excluded by mode policy";
	default:
		return "unrecognized no-photo reason";
	}
}

static void Cli_PrintEventPhotoStatus(const IncidentJournalRecord_t *event) {
	PhotoStore_Metadata_t metadata;
	PhotoStore_Status_t status = PhotoStore_Find(event->sequence, &metadata);
	if (status == PHOTO_STORE_STATUS_OK) {
		const char *format =
				(metadata.pixel_format == PHOTO_STORE_PIXEL_FORMAT_RGB565) ?
						"RGB565" :
				(metadata.pixel_format == PHOTO_STORE_PIXEL_FORMAT_YUV422) ?
						"YUV422" :
				(metadata.pixel_format == PHOTO_STORE_PIXEL_FORMAT_GRAY8) ?
						"GRAY8" : "UNKNOWN";
		Cli_Printf("photo=YES size=%ux%u format=%s bytes=%lu crc32=0x%08lX\r\n",
				(unsigned int) metadata.width, (unsigned int) metadata.height,
				format, (unsigned long) metadata.frame_bytes,
				(unsigned long) metadata.payload_crc32);
	} else if (status == PHOTO_STORE_STATUS_NOT_FOUND) {
		if (event->photo_outcome == INCIDENT_PHOTO_OUTCOME_NO_PHOTO) {
			Cli_Printf("photo=NO reason=%s\r\n",
					Cli_PhotoReasonText(event->photo_reason));
		} else if (event->photo_outcome == INCIDENT_PHOTO_OUTCOME_PENDING) {
			Cli_TxWrite(
					"photo=NO reason=photo processing was interrupted or not finalized\r\n");
		} else if (event->photo_outcome == INCIDENT_PHOTO_OUTCOME_STORED) {
			Cli_TxWrite(
					"photo=NO reason=journal says photo was stored, but its Flash extent is missing (interrupted reclaim or storage inconsistency)\r\n");
		} else if (event->photo_outcome == INCIDENT_PHOTO_OUTCOME_LEGACY) {
			Cli_TxWrite(
					"photo=NO reason=not recorded (legacy event; original reason unavailable)\r\n");
		} else {
			Cli_TxWrite(
					"photo=UNKNOWN reason=journal photo outcome code is invalid\r\n");
		}
	} else if (status == PHOTO_STORE_STATUS_NOT_READY) {
		Cli_TxWrite(
				"photo=UNKNOWN reason=photo storage recovery is not complete\r\n");
	} else if (status == PHOTO_STORE_STATUS_BUSY) {
		Cli_TxWrite(
				"photo=UNKNOWN reason=Flash is busy; retry the command\r\n");
	} else {
		Cli_TxWrite(
				"photo=UNKNOWN reason=photo storage reported an I/O/metadata error\r\n");
	}
}

static void Cli_HandlePhoto(char *arguments) {
	PhotoStore_Stats_t photo_stats;
	PhotoCaptureService_Stats_t capture_stats;
	PhotoStore_Status_t status;
	uint32_t event_id;

	if (strncmp(arguments, "selftest", 8U) == 0
			&& (arguments[8] == '\0' || isspace((unsigned char ) arguments[8]))) {
		Cli_TxWrite(
				PhotoStore_RunFormatSelfTest() ?
						"[PHOTO TEST] metadata/CRC format self-test PASS\r\n" :
						"[PHOTO TEST] metadata/CRC format self-test FAIL\r\n");
		return;
	}

	if (strncmp(arguments, "status", 6U) == 0
			&& (arguments[6] == '\0' || isspace((unsigned char ) arguments[6]))) {
		PhotoStore_GetStats(&photo_stats);
		PhotoCaptureService_GetStats(&capture_stats);
		Cli_Printf(
				"[PHOTO STATUS] storage=%s scanned=%lu occupied=%lu free=%lu photos=%lu index=%u release_pending=%u next=0x%06lX io_errors=%lu\r\n",
				Cli_PhotoStoreStateName(photo_stats.state),
				(unsigned long) photo_stats.scanned_sectors,
				(unsigned long) photo_stats.occupied_sectors,
				(unsigned long) photo_stats.free_sectors,
				(unsigned long) photo_stats.committed_photos,
				(unsigned int) photo_stats.index_entries,
				(unsigned int) photo_stats.release_pending,
				(unsigned long) photo_stats.next_free_address,
				(unsigned long) photo_stats.io_errors);
		Cli_Printf(
				"[PHOTO STATUS] camera=%s mode=%ux%u frame_bytes=%lu queue=%lu active_event=%lu\r\n",
				capture_stats.camera_available ?
						"READY" : "UNAVAILABLE (degraded mode)",
				(unsigned int) capture_stats.camera_width,
				(unsigned int) capture_stats.camera_height,
				(unsigned long) capture_stats.frame_bytes,
				(unsigned long) capture_stats.queue_depth,
				(unsigned long) capture_stats.active_event_id);
		Cli_Printf(
				"[PHOTO COUNTERS] observed=%lu stored=%lu cooldown=%lu camera_unavailable=%lu capture_failed=%lu storage_skipped=%lu queue_dropped=%lu\r\n",
				(unsigned long) capture_stats.events_observed,
				(unsigned long) capture_stats.photos_stored,
				(unsigned long) capture_stats.skipped_cooldown,
				(unsigned long) capture_stats.skipped_camera_unavailable,
				(unsigned long) capture_stats.capture_failures,
				(unsigned long) capture_stats.skipped_store_error,
				(unsigned long) capture_stats.queue_dropped);
		return;
	}

	if (strncmp(arguments, "id", 2U) == 0
			&& (arguments[2] == '\0' || isspace((unsigned char ) arguments[2]))) {
		char *id_text = arguments + 2;
		IncidentJournalRecord_t event_record;
		IncidentJournalStatus_t event_status;
		while (isspace((unsigned char ) *id_text))
			++id_text;
		if (!Cli_ParseEventId(id_text, &event_id)) {
			Cli_TxWrite("Usage: photo id <event_id>\r\n");
			return;
		}
		if (App_GetState() != APP_STATE_DISARMED) {
			Cli_TxWrite(
					"ERR: photo download is available only in DISARMED.\r\n");
			return;
		}
		if (PhotoCaptureService_IsBusy()) {
			Cli_TxWrite(
					"ERR: photo capture/storage is still active; retry when photo status reports queue=0.\r\n");
			return;
		}
		status = PhotoStore_BeginReadSession();
		if (status != PHOTO_STORE_STATUS_OK) {
			Cli_TxWrite(
					"ERR: photo storage is busy or not ready; retry later.\r\n");
			return;
		}
		s_photo_tx_read_lock = true;
		event_status = IncidentJournal_ReadBySequence(event_id, &event_record);
		if (event_status != INCIDENT_JOURNAL_OK) {
			Cli_FinishPhotoTransfer();
			if (event_status == INCIDENT_JOURNAL_BUSY) {
				Cli_TxWrite("ERR: Flash is busy; retry the photo command.\r\n");
			} else if (event_status == INCIDENT_JOURNAL_NOT_FOUND) {
				Cli_Printf(
						"ERR: event #%lu is no longer retained in the journal.\r\n",
						(unsigned long) event_id);
			} else {
				Cli_TxWrite("ERR: the event journal could not be read.\r\n");
			}
			return;
		}
		status = PhotoStore_Find(event_id, &s_photo_tx_metadata);
		if (status != PHOTO_STORE_STATUS_OK) {
			Cli_FinishPhotoTransfer();
			if (status == PHOTO_STORE_STATUS_NOT_FOUND) {
				Cli_Printf("[PHOTO] event #%lu has no stored photo. ",
						(unsigned long) event_id);
				if (event_record.photo_outcome
						== INCIDENT_PHOTO_OUTCOME_NO_PHOTO) {
					Cli_Printf("Reason: %s.\r\n",
							Cli_PhotoReasonText(event_record.photo_reason));
				} else if (event_record.photo_outcome
						== INCIDENT_PHOTO_OUTCOME_PENDING) {
					Cli_TxWrite(
							"Reason: photo processing was interrupted or not finalized (for example, power loss during capture/write).\r\n");
				} else if (event_record.photo_outcome
						== INCIDENT_PHOTO_OUTCOME_STORED) {
					Cli_TxWrite(
							"Reason: the journal says a photo was stored, but its Flash extent is missing (interrupted reclaim or storage inconsistency).\r\n");
				} else if (event_record.photo_outcome
						== INCIDENT_PHOTO_OUTCOME_LEGACY) {
					Cli_TxWrite(
							"Reason: legacy event; the original no-photo reason was not stored.\r\n");
				} else {
					Cli_TxWrite(
							"Reason: journal photo outcome is invalid or unsupported.\r\n");
				}
			} else {
				Cli_TxWrite(
						"ERR: photo metadata cannot be read because photo storage is busy or unavailable.\r\n");
			}
			return;
		}
		s_photo_tx_all_mode = false;
		s_photo_all_sent = 0U;
		s_photo_abort_requested = 0U;
		s_photo_tx_abort_after_chunk = false;
		s_photo_chunk_retry_count = 0U;
		s_photo_chunk_ack_received = 0U;
		s_photo_chunk_nack_received = 0U;
		s_photo_tx_current_active = true;
		s_photo_tx_internal_write = true;
		s_photo_tx_offset = 0U;
		s_photo_tx_crc = 0xFFFFFFFFUL;
		s_photo_tx_failed = false;
		s_photo_tx_start_tick = HAL_GetTick();
		s_photo_tx_timeout_ms = Cli_PhotoTransferTimeoutMs(
				s_photo_tx_metadata.frame_bytes);
		s_photo_tx_state = CLI_PHOTO_TX_WAIT_HEADER;
		Cli_EnterPhotoProtocolMode(false);
		Cli_Printf("IRPHOTO4,%lu,%u,%u,%u,%u,%lu,%08lX,%u\r\n",
				(unsigned long) s_photo_tx_metadata.event_id,
				(unsigned int) s_photo_tx_metadata.width,
				(unsigned int) s_photo_tx_metadata.height,
				(unsigned int) s_photo_tx_metadata.pixel_format,
				(unsigned int) s_photo_tx_metadata.bytes_per_pixel,
				(unsigned long) s_photo_tx_metadata.frame_bytes,
				(unsigned long) s_photo_tx_metadata.payload_crc32,
				(unsigned int) CLI_PHOTO_CHUNK_SIZE);
		s_photo_tx_internal_write = false;
		return;
	}

	if (strcmp(arguments, "all") == 0) {
		if (App_GetState() != APP_STATE_DISARMED) {
			Cli_TxWrite(
					"ERR: photo download is available only in DISARMED.\r\n");
			return;
		}
		if (PhotoCaptureService_IsBusy()) {
			Cli_TxWrite(
					"ERR: photo capture/storage is still active; retry later.\r\n");
			return;
		}
		PhotoStore_GetStats(&photo_stats);
		if (photo_stats.state != PHOTO_STORE_STATE_READY) {
			Cli_TxWrite("ERR: photo storage recovery is not complete.\r\n");
			return;
		}
		status = PhotoStore_BeginReadSession();
		if (status != PHOTO_STORE_STATUS_OK) {
			Cli_TxWrite("ERR: photo storage is busy; retry later.\r\n");
			return;
		}
		s_photo_tx_read_lock = true;
		s_photo_abort_requested = 0U;
		s_photo_tx_abort_after_chunk = false;
		s_photo_tx_current_active = false;
		s_photo_tx_all_mode = true;
		s_photo_all_index = 0U;
		s_photo_all_count = photo_stats.index_entries;
		s_photo_all_sent = 0U;
		s_photo_tx_state = CLI_PHOTO_TX_ALL_NEXT;
		s_photo_tx_start_tick = HAL_GetTick();
		Cli_EnterPhotoProtocolMode(false);
		s_photo_tx_internal_write = true;
		Cli_Printf("IRPHOTO_ALL_BEGIN,4,%u\r\n",
				(unsigned int) s_photo_all_count);
		s_photo_tx_internal_write = false;
		return;
	}

	Cli_TxWrite(
			"Usage: photo status | photo id <event_id> | photo all | photo selftest\r\n");
}

static void Cli_HandlePhotos(char *arguments) {
	IncidentJournalRecord_t records[INCIDENT_JOURNAL_RECENT_MAX];
	uint8_t requested = 10U;
	uint8_t count = 0U;
	uint8_t i;
	IncidentJournalStatus_t journal_status;
	if (strcmp(arguments, "list") == 0) {
		Cli_StartPhotoList();
		return;
	}
	if (App_GetState() != APP_STATE_DISARMED) {
		Cli_TxWrite("ERR: photo listing is available only in DISARMED.\r\n");
		return;
	}
	if (*arguments != '\0') {
		uint32_t parsed;
		if (!Cli_ParseEventId(arguments,
				&parsed) || parsed > INCIDENT_JOURNAL_RECENT_MAX) {
			Cli_TxWrite("Usage: photos [1..30] | photos list\r\n");
			return;
		}
		requested = (uint8_t) parsed;
	}
	journal_status = IncidentJournal_ReadRecent(records, requested, &count);
	if (journal_status != INCIDENT_JOURNAL_OK) {
		Cli_TxWrite(
				"[PHOTO] cannot list events: journal is busy or unavailable; retry later.\r\n");
		return;
	}
	Cli_Printf("[PHOTOS] retained_events=%u (newest first)\r\n",
			(unsigned int) count);
	for (i = 0U; i < count; ++i) {
		Cli_Printf(
				"[EVENT] id=%lu type=%s mode=%s tick_ms=%lu value=%ld threshold=%ld ",
				(unsigned long) records[i].sequence,
				IncidentDetector_TypeName((IncidentType_t) records[i].type),
				(records[i].mode == CONFIG_MODE_SITE) ? "SITE" : "CARGO",
				(unsigned long) records[i].timestamp_ms,
				(long) records[i].value, (long) records[i].threshold);
		Cli_PrintEventPhotoStatus(&records[i]);
	}
}

static void Cli_ExecuteLine(char *line) {
	char *start = line;
	char *end;
	char *argument;
	while (isspace((unsigned char)*start) != 0) {
		start++;
	}
	end = start + strlen(start);
	while ((end > start) && (isspace((unsigned char)end[-1]) != 0)) {
		*--end = '\0';
	}
	if (*start == '\0') {
		return;
	}
	argument = start;
	while ((*argument != '\0') && (isspace((unsigned char)*argument) == 0)) {
		*argument = (char) tolower((unsigned char )*argument);
		argument++;
	}
	if (*argument != '\0')

	{
		*argument++ = '\0';
		while (isspace((unsigned char)*argument) != 0) {
			argument++;
		}
	}
	if (strcmp(start, "help") == 0) {
		if (*argument == '\0')
			Cli_PrintHelp();
		else
			Cli_TxWrite("ERR: help takes no arguments\r\n");
	} else if (strcmp(start, "status") == 0) {
		if (*argument == '\0')
			Cli_PrintStatus();
		else
			Cli_TxWrite("ERR: status takes no arguments\r\n");
	} else if (strcmp(start, "config") == 0) {
		Cli_HandleConfig(argument);
	} else if (strcmp(start, "journal") == 0) {
		Cli_HandleJournal(argument);
	} else if (strcmp(start, "led") == 0) {
		Cli_HandleLed(argument);
	} else if (strcmp(start, "photos") == 0) {
		Cli_HandlePhotos(argument);
	} else if (strcmp(start, "photo") == 0) {
		Cli_HandlePhoto(argument);
	} else if ((strcmp(start, "seal") == 0) || (strcmp(start, "unseal") == 0)) {
		Cli_HandlePasswordCommand(start, argument);
	} else if (strcmp(start, "erase") == 0) {
		Cli_HandleErase(argument);
	} else {
		Cli_TxWrite("ERR: unknown command. Type 'help'.\r\n");
	}
}
static uint8_t Cli_PrefixMatchesIgnoreCase(const char *text, const char *prefix,
		size_t length) {
	size_t i;
	for (i = 0U; i < length; i++) {
		if (tolower((unsigned char )text[i]) !=
		tolower((unsigned char)prefix[i])) {
			return 0U;
		}
	}
	return 1U;
}
static uint8_t Cli_IsPasswordCharacter(void) {
	size_t length = s_line_length;
	size_t index;
	size_t token_start;
	size_t token_end;
	const char *commands[] = { "seal ", "unseal " };
	size_t command_lengths[] = { 5U, 7U };
	unsigned i;
	for (i = 0U; i < 2U; i++)

	{
		if ((length > command_lengths[i])
				&& (Cli_PrefixMatchesIgnoreCase(s_line, commands[i],
						command_lengths[i]) != 0U)) {
			return 1U;
		}
	}
	if ((length < 6U)
			|| (Cli_PrefixMatchesIgnoreCase(s_line, "erase ", 6U) == 0U)) {
		return 0U;
	}
	token_start = 6U;
	token_end = token_start;
	while ((token_end < length)
			&& (isspace((unsigned char)s_line[token_end]) == 0)) {
		token_end++;
	}
	if (token_end == length) {
		return 0U;
	}
	index = token_end;
	while ((index < length) && (isspace((unsigned char)s_line[index]) != 0)) {
		index++;
	}
	/* A delimiter after the scope means the next received byte is password. */
	return (uint8_t) (length > index);
}
static void Cli_ProcessByte(uint8_t byte) {
	if (byte == (uint8_t) '\n') {
		if (s_last_was_cr != 0U) {
			s_last_was_cr = 0U;
			return;
		}
	} else {
		s_last_was_cr = 0U;
	}
	if ((byte == (uint8_t) '\r') || (byte == (uint8_t) '\n')) {
		s_last_was_cr = (byte == (uint8_t) '\r') ? 1U : 0U;
		Cli_TxWrite("\r\n");
		if (s_discard_line != 0U) {
			s_discard_line = 0U;
			memset(s_line, 0, sizeof(s_line));
			s_line_length = 0U;
			Cli_TxWrite("ERR: command line too long; discarded\r\n");
		} else if (s_confirmation_active != 0U) {
			s_line[s_line_length] = '\0';
			Cli_HandleConfirmationAnswer(s_line);
			memset(s_line, 0, sizeof(s_line));
			s_line_length = 0U;

		} else if (s_line_length > 0U) {
			s_line[s_line_length] = '\0';
			Cli_ExecuteLine(s_line);
			memset(s_line, 0, sizeof(s_line));
			s_line_length = 0U;
		}
		if ((s_confirmation_active == 0U) && (s_maintenance_lock == 0U)
				&& (s_journal_export_active == 0U)) {
			Cli_SendPrompt();
		}
		return;
	}
	if ((byte == 0x08U) || (byte == 0x7FU)) {
		if ((s_line_length > 0U) && (s_discard_line == 0U)) {
			s_line_length--;
			s_line[s_line_length] = '\0';
			Cli_TxWrite("\b \b");
		}
		return;
	}
	if ((byte < 0x20U) || (byte > 0x7EU))
		return;
	if (s_discard_line != 0U)
		return;
	if (s_line_length < (CLI_LINE_SIZE - 1U)) {
		s_line[s_line_length++] = (char) byte;
		{
			char echo[2];
			echo[0] = (Cli_IsPasswordCharacter() != 0U) ? '*' : (char) byte;
			echo[1] = '\0';
			Cli_TxWrite(echo);
		}
	} else {
		s_discard_line = 1U;
	}
}
static void Cli_SendPrompt(void) {
	Cli_TxWrite("IR> ");
}
bool CliService_Init(void) {
	memset(&s_rx, 0, sizeof(s_rx));
	memset(&s_tx, 0, sizeof(s_tx));
	memset(s_line, 0, sizeof(s_line));
	s_rx_byte = 0U;
	s_line_length = 0U;
	s_discard_line = 0U;
	s_last_was_cr = 0U;
	s_rx_overflow_count = 0U;
	s_tx_drop_count = 0U;
	s_tx_queue_overflow_count = 0U;
	s_tx_start_error_count = 0U;
	s_rx_restart_error_count = 0U;
	s_last_config_save_status = ConfigService_GetSaveStatus();
	s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
	s_erase_confirmation_deadline = 0U;
	s_erase_operation_active = 0U;

	s_erase_waiting_config_reset = 0U;
	s_factory_reset_pending = 0U;
	s_confirmation_active = 0U;
	s_maintenance_lock = 0U;
	s_last_erase_progress_report = 0U;
	s_journal_export_state = CLI_JOURNAL_EXPORT_IDLE;
	s_journal_export_active = 0U;
	memset(&s_journal_cursor, 0, sizeof(s_journal_cursor));
	s_journal_export_total = 0U;
	s_journal_export_sent = 0U;
	s_journal_export_first_id = 0U;
	s_journal_export_last_id = 0U;
	s_photo_tx_state = CLI_PHOTO_TX_IDLE;
	s_photo_tx_active = false;
	s_photo_abort_requested = 0U;
	s_photo_tx_internal_write = false;
	s_photo_tx_failed = false;
	s_photo_tx_abort_after_chunk = false;
	s_photo_tx_all_mode = false;
	s_photo_tx_read_lock = false;
	s_photo_tx_current_active = false;
	s_photo_chunk_length = 0U;
	s_photo_chunk_crc = 0U;
	s_photo_chunk_ack_received = 0U;
	s_photo_chunk_nack_received = 0U;
	s_photo_chunk_retry_count = 0U;
	s_photo_chunk_ack_deadline = 0U;
	s_photo_list_state = CLI_PHOTO_LIST_IDLE;
	s_photo_list_active = false;
	s_photo_list_abort_requested = 0U;
	s_photo_list_read_lock = false;
	s_photo_list_index = 0U;
	s_photo_list_count = 0U;
	s_photo_list_sent = 0U;
	s_photo_list_skipped = 0U;
	s_photo_all_index = 0U;
	s_photo_all_count = 0U;
	s_photo_all_sent = 0U;
	s_photo_deferred_length = 0U;
	s_photo_deferred_text[0] = '\0';
	s_initialized = 1U;
	if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
		s_initialized = 0U;
		return false;
	}
	Cli_TxWrite("\r\nIncident Recorder CLI ready. Type 'help'.\r\n");
	Cli_SendPrompt();
	return true;
}
void CliService_Process(void) {
	uint8_t byte;
	uint8_t processed = 0U;
	ConfigSaveStatus_t save_status;
	StorageMaintenanceStatus_t erase_status;
	uint16_t completed_sectors;
	uint32_t failed_address;
	char notice[CLI_TX_FORMAT_SIZE];
	/* During destructive maintenance the RX ISR discards bytes and the parser
	 * is deliberately not run. No command can be accepted until completion. */
	if ((s_maintenance_lock == 0U) && (s_journal_export_active == 0U)
			&& !s_photo_tx_active && !s_photo_list_active) {
		while (processed < CLI_PROCESS_BUDGET) {
			if (Cli_TxFreeSpace() < CLI_TX_COMMAND_RESERVE)
				break;
			if (Cli_RxPop(&byte) == 0U)
				break;
			Cli_ProcessByte(byte);
			processed++;
			if ((s_maintenance_lock != 0U) || (s_journal_export_active != 0U)
					|| s_photo_tx_active || s_photo_list_active)
				break;
		}
	}
	if ((s_confirmation_active != 0U)
			&& ((int32_t) (HAL_GetTick() - s_erase_confirmation_deadline) >= 0)) {
		s_pending_erase_scope = CLI_ERASE_SCOPE_NONE;
		s_confirmation_active = 0U;
		Cli_AsyncNotice(
				"Destructive operation cancelled: confirmation timed out.");
	}
	save_status = ConfigService_GetSaveStatus();
	if ((s_last_config_save_status == CONFIG_SAVE_IN_PROGRESS)
			&& (save_status == CONFIG_SAVE_OK)) {
		if (s_factory_reset_pending != 0U) {
			Cli_FinishMaintenance(
					"[ERASE] factory reset complete; defaults restored, generation=0, old configuration slot verified erased.");
			StorageMaintenance_AcknowledgeComplete();
			Cli_ReinitializeJournal();
		} else {
			Cli_AsyncNotice(
					"[CONFIG] persistent save complete; new generation active.");
		}
	} else if ((s_last_config_save_status == CONFIG_SAVE_IN_PROGRESS)
			&& (save_status == CONFIG_SAVE_ERROR)) {
		if (s_factory_reset_pending != 0U)

		{
			Cli_FinishMaintenance(
					"[ERASE] ERROR: factory reset or old-slot erase verification failed; inspect Flash before further use.");
			StorageMaintenance_AcknowledgeComplete();
			Cli_ReinitializeJournal();
		} else {
			Cli_AsyncNotice(
					"[CONFIG] ERROR: save failed; previous configuration retained.");
		}
	}
	s_last_config_save_status = save_status;
	if (s_erase_operation_active != 0U) {
		erase_status = StorageMaintenance_GetStatus();
		completed_sectors = StorageMaintenance_GetCompletedSectors();
		if ((completed_sectors
				>= (uint16_t) (s_last_erase_progress_report + 256U))
				&& (erase_status == STORAGE_MAINTENANCE_IN_PROGRESS)) {
			s_last_erase_progress_report = (uint16_t) (completed_sectors
					& 0xFF00U);
			(void) snprintf(notice, sizeof(notice),
					"[ERASE] verified sectors: %u/2046.",
					(unsigned) completed_sectors);
			Cli_PrintProgress(notice);
		} else if (erase_status == STORAGE_MAINTENANCE_COMPLETE) {
			s_erase_operation_active = 0U;
			if (s_erase_waiting_config_reset != 0U) {
				ConfigResult_t result = ConfigService_FactoryReset();
				s_erase_waiting_config_reset = 0U;
				if (result == CONFIG_RESULT_OK) {
					s_factory_reset_pending = 1U;
					Cli_PrintProgress(
							"[ERASE] data partition erase and read-back verification complete; resetting factory configuration.");
				} else {
					Cli_FinishMaintenance(
							"[ERASE] ERROR: data partition was erased, but factory configuration reset could not start.");
					StorageMaintenance_AcknowledgeComplete();
				}
			} else {
				Cli_FinishMaintenance(
						"[ERASE] data partition erased and every sector verified (all bytes = 0xFF).");
				StorageMaintenance_AcknowledgeComplete();
				Cli_ReinitializeJournal();
			}
		} else if (erase_status == STORAGE_MAINTENANCE_ERROR) {
			failed_address = StorageMaintenance_GetFailedAddress();
			(void) snprintf(notice, sizeof(notice),
					"[ERASE] ERROR: erase/read-back verification failed at 0x%06lX after %u verified sectors.",
					(unsigned long) failed_address,
					(unsigned) completed_sectors);
			Cli_FinishMaintenance(notice);

		}
	}
	Cli_ProcessJournalExport();
	Cli_ProcessPhotoTransfer();
	Cli_ProcessPhotoList();
	Cli_TxKick();
}
uint32_t CliService_GetRxOverflowCount(void) {
	return s_rx_overflow_count;
}
uint32_t CliService_GetTxDropCount(void) {
	return s_tx_drop_count;
}
/* HAL callback: keep the ISR short; parsing and command execution stay in
 superloop. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
	uint16_t head;
	uint16_t next;
	if ((uart == &huart1) && (s_initialized != 0U)) {
		if (s_photo_tx_active || s_photo_list_active) {
			if (s_rx_byte == 0x03U) {
				if (s_photo_tx_active)
					s_photo_abort_requested = 1U;
				else
					s_photo_list_abort_requested = 1U;
			} else if (s_photo_tx_active) {
				/* ACK/NACK may arrive immediately after the final payload byte,
				 * before the super loop has advanced WAIT_CHUNK_DATA to WAIT_ACK. */
				if (s_rx_byte == CLI_PHOTO_ACK_BYTE)
					s_photo_chunk_ack_received = 1U;
				else if (s_rx_byte == CLI_PHOTO_NACK_BYTE)
					s_photo_chunk_nack_received = 1U;
			}
			s_rx_byte = 0U;
			if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK)
				s_rx_restart_error_count++;
			return;
		}
		if ((s_maintenance_lock != 0U) || (s_journal_export_active != 0U)) {
			s_rx_byte = 0U;
			if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
				s_rx_restart_error_count++;
			}
			return;
		}
		head = s_rx.head;
		next = Cli_NextIndex(head, CLI_RX_RING_SIZE);
		if (next != s_rx.tail) {
			s_rx.data[head] = s_rx_byte;
			s_rx.head = next;
		} else {
			s_rx_overflow_count++;
		}
		s_rx_byte = 0U;
		if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
			s_rx_restart_error_count++;
		}
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
	if ((uart == &huart1) && (s_tx.transfer_active != 0U)) {
		s_tx.tail = (uint16_t) ((s_tx.tail + s_tx.active_length)
				& (CLI_TX_RING_SIZE - 1U));
		s_tx.active_length = 0U;
		s_tx.transfer_active = 0U;
		Cli_TxKick();
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
	if ((uart == &huart1) && (s_initialized != 0U)) {
		s_rx_byte = 0U;

		if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
			s_rx_restart_error_count++;
		}
	}
}
bool CliService_IsMaintenanceLocked(void) {
	return (s_maintenance_lock != 0U) || (s_journal_export_active != 0U)
			|| s_photo_tx_active || s_photo_list_active;
}
