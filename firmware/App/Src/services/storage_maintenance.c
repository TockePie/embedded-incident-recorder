#include "services/storage_maintenance.h"

#include "drivers/w25q64.h"
#include "spi.h"

#define STORAGE_DATA_START_ADDRESS   0x002000UL
#define STORAGE_DATA_END_ADDRESS     0x800000UL
#define STORAGE_SECTOR_SIZE          4096UL
#define STORAGE_DATA_SECTOR_COUNT    2046U

typedef enum {
	ERASE_STATE_IDLE = 0,
	ERASE_STATE_START_SECTOR,
	ERASE_STATE_WAIT_SECTOR,
	ERASE_STATE_VERIFY_SECTOR
} EraseState_t;

static EraseState_t s_state;
static StorageMaintenanceStatus_t s_status;
static uint32_t s_address;
static uint16_t s_completed_sectors;
static uint32_t s_failed_address;

bool StorageMaintenance_StartDataPartitionErase(void) {
	if (s_status != STORAGE_MAINTENANCE_IDLE) {
		return false;
	}

	s_address = STORAGE_DATA_START_ADDRESS;
	s_completed_sectors = 0U;
	s_failed_address = 0xFFFFFFFFUL;
	s_state = ERASE_STATE_START_SECTOR;
	s_status = STORAGE_MAINTENANCE_IN_PROGRESS;
	return true;
}

void StorageMaintenance_Process(void) {
	W25Q64_Status_t flash_status;
	bool busy = false;

	if (s_status != STORAGE_MAINTENANCE_IN_PROGRESS) {
		return;
	}

	if (s_state == ERASE_STATE_START_SECTOR) {
		flash_status = W25Q64_StartSectorErase(&hspi2, s_address);
		if (flash_status != W25Q64_STATUS_OK) {
			s_status = STORAGE_MAINTENANCE_ERROR;
			s_state = ERASE_STATE_IDLE;
			return;
		}
		s_state = ERASE_STATE_WAIT_SECTOR;
		return;
	}

	if (s_state == ERASE_STATE_WAIT_SECTOR) {
		flash_status = W25Q64_IsBusy(&hspi2, &busy);
		if (flash_status != W25Q64_STATUS_OK) {
			s_status = STORAGE_MAINTENANCE_ERROR;
			s_state = ERASE_STATE_IDLE;
			return;
		}
		if (busy) {
			return;
		}

		s_state = ERASE_STATE_VERIFY_SECTOR;
		return;
	}

	if (s_state == ERASE_STATE_VERIFY_SECTOR) {
		flash_status = W25Q64_VerifyErased(&hspi2, s_address,
		STORAGE_SECTOR_SIZE);
		if (flash_status != W25Q64_STATUS_OK) {
			s_failed_address = s_address;
			s_status = STORAGE_MAINTENANCE_ERROR;
			s_state = ERASE_STATE_IDLE;
			return;
		}

		/* Count only sectors whose complete 4 KiB range read back as 0xFF. */
		s_completed_sectors++;
		s_address += STORAGE_SECTOR_SIZE;
		if (s_address >= STORAGE_DATA_END_ADDRESS) {
			s_status = STORAGE_MAINTENANCE_COMPLETE;
			s_state = ERASE_STATE_IDLE;
		} else {
			s_state = ERASE_STATE_START_SECTOR;
		}
		return;
	}

	s_failed_address = s_address;
	s_status = STORAGE_MAINTENANCE_ERROR;
	s_state = ERASE_STATE_IDLE;
}

void StorageMaintenance_AcknowledgeComplete(void) {
	if (s_status == STORAGE_MAINTENANCE_COMPLETE) {
		s_status = STORAGE_MAINTENANCE_IDLE;
		s_state = ERASE_STATE_IDLE;
	}
}

StorageMaintenanceStatus_t StorageMaintenance_GetStatus(void) {
	return s_status;
}

uint16_t StorageMaintenance_GetCompletedSectors(void) {
	return s_completed_sectors;
}

uint32_t StorageMaintenance_GetFailedAddress(void) {
	return s_failed_address;
}
