#ifndef STORAGE_MAINTENANCE_H
#define STORAGE_MAINTENANCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	STORAGE_MAINTENANCE_IDLE = 0,
	STORAGE_MAINTENANCE_IN_PROGRESS,
	STORAGE_MAINTENANCE_COMPLETE,
	STORAGE_MAINTENANCE_ERROR
} StorageMaintenanceStatus_t;

/* Erase data sectors 2 through 2047 while preserving configuration sectors. */
bool StorageMaintenance_StartDataPartitionErase(void);
void StorageMaintenance_Process(void);
void StorageMaintenance_AcknowledgeComplete(void);
StorageMaintenanceStatus_t StorageMaintenance_GetStatus(void);
uint16_t StorageMaintenance_GetCompletedSectors(void);
uint32_t StorageMaintenance_GetFailedAddress(void);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_MAINTENANCE_H */
