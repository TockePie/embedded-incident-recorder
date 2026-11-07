#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEALTH_MONITOR_MPU_STALE_LIMIT_MS   150U
#define HEALTH_MONITOR_STARTUP_GRACE_MS     500U
#define HEALTH_MONITOR_FLASH_PERIOD_MS     2000U
#define HEALTH_MONITOR_CAMERA_PERIOD_MS    2000U

typedef enum {
	HEALTH_DEVICE_UNKNOWN = 0U,
	HEALTH_DEVICE_OK,
	HEALTH_DEVICE_NOT_INITIALIZED,
	HEALTH_DEVICE_DEGRADED,
	HEALTH_DEVICE_FAILED,
	HEALTH_DEVICE_BUSY
} HealthDeviceState_t;

typedef struct {
	HealthDeviceState_t mpu_state;
	HealthDeviceState_t flash_state;
	HealthDeviceState_t camera_state;
	bool photos_available;
	uint32_t mpu_sample_age_ms;
	uint32_t flash_error_count;
	uint32_t camera_error_count;
	uint32_t queue_drop_count;
	uint32_t uart_rx_overflow_count;
	uint32_t uart_tx_drop_count;
	uint32_t uart_hal_error_count;
	uint32_t uart_hal_error_code;
	uint32_t fault_code;
} HealthMonitorSnapshot_t;

void HealthMonitor_Init(uint32_t now_ms);
void HealthMonitor_Process(uint32_t now_ms);
void HealthMonitor_GetSnapshot(HealthMonitorSnapshot_t *snapshot);
const char *HealthMonitor_DeviceStateName(HealthDeviceState_t state);

#ifdef __cplusplus
}
#endif

#endif /* HEALTH_MONITOR_H */
