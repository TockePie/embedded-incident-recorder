#ifndef INCIDENT_DETECTOR_H
#define INCIDENT_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers/mpu6050.h"
#include "services/config_service.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INCIDENT_QUEUE_CAPACITY 20U

typedef enum {
	INCIDENT_TYPE_SHOCK = 1U,
	INCIDENT_TYPE_TILT = 2U,
	INCIDENT_TYPE_TEMPERATURE = 3U,
	INCIDENT_TYPE_VIBRATION = 4U
} IncidentType_t;

typedef struct {
	uint32_t sequence;
	uint32_t timestamp_ms;
	ConfigMode_t mode;
	IncidentType_t type;
	/* SHOCK/VIBRATION: mg; TILT: centidegrees; TEMPERATURE: milli-degrees C. */
	int32_t value;
	int32_t threshold;
} IncidentEvent_t;

typedef struct {
	uint32_t samples_processed;
	uint32_t orientation_ready;
	uint32_t queue_count;
	uint32_t queue_dropped;
	uint32_t shock_events;
	uint32_t tilt_events;
	uint32_t temperature_events;
	uint32_t vibration_events;
} IncidentDetectorStats_t;

void IncidentDetector_Init(void);
void IncidentDetector_ProcessSample(bool armed, ConfigMode_t mode,
		const ConfigProfile_t *profile, const MPU6050_Sample_t *sample,
		uint32_t sample_sequence, uint32_t sample_tick_ms);
bool IncidentDetector_Peek(uint8_t index, IncidentEvent_t *event);
bool IncidentDetector_Pop(IncidentEvent_t *event);
void IncidentDetector_GetStats(IncidentDetectorStats_t *stats);
const char *IncidentDetector_TypeName(IncidentType_t type);

#ifdef __cplusplus
}
#endif

#endif /* INCIDENT_DETECTOR_H */
