#ifndef LED_SERVICE_H
#define LED_SERVICE_H

#include <stdint.h>
#include "app/app_state.h"

#ifdef __cplusplus
extern "C" {
#endif

void LedService_Init(void);
void LedService_Update(AppState_t state, uint32_t now_ms);
/* Starts a three-pulse, non-blocking incident indication. */
void LedService_TriggerIncidentPattern(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* LED_SERVICE_H */
