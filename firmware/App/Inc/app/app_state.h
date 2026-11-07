#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	APP_STATE_DISARMED = 0,
	APP_STATE_ARMED,
	APP_STATE_FAULT
} AppState_t;

typedef struct {
	AppState_t state;
	bool toggle_requested;
	uint32_t fault_code;
} AppStateManager_t;

void AppState_Init(AppStateManager_t *manager);
void AppState_RequestToggle(AppStateManager_t *manager);
void AppState_Process(AppStateManager_t *manager);
void AppState_EnterFault(AppStateManager_t *manager, uint32_t fault_code);
AppState_t AppState_Get(const AppStateManager_t *manager);
uint32_t AppState_GetFaultCode(const AppStateManager_t *manager);

#ifdef __cplusplus
}
#endif

#endif /* APP_STATE_H */
