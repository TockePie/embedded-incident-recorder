#include "app/app_state.h"
#include <stddef.h>

void AppState_Init(AppStateManager_t *manager) {
	if (manager == NULL) {
		return;
	}

	manager->state = APP_STATE_DISARMED;
	manager->toggle_requested = false;
	manager->fault_code = 0U;
}

void AppState_RequestToggle(AppStateManager_t *manager) {
	if ((manager != NULL) && (manager->state != APP_STATE_FAULT)) {
		manager->toggle_requested = true;
	}
}

void AppState_Process(AppStateManager_t *manager) {
	if (manager == NULL) {
		return;
	}

	if (manager->state == APP_STATE_FAULT) {
		manager->toggle_requested = false;
		return;
	}

	if (manager->toggle_requested != false) {
		manager->toggle_requested = false;
		manager->state =
				(manager->state == APP_STATE_DISARMED) ?
						APP_STATE_ARMED : APP_STATE_DISARMED;
	}
}

void AppState_EnterFault(AppStateManager_t *manager, uint32_t fault_code) {
	if (manager == NULL) {
		return;
	}

	manager->fault_code = fault_code;
	manager->toggle_requested = false;
	manager->state = APP_STATE_FAULT;
}

AppState_t AppState_Get(const AppStateManager_t *manager) {
	return (manager != NULL) ? manager->state : APP_STATE_FAULT;
}

uint32_t AppState_GetFaultCode(const AppStateManager_t *manager) {
	return (manager != NULL) ? manager->fault_code : 0xFFFFFFFFUL;
}
