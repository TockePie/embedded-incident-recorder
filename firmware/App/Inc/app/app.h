#ifndef APP_H
#define APP_H

#include <stdint.h>

#include "app/app_state.h"
#include "services/self_test.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the application and execute startup diagnostics. */
void App_Init(void);

/** Execute one foreground super-loop iteration. */
void App_RunOnce(void);

/** Return the result of the startup self-test. */
SelfTestOverallStatus_t App_GetStartupStatus(void);

/** Return the current application state. */
AppState_t App_GetState(void);

/** Enter the latched FAULT state until reset or explicit recovery. */
void App_EnterFault(uint32_t fault_code);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
