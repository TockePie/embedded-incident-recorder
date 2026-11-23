#ifndef APP_RTOS_SYNC_H
#define APP_RTOS_SYNC_H

#include <stdbool.h>

bool RtosSync_Init(void);

void RtosSync_LockFlash(void);
void RtosSync_UnlockFlash(void);

void RtosSync_LockDiagUart(void);
void RtosSync_UnlockDiagUart(void);

void RtosSync_LockState(void);
void RtosSync_UnlockState(void);

#endif /* APP_RTOS_SYNC_H */
