#ifndef CLI_SERVICE_H
#define CLI_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USART1 command interface: PA9 TX, PA10 RX, 115200 baud, 8-N-1. */
bool CliService_Init(void);
void CliService_Process(void);
uint32_t CliService_GetRxOverflowCount(void);
uint32_t CliService_GetTxDropCount(void);
bool CliService_IsMaintenanceLocked(void);

#ifdef __cplusplus
}
#endif

#endif /* CLI_SERVICE_H */
