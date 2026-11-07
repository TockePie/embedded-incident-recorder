#ifndef DIAG_UART_H
#define DIAG_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Send a formatted diagnostic line over USART2 (ST-LINK virtual COM port).
 * This blocking function is intended for foreground diagnostics and must not
 * be called from an interrupt handler.
 */
void DiagUART_Printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_UART_H */
