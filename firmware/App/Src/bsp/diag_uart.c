#include "bsp/diag_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"

#define DIAG_UART_TIMEOUT_MS 1000U
#define DIAG_UART_BUFFER_SIZE 192U

void DiagUART_Printf(const char *format, ...) {
	char buffer[DIAG_UART_BUFFER_SIZE];
	va_list args;
	int formatted_length;
	uint16_t transmit_length;

	if (format == NULL) {
		return;
	}

	va_start(args, format);
	formatted_length = vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	if (formatted_length <= 0) {
		return;
	}

	if ((size_t) formatted_length >= sizeof(buffer)) {
		transmit_length = (uint16_t) (sizeof(buffer) - 1U);
	} else {
		transmit_length = (uint16_t) formatted_length;
	}

	(void) HAL_UART_Transmit(&huart2, (uint8_t*) buffer, transmit_length,
	DIAG_UART_TIMEOUT_MS);
}
