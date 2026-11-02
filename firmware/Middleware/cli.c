#include "cli.h"
#include "super_loop.h"
#include <stdio.h>
#include <string.h>

static RingBuffer_t rx_ring = {0};
static char cmd_line[CLI_BUF_SIZE];
static uint8_t cmd_pos = 0;

void CLI_Init(void) {
    rx_ring.head = 0;
    rx_ring.tail = 0;
}

// Викликається з ISR переривання UART
void CLI_OnRxByte(char ch) {
    uint16_t next = (rx_ring.head + 1) % CLI_BUF_SIZE;
    if (next != rx_ring.tail) { // Буфер не повний
        rx_ring.buffer[rx_ring.head] = ch;
        rx_ring.head = next;
    }
}

static bool CLI_PopByte(char *ch) {
    if (rx_ring.head == rx_ring.tail) return false;
    *ch = rx_ring.buffer[rx_ring.tail];
    rx_ring.tail = (rx_ring.tail + 1) % CLI_BUF_SIZE;
    return true;
}

static void CLI_ParseCommand(char *cmd) {
    if (strcmp(cmd, "STATS") == 0) {
        printf("[CLI] Command: STATS executed\r\n");
    } else if (strcmp(cmd, "CAPTURE") == 0) {
        printf("[CLI] Command: Manual Capture Triggered\r\n");
    } else if (strcmp(cmd, "HELP") == 0) {
        printf("[CLI] Available commands: STATS, CAPTURE, HELP\r\n");
    } else {
        printf("[CLI] Unknown command: %s\r\n", cmd);
    }
}

void Task_CLI(void) {
    char ch;
    while (CLI_PopByte(&ch)) {
        if (ch == '\r' || ch == '\n') {
            if (cmd_pos > 0) {
                cmd_line[cmd_pos] = '\0';
                CLI_ParseCommand(cmd_line);
                cmd_pos = 0;
            }
        } else if (cmd_pos < CLI_BUF_SIZE - 1) {
            cmd_line[cmd_pos++] = ch;
        }
    }
}
