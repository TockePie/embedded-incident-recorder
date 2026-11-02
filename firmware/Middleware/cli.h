#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include <stdbool.h>

#define CLI_BUF_SIZE 128

typedef struct {
    char buffer[CLI_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
} RingBuffer_t;

void CLI_Init(void);
void CLI_OnRxByte(char ch);
void Task_CLI(void);

#endif // CLI_H
