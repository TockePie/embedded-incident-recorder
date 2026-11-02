#ifndef SUPER_LOOP_H
#define SUPER_LOOP_H

#include <stdint.h>
#include <stdbool.h>

#define SENSOR_POLL_PERIOD_US   (10000u)   
#define STATS_REPORT_PERIOD_US  (2000000u) 

typedef enum {
    STATE_IDLE = 0,
    STATE_TRIGGERED,
    STATE_CAPTURING,
    STATE_SAVING_TO_FLASH,
    STATE_ERROR
} AppState_t;

void SuperLoop_Init(void);
void SuperLoop_Run(void);
void SuperLoop_OnGPIO_Interrupt(uint16_t pin);

#endif 
