#ifndef USER_BUTTON_H
#define USER_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USER_BUTTON_DEBOUNCE_MS 30U

void UserButton_Init(uint32_t now_ms);

/** Return true once for each debounced press transition. */
bool UserButton_Process(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* USER_BUTTON_H */
