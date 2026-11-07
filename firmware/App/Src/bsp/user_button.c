#include "bsp/user_button.h"
#include "main.h"

/* NUCLEO-F411RE B1 USER button: PC13 is HIGH while pressed. */
#define USER_BUTTON_PRESSED_LEVEL GPIO_PIN_SET

static GPIO_PinState s_raw_state;
static GPIO_PinState s_stable_state;
static uint32_t s_last_raw_change_ms;

void UserButton_Init(uint32_t now_ms) {
	s_raw_state = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
	s_stable_state = s_raw_state;
	s_last_raw_change_ms = now_ms;
}

bool UserButton_Process(uint32_t now_ms) {
	const GPIO_PinState current = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port,
	USER_BUTTON_Pin);

	if (current != s_raw_state) {
		s_raw_state = current;
		s_last_raw_change_ms = now_ms;
	}

	if ((s_raw_state != s_stable_state)
			&& ((uint32_t) (now_ms - s_last_raw_change_ms)
					>= USER_BUTTON_DEBOUNCE_MS)) {
		s_stable_state = s_raw_state;
		return (s_stable_state == USER_BUTTON_PRESSED_LEVEL);
	}

	return false;
}
