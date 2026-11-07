#include "services/led_service.h"
#include "main.h"

#define LED_DISARMED_HALF_PERIOD_MS 500U
#define LED_FAULT_HALF_PERIOD_MS    100U
#define LED_INCIDENT_PULSE_MS       100U
#define LED_INCIDENT_PULSE_COUNT    3U
#define LED_INCIDENT_PATTERN_MS     (2U * LED_INCIDENT_PULSE_MS * LED_INCIDENT_PULSE_COUNT)

static uint8_t s_incident_pattern_active;
static uint32_t s_incident_pattern_start_ms;

static void led_write(GPIO_PinState state) {
	HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, state);
}

void LedService_Init(void) {
	s_incident_pattern_active = 0U;
	s_incident_pattern_start_ms = 0U;
	led_write(GPIO_PIN_RESET);
}

void LedService_TriggerIncidentPattern(uint32_t now_ms) {
	s_incident_pattern_start_ms = now_ms;
	s_incident_pattern_active = 1U;
}

void LedService_Update(AppState_t state, uint32_t now_ms) {
	GPIO_PinState output = GPIO_PIN_RESET;

	/* FAULT has the highest indication priority and is never masked by an
	 * incident pulse. The incident pattern has priority over normal states. */
	if (state == APP_STATE_FAULT) {
		output = (((now_ms / LED_FAULT_HALF_PERIOD_MS) & 1U) == 0U) ?
				GPIO_PIN_SET : GPIO_PIN_RESET;
	} else if (s_incident_pattern_active != 0U) {
		uint32_t elapsed = now_ms - s_incident_pattern_start_ms;
		if (elapsed < LED_INCIDENT_PATTERN_MS) {
			uint32_t phase = elapsed % (2U * LED_INCIDENT_PULSE_MS);
			output = (phase < LED_INCIDENT_PULSE_MS) ? GPIO_PIN_SET : GPIO_PIN_RESET;
		} else {
			s_incident_pattern_active = 0U;
		}
	}

	if ((state != APP_STATE_FAULT) && (s_incident_pattern_active == 0U)) {
		switch (state) {
		case APP_STATE_DISARMED:
			output = (((now_ms / LED_DISARMED_HALF_PERIOD_MS) & 1U) == 0U) ?
					GPIO_PIN_SET : GPIO_PIN_RESET;
			break;
		case APP_STATE_ARMED:
			output = GPIO_PIN_SET;
			break;
		default:
			output = GPIO_PIN_RESET;
			break;
		}
	}

	led_write(output);
}
