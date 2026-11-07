#ifndef HEALTH_POLICY_H
#define HEALTH_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEALTH_POLICY_FLASH_ERROR_LIMIT   2U
#define HEALTH_POLICY_CAMERA_ERROR_LIMIT  3U

typedef enum {
	HEALTH_POLICY_ACTION_NONE = 0U,
	HEALTH_POLICY_ACTION_MPU_FAULT = (1U << 0),
	HEALTH_POLICY_ACTION_FLASH_FAULT = (1U << 1),
	HEALTH_POLICY_ACTION_CAMERA_DEGRADED = (1U << 2),
	HEALTH_POLICY_ACTION_QUEUE_WARNING = (1U << 3),
	HEALTH_POLICY_ACTION_UART_WARNING = (1U << 4)
} HealthPolicyAction_t;

typedef struct {
	bool mpu_sample_fresh;
	uint8_t flash_error_streak;
	uint8_t camera_error_streak;
	bool queue_drop_delta;
	bool uart_error_delta;
} HealthPolicyInput_t;

/* Convert a health snapshot into actions for the application fault policy. */
uint32_t HealthPolicy_Evaluate(const HealthPolicyInput_t *input);

/* Validate the policy logic without accessing hardware. */
bool HealthPolicy_RunSelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* HEALTH_POLICY_H */
