#include "services/health_policy.h"

uint32_t HealthPolicy_Evaluate(const HealthPolicyInput_t *input) {
	uint32_t actions = HEALTH_POLICY_ACTION_NONE;

	if (input == 0) {
		return HEALTH_POLICY_ACTION_MPU_FAULT | HEALTH_POLICY_ACTION_FLASH_FAULT;
	}

	if (!input->mpu_sample_fresh) {
		actions |= HEALTH_POLICY_ACTION_MPU_FAULT;
	}

	if (input->flash_error_streak >= HEALTH_POLICY_FLASH_ERROR_LIMIT) {
		actions |= HEALTH_POLICY_ACTION_FLASH_FAULT;
	}

	if (input->camera_error_streak >= HEALTH_POLICY_CAMERA_ERROR_LIMIT) {
		actions |= HEALTH_POLICY_ACTION_CAMERA_DEGRADED;
	}

	if (input->queue_drop_delta) {
		actions |= HEALTH_POLICY_ACTION_QUEUE_WARNING;
	}

	if (input->uart_error_delta) {
		actions |= HEALTH_POLICY_ACTION_UART_WARNING;
	}

	return actions;
}

bool HealthPolicy_RunSelfTest(void) {
	HealthPolicyInput_t input = { 0 };
	uint32_t actions;

	input.mpu_sample_fresh = true;
	actions = HealthPolicy_Evaluate(&input);
	if (actions != HEALTH_POLICY_ACTION_NONE) {
		return false;
	}

	input.mpu_sample_fresh = false;
	actions = HealthPolicy_Evaluate(&input);
	if ((actions & HEALTH_POLICY_ACTION_MPU_FAULT) == 0U) {
		return false;
	}

	input.mpu_sample_fresh = true;
	input.flash_error_streak = HEALTH_POLICY_FLASH_ERROR_LIMIT;
	actions = HealthPolicy_Evaluate(&input);
	if ((actions & HEALTH_POLICY_ACTION_FLASH_FAULT) == 0U) {
		return false;
	}

	input.flash_error_streak = 0U;
	input.camera_error_streak = HEALTH_POLICY_CAMERA_ERROR_LIMIT;
	actions = HealthPolicy_Evaluate(&input);
	if ((actions & HEALTH_POLICY_ACTION_CAMERA_DEGRADED) == 0U) {
		return false;
	}

	input.camera_error_streak = 0U;
	input.queue_drop_delta = true;
	input.uart_error_delta = true;
	actions = HealthPolicy_Evaluate(&input);
	if (((actions & HEALTH_POLICY_ACTION_QUEUE_WARNING) == 0U)
			|| ((actions & HEALTH_POLICY_ACTION_UART_WARNING) == 0U)
			|| ((actions
					& (HEALTH_POLICY_ACTION_MPU_FAULT
							| HEALTH_POLICY_ACTION_FLASH_FAULT)) != 0U)) {
		return false;
	}

	return true;
}
