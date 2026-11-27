#include "app/app.h"

#include "app/rtos_sync.h"
#include "app/app_scheduler.h"
#include "metrics/metrics.h"
#include "app/app_state.h"
#include "bsp/diag_uart.h"
#include "bsp/user_button.h"
#include "services/cli_service.h"
#include "services/config_service.h"
#include "services/health_monitor.h"
#include "services/imu_service.h"
#include "services/incident_detector.h"
#include "services/incident_journal.h"
#include "services/photo_store.h"
#include "services/photo_capture_service.h"
#include "services/led_service.h"
#include "services/storage_maintenance.h"
#include "drivers/ov7670_dvp_capture.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>

#define APP_TASK_BUTTON_PERIOD_MS       5U
#define APP_TASK_STATE_PERIOD_MS        5U
#define APP_TASK_LED_PERIOD_MS         10U
#define APP_TASK_HEALTH_PERIOD_MS     100U
#define APP_TASK_CLI_PERIOD_MS          5U
#define APP_TASK_CONFIG_PERIOD_MS       5U
#define APP_TASK_JOURNAL_PERIOD_MS      5U

#define APP_FAULT_STARTUP_SELF_TEST      0x00010001UL
#define APP_FAULT_SCHEDULER_SETUP        0x00010002UL
#define APP_FAULT_CONFIG_INIT            0x00030001UL
#define APP_FAULT_CONFIG_SAVE            0x00030002UL
#define APP_FAULT_STORAGE_MAINTENANCE    0x00030003UL
#define APP_FAULT_JOURNAL_INIT            0x00060001UL

static SelfTestOverallStatus_t s_startup_status = SELF_TEST_OVERALL_FAIL;
static AppScheduler_t s_scheduler;
static AppStateManager_t s_state_manager;
static bool s_initialized;
static uint32_t s_led_event_total;

static void AppTask_Button(uint32_t now_ms, void *context) {
	(void) context;

	if (UserButton_Process(now_ms)) {
		Metrics_ProbeSet(METRICS_PROBE_BUTTON, true);
		(void) Metrics_EventPush(METRICS_EVENT_BUTTON_PRESS,
				(uint32_t) AppState_Get(&s_state_manager));
		/* Keep the system DISARMED while a Flash save/erase is in progress.
		 AppState_RequestToggle() independently rejects requests in FAULT. */
		if ((ConfigService_GetSaveStatus() != CONFIG_SAVE_IN_PROGRESS)
				&& (StorageMaintenance_GetStatus() == STORAGE_MAINTENANCE_IDLE)
				&& (CliService_IsMaintenanceLocked() == false)) {
			AppState_RequestToggle(&s_state_manager);
		}
		Metrics_ProbeSet(METRICS_PROBE_BUTTON, false);
	}
}

static void AppTask_StateManager(uint32_t now_ms, void *context) {
	AppState_t previous_state;
	AppState_t current_state;
	(void) now_ms;
	(void) context;
	previous_state = AppState_Get(&s_state_manager);
	AppState_Process(&s_state_manager);
	current_state = AppState_Get(&s_state_manager);
	if (current_state != previous_state) {
		const uint32_t argument = ((uint32_t) previous_state << 16U)
				| (uint32_t) current_state;
		(void) Metrics_EventPush(METRICS_EVENT_STATE_CHANGE, argument);
	}
}

// static void AppTask_Led(uint32_t now_ms, void *context) {
// 	(void) context;
// 	LedService_Update(AppState_Get(&s_state_manager), now_ms);
// }

static void AppTask_CLI(uint32_t now_ms, void *context) {
	(void) now_ms;
	(void) context;
	CliService_Process();
}

static void AppTask_Config(uint32_t now_ms, void *context) {
	(void) now_ms;
	(void) context;

	ConfigService_Process();
	StorageMaintenance_Process();

	if (ConfigService_GetSaveStatus() == CONFIG_SAVE_ERROR) {
		App_EnterFault(APP_FAULT_CONFIG_SAVE);
	}

	if (StorageMaintenance_GetStatus() == STORAGE_MAINTENANCE_ERROR) {
		App_EnterFault(APP_FAULT_STORAGE_MAINTENANCE);
	}
}

static void AppTask_Journal(uint32_t now_ms, void *context) {
	(void) now_ms;
	(void) context;
	IncidentJournal_Process();
	PhotoStore_Process();
	PhotoCaptureService_Process();
}

static void AppTask_HealthMonitor(uint32_t now_ms, void *context) {
	(void) context;
	HealthMonitor_Process(now_ms);
}

static bool App_RegisterTasks(uint32_t now_ms) {
	// if (AppScheduler_AddPeriodic(&s_scheduler, AppTask_Button, NULL,
	// APP_TASK_BUTTON_PERIOD_MS, now_ms,
	// 		METRICS_EXEC_TASK_BUTTON) != APP_SCHEDULER_OK) {
	// 	return false;
	// }
 //
	// if (AppScheduler_AddPeriodic(&s_scheduler, AppTask_StateManager, NULL,
	// APP_TASK_STATE_PERIOD_MS, now_ms,
	// 		METRICS_EXEC_TASK_STATE_MANAGER) != APP_SCHEDULER_OK) {
	// 	return false;
	// }

	// if (AppScheduler_AddPeriodic(&s_scheduler, AppTask_Led, NULL,
	// APP_TASK_LED_PERIOD_MS, now_ms,
	// 		METRICS_EXEC_TASK_LED) != APP_SCHEDULER_OK) {
	// 	return false;
	// }

	if (AppScheduler_AddPeriodic(&s_scheduler, AppTask_CLI, NULL,
	APP_TASK_CLI_PERIOD_MS, now_ms,
			METRICS_EXEC_TASK_CLI) != APP_SCHEDULER_OK) {
		return false;
	}

	if (AppScheduler_AddPeriodic(&s_scheduler, AppTask_Config, NULL,
	APP_TASK_CONFIG_PERIOD_MS, now_ms,
			METRICS_EXEC_TASK_CONFIG) != APP_SCHEDULER_OK) {
		return false;
	}

	if (AppScheduler_AddPeriodic(&s_scheduler, AppTask_Journal, NULL,
	APP_TASK_JOURNAL_PERIOD_MS, now_ms,
			METRICS_EXEC_TASK_JOURNAL) != APP_SCHEDULER_OK) {
		return false;
	}

	// if (AppScheduler_AddPeriodic(&s_scheduler, AppTask_HealthMonitor, NULL,
	// APP_TASK_HEALTH_PERIOD_MS, now_ms,
	// 		METRICS_EXEC_TASK_HEALTH_MONITOR) != APP_SCHEDULER_OK) {
	// 	return false;
	// }

	return true;
}

void App_Init(void) {
	const ConfigData_t *config = NULL;
	bool sealed = false;
	uint32_t generation = 0U;
	uint32_t now_ms;

	s_initialized = false;
	s_led_event_total = 0U;
	IncidentDetector_Init();
	AppState_Init(&s_state_manager);
	AppScheduler_Init(&s_scheduler);
	LedService_Init();

	DiagUART_Printf("\r\nIncident Recorder firmware starting...\r\n");
	s_startup_status = SelfTest_Run();

	/* Ignore any button activity during the blocking startup self-test. */
	UserButton_Init(HAL_GetTick());

	if (s_startup_status == SELF_TEST_OVERALL_FAIL) {
		App_EnterFault(APP_FAULT_STARTUP_SELF_TEST);
		DiagUART_Printf(
				"[APP] Critical startup self-test failure: system latched in FAULT.\r\n");
	} else if (s_startup_status == SELF_TEST_OVERALL_DEGRADED) {
		DiagUART_Printf(
				"[APP] Startup completed with limitations; initial state=DISARMED.\r\n");
	} else {
		DiagUART_Printf(
				"[APP] Startup checks passed; initial state=DISARMED.\r\n");
	}

	if (!ConfigService_Init()) {
		App_EnterFault(APP_FAULT_CONFIG_INIT);
		DiagUART_Printf(
				"[CONFIG] W25Q64 configuration read failed; system latched in FAULT.\r\n");
	} else {
		ConfigService_Get(&config, &sealed, &generation);
		if (config != NULL) {
			DiagUART_Printf(
					"[CONFIG] loaded mode=%s sealed=%u generation=%lu.\r\n",
					(config->mode == CONFIG_MODE_SITE) ? "SITE" : "CARGO",
					(unsigned int) sealed, (unsigned long) generation);
			if (generation == 0U) {
				DiagUART_Printf(
						"[CONFIG] generation=0: factory/default configuration baseline; system remains DISARMED until explicitly armed.\r\n");
			}
		} else {
			App_EnterFault(APP_FAULT_CONFIG_INIT);
			DiagUART_Printf(
					"[CONFIG] configuration service returned no active configuration; system latched in FAULT.\r\n");
		}
	}

	if (!CliService_Init()) {
		DiagUART_Printf(
				"[CLI] USART1 receive interrupt could not be started; CLI unavailable, autonomous operation retained.\r\n");
	} else {
		DiagUART_Printf(
				"[CLI] USART1 command interface ready (115200 8-N-1, PA9/PA10).\r\n");
	}

	now_ms = HAL_GetTick();
	HealthMonitor_Init(now_ms);

	if (IncidentJournal_Init() != INCIDENT_JOURNAL_OK) {
		App_EnterFault(APP_FAULT_JOURNAL_INIT);
		DiagUART_Printf(
				"[JOURNAL] initialization/recovery failed; system latched in FAULT.\r\n");
	}

	if (PhotoStore_Init() == PHOTO_STORE_STATUS_OK) {
		DiagUART_Printf(
				"[PHOTO] W25Q64 photo partition recovery started (cooperative).\r\n");
	} else {
		DiagUART_Printf(
				"[PHOTO] photo storage initialization failed; event journal remains available.\r\n");
	}

	/* Camera is a non-critical capability. A failed SCCB/configuration bring-up
	 * leaves event detection and journaling operational in degraded mode. */
	(void) PhotoCaptureService_Init();

	/* Startup services above may perform blocking Flash recovery and camera
	 * initialization. Refresh the scheduler epoch immediately before task
	 * registration so startup time is not misreported as missed task periods. */
	now_ms = HAL_GetTick();
	if (!App_RegisterTasks(now_ms)) {
		App_EnterFault(APP_FAULT_SCHEDULER_SETUP);
		DiagUART_Printf(
				"[APP] Scheduler initialization error: system latched in FAULT.\r\n");
		return;
	}

	s_initialized = true;
}

void App_RunOnce(void) {
	if (s_initialized) {
		const ConfigData_t *config = NULL;
		bool sealed = false;
		uint32_t generation = 0U;
		const MPU6050_Sample_t *sample;
		uint32_t sequence;
		uint32_t sample_tick;

		/* Poll DVP frame state/timeouts on every super-loop pass. Individual
		 * HREF line finalization is handled immediately in the HREF EXTI ISR;
		 * do not move this frame-state poll into the 5 ms scheduler. */
		Metrics_ExecBegin(METRICS_EXEC_FOREGROUND_CAMERA, 0U);
		Metrics_ProbeSet(METRICS_PROBE_CAMERA, true);
		OV7670_DvpCapture_Process();
		Metrics_ProbeSet(METRICS_PROBE_CAMERA, false);
		Metrics_ExecEnd(METRICS_EXEC_FOREGROUND_CAMERA);

		/* Acquire and classify first: CLI/Flash foreground work must not delay
		 processing of a sample that has already arrived from the MPU-6050. */
		Metrics_ExecBegin(METRICS_EXEC_FOREGROUND_IMU, 0U);
		Metrics_ProbeSet(METRICS_PROBE_IMU, true);
		ImuService_RunOnce();
		Metrics_ProbeSet(METRICS_PROBE_IMU, false);
		Metrics_ExecEnd(METRICS_EXEC_FOREGROUND_IMU);
		ConfigService_Get(&config, &sealed, &generation);
		(void) sealed;
		(void) generation;

		sample = ImuService_GetLatestSample();
		sequence = ImuService_GetSampleSequence();
		sample_tick = ImuService_GetLatestSampleTick();

		if (config != NULL) {
			const ConfigProfile_t *profile =
					(config->mode == CONFIG_MODE_SITE) ?
							&config->site : &config->cargo;

			Metrics_ExecBegin(METRICS_EXEC_FOREGROUND_DETECTOR, 0U);
			IncidentDetector_ProcessSample(
					(AppState_Get(&s_state_manager) == APP_STATE_ARMED),
					(ConfigMode_t) config->mode, profile, sample, sequence,
					sample_tick);
			Metrics_ExecEnd(METRICS_EXEC_FOREGROUND_DETECTOR);
			{
				IncidentDetectorStats_t detector_stats;
				uint32_t event_total;
				IncidentDetector_GetStats(&detector_stats);
				event_total = detector_stats.shock_events
						+ detector_stats.tilt_events
						+ detector_stats.temperature_events
						+ detector_stats.vibration_events;
				if (event_total != s_led_event_total) {
					s_led_event_total = event_total;
					(void) Metrics_EventPush(
							METRICS_EVENT_INCIDENT_COUNT_CHANGED, event_total);
					LedService_TriggerIncidentPattern(HAL_GetTick());
				}
			}
		}

		AppScheduler_Run(&s_scheduler, HAL_GetTick());
	} else {
		/* Keep the LED fault indication active even if scheduler setup failed. */
		LedService_Update(AppState_Get(&s_state_manager), HAL_GetTick());
	}
}

SelfTestOverallStatus_t App_GetStartupStatus(void) {
	return s_startup_status;
}

AppState_t App_GetState(void) {
	return AppState_Get(&s_state_manager);
}

void App_EnterFault(uint32_t fault_code) {
	if (AppState_Get(&s_state_manager) != APP_STATE_FAULT) {
		AppState_EnterFault(&s_state_manager, fault_code);
		(void) Metrics_EventPush(METRICS_EVENT_FAULT_ENTERED, fault_code);
	}
}

void App_StepButton(uint32_t now_ms) {
	AppTask_Button(now_ms, NULL);
}

void App_StepStateManager(uint32_t now_ms) {
	AppTask_StateManager(now_ms, NULL);
}

void App_StepHealthMonitor(uint32_t now_ms) {
	AppTask_HealthMonitor(now_ms, NULL);
}

void App_StepImu(void)
{
	const ConfigData_t *config = NULL;
	bool sealed = false;
	uint32_t generation = 0U;
	const MPU6050_Sample_t *sample;
	uint32_t sequence;
	uint32_t sample_tick;
	bool is_armed;

	if (!s_initialized) {
		return;
	}

	Metrics_ExecBegin(METRICS_EXEC_FOREGROUND_IMU, 0U);
	Metrics_ProbeSet(METRICS_PROBE_IMU, true);
	ImuService_RunOnce();
	Metrics_ProbeSet(METRICS_PROBE_IMU, false);
	Metrics_ExecEnd(METRICS_EXEC_FOREGROUND_IMU);
	
	/* Metric: interval between new IMU samples. Placed right after the
	 * read and before RtosSync_LockFlash(), so that waiting for the Flash
	 * mutex is not counted into the sample interval. */
	Metrics_ImuSampleObserve(ImuService_GetSampleSequence(),
			ImuService_GetLatestSampleTick(), METRICS_IMU_SAMPLE_PERIOD_MS);

	RtosSync_LockFlash();
	ConfigService_Get(&config, &sealed, &generation);
	RtosSync_UnlockFlash();
	(void)sealed;
	(void)generation;

	sample = ImuService_GetLatestSample();
	sequence = ImuService_GetSampleSequence();
	sample_tick = ImuService_GetLatestSampleTick();

	if (config != NULL) {
		const ConfigProfile_t *profile =
		(config->mode == CONFIG_MODE_SITE) ? &config->site : &config->cargo;

		RtosSync_LockState();
		is_armed = (AppState_Get(&s_state_manager) == APP_STATE_ARMED);
		RtosSync_UnlockState();

		Metrics_ExecBegin(METRICS_EXEC_FOREGROUND_DETECTOR, 0U);
		IncidentDetector_ProcessSample(
			is_armed,
			(ConfigMode_t)config->mode, profile, sample, sequence, sample_tick);
		Metrics_ExecEnd(METRICS_EXEC_FOREGROUND_DETECTOR);

		{
			IncidentDetectorStats_t detector_stats;
			uint32_t event_total;
			IncidentDetector_GetStats(&detector_stats);
			event_total = detector_stats.shock_events
			+ detector_stats.tilt_events
			+ detector_stats.temperature_events
			+ detector_stats.vibration_events;
			if (event_total != s_led_event_total) {
				s_led_event_total = event_total;
				(void)Metrics_EventPush(METRICS_EVENT_INCIDENT_COUNT_CHANGED, event_total);
				RtosSync_LockState();
				LedService_TriggerIncidentPattern(HAL_GetTick());
				RtosSync_UnlockState();
			}
		}
	}
}

void App_StepCamera(void)
{
	if (!s_initialized) {
		return;
	}

	Metrics_ExecBegin(METRICS_EXEC_FOREGROUND_CAMERA, 0U);
	Metrics_ProbeSet(METRICS_PROBE_CAMERA, true);
	OV7670_DvpCapture_Process();
	Metrics_ProbeSet(METRICS_PROBE_CAMERA, false);
	Metrics_ExecEnd(METRICS_EXEC_FOREGROUND_CAMERA);
}

void App_StepStorage(void)
{
	if (!s_initialized) {
		return;
	}

	RtosSync_LockFlash();

	Metrics_ExecBegin(METRICS_EXEC_TASK_CONFIG, 5U);
	ConfigService_Process();
	StorageMaintenance_Process();
	if (ConfigService_GetSaveStatus() == CONFIG_SAVE_ERROR) {
		App_EnterFault(APP_FAULT_CONFIG_SAVE);
	}
	if (StorageMaintenance_GetStatus() == STORAGE_MAINTENANCE_ERROR) {
		App_EnterFault(APP_FAULT_STORAGE_MAINTENANCE);
	}
	Metrics_ExecEnd(METRICS_EXEC_TASK_CONFIG);

	Metrics_ExecBegin(METRICS_EXEC_TASK_JOURNAL, 5U);
	IncidentJournal_Process();
	PhotoStore_Process();
	PhotoCaptureService_Process();
	Metrics_ExecEnd(METRICS_EXEC_TASK_JOURNAL);

	RtosSync_UnlockFlash();
}

void App_StepCli(void)
{
	if (!s_initialized) {
		return;
	}

	Metrics_ExecBegin(METRICS_EXEC_TASK_CLI, 5U);
	RtosSync_LockFlash();
	CliService_Process();
	RtosSync_UnlockFlash();
	Metrics_ExecEnd(METRICS_EXEC_TASK_CLI);
}
