#include "services/self_test.h"

#include <string.h>

#include "bsp/diag_uart.h"
#include "drivers/mpu6050.h"
#include "drivers/ov7670.h"
#include "drivers/w25q64.h"
#include "services/imu_service.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"

#define SELF_TEST_BANNER_LINE       "===============================================\r\n"
#define IMU_CALIBRATION_SAMPLES     MPU6050_DEFAULT_CAL_SAMPLES
#define IMU_CALIBRATION_PERIOD_MS   5U

static SelfTestReport_t s_report;

static void SelfTest_Mpu6050(void) {
	MPU6050_Identity_t identity = { 0U, 0U };
	MPU6050_CalibrationInfo_t calibration;
	MPU6050_Status_t status;
	const MPU6050_Handle_t *device;

	memset(&calibration, 0, sizeof(calibration));
	s_report.mpu6050_accel_scale_correction = 1.0f;

	status = MPU6050_Probe(&hi2c1, &identity);
	s_report.mpu6050_address_7bit = identity.address_7bit;
	s_report.mpu6050_who_am_i = identity.who_am_i;

	if (status != MPU6050_STATUS_OK) {
		s_report.mpu6050_ok = false;
		if (status == MPU6050_STATUS_NOT_FOUND) {
			DiagUART_Printf(
					"[IMU] MPU-6050 not found at 0x68 or 0x69 -> FAIL\r\n");
		} else if (status == MPU6050_STATUS_ID_MISMATCH) {
			DiagUART_Printf(
					"[IMU] I2C addr=0x%02X responded, WHO_AM_I=0x%02X (expected 0x68) -> FAIL\r\n",
					identity.address_7bit, identity.who_am_i);
		} else {
			DiagUART_Printf("[IMU] I2C communication error -> FAIL\r\n");
		}
		return;
	}

	DiagUART_Printf(
			"[IMU] MPU-6050 I2C1 addr=0x%02X WHO_AM_I=0x%02X -> PASS\r\n",
			identity.address_7bit, identity.who_am_i);

	status = ImuService_Init(&hi2c1, identity.address_7bit);
	if (status != MPU6050_STATUS_OK) {
		s_report.mpu6050_ok = false;
		DiagUART_Printf(
				"[IMU] configuration/read-back verification failed (status=%u) -> FAIL\r\n",
				(unsigned int) status);
		return;
	}

	s_report.mpu6050_ok = true;
	device = ImuService_GetDevice();
	if (device != NULL) {
		DiagUART_Printf(
				"[IMU] Config accel=+/-16g gyro=+/-500dps rate=%uHz DLPF_CFG=%u -> PASS\r\n",
				(unsigned int) MPU6050_GetSampleRateHz(device),
				(unsigned int) device->config.dlpf);
	}

	status = ImuService_CalibrateStationary(IMU_CALIBRATION_SAMPLES,
	IMU_CALIBRATION_PERIOD_MS, &calibration);
	s_report.mpu6050_gyro_calibrated = ImuService_IsGyroCalibrated();
	s_report.mpu6050_accel_scale_calibrated =
			ImuService_IsAccelScaleCalibrated();
	s_report.mpu6050_gravity_norm_before_g =
			calibration.accel_gravity_norm_before_g;
	s_report.mpu6050_accel_scale_correction =
			calibration.accel_scale_correction;
	s_report.mpu6050_max_gyro_stddev_dps = calibration.max_gyro_stddev_dps;
	s_report.mpu6050_max_accel_stddev_g = calibration.max_accel_stddev_g;
	memcpy(s_report.mpu6050_gyro_bias_dps, calibration.gyro_bias_dps,
			sizeof(s_report.mpu6050_gyro_bias_dps));

	if (status == MPU6050_STATUS_OK) {
		DiagUART_Printf(
				"[IMU] Stationary calibration samples=%u gyro_bias[mdps]=%+ld,%+ld,%+ld -> PASS\r\n",
				IMU_CALIBRATION_SAMPLES,
				(long) (calibration.gyro_bias_dps[0] * 1000.0f),
				(long) (calibration.gyro_bias_dps[1] * 1000.0f),
				(long) (calibration.gyro_bias_dps[2] * 1000.0f));
		DiagUART_Printf(
				"[IMU] Accel gravity reference=%ldmg common_scale=%ld/1000 -> APPLIED (startup orientation)\r\n",
				(long) (calibration.accel_gravity_norm_before_g * 1000.0f),
				(long) (calibration.accel_scale_correction * 1000.0f));
	} else if (status == MPU6050_STATUS_NOT_STATIONARY) {
		DiagUART_Printf(
				"[IMU] Startup calibration rejected: gyro_std=%ldmdps accel_std=%ldmg mean_gyro_candidate=%+ld,%+ld,%+ld mdps -> WARN\r\n",
				(long) (calibration.max_gyro_stddev_dps * 1000.0f),
				(long) (calibration.max_accel_stddev_g * 1000.0f),
				(long) (calibration.gyro_bias_dps[0] * 1000.0f),
				(long) (calibration.gyro_bias_dps[1] * 1000.0f),
				(long) (calibration.gyro_bias_dps[2] * 1000.0f));
	} else if (status == MPU6050_STATUS_GRAVITY_OUT_OF_RANGE) {
		DiagUART_Printf(
				"[IMU] Calibration gravity magnitude=%ldmg outside allowed range; gyro bias not accepted -> WARN\r\n",
				(long) (calibration.accel_gravity_norm_before_g * 1000.0f));
	} else {
		DiagUART_Printf(
				"[IMU] Calibration bus/driver error (status=%u) -> FAIL\r\n",
				(unsigned int) status);
		s_report.mpu6050_ok = false;
	}
}

static void SelfTest_Camera(void) {
	OV7670_Diagnostics_t diag;
	OV7670_Status_t status;

	memset(&diag, 0, sizeof(diag));
	status = OV7670_ProbeEx(&htim1, TIM_CHANNEL_1, &s_report.ov7670_pid,
			&s_report.ov7670_ver, &diag);
	s_report.ov7670_ok = (status == OV7670_STATUS_OK);

	if (status == OV7670_STATUS_OK) {
		DiagUART_Printf(
				"[CAM] OV7670 XCLK=12.5MHz PA8_SPEED=HIGH PID=0x%02X VER=0x%02X MID=0x%02X%02X SCCB_IDLE=%u/%u -> PASS\r\n",
				(unsigned int) s_report.ov7670_pid,
				(unsigned int) s_report.ov7670_ver, (unsigned int) diag.midh,
				(unsigned int) diag.midl, (unsigned int) diag.sioc_idle_high,
				(unsigned int) diag.siod_idle_high);
	} else if (status == OV7670_STATUS_XCLK_ERROR) {
		DiagUART_Printf(
				"[CAM] OV7670 XCLK setup/start failed (pin_configured=%u pwm_started=%u) -> FAIL\r\n",
				(unsigned int) diag.xclk_pin_configured,
				(unsigned int) diag.xclk_started);
	} else if (status == OV7670_STATUS_BUS_STUCK) {
		DiagUART_Printf(
				"[CAM] SCCB bus not idle: SIOC=%u SIOD=%u (both must be HIGH) -> FAIL\r\n",
				(unsigned int) diag.sioc_idle_high,
				(unsigned int) diag.siod_idle_high);
	} else if (status == OV7670_STATUS_ID_READ_ERROR) {
		DiagUART_Printf(
				"[CAM] SCCB read failed: PID=0x%02X VER=0x%02X MID=0x%02X%02X idle=%u/%u DC=%u,%u,%u -> FAIL\r\n",
				(unsigned int) s_report.ov7670_pid,
				(unsigned int) s_report.ov7670_ver, (unsigned int) diag.midh,
				(unsigned int) diag.midl, (unsigned int) diag.sioc_idle_high,
				(unsigned int) diag.siod_idle_high,
				(unsigned int) diag.dc_write_address,
				(unsigned int) diag.dc_register_address,
				(unsigned int) diag.dc_read_address);
	} else {
		DiagUART_Printf(
				"[CAM] OV7670 ID mismatch: PID=0x%02X VER=0x%02X (expected 0x76/0x73 or 0x76/0x70) -> FAIL\r\n",
				(unsigned int) s_report.ov7670_pid,
				(unsigned int) s_report.ov7670_ver);
	}
}

static void SelfTest_Flash(void) {
	W25Q64_StatusRegisters_t flash_status;
	W25Q64_Status_t status;

	status = W25Q64_Probe(&hspi2, s_report.w25q64_jedec_id);
	if (status != W25Q64_STATUS_OK) {
		s_report.w25q64_ok = false;
		if (status == W25Q64_STATUS_UNEXPECTED_ID) {
			DiagUART_Printf(
					"[FLASH] SPI responds, unexpected JEDEC=%02X %02X %02X (expected EF 40 17) -> FAIL\r\n",
					s_report.w25q64_jedec_id[0], s_report.w25q64_jedec_id[1],
					s_report.w25q64_jedec_id[2]);
		} else if (status == W25Q64_STATUS_TIMEOUT) {
			DiagUART_Printf(
					"[FLASH] W25Q64 BUSY timeout during startup -> FAIL\r\n");
		} else {
			DiagUART_Printf(
					"[FLASH] SPI2 communication error (status=%u) -> FAIL\r\n",
					(unsigned int) status);
		}
		return;
	}

	status = W25Q64_ReadStatusRegisters(&hspi2, &flash_status);
	if (status != W25Q64_STATUS_OK) {
		s_report.w25q64_ok = false;
		DiagUART_Printf(
				"[FLASH] JEDEC ID read, but status-register read failed -> FAIL\r\n");
		return;
	}

	s_report.w25q64_status1 = flash_status.status1;
	s_report.w25q64_status2 = flash_status.status2;
	s_report.w25q64_ok = !flash_status.busy;

	DiagUART_Printf(
			"[FLASH] W25Q64 SPI2 JEDEC=%02X %02X %02X SR1=%02X SR2=%02X BUSY=%u WEL=%u BP=%u -> %s\r\n",
			s_report.w25q64_jedec_id[0], s_report.w25q64_jedec_id[1],
			s_report.w25q64_jedec_id[2], flash_status.status1,
			flash_status.status2, flash_status.busy ? 1U : 0U,
			flash_status.write_enable_latch ? 1U : 0U,
			(unsigned int) flash_status.block_protect_bits,
			s_report.w25q64_ok ? "PASS" : "FAIL");

	if (flash_status.write_enable_latch) {
		DiagUART_Printf(
				"[FLASH] INFO: WEL is set; no array modification is performed by self-test\r\n");
	}
	if ((flash_status.block_protect_bits != 0U)
			|| flash_status.complement_protect || flash_status.sector_protect) {
		DiagUART_Printf(
				"[FLASH] INFO: protection configuration is active (SR1/SR2); write tests may be restricted\r\n");
	}
}

SelfTestOverallStatus_t SelfTest_Run(void) {
	memset(&s_report, 0, sizeof(s_report));
	s_report.mpu6050_accel_scale_correction = 1.0f;

	DiagUART_Printf("\r\n" SELF_TEST_BANNER_LINE);
	DiagUART_Printf(" INCIDENT RECORDER - STARTUP SELF-TEST\r\n");
	DiagUART_Printf(SELF_TEST_BANNER_LINE);
	DiagUART_Printf("[INFO] Diagnostic console: USART2, 115200 8N1\r\n");
	DiagUART_Printf(
			"[INFO] Keep device stationary during startup calibration\r\n");

	SelfTest_Mpu6050();
	SelfTest_Camera();
	SelfTest_Flash();

	if ((!s_report.mpu6050_ok) || (!s_report.w25q64_ok)) {
		s_report.overall = SELF_TEST_OVERALL_FAIL;
		DiagUART_Printf(
				"-----------------------------------------------\r\nSELF-TEST: FAIL\r\nSYSTEM: NOT READY - required sensor or storage unavailable\r\n");
	} else if ((!s_report.ov7670_ok) || (!s_report.mpu6050_gyro_calibrated)
			|| (!s_report.mpu6050_accel_scale_calibrated)) {
		s_report.overall = SELF_TEST_OVERALL_DEGRADED;
		DiagUART_Printf(
				"-----------------------------------------------\r\nSELF-TEST: DEGRADED\r\n");
		if (!s_report.ov7670_ok) {
			DiagUART_Printf("[WARN] Camera unavailable - photos disabled\r\n");
		}
		if (!s_report.mpu6050_gyro_calibrated) {
			DiagUART_Printf(
					"[WARN] Gyro bias not calibrated - angular-rate readings may be biased\r\n");
		}
		if (!s_report.mpu6050_accel_scale_calibrated) {
			DiagUART_Printf(
					"[WARN] Accelerometer gravity-scale calibration unavailable\r\n");
		}
		DiagUART_Printf("SYSTEM: READY WITH LIMITATIONS\r\n");
	} else {
		s_report.overall = SELF_TEST_OVERALL_PASS;
		DiagUART_Printf(
				"-----------------------------------------------\r\nSELF-TEST: PASS\r\nSYSTEM: READY\r\n");
	}

	DiagUART_Printf(SELF_TEST_BANNER_LINE "\r\n");
	return s_report.overall;
}

const SelfTestReport_t* SelfTest_GetReport(void) {
	return &s_report;
}
