#include "services/incident_detector.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define ID_ORIENTATION_STABLE_SAMPLES 60U       /* 300 ms at 200 Hz */
#define ID_ORIENTATION_ACCEL_MIN_G 0.70f
#define ID_ORIENTATION_ACCEL_MAX_G 1.30f
#define ID_ORIENTATION_GYRO_MAX_DPS 5.0f
#define ID_GRAVITY_ALPHA 0.05f
#define ID_TILT_CONFIRM_MS 500U
#define ID_TILT_CLEAR_MS 1000U
#define ID_TILT_HYSTERESIS_DEG 5.0f
#define ID_TILT_GYRO_STABLE_DPS 15.0f
#define ID_SHOCK_CLEAR_MS 100U
#define ID_SHOCK_REFRACTORY_MS 500U
#define ID_SHOCK_RESET_RATIO 0.60f
#define ID_TEMP_CONFIRM_MS 2000U
#define ID_TEMP_CLEAR_MS 5000U
#define ID_TEMP_HYSTERESIS_C 2.0f
#define ID_VIBRATION_WINDOW 50U                /* 250 ms at 200 Hz */
#define ID_VIBRATION_CONFIRM_MS 500U
#define ID_VIBRATION_CLEAR_MS 700U
#define ID_VIBRATION_CLEAR_RATIO 0.60f
#define ID_VIBRATION_MIN_CROSSINGS 6U
#define ID_VIBRATION_CLEAR_CROSSINGS 3U
#define ID_VIBRATION_DEADBAND_G 0.020f
#define ID_VIBRATION_MIN_RMS_G 0.020f
#define ID_VIBRATION_LP_FREEZE_G 0.50f

typedef struct {
	IncidentEvent_t items[INCIDENT_QUEUE_CAPACITY];
	uint8_t head;
	uint8_t count;
	uint32_t next_sequence;
	uint32_t dropped;
} IncidentQueue_t;

static IncidentQueue_t s_queue;
static IncidentDetectorStats_t s_stats;
static uint8_t s_was_armed;
static ConfigMode_t s_mode;
static uint32_t s_last_sample_sequence;
static uint8_t s_have_sample_sequence;
static float s_gravity[3];
static float s_shock_gravity[3];
static uint8_t s_gravity_initialized;
static uint16_t s_orientation_stable_count;
static float s_orientation_sum[3];
static float s_baseline[3];
static uint8_t s_orientation_ready;
static uint32_t s_tilt_condition_since;
static uint32_t s_tilt_clear_since;
static uint8_t s_tilt_active;
static uint32_t s_shock_clear_since;
static uint32_t s_last_shock_tick;
static uint8_t s_shock_active;
static uint8_t s_shock_has_fired;
static uint32_t s_temp_condition_since;
static uint32_t s_temp_clear_since;
static uint8_t s_temp_active;
static float s_vibration_sq[ID_VIBRATION_WINDOW];
static uint8_t s_vibration_crossings[ID_VIBRATION_WINDOW];
static uint8_t s_vibration_index;
static uint8_t s_vibration_fill;
static float s_vibration_sum_sq;
static uint16_t s_vibration_sum_crossings;
static int8_t s_axis_sign[3];
static uint32_t s_vibration_condition_since;
static uint32_t s_vibration_clear_since;
static uint8_t s_vibration_active;

static float vec_norm(const float v[3]) {
	return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

static float gyro_norm(const MPU6050_Sample_t *sample) {
	return sqrtf(
			(sample->gyro_dps[0] * sample->gyro_dps[0])
					+ (sample->gyro_dps[1] * sample->gyro_dps[1])
					+ (sample->gyro_dps[2] * sample->gyro_dps[2]));
}

static uint8_t normalize(const float in[3], float out[3]) {
	const float n = vec_norm(in);
	if (n < 0.10f)
		return 0U;
	out[0] = in[0] / n;
	out[1] = in[1] / n;
	out[2] = in[2] / n;
	return 1U;
}

static void queue_push(IncidentType_t type, ConfigMode_t mode, uint32_t tick,
		int32_t value, int32_t threshold) {
	uint8_t tail;
	IncidentEvent_t *event;
	if (s_queue.count >= INCIDENT_QUEUE_CAPACITY) {
		++s_queue.dropped;
		s_stats.queue_dropped = s_queue.dropped;
		return; /* Drop newest; preserve chronology of already queued events. */
	}
	tail = (uint8_t) ((s_queue.head + s_queue.count) % INCIDENT_QUEUE_CAPACITY);
	event = &s_queue.items[tail];
	event->sequence = ++s_queue.next_sequence;
	event->timestamp_ms = tick;
	event->mode = mode;
	event->type = type;
	event->value = value;
	event->threshold = threshold;
	++s_queue.count;
	s_stats.queue_count = s_queue.count;
	switch (type) {
	case INCIDENT_TYPE_SHOCK:
		++s_stats.shock_events;
		break;
	case INCIDENT_TYPE_TILT:
		++s_stats.tilt_events;
		break;
	case INCIDENT_TYPE_TEMPERATURE:
		++s_stats.temperature_events;
		break;
	case INCIDENT_TYPE_VIBRATION:
		++s_stats.vibration_events;
		break;
	default:
		break;
	}
}

static void reset_detection_state(void) {
	memset(s_gravity, 0, sizeof(s_gravity));
	memset(s_shock_gravity, 0, sizeof(s_shock_gravity));
	memset(s_orientation_sum, 0, sizeof(s_orientation_sum));
	memset(s_baseline, 0, sizeof(s_baseline));
	memset(s_vibration_sq, 0, sizeof(s_vibration_sq));
	memset(s_vibration_crossings, 0, sizeof(s_vibration_crossings));
	memset(s_axis_sign, 0, sizeof(s_axis_sign));
	s_gravity_initialized = 0U;
	s_orientation_stable_count = 0U;
	s_orientation_ready = 0U;
	s_tilt_condition_since = 0U;
	s_tilt_clear_since = 0U;
	s_tilt_active = 0U;
	s_shock_clear_since = 0U;
	s_last_shock_tick = 0U;
	s_shock_active = 0U;
	s_shock_has_fired = 0U;
	s_temp_condition_since = 0U;
	s_temp_clear_since = 0U;
	s_temp_active = 0U;
	s_vibration_index = 0U;
	s_vibration_fill = 0U;
	s_vibration_sum_sq = 0.0f;
	s_vibration_sum_crossings = 0U;
	s_vibration_condition_since = 0U;
	s_vibration_clear_since = 0U;
	s_vibration_active = 0U;
	s_have_sample_sequence = 0U;
	s_stats.orientation_ready = 0U;
}

void IncidentDetector_Init(void) {
	memset(&s_queue, 0, sizeof(s_queue));
	memset(&s_stats, 0, sizeof(s_stats));
	s_was_armed = 0U;
	s_mode = CONFIG_MODE_CARGO;
	reset_detection_state();
}

static void process_orientation(const MPU6050_Sample_t *sample,
		const ConfigProfile_t *profile, uint32_t tick) {
	float current[3];
	float dot;
	float angle_deg;
	float threshold;
	const float a_norm = vec_norm(sample->accel_g);
	const float g_norm = gyro_norm(sample);
	unsigned i;

	if (s_orientation_ready == 0U) {
		if ((a_norm >= ID_ORIENTATION_ACCEL_MIN_G)
				&& (a_norm <= ID_ORIENTATION_ACCEL_MAX_G)
				&& (g_norm <= ID_ORIENTATION_GYRO_MAX_DPS)) {
			for (i = 0U; i < 3U; ++i)
				s_orientation_sum[i] += s_gravity[i];
			if (s_orientation_stable_count < ID_ORIENTATION_STABLE_SAMPLES)
				++s_orientation_stable_count;
		} else {
			s_orientation_stable_count = 0U;
			memset(s_orientation_sum, 0, sizeof(s_orientation_sum));
		}
		if (s_orientation_stable_count >= ID_ORIENTATION_STABLE_SAMPLES) {
			float mean[3];
			for (i = 0U; i < 3U; ++i)
				mean[i] = s_orientation_sum[i]
						/ (float) ID_ORIENTATION_STABLE_SAMPLES;
			if (normalize(mean, s_baseline) != 0U) {
				s_orientation_ready = 1U;
				s_stats.orientation_ready = 1U;
			}
		}
		return;
	}

	if (normalize(s_gravity, current) == 0U)
		return;
	dot = (s_baseline[0] * current[0]) + (s_baseline[1] * current[1])
			+ (s_baseline[2] * current[2]);
	if (dot > 1.0f)
		dot = 1.0f;
	if (dot < -1.0f)
		dot = -1.0f;
	angle_deg = acosf(dot) * 57.2957795f;
	threshold = (float) profile->tilt_threshold_deg;

	if (s_tilt_active == 0U) {
		if ((angle_deg >= threshold) && (g_norm <= ID_TILT_GYRO_STABLE_DPS)) {
			if (s_tilt_condition_since == 0U)
				s_tilt_condition_since = tick;
			if ((uint32_t) (tick - s_tilt_condition_since) >= ID_TILT_CONFIRM_MS) {
				queue_push(INCIDENT_TYPE_TILT, s_mode, tick,
						(int32_t) (angle_deg * 100.0f),
						(int32_t) (threshold * 100.0f));
				s_tilt_active = 1U;
				s_tilt_clear_since = 0U;
			}
		} else
			s_tilt_condition_since = 0U;
	} else {
		const float clear_threshold =
				(threshold > ID_TILT_HYSTERESIS_DEG) ?
						threshold - ID_TILT_HYSTERESIS_DEG : threshold * 0.80f;
		if (angle_deg < clear_threshold) {
			if (s_tilt_clear_since == 0U)
				s_tilt_clear_since = tick;
			if ((uint32_t) (tick - s_tilt_clear_since) >= ID_TILT_CLEAR_MS) {
				s_tilt_active = 0U;
				s_tilt_condition_since = 0U;
				s_tilt_clear_since = 0U;
			}
		} else
			s_tilt_clear_since = 0U;
	}
}

static void process_shock(const MPU6050_Sample_t *sample,
		const float dynamic[3], const ConfigProfile_t *profile, uint32_t tick) {
	const float dynamic_mg = vec_norm(dynamic) * 1000.0f;
	const float threshold = (float) profile->impact_threshold_mg;
	const float clear_threshold = threshold * ID_SHOCK_RESET_RATIO;

	if ((s_shock_active == 0U) && (dynamic_mg >= threshold)
			&& ((s_shock_has_fired == 0U)
					|| ((uint32_t) (tick - s_last_shock_tick)
							>= ID_SHOCK_REFRACTORY_MS))) {
		queue_push(INCIDENT_TYPE_SHOCK, s_mode, tick,
				(int32_t) (dynamic_mg + 0.5f), (int32_t) threshold);
		s_shock_active = 1U;
		s_shock_has_fired = 1U;
		s_last_shock_tick = tick;
		s_shock_clear_since = 0U;
	} else if (s_shock_active != 0U) {
		if (dynamic_mg < clear_threshold) {
			if (s_shock_clear_since == 0U)
				s_shock_clear_since = tick;
			if ((uint32_t) (tick - s_shock_clear_since) >= ID_SHOCK_CLEAR_MS) {
				s_shock_active = 0U;
				s_shock_clear_since = 0U;
			}
		} else
			s_shock_clear_since = 0U;
	}
	(void) sample;
}

static uint8_t axis_crossings(const float dynamic[3]) {
	uint8_t count = 0U;
	unsigned i;
	for (i = 0U; i < 3U; ++i) {
		int8_t sign = 0;
		if (dynamic[i] > ID_VIBRATION_DEADBAND_G)
			sign = 1;
		else if (dynamic[i] < -ID_VIBRATION_DEADBAND_G)
			sign = -1;
		if ((sign != 0) && (s_axis_sign[i] != 0) && (sign != s_axis_sign[i]))
			++count;
		if (sign != 0)
			s_axis_sign[i] = sign;
	}
	return count;
}

static void process_vibration(const float dynamic[3],
		const ConfigProfile_t *profile, uint32_t tick) {
	const float magnitude = vec_norm(dynamic);
	const float threshold = (float) profile->impact_threshold_mg / 1000.0f;
	const uint8_t crossings = axis_crossings(dynamic);
	float rms;
	uint8_t active_condition;

	if (s_vibration_fill == ID_VIBRATION_WINDOW) {
		s_vibration_sum_sq -= s_vibration_sq[s_vibration_index];
		s_vibration_sum_crossings = (uint16_t) (s_vibration_sum_crossings
				- s_vibration_crossings[s_vibration_index]);
	} else
		++s_vibration_fill;
	s_vibration_sq[s_vibration_index] = magnitude * magnitude;
	s_vibration_crossings[s_vibration_index] = crossings;
	s_vibration_sum_sq += magnitude * magnitude;
	s_vibration_sum_crossings = (uint16_t) (s_vibration_sum_crossings
			+ crossings);
	s_vibration_index = (uint8_t) ((s_vibration_index + 1U)
			% ID_VIBRATION_WINDOW);

	if (s_vibration_fill < ID_VIBRATION_WINDOW)
		return;
	rms = sqrtf(s_vibration_sum_sq / (float) ID_VIBRATION_WINDOW);
	active_condition = (uint8_t) ((rms >= threshold)
			&& (rms >= ID_VIBRATION_MIN_RMS_G)
			&& (s_vibration_sum_crossings >= ID_VIBRATION_MIN_CROSSINGS));

	if (s_vibration_active == 0U) {
		if (active_condition != 0U) {
			if (s_vibration_condition_since == 0U)
				s_vibration_condition_since = tick;
			if ((uint32_t) (tick - s_vibration_condition_since)
					>= ID_VIBRATION_CONFIRM_MS) {
				queue_push(INCIDENT_TYPE_VIBRATION, s_mode, tick,
						(int32_t) (rms * 1000.0f + 0.5f),
						(int32_t) (threshold * 1000.0f));
				s_vibration_active = 1U;
				s_vibration_clear_since = 0U;
			}
		} else
			s_vibration_condition_since = 0U;
	} else {
		if ((rms < (threshold * ID_VIBRATION_CLEAR_RATIO))
				|| (s_vibration_sum_crossings < ID_VIBRATION_CLEAR_CROSSINGS)) {
			if (s_vibration_clear_since == 0U)
				s_vibration_clear_since = tick;
			if ((uint32_t) (tick - s_vibration_clear_since)
					>= ID_VIBRATION_CLEAR_MS) {
				s_vibration_active = 0U;
				s_vibration_condition_since = 0U;
				s_vibration_clear_since = 0U;
			}
		} else
			s_vibration_clear_since = 0U;
	}
}

static void process_temperature(const MPU6050_Sample_t *sample,
		const ConfigProfile_t *profile, uint32_t tick) {
	const float temp = sample->temperature_c;
	const uint8_t outside = (uint8_t) ((temp < (float) profile->temp_min_c)
			|| (temp > (float) profile->temp_max_c));
	if (s_temp_active == 0U) {
		if (outside != 0U) {
			if (s_temp_condition_since == 0U)
				s_temp_condition_since = tick;
			if ((uint32_t) (tick - s_temp_condition_since) >= ID_TEMP_CONFIRM_MS) {
				const float bound =
						(temp < (float) profile->temp_min_c) ?
								(float) profile->temp_min_c :
								(float) profile->temp_max_c;
				queue_push(INCIDENT_TYPE_TEMPERATURE, s_mode, tick,
						(int32_t) (temp * 1000.0f),
						(int32_t) (bound * 1000.0f));
				s_temp_active = 1U;
				s_temp_clear_since = 0U;
			}
		} else
			s_temp_condition_since = 0U;
	} else {
		const float low_clear = (float) profile->temp_min_c
				+ ID_TEMP_HYSTERESIS_C;
		const float high_clear = (float) profile->temp_max_c
				- ID_TEMP_HYSTERESIS_C;
		if ((temp >= low_clear) && (temp <= high_clear)) {
			if (s_temp_clear_since == 0U)
				s_temp_clear_since = tick;
			if ((uint32_t) (tick - s_temp_clear_since) >= ID_TEMP_CLEAR_MS) {
				s_temp_active = 0U;
				s_temp_condition_since = 0U;
				s_temp_clear_since = 0U;
			}
		} else
			s_temp_clear_since = 0U;
	}
}

void IncidentDetector_ProcessSample(bool armed, ConfigMode_t mode,
		const ConfigProfile_t *profile, const MPU6050_Sample_t *sample,
		uint32_t sample_sequence, uint32_t sample_tick_ms) {
	float dynamic[3];
	float dynamic_norm;
	unsigned i;

	if ((armed == false) || (profile == NULL)) {
		if (s_was_armed != 0U)
			reset_detection_state();
		s_was_armed = 0U;
		return;
	}
	if ((sample == NULL)
			|| ((mode != CONFIG_MODE_CARGO) && (mode != CONFIG_MODE_SITE)))
		return;
	if ((s_was_armed == 0U) || (mode != s_mode)) {
		reset_detection_state();
		s_was_armed = 1U;
		s_mode = mode;
	}
	if ((s_have_sample_sequence != 0U)
			&& (sample_sequence == s_last_sample_sequence))
		return;
	s_have_sample_sequence = 1U;
	s_last_sample_sequence = sample_sequence;
	++s_stats.samples_processed;

	if (s_gravity_initialized == 0U) {
		for (i = 0U; i < 3U; ++i) {
			s_gravity[i] = sample->accel_g[i];
			s_shock_gravity[i] = sample->accel_g[i];
		}
		s_gravity_initialized = 1U;
	}

	/* The orientation low-pass must follow a sustained tilt. A separate shock
	 baseline is frozen during transients so an impact cannot be averaged away. */
	dynamic[0] = sample->accel_g[0] - s_shock_gravity[0];
	dynamic[1] = sample->accel_g[1] - s_shock_gravity[1];
	dynamic[2] = sample->accel_g[2] - s_shock_gravity[2];
	dynamic_norm = vec_norm(dynamic);
	if ((dynamic_norm < ID_VIBRATION_LP_FREEZE_G)
			|| ((gyro_norm(sample) < 5.0f)
					&& (vec_norm(sample->accel_g) >= ID_ORIENTATION_ACCEL_MIN_G)
					&& (vec_norm(sample->accel_g) <= ID_ORIENTATION_ACCEL_MAX_G))) {
		for (i = 0U; i < 3U; ++i)
			s_shock_gravity[i] += ID_GRAVITY_ALPHA
					* (sample->accel_g[i] - s_shock_gravity[i]);
	}

	for (i = 0U; i < 3U; ++i)
		s_gravity[i] += ID_GRAVITY_ALPHA * (sample->accel_g[i] - s_gravity[i]);
	dynamic[0] = sample->accel_g[0] - s_gravity[0];
	dynamic[1] = sample->accel_g[1] - s_gravity[1];
	dynamic[2] = sample->accel_g[2] - s_gravity[2];

	/* Temperature is monitored in BOTH profiles. The MPU-6050 value is die
	 temperature, not ambient/cargo temperature. */
	process_temperature(sample, profile, sample_tick_ms);

	if (mode == CONFIG_MODE_CARGO) {
		const float shock_dynamic[3] = { sample->accel_g[0]
				- s_shock_gravity[0], sample->accel_g[1] - s_shock_gravity[1],
				sample->accel_g[2] - s_shock_gravity[2] };
		process_shock(sample, shock_dynamic, profile, sample_tick_ms);
		process_orientation(sample, profile, sample_tick_ms);
	} else {
		process_vibration(dynamic, profile, sample_tick_ms);
	}
}

bool IncidentDetector_Peek(uint8_t index, IncidentEvent_t *event) {
	uint8_t position;
	if ((event == NULL) || (index >= s_queue.count))
		return false;
	position = (uint8_t) ((s_queue.head + index) % INCIDENT_QUEUE_CAPACITY);
	*event = s_queue.items[position];
	return true;
}

bool IncidentDetector_Pop(IncidentEvent_t *event) {
	if ((event == NULL) || (s_queue.count == 0U))
		return false;
	*event = s_queue.items[s_queue.head];
	s_queue.head = (uint8_t) ((s_queue.head + 1U) % INCIDENT_QUEUE_CAPACITY);
	--s_queue.count;
	s_stats.queue_count = s_queue.count;
	return true;
}

void IncidentDetector_GetStats(IncidentDetectorStats_t *stats) {
	if (stats == NULL)
		return;
	*stats = s_stats;
	stats->queue_count = s_queue.count;
	stats->queue_dropped = s_queue.dropped;
}

const char* IncidentDetector_TypeName(IncidentType_t type) {
	switch (type) {
	case INCIDENT_TYPE_SHOCK:
		return "SHOCK";
	case INCIDENT_TYPE_TILT:
		return "TILT";
	case INCIDENT_TYPE_TEMPERATURE:
		return "TEMPERATURE";
	case INCIDENT_TYPE_VIBRATION:
		return "VIBRATION";
	default:
		return "UNKNOWN";
	}
}
