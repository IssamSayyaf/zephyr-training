/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Nintendo Nunchuk I2C Driver for nRF52840 - CORRECTED VERSION
 */

#define DT_DRV_COMPAT nintendo_nunchuk_extended

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <stdlib.h>
#include <drivers/nunchuk.h>

#ifdef CONFIG_NUNCHUK_GESTURES
#include <math.h>
#endif

LOG_MODULE_REGISTER(nunchuk, CONFIG_NUNCHUK_LOG_LEVEL);

/* Nunchuk I2C commands for different variants */
#define NUNCHUK_BLACK_CMD1          0xF0
#define NUNCHUK_BLACK_VAL1          0x55
#define NUNCHUK_BLACK_CMD2          0xFB
#define NUNCHUK_BLACK_VAL2          0x00

#define NUNCHUK_WHITE_CMD1          0x40
#define NUNCHUK_WHITE_VAL1          0x00
#define NUNCHUK_WHITE_CMD2          0x00

#define NUNCHUK_READ_CMD            0x00
#define NUNCHUK_DATA_SIZE           6

/* Timing constants optimized for nRF52840 */
#define NUNCHUK_INIT_DELAY_MS       2
#define NUNCHUK_READ_DELAY_US       200

/* Sensor constants */
#define NUNCHUK_JOYSTICK_CENTER     127
#define NUNCHUK_ACCEL_1G            512

#ifdef CONFIG_NUNCHUK_GESTURES
#define NUNCHUK_SHAKE_THRESHOLD     100
#define NUNCHUK_TILT_THRESHOLD      150
#define NUNCHUK_GESTURE_HISTORY     8
#endif

/**
 * @brief Device configuration (from device tree)
 */
struct nunchuk_config {
	struct i2c_dt_spec i2c;
	uint32_t poll_interval_ms;
	uint8_t deadzone;
	bool enable_gestures;
	const char *nunchuk_type;
};

/**
 * @brief Calibration data
 */
struct nunchuk_calibration {
	uint8_t joystick_x_center;
	uint8_t joystick_y_center;
	uint16_t accel_x_zero;
	uint16_t accel_y_zero;
	uint16_t accel_z_zero;
	bool is_calibrated;
};

/**
 * @brief Device runtime data - CORRECTED VERSION
 */
struct nunchuk_data_private {
	/* Work queue for periodic polling */
	struct k_work_delayable work;
	
	/* CORRECTION: Store device pointer for work handler access */
	const struct device *device;
	
	/* Latest sensor data */
	struct nunchuk_data sensor_data;
	
	/* Calibration data */
	struct nunchuk_calibration cal;
	
	/* Callbacks */
	nunchuk_data_callback_t data_callback;
	void *data_callback_user_data;
	nunchuk_gesture_callback_t gesture_callback;
	void *gesture_callback_user_data;
	
	/* Driver state */
	bool initialized;
	struct k_mutex data_mutex;
	
#ifdef CONFIG_NUNCHUK_GESTURES
	/* Gesture detection history */
	struct nunchuk_data gesture_history[NUNCHUK_GESTURE_HISTORY];
	uint8_t gesture_index;
#endif
};

/**
 * @brief Initialize the nunchuk device with correct command sequence
 */
static int nunchuk_device_init(const struct device *dev)
{
	const struct nunchuk_config *cfg = dev->config;
	bool is_white = (strcmp(cfg->nunchuk_type, "white") == 0);
	uint8_t cmd_data[2];
	int ret;

	LOG_INF("Initializing %s nunchuk", is_white ? "white" : "black");

	if (is_white) {
		/* White nunchuk: 0x40, 0x00 then 0x00 */
		cmd_data[0] = NUNCHUK_WHITE_CMD1;
		cmd_data[1] = NUNCHUK_WHITE_VAL1;
		ret = i2c_write_dt(&cfg->i2c, cmd_data, 2);
		if (ret < 0) {
			LOG_ERR("White nunchuk init cmd1 failed: %d", ret);
			return ret;
		}
		
		k_msleep(NUNCHUK_INIT_DELAY_MS);
		
		/* Second command is single byte */
		// uint8_t single_cmd = NUNCHUK_WHITE_CMD2;
		cmd_data[0] = NUNCHUK_WHITE_CMD2;
		cmd_data[1] = NUNCHUK_WHITE_CMD2;

		ret = i2c_write_dt(&cfg->i2c, cmd_data, 2);
		if (ret < 0) {
			LOG_ERR("White nunchuk init cmd2 failed: %d", ret);
			return ret;
		}
	} else {
		/* Black nunchuk: 0xF0, 0x55 then 0xFB, 0x00 */
		cmd_data[0] = NUNCHUK_BLACK_CMD1;
		cmd_data[1] = NUNCHUK_BLACK_VAL1;
		ret = i2c_write_dt(&cfg->i2c, cmd_data, 2);
		if (ret < 0) {
			LOG_ERR("Black nunchuk init cmd1 failed: %d", ret);
			return ret;
		}
		
		k_msleep(NUNCHUK_INIT_DELAY_MS);
		
		cmd_data[0] = NUNCHUK_BLACK_CMD2;
		cmd_data[1] = NUNCHUK_BLACK_VAL2;
		ret = i2c_write_dt(&cfg->i2c, cmd_data, 2);
		if (ret < 0) {
			LOG_ERR("Black nunchuk init cmd2 failed: %d", ret);
			return ret;
		}
	}

	k_msleep(NUNCHUK_INIT_DELAY_MS);
	
	/* Test read to verify device responds */
	uint8_t test_data[NUNCHUK_DATA_SIZE];
	uint8_t read_cmd = NUNCHUK_READ_CMD;
	
	ret = i2c_write_dt(&cfg->i2c, &read_cmd, 1);
	if (ret < 0) {
		LOG_ERR("Test read command failed: %d", ret);
		return ret;
	}
	
	k_usleep(NUNCHUK_READ_DELAY_US);
	
	ret = i2c_read_dt(&cfg->i2c, test_data, NUNCHUK_DATA_SIZE);
	if (ret < 0) {
		LOG_ERR("Test read data failed: %d", ret);
		return ret;
	}

	LOG_INF("Nunchuk initialization successful");
	return 0;
}

/**
 * @brief Decode raw 6-byte nunchuk data packet
 */
static void nunchuk_decode_data(const uint8_t *raw, struct nunchuk_data *data)
{
	/* Joystick data (bytes 0-1) */
	data->joystick_x = raw[0];
	data->joystick_y = raw[1];

	/* Accelerometer data (10-bit, split across bytes) */
	data->accel_x = (raw[2] << 2) | ((raw[5] >> 2) & 0x03);
	data->accel_y = (raw[3] << 2) | ((raw[5] >> 4) & 0x03);
	data->accel_z = (raw[4] << 2) | ((raw[5] >> 6) & 0x03);

	/* Button data (inverted in byte 5) */
	data->button_c = !(raw[5] & 0x02);
	data->button_z = !(raw[5] & 0x01);
	
	/* Add timestamp */
	data->timestamp = k_uptime_get_32();
}

/**
 * @brief Apply calibration and deadzone to sensor data
 */
static void nunchuk_apply_calibration(struct nunchuk_data *data,
				     const struct nunchuk_calibration *cal,
				     uint8_t deadzone)
{
	if (!cal->is_calibrated) {
		return;
	}

	/* Apply joystick deadzone around calibrated center */
	int16_t x_diff = (int16_t)data->joystick_x - cal->joystick_x_center;
	int16_t y_diff = (int16_t)data->joystick_y - cal->joystick_y_center;

	if (abs(x_diff) < deadzone) {
		data->joystick_x = cal->joystick_x_center;
	}
	if (abs(y_diff) < deadzone) {
		data->joystick_y = cal->joystick_y_center;
	}

	/* Apply accelerometer zero-point calibration */
	data->accel_x = (data->accel_x > cal->accel_x_zero) ?
			data->accel_x - cal->accel_x_zero : 0;
	data->accel_y = (data->accel_y > cal->accel_y_zero) ?
			data->accel_y - cal->accel_y_zero : 0;
	data->accel_z = (data->accel_z > cal->accel_z_zero) ?
			data->accel_z - cal->accel_z_zero : 0;
}

#ifdef CONFIG_NUNCHUK_GESTURES
/**
 * @brief Detect gestures using FPU calculations
 */
static enum nunchuk_gesture nunchuk_detect_gesture(
	struct nunchuk_data_private *priv,
	const struct nunchuk_data *current)
{
	/* Store current data in history */
	priv->gesture_history[priv->gesture_index] = *current;
	priv->gesture_index = (priv->gesture_index + 1) % NUNCHUK_GESTURE_HISTORY;

	/* Need full history buffer for gesture detection */
	static bool history_full = false;
	if (priv->gesture_index == 0) {
		history_full = true;
	}
	if (!history_full) {
		return NUNCHUK_GESTURE_NONE;
	}

	/* Calculate acceleration variance for shake detection */
	float accel_x_variance = 0, accel_y_variance = 0, accel_z_variance = 0;
	float mean_x = 0, mean_y = 0, mean_z = 0;

	/* Calculate means */
	for (int i = 0; i < NUNCHUK_GESTURE_HISTORY; i++) {
		mean_x += priv->gesture_history[i].accel_x;
		mean_y += priv->gesture_history[i].accel_y;
		mean_z += priv->gesture_history[i].accel_z;
	}
	mean_x /= NUNCHUK_GESTURE_HISTORY;
	mean_y /= NUNCHUK_GESTURE_HISTORY;
	mean_z /= NUNCHUK_GESTURE_HISTORY;

	/* Calculate variances */
	for (int i = 0; i < NUNCHUK_GESTURE_HISTORY; i++) {
		float dx = priv->gesture_history[i].accel_x - mean_x;
		float dy = priv->gesture_history[i].accel_y - mean_y;
		float dz = priv->gesture_history[i].accel_z - mean_z;
		accel_x_variance += dx * dx;
		accel_y_variance += dy * dy;
		accel_z_variance += dz * dz;
	}

	/* Detect shakes based on variance */
	if (accel_x_variance > NUNCHUK_SHAKE_THRESHOLD &&
	    accel_x_variance > accel_y_variance &&
	    accel_x_variance > accel_z_variance) {
		return NUNCHUK_GESTURE_SHAKE_X;
	}
	if (accel_y_variance > NUNCHUK_SHAKE_THRESHOLD &&
	    accel_y_variance > accel_z_variance) {
		return NUNCHUK_GESTURE_SHAKE_Y;
	}
	if (accel_z_variance > NUNCHUK_SHAKE_THRESHOLD) {
		return NUNCHUK_GESTURE_SHAKE_Z;
	}

	/* Detect tilts based on current reading */
	if (current->accel_x > NUNCHUK_ACCEL_1G + NUNCHUK_TILT_THRESHOLD) {
		return NUNCHUK_GESTURE_TILT_RIGHT;
	}
	if (current->accel_x < NUNCHUK_ACCEL_1G - NUNCHUK_TILT_THRESHOLD) {
		return NUNCHUK_GESTURE_TILT_LEFT;
	}
	if (current->accel_y > NUNCHUK_ACCEL_1G + NUNCHUK_TILT_THRESHOLD) {
		return NUNCHUK_GESTURE_TILT_FORWARD;
	}
	if (current->accel_y < NUNCHUK_ACCEL_1G - NUNCHUK_TILT_THRESHOLD) {
		return NUNCHUK_GESTURE_TILT_BACK;
	}

	return NUNCHUK_GESTURE_NONE;
}
#endif

/**
 * @brief Read sensor data from nunchuk
 */
static int nunchuk_read_sensor_data(const struct device *dev,
				   struct nunchuk_data *data)
{
	const struct nunchuk_config *cfg = dev->config;
	struct nunchuk_data_private *priv = dev->data;
	uint8_t raw_data[NUNCHUK_DATA_SIZE];
	uint8_t read_cmd = NUNCHUK_READ_CMD;
	int ret;

	/* Send read command */
	ret = i2c_write_dt(&cfg->i2c, &read_cmd, 1);
	if (ret < 0) {
		return ret;
	}

	/* Wait for data to be ready */
	k_usleep(NUNCHUK_READ_DELAY_US);

	/* Read the data */
	ret = i2c_read_dt(&cfg->i2c, raw_data, NUNCHUK_DATA_SIZE);
	if (ret < 0) {
		return ret;
	}

	/* Decode and apply calibration */
	nunchuk_decode_data(raw_data, data);
	nunchuk_apply_calibration(data, &priv->cal, cfg->deadzone);

	return 0;
}

/**
 * @brief Work handler - called periodically to poll nunchuk
 * CORRECTED VERSION: Uses stored device pointer instead of CONTAINER_OF
 */
static void nunchuk_work_handler(struct k_work *work)
{
	/* Step 1: Convert work type */
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	
	/* Step 2: Get private data from work member */
	struct nunchuk_data_private *priv = CONTAINER_OF(dwork,
							 struct nunchuk_data_private,
							 work);
	
	/* Step 3: Get device from stored pointer - CORRECTED! */
	const struct device *dev = priv->device;
	const struct nunchuk_config *cfg = dev->config;
	
	struct nunchuk_data new_data;
	int ret;

	/* Read new sensor data */
	ret = nunchuk_read_sensor_data(dev, &new_data);
	if (ret != 0) {
		LOG_WRN("Failed to read sensor data: %d", ret);
		goto schedule_next;
	}

	/* Update stored data with mutex protection */
	k_mutex_lock(&priv->data_mutex, K_FOREVER);
	priv->sensor_data = new_data;
	k_mutex_unlock(&priv->data_mutex);

	/* Call data callback if registered */
	if (priv->data_callback) {
		priv->data_callback(dev, &new_data, priv->data_callback_user_data);
	}

#ifdef CONFIG_NUNCHUK_GESTURES
	/* Detect and report gestures */
	if (cfg->enable_gestures && priv->gesture_callback) {
		enum nunchuk_gesture gesture = nunchuk_detect_gesture(priv, &new_data);
		if (gesture != NUNCHUK_GESTURE_NONE) {
			priv->gesture_callback(dev, gesture,
					      priv->gesture_callback_user_data);
		}
	}
#endif

schedule_next:
	/* Schedule next reading */
	k_work_schedule(&priv->work, K_MSEC(cfg->poll_interval_ms));
}

/* Public API implementation */

int nunchuk_read(const struct device *dev, struct nunchuk_data *data)
{
	struct nunchuk_data_private *priv = dev->data;

	if (!priv->initialized || !data) {
		return -EINVAL;
	}

	k_mutex_lock(&priv->data_mutex, K_FOREVER);
	*data = priv->sensor_data;
	k_mutex_unlock(&priv->data_mutex);

	return 0;
}

int nunchuk_set_data_callback(const struct device *dev,
			     nunchuk_data_callback_t callback,
			     void *user_data)
{
	struct nunchuk_data_private *priv = dev->data;

	if (!priv->initialized) {
		return -ENODEV;
	}

	priv->data_callback = callback;
	priv->data_callback_user_data = user_data;

	return 0;
}

int nunchuk_set_gesture_callback(const struct device *dev,
				nunchuk_gesture_callback_t callback,
				void *user_data)
{
	struct nunchuk_data_private *priv = dev->data;

	if (!priv->initialized) {
		return -ENODEV;
	}

#ifdef CONFIG_NUNCHUK_GESTURES
	priv->gesture_callback = callback;
	priv->gesture_callback_user_data = user_data;
	return 0;
#else
	return -ENOTSUP;
#endif
}

int nunchuk_calibrate(const struct device *dev)
{
	struct nunchuk_data_private *priv = dev->data;
	struct nunchuk_data cal_data;
	uint32_t samples = 20;
	uint32_t sum_jx = 0, sum_jy = 0;
	uint32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
	int ret;

	if (!priv->initialized) {
		return -ENODEV;
	}

	LOG_INF("Starting calibration - hold nunchuk steady");

	/* Take multiple samples for accuracy */
	for (uint32_t i = 0; i < samples; i++) {
		ret = nunchuk_read_sensor_data(dev, &cal_data);
		if (ret < 0) {
			LOG_ERR("Calibration read failed: %d", ret);
			return ret;
		}
		
		sum_jx += cal_data.joystick_x;
		sum_jy += cal_data.joystick_y;
		sum_ax += cal_data.accel_x;
		sum_ay += cal_data.accel_y;
		sum_az += cal_data.accel_z;
		
		k_msleep(10);
	}

	/* Calculate calibration values */
	priv->cal.joystick_x_center = sum_jx / samples;
	priv->cal.joystick_y_center = sum_jy / samples;
	priv->cal.accel_x_zero = sum_ax / samples;
	priv->cal.accel_y_zero = sum_ay / samples;
	priv->cal.accel_z_zero = sum_az / samples;
	priv->cal.is_calibrated = true;

	LOG_INF("Calibration complete - center: (%d,%d), accel_zero: (%d,%d,%d)",
		priv->cal.joystick_x_center, priv->cal.joystick_y_center,
		priv->cal.accel_x_zero, priv->cal.accel_y_zero, priv->cal.accel_z_zero);

	return 0;
}

/**
 * @brief Driver initialization function - CORRECTED VERSION
 */
static int nunchuk_init(const struct device *dev)
{
	const struct nunchuk_config *cfg = dev->config;
	struct nunchuk_data_private *priv = dev->data;
	int ret;

	LOG_INF("Initializing nunchuk driver");

	/* CORRECTION: Store device pointer in private data */
	priv->device = dev;

	/* Verify I2C device is ready */
	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	/* Initialize synchronization */
	ret = k_mutex_init(&priv->data_mutex);
	if (ret < 0) {
		LOG_ERR("Failed to init mutex: %d", ret);
		return ret;
	}

	/* Initialize work handler */
	k_work_init_delayable(&priv->work, nunchuk_work_handler);

	/* Initialize the hardware */
	ret = nunchuk_device_init(dev);
	if (ret < 0) {
		LOG_ERR("Hardware init failed: %d", ret);
		return ret;
	}

	/* Set default calibration values */
	priv->cal.joystick_x_center = NUNCHUK_JOYSTICK_CENTER;
	priv->cal.joystick_y_center = NUNCHUK_JOYSTICK_CENTER;
	priv->cal.accel_x_zero = NUNCHUK_ACCEL_1G;
	priv->cal.accel_y_zero = NUNCHUK_ACCEL_1G;
	priv->cal.accel_z_zero = NUNCHUK_ACCEL_1G;
	priv->cal.is_calibrated = false;

	/* Clear callbacks */
	priv->data_callback = NULL;
	priv->data_callback_user_data = NULL;
	priv->gesture_callback = NULL;
	priv->gesture_callback_user_data = NULL;

#ifdef CONFIG_NUNCHUK_GESTURES
	/* Initialize gesture detection */
	priv->gesture_index = 0;
	memset(priv->gesture_history, 0, sizeof(priv->gesture_history));
#endif

	priv->initialized = true;

	/* Start periodic polling */
	k_work_schedule(&priv->work, K_MSEC(cfg->poll_interval_ms));

	LOG_INF("Nunchuk driver ready");
	return 0;
}

/* Device instantiation macro - unchanged, correct as-is */
#define NUNCHUK_INIT(inst) \
	static const struct nunchuk_config nunchuk_config_##inst = { \
		.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.poll_interval_ms = DT_INST_PROP(inst, poll_interval_ms), \
		.deadzone = DT_INST_PROP(inst, deadzone), \
		.enable_gestures = DT_INST_PROP(inst, enable_gestures), \
		.nunchuk_type = DT_INST_PROP(inst, nunchuk_type), \
	}; \
	static struct nunchuk_data_private nunchuk_data_##inst; \
	DEVICE_DT_INST_DEFINE(inst, nunchuk_init, NULL, \
			      &nunchuk_data_##inst, &nunchuk_config_##inst, \
			      POST_KERNEL, CONFIG_NUNCHUK_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(NUNCHUK_INIT)