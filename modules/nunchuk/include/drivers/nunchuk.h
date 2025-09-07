/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Nintendo Nunchuk I2C Driver API
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_NUNCHUK_H_
#define ZEPHYR_INCLUDE_DRIVERS_NUNCHUK_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Nunchuk sensor data structure
 * 
 * Contains all sensor readings from a single poll of the nunchuk.
 */
struct nunchuk_data {
	/** Joystick X position (0-255, center=127) */
	uint8_t joystick_x;
	/** Joystick Y position (0-255, center=127) */
	uint8_t joystick_y;
	/** Accelerometer X reading (0-1023, ~512=1G) */
	uint16_t accel_x;
	/** Accelerometer Y reading (0-1023, ~512=1G) */
	uint16_t accel_y;
	/** Accelerometer Z reading (0-1023, ~512=1G) */
	uint16_t accel_z;
	/** C button state (true=pressed) */
	bool button_c;
	/** Z button state (true=pressed) */
	bool button_z;
	/** Timestamp of reading in milliseconds */
	uint32_t timestamp;
};

/**
 * @brief Nunchuk gesture types (requires CONFIG_NUNCHUK_GESTURES=y)
 */
enum nunchuk_gesture {
	NUNCHUK_GESTURE_NONE,         /**< No gesture detected */
	NUNCHUK_GESTURE_SHAKE_X,      /**< Shake along X-axis */
	NUNCHUK_GESTURE_SHAKE_Y,      /**< Shake along Y-axis */
	NUNCHUK_GESTURE_SHAKE_Z,      /**< Shake along Z-axis */
	NUNCHUK_GESTURE_TILT_LEFT,    /**< Tilt left */
	NUNCHUK_GESTURE_TILT_RIGHT,   /**< Tilt right */
	NUNCHUK_GESTURE_TILT_FORWARD, /**< Tilt forward */
	NUNCHUK_GESTURE_TILT_BACK,    /**< Tilt backward */
};

/**
 * @brief Callback function type for nunchuk data updates
 * 
 * Called periodically with new sensor data.
 * 
 * @param dev The nunchuk device
 * @param data Pointer to the latest sensor data
 * @param user_data User-provided data passed to callback
 */
typedef void (*nunchuk_data_callback_t)(const struct device *dev, 
					const struct nunchuk_data *data, 
					void *user_data);

/**
 * @brief Callback function type for gesture detection
 * 
 * Called when a gesture is detected (if gestures are enabled).
 * 
 * @param dev The nunchuk device
 * @param gesture The detected gesture type
 * @param user_data User-provided data passed to callback
 */
typedef void (*nunchuk_gesture_callback_t)(const struct device *dev, 
					   enum nunchuk_gesture gesture,
					   void *user_data);

/**
 * @brief Read current nunchuk sensor data
 * 
 * Returns the most recently cached sensor data. This is non-blocking
 * and returns immediately with the last polled values.
 * 
 * @param dev Nunchuk device pointer
 * @param data Pointer to store the sensor data
 * @return 0 on success, negative errno on failure
 */
int nunchuk_read(const struct device *dev, struct nunchuk_data *data);

/**
 * @brief Set callback for data updates
 * 
 * Register a callback function that will be called each time new
 * sensor data is available (every poll-interval-ms).
 * 
 * @param dev Nunchuk device pointer
 * @param callback Callback function (NULL to disable)
 * @param user_data User data passed to callback
 * @return 0 on success, negative errno on failure
 */
int nunchuk_set_data_callback(const struct device *dev, 
			     nunchuk_data_callback_t callback, 
			     void *user_data);

/**
 * @brief Set callback for gesture detection
 * 
 * Register a callback function for gesture events. Only works if
 * CONFIG_NUNCHUK_GESTURES=y and enable-gestures is set in device tree.
 * 
 * @param dev Nunchuk device pointer
 * @param callback Callback function (NULL to disable)
 * @param user_data User data passed to callback
 * @return 0 on success, negative errno on failure
 */
int nunchuk_set_gesture_callback(const struct device *dev, 
				nunchuk_gesture_callback_t callback, 
				void *user_data);

/**
 * @brief Calibrate the nunchuk
 * 
 * Performs calibration by taking multiple samples of the current
 * position and using them as the neutral/center reference.
 * Hold the nunchuk in a neutral position when calling this.
 * 
 * @param dev Nunchuk device pointer
 * @return 0 on success, negative errno on failure
 */
int nunchuk_calibrate(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_NUNCHUK_H_ */