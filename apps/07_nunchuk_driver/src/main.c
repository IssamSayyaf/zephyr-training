/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Nunchuk Driver Demo Application for nRF52840
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <drivers/nunchuk.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Device handles */
static const struct device *nunchuk_dev = DEVICE_DT_GET(DT_NODELABEL(nunchuk));

/* LED GPIO specs for nRF52840 DK */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

/* Demo state */
static bool calibration_requested = false;
static uint32_t data_count = 0;
static uint32_t gesture_count = 0;

/**
 * @brief Update LEDs based on nunchuk state
 */
static void update_leds(const struct nunchuk_data *data)
{
	/* LED0: C button */
	gpio_pin_set_dt(&led0, data->button_c);
	
	/* LED1: Z button */
	gpio_pin_set_dt(&led1, data->button_z);
	
	/* LED2: Joystick moved from center */
	bool joystick_active = (abs(data->joystick_x - 127) > 20) ||
			       (abs(data->joystick_y - 127) > 20);
	gpio_pin_set_dt(&led2, joystick_active);
	
	/* LED3: High acceleration detected */
	uint32_t accel_total = data->accel_x + data->accel_y + data->accel_z;
	gpio_pin_set_dt(&led3, accel_total > 1800);
}

/**
 * @brief Nunchuk data callback - called every poll interval
 */
static void nunchuk_data_callback(const struct device *dev,
				 const struct nunchuk_data *data,
				 void *user_data)
{
	static bool last_c_button = false;
	
	data_count++;
	
	/* Update visual feedback */
	update_leds(data);
	
	/* Request calibration on C button press */
	if (data->button_c && !last_c_button) {
		calibration_requested = true;
		LOG_INF("🎮 C button pressed - calibration requested");
	}
	last_c_button = data->button_c;
	
	/* Log data periodically */
	if (data_count % 50 == 0) {
		LOG_INF("📊 Data #%u - Joy:(%d,%d) Accel:(%d,%d,%d) Btns: C=%s Z=%s",
			data_count,
			data->joystick_x, data->joystick_y,
			data->accel_x, data->accel_y, data->accel_z,
			data->button_c ? "ON" : "off",
			data->button_z ? "ON" : "off");
	}
	
	/* Z button detailed info */
	if (data->button_z) {
		LOG_INF("🚀 Z button: timestamp=%u ms", data->timestamp);
	}
}

/**
 * @brief Gesture detection callback
 */
static void nunchuk_gesture_callback(const struct device *dev,
				    enum nunchuk_gesture gesture,
				    void *user_data)
{
	const char *names[] = {
		"None", "Shake-X", "Shake-Y", "Shake-Z",
		"Tilt-Left", "Tilt-Right", "Tilt-Forward", "Tilt-Back"
	};
	
	gesture_count++;
	
	if (gesture < ARRAY_SIZE(names)) {
		LOG_INF("🎯 Gesture #%u: %s", gesture_count, names[gesture]);
	}
	
	/* Flash all LEDs on gesture */
	for (int i = 0; i < 2; i++) {
		gpio_pin_set_dt(&led0, 1);
		gpio_pin_set_dt(&led1, 1);
		gpio_pin_set_dt(&led2, 1);
		gpio_pin_set_dt(&led3, 1);
		k_msleep(100);
		gpio_pin_set_dt(&led0, 0);
		gpio_pin_set_dt(&led1, 0);
		gpio_pin_set_dt(&led2, 0);
		gpio_pin_set_dt(&led3, 0);
		k_msleep(100);
	}
}

/**
 * @brief Initialize GPIO LEDs
 */
static int init_leds(void)
{
	int ret = 0;
	
	/* Configure all LEDs as outputs */
	ret |= gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	ret |= gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
	
	if (ret < 0) {
		LOG_ERR("Failed to configure LEDs");
		return ret;
	}
	
	/* Test pattern */
	for (int i = 0; i < 4; i++) {
		gpio_pin_set_dt(&led0, i == 0);
		gpio_pin_set_dt(&led1, i == 1);
		gpio_pin_set_dt(&led2, i == 2);
		gpio_pin_set_dt(&led3, i == 3);
		k_msleep(150);
	}
	
	/* Turn off all LEDs */
	gpio_pin_set_dt(&led0, 0);
	gpio_pin_set_dt(&led1, 0);
	gpio_pin_set_dt(&led2, 0);
	gpio_pin_set_dt(&led3, 0);
	
	LOG_INF("💡 LEDs initialized");
	return 0;
}

/**
 * @brief Main application entry point
 */
int main(void)
{
	int ret;
	
	LOG_INF("🚀 Nunchuk Demo for nRF52840");
	LOG_INF("📦 Features: FPU gestures, visual feedback, auto-calibration");
	
	/* Enable USB console */
	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_WRN("USB enable failed: %d", ret);
	} else {
		LOG_INF("🔌 USB console ready");
	}
	
	/* Initialize LEDs */
	ret = init_leds();
	if (ret < 0) {
		return ret;
	}
	
	/* Check nunchuk device */
	if (!device_is_ready(nunchuk_dev)) {
		LOG_ERR("❌ Nunchuk device not ready");
		return -1;
	}
	
	/* Set up data callback */
	ret = nunchuk_set_data_callback(nunchuk_dev, nunchuk_data_callback, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to set data callback: %d", ret);
		return ret;
	}
	
	/* Set up gesture callback */
	ret = nunchuk_set_gesture_callback(nunchuk_dev, nunchuk_gesture_callback, NULL);
	if (ret < 0) {
		LOG_WRN("Gesture callback not supported: %d", ret);
	}
	
	LOG_INF("✅ Nunchuk demo ready!");
	LOG_INF("🎮 Controls:");
	LOG_INF("  🔴 C button = Auto-calibrate");
	LOG_INF("  🔵 Z button = Debug output");
	LOG_INF("  🕹️  Joystick = LED2 lights up");
	LOG_INF("  🎯 Gestures = LED flash animation");
	
	/* Main loop */
	while (1) {
		/* Handle calibration requests */
		if (calibration_requested) {
			LOG_INF("🎯 Calibrating - hold nunchuk steady...");
			k_sleep(K_MSEC(1000));  /* Give user time */
			
			ret = nunchuk_calibrate(nunchuk_dev);
			if (ret == 0) {
				LOG_INF("✅ Calibration successful!");
				/* Success blink */
				for (int i = 0; i < 3; i++) {
					gpio_pin_set_dt(&led1, 1);
					k_msleep(100);
					gpio_pin_set_dt(&led1, 0);
					k_msleep(100);
				}
			} else {
				LOG_ERR("❌ Calibration failed: %d", ret);
				/* Error blink */
				for (int i = 0; i < 3; i++) {
					gpio_pin_set_dt(&led0, 1);
					k_msleep(100);
					gpio_pin_set_dt(&led0, 0);
					k_msleep(100);
				}
			}
			calibration_requested = false;
		}
		
		/* Manual data read example */
		struct nunchuk_data manual_data;
		ret = nunchuk_read(nunchuk_dev, &manual_data);
		if (ret == 0) {
			/* Data available for custom processing */
		}
		
		/* Sleep while work queue handles periodic polling */
		k_sleep(K_SECONDS(1));
	}
	
	return 0;
}