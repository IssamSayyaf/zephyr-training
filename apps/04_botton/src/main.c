/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define BUTTON0_NODE DT_ALIAS(button6)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);

int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	if (!gpio_is_ready_dt(&button)) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret < 0) {
		return 0;
	}
	while (1) {
		if (gpio_pin_get_dt(&button) == 0) {
			k_msleep(10);
			if (gpio_pin_get_dt(&button) == 0) {
				led_state = !led_state;
				ret = gpio_pin_set_dt(&led, (int)led_state);
				if (ret < 0) {
					return 0;
				}
				printf("LED state: %s\n", led_state ? "ON" : "OFF");
				while (gpio_pin_get_dt(&button) == 0) {
					k_msleep(10);
				}
			}
		}
	}
	return 0;
}
