#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>

// Settings
static const int32_t sleep_time_ms = 100;

// Get Devicetree configurations
#define MY_ADC_CH DT_ALIAS(my_adc_channel)
#define MY_ADC DT_ALIAS(my_adc)

static const struct device *adc = DEVICE_DT_GET(MY_ADC);
static const struct adc_channel_cfg adc_ch = ADC_CHANNEL_CFG_DT(MY_ADC_CH);
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

#define VDD_MV 3000

int main(void)
{
	int ret;
	uint16_t buf;
	uint16_t val_mv;
	uint32_t pulse_ns;

	// Buffer and options for ADC (defined in adc.h)
	struct adc_sequence seq = {
		.channels = BIT(adc_ch.channel_id),
		.buffer = &buf,
		.buffer_size = sizeof(buf),
		.resolution = DT_PROP(MY_ADC_CH, zephyr_resolution)
	};

	// Make sure that the ADC was initialized
	if (!device_is_ready(adc)) {
		printk("ADC peripheral is not ready\r\n");
		return 0;
	}

	// Configure ADC channel
	ret = adc_channel_setup(adc, &adc_ch);
	if (ret < 0) {
		printk("Could not set up ADC\r\n");
		return 0;
	}

	// Make sure that the PWM LED was initialized
	if (!pwm_is_ready_dt(&pwm_led)) {
		printk("PWM is not ready\r\n");
		return 0;
	}

	// Do forever
	while (1) {
		// Sample ADC
		ret = adc_read(adc, &seq);
		if (ret < 0) {
			printk("Could not read ADC: %d\r\n", ret);
			continue;
		}

		// Calculate ADC value (mV)
		val_mv = buf * VDD_MV / (1 << seq.resolution);

		// Calculate pulse width proportional to ADC reading
		// Scale ADC value (0 to max_resolution) to pulse width (0 to period)
		pulse_ns = ((uint64_t)buf * pwm_led.period) / (1 << seq.resolution);

		// Print ADC value
		printk("Raw: %u, mV: %u\r\n", buf, val_mv);
		printk("Pulse: %u ns\r\n", pulse_ns);

		// Set LED PWM pulse width
		ret = pwm_set_dt(&pwm_led, pwm_led.period, pulse_ns);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}

		// Sleep
		k_msleep(sleep_time_ms);
	}
	return 0;
}