# Tutorial: ADC + PWM - Analog Input Controlling LED Brightness

## Table of Contents
- [Introduction](#introduction)
- [What You'll Learn](#what-youll-learn)
- [System Overview](#system-overview)
- [Prerequisites](#prerequisites)
- [Project Structure](#project-structure)
- [Device Tree Configuration](#device-tree-configuration)
- [Kconfig Configuration](#kconfig-configuration)
- [Code Walkthrough](#code-walkthrough)
- [Understanding the Integration](#understanding-the-integration)
- [Building and Running](#building-and-running)
- [Troubleshooting](#troubleshooting)
- [Exercises](#exercises)

## Introduction

This tutorial demonstrates how to **combine ADC and PWM** peripherals to create an interactive system: an analog input (potentiometer) controls LED brightness in real-time. This is a practical example of closed-loop control where sensor input directly affects actuator output.

## What You'll Learn

✅ **Peripheral Integration**: Combining ADC and PWM in a single application  
✅ **Real-time Control**: Reading analog input and updating PWM output  
✅ **Scaling and Mapping**: Converting ADC values to PWM duty cycle  
✅ **PWM for LED Dimming**: Using pulse-width modulation for brightness control  
✅ **Device Tree Configuration**: Defining both ADC and PWM peripherals  

## System Overview

### Hardware Flow

```
Potentiometer (P0.04) → ADC Reading → Scale to PWM → LED Brightness (P0.13)
```

### Control Loop

```
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│  Read ADC   │ ───> │   Scale     │ ───> │  Set PWM    │
│  (0-4095)   │      │  (0-period) │      │  Duty Cycle │
└─────────────┘      └─────────────┘      └─────────────┘
       ▲                                          │
       │                                          ▼
       └──────────── Sleep 100ms ────────── LED Brightness
```

### Functionality

- **Potentiometer at 0%** → ADC reads ~0 → PWM duty cycle 0% → LED OFF
- **Potentiometer at 50%** → ADC reads ~2048 → PWM duty cycle 50% → LED half brightness
- **Potentiometer at 100%** → ADC reads ~4095 → PWM duty cycle 100% → LED full brightness

## Prerequisites

Before starting this tutorial, you should understand:

1. **ADC basics** (see `05_adc/README.md`):
   - ADC configuration and reading
   - Resolution and reference voltage
   - Converting raw values to voltage

2. **PWM basics** (concepts covered here):
   - Pulse Width Modulation for brightness control
   - Period and pulse width relationship
   - Duty cycle calculation

3. **Device Tree** (covered in previous tutorials):
   - Overlays and aliases
   - Pin configuration

## Project Structure

```
06_adc_pwm/
├── CMakeLists.txt          # Build configuration
├── prj.conf                # Enable ADC and PWM drivers
├── board/
│   └── app.overlay         # ADC + PWM Device Tree configuration
├── src/
│   └── main.c              # Application: ADC → PWM control
└── README.md               # This file
```

## Device Tree Configuration

### File: `board/app.overlay`

#### 1. **Aliases for Easy Access**

```dts
/ {
    aliases {
        my-adc = &adc;                  /* ADC device */
        my-adc-channel = &adc0_ch0;     /* ADC channel 0 */
        pwm-led0 = &pwm_led0;           /* PWM-controlled LED */
    };
```

- Provides portable names for peripherals
- Used in code with `DT_ALIAS()` macro

#### 2. **PWM LED Definition**

```dts
    /* PWM-controlled LED */
    pwmleds {
        compatible = "pwm-leds";        /* Standard PWM LED binding */
        pwm_led0: pwm_led_0 {
            /* PWM device, channel, period, polarity */
            pwms = <&pwm0 0 PWM_MSEC(1) PWM_POLARITY_INVERTED>;
            label = "PWM LED 0";
        };
    };
};
```

**Breakdown:**

- **`compatible = "pwm-leds"`**: Uses Zephyr's PWM LED driver
- **`pwms = <&pwm0 0 PWM_MSEC(1) PWM_POLARITY_INVERTED>`**:
  - `&pwm0`: Reference to PWM peripheral 0
  - `0`: PWM channel 0
  - `PWM_MSEC(1)`: Period = 1 millisecond (1 kHz frequency)
  - `PWM_POLARITY_INVERTED`: LED is active-low (common cathode)

**Why inverted polarity?**
- nRF52840 DK LEDs are **active-low** (cathode connected to GPIO, anode to VDD)
- GPIO HIGH → LED OFF
- GPIO LOW → LED ON
- `PWM_POLARITY_INVERTED` automatically handles this

#### 3. **Disable Default GPIO LED**

```dts
/* Repurpose LED1 (P0.13) for PWM, disable GPIO mode */
&led0 {
    status = "disabled";
};
```

- LED0 is typically P0.13 in GPIO mode
- We need P0.13 for PWM, so disable GPIO LED driver

#### 4. **Enable and Configure PWM0**

```dts
&pwm0 {
    status = "okay";
    pinctrl-0 = <&pwm0_default>;        /* Active state pin config */
    pinctrl-1 = <&pwm0_sleep>;          /* Low-power state pin config */
    pinctrl-names = "default", "sleep";
};
```

- Enables PWM peripheral 0
- References pin control states (defined next)

#### 5. **Pin Control Configuration**

```dts
&pinctrl {
    pwm0_default: pwm0_default {
        group1 {
            /* Route PWM channel 0 to P0.13 */
            psels = <NRF_PSEL(PWM_OUT0, 0, 13)>;
        };
    };

    pwm0_sleep: pwm0_sleep {
        group1 {
            psels = <NRF_PSEL(PWM_OUT0, 0, 13)>;
            low-power-enable;               /* Reduce power in sleep */
        };
    };
};
```

**Understanding `NRF_PSEL`:**

```c
NRF_PSEL(PWM_OUT0, 0, 13)
         ^^^^^^^^  ^  ^^
         Signal    Port Pin
```

- **PWM_OUT0**: PWM channel 0 output signal
- **0**: GPIO port 0
- **13**: Pin 13 (P0.13)

Maps PWM hardware channel to physical pin.

#### 6. **ADC Configuration**

```dts
&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    adc0_ch0: channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1_4";
        zephyr,reference = "ADC_REF_VDD_1_4";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN2>;   /* P0.04 */
        zephyr,resolution = <12>;                    /* 12-bit: 0-4095 */
    };
};
```

- Same ADC configuration as `05_adc` example
- Reads analog input on P0.04 (AIN2)
- 12-bit resolution with full VDD range (0-3.3V)

## Kconfig Configuration

### File: `prj.conf`

```properties
CONFIG_ADC=y           # Enable ADC driver
CONFIG_PWM=y           # Enable PWM subsystem
CONFIG_PWM_NRFX=y      # Enable nRF PWM driver
CONFIG_GPIO=y          # Enable GPIO (for button, if needed)
CONFIG_LOG=y           # Enable logging
CONFIG_SERIAL=y        # Serial console
CONFIG_UART_CONSOLE=y  # UART console for printk
```

**Key Configuration:**

- **CONFIG_PWM=y**: Enables PWM API
- **CONFIG_PWM_NRFX=y**: nRF52840-specific PWM driver
- Both ADC and PWM drivers enabled simultaneously

## Code Walkthrough

### File: `src/main.c`

#### 1. **Include Headers and Macros**

```c
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>

// Settings
static const int32_t sleep_time_ms = 100;  /* Sample rate: 10 Hz */

// Get Device Tree configurations
#define MY_ADC_CH DT_ALIAS(my_adc_channel)
#define MY_ADC DT_ALIAS(my_adc)
#define MY_PWM DT_ALIAS(pwm_led0)

#define VDD_MV 3000  /* VDD voltage in millivolts (adjust if different) */
```

- `<zephyr/drivers/pwm.h>`: PWM API functions
- `MY_PWM`: Alias to `pwm_led0` node defined in overlay

#### 2. **Get Device Handles**

```c
static const struct device *adc = DEVICE_DT_GET(MY_ADC);
static const struct adc_channel_cfg adc_ch = ADC_CHANNEL_CFG_DT(MY_ADC_CH);
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(MY_PWM);
```

**New: `PWM_DT_SPEC_GET()`**

- Extracts PWM configuration from Device Tree
- Populates `struct pwm_dt_spec` with:
  - Device pointer
  - PWM channel
  - Period (in nanoseconds)
  - Flags (polarity)

**Structure:**

```c
struct pwm_dt_spec {
    const struct device *dev;  /* PWM device */
    uint32_t channel;          /* PWM channel (0) */
    uint32_t period;           /* Period in ns (1,000,000 for 1ms) */
    pwm_flags_t flags;         /* PWM_POLARITY_INVERTED */
};
```

#### 3. **Prepare ADC Sequence**

```c
int main(void) {
    int ret;
    uint16_t buf;          /* ADC reading buffer */
    uint16_t val_mv;       /* ADC value in millivolts */
    uint32_t pulse_ns;     /* PWM pulse width in nanoseconds */

    struct adc_sequence seq = {
        .channels = BIT(adc_ch.channel_id),
        .buffer = &buf,
        .buffer_size = sizeof(buf),
        .resolution = DT_PROP(MY_ADC_CH, zephyr_resolution)  /* 12 */
    };
```

- Standard ADC sequence setup (see `05_adc` tutorial)

#### 4. **Initialize ADC**

```c
    // Make sure ADC is ready
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
```

- Verify ADC device is ready
- Apply channel configuration

#### 5. **Initialize PWM**

```c
    // Make sure PWM LED is ready
    if (!pwm_is_ready_dt(&pwm_led)) {
        printk("PWM is not ready\r\n");
        return 0;
    }
```

**New API: `pwm_is_ready_dt()`**

- Checks if PWM device is initialized
- Takes `pwm_dt_spec` structure (includes device pointer)

#### 6. **Control Loop: ADC → PWM**

```c
    while (1) {
        // Sample ADC
        ret = adc_read(adc, &seq);
        if (ret < 0) {
            printk("Could not read ADC: %d\r\n", ret);
            continue;
        }

        // Calculate ADC value in millivolts
        val_mv = buf * VDD_MV / (1 << seq.resolution);

        // Calculate pulse width proportional to ADC reading
        // Scale ADC value (0 to 4095) to pulse width (0 to period)
        pulse_ns = ((uint64_t)buf * pwm_led.period) / (1 << seq.resolution);

        // Print values
        printk("Raw: %u, mV: %u\r\n", buf, val_mv);
        printk("Pulse: %u ns\r\n", pulse_ns);

        // Set LED PWM pulse width
        ret = pwm_set_dt(&pwm_led, pwm_led.period, pulse_ns);
        if (ret) {
            printk("Error %d: failed to set pulse width\n", ret);
            return 0;
        }

        // Sleep 100ms before next reading
        k_msleep(sleep_time_ms);
    }
    return 0;
}
```

### Understanding the PWM Scaling Formula

```c
pulse_ns = ((uint64_t)buf * pwm_led.period) / (1 << seq.resolution);
```

**Breaking it down:**

- **`buf`**: ADC raw value (0 to 4095 for 12-bit)
- **`pwm_led.period`**: PWM period in nanoseconds (1,000,000 ns = 1 ms)
- **`(1 << seq.resolution)`**: Maximum ADC value (4096 for 12-bit)
- **Cast to `uint64_t`**: Prevent overflow during multiplication

**Example calculations:**

| ADC Value | Formula | Pulse Width | Duty Cycle | LED Brightness |
|-----------|---------|-------------|------------|----------------|
| 0 | (0 × 1,000,000) / 4096 | 0 ns | 0% | OFF |
| 1024 | (1024 × 1,000,000) / 4096 | 250,000 ns | 25% | Dim |
| 2048 | (2048 × 1,000,000) / 4096 | 500,000 ns | 50% | Half |
| 4095 | (4095 × 1,000,000) / 4096 | 999,756 ns | ~100% | Full |

### PWM API: `pwm_set_dt()`

```c
ret = pwm_set_dt(&pwm_led, pwm_led.period, pulse_ns);
```

**Parameters:**

1. **`&pwm_led`**: Pointer to `pwm_dt_spec` (contains device, channel, flags)
2. **`pwm_led.period`**: PWM period in nanoseconds (constant, 1 ms)
3. **`pulse_ns`**: Pulse width in nanoseconds (variable, 0 to period)

**Duty Cycle = `pulse_ns / period`**

- `pulse_ns = 0` → 0% duty cycle → LED OFF
- `pulse_ns = period/2` → 50% duty cycle → LED half brightness
- `pulse_ns = period` → 100% duty cycle → LED full brightness

## Understanding the Integration

### PWM Fundamentals

**Pulse Width Modulation (PWM)** controls average power by rapidly switching on/off:

```
Period = 1 ms (1000 µs)
Frequency = 1 / Period = 1 kHz

Duty Cycle 25%:
  ┌──┐     ┌──┐     ┌──┐
──┘  └─────┘  └─────┘  └─────
  250µs on, 750µs off

Duty Cycle 75%:
  ┌──────┐  ┌──────┐  ┌──────┐
──┘      └──┘      └──┘      └──
  750µs on, 250µs off
```

**For LED brightness:**

- Human eye perceives average brightness
- Higher duty cycle = brighter LED
- PWM frequency (1 kHz) fast enough to avoid visible flicker

### ADC to PWM Mapping

```
ADC Domain              PWM Domain
(Digital Values)        (Time)

0 ────────────────────> 0 ns (0% duty)
                        LED OFF

2048 ──────────────────> 500,000 ns (50% duty)
(Half-scale)            LED half brightness

4095 ──────────────────> 1,000,000 ns (100% duty)
(Max)                   LED full brightness
```

### Closed-Loop Control System

```
     ┌─────────────────────────────────────┐
     │                                     │
     ▼                                     │
┌─────────┐    ┌────────┐    ┌──────┐    │
│ Sensor  │───>│ ADC    │───>│ PWM  │───>│ LED
│ (Pot)   │    │ Read   │    │ Set  │    │
└─────────┘    └────────┘    └──────┘    │
                    │                      │
                    └──────────────────────┘
                  Every 100ms (10 Hz)
```

- **Open-loop**: No feedback (PWM runs independently)
- **Closed-loop (future)**: Could add light sensor to maintain target brightness

## Building and Running

### Build Commands

```bash
cd ~/Zephyr_tutorial/apps/06_adc_pwm
west build -b nrf52840dk/nrf52840 -p
```

### Flash to Board

```bash
west flash
```

### View Output

```bash
screen /dev/ttyACM0 115200
# Or
minicom -D /dev/ttyACM0
```

### Expected Output

```
Raw: 0, mV: 0
Pulse: 0 ns
Raw: 512, mV: 375
Pulse: 125000 ns
Raw: 2048, mV: 1500
Pulse: 500000 ns
Raw: 4095, mV: 2998
Pulse: 999756 ns
```

### Hardware Setup

```
              VDD (3.3V)
                |
                ┴
        Potentiometer
               ╱│╲
              ╱ │ ╲
        ────┴  │  ┴────
               │
               └─────> P0.04 (ADC Input)
               
               
LED1 (P0.13) controlled by PWM
```

**Test:**

1. Rotate potentiometer fully counterclockwise → ADC reads 0 → LED OFF
2. Rotate to middle position → ADC reads ~2048 → LED half brightness
3. Rotate fully clockwise → ADC reads ~4095 → LED full brightness

## Troubleshooting

### Problem: LED not responding to potentiometer

**Causes:**
1. PWM not configured correctly
2. ADC reading not working
3. Wiring issue

**Solutions:**

1. **Check PWM initialization:**
   ```c
   if (!pwm_is_ready_dt(&pwm_led)) {
       printk("PWM not ready\n");  // Should not appear
   }
   ```

2. **Verify ADC readings:**
   - Check serial output for `Raw:` values
   - Should change when rotating potentiometer
   - If stuck at 0 or 4095, check ADC wiring

3. **Test PWM independently:**
   ```c
   // Set fixed 50% duty cycle for testing
   pwm_set_dt(&pwm_led, pwm_led.period, pwm_led.period / 2);
   ```

---

### Problem: LED always full brightness or off

**Cause**: PWM polarity incorrect

**Solution**: Check `app.overlay`:

```dts
pwms = <&pwm0 0 PWM_MSEC(1) PWM_POLARITY_INVERTED>;
                            ^^^^^^^^^^^^^^^^^^^^^^^
```

- nRF52840 DK LEDs are **active-low**
- Use `PWM_POLARITY_INVERTED`
- If LED behavior is reversed, try `PWM_POLARITY_NORMAL`

---

### Problem: LED flickers or unstable

**Causes:**
1. Noisy ADC readings
2. PWM frequency too low

**Solutions:**

1. **Average ADC readings:**
   ```c
   #define AVG_SAMPLES 5
   uint32_t sum = 0;
   for (int i = 0; i < AVG_SAMPLES; i++) {
       adc_read(adc, &seq);
       sum += buf;
       k_msleep(5);
   }
   buf = sum / AVG_SAMPLES;
   ```

2. **Increase PWM frequency:**
   ```dts
   /* Change period from 1ms to 0.1ms (10 kHz) */
   pwms = <&pwm0 0 PWM_USEC(100) PWM_POLARITY_INVERTED>;
   ```

---

### Problem: Build error "PWM_DT_SPEC_GET undeclared"

**Cause**: Missing PWM driver configuration

**Solution**: Verify `prj.conf`:

```properties
CONFIG_PWM=y
CONFIG_PWM_NRFX=y
```

---

### Problem: Integer overflow in pulse calculation

**Symptom**: Incorrect pulse width values, LED behavior erratic

**Cause**: Multiplication overflow

**Solution**: Cast to 64-bit:

```c
pulse_ns = ((uint64_t)buf * pwm_led.period) / (1 << seq.resolution);
           ^^^^^^^^^^^
           Essential for large values
```

---

### Problem: "PWM is not ready" error

**Cause**: PWM peripheral not enabled in Device Tree

**Solution**: Check `app.overlay`:

```dts
&pwm0 {
    status = "okay";  /* Must be "okay" */
```

Also verify pin control configuration exists.

## Exercises

### Exercise 1: Reverse LED Control

**Goal**: LED brightness increases as potentiometer decreases (inverse control)

```c
// Instead of:
pulse_ns = ((uint64_t)buf * pwm_led.period) / (1 << seq.resolution);

// Use:
uint16_t inverted_buf = ((1 << seq.resolution) - 1) - buf;
pulse_ns = ((uint64_t)inverted_buf * pwm_led.period) / (1 << seq.resolution);
```

**Result**: Potentiometer at 0% → LED full brightness

---

### Exercise 2: Non-Linear Response

**Goal**: Implement exponential brightness curve (more sensitivity at low end)

```c
// Square the normalized value for exponential curve
uint32_t normalized = ((uint64_t)buf * 1000) / (1 << seq.resolution);
uint32_t squared = (normalized * normalized) / 1000;
pulse_ns = (squared * pwm_led.period) / 1000;
```

**Explanation**: Squaring creates exponential response, makes low-end more responsive

---

### Exercise 3: Control Multiple LEDs

**Goal**: Control 2 LEDs with different brightness patterns

1. **Add second PWM LED in `app.overlay`:**

```dts
pwm_led1: pwm_led_1 {
    pwms = <&pwm0 1 PWM_MSEC(1) PWM_POLARITY_INVERTED>;
    label = "PWM LED 1";
};

// Add alias
pwm-led1 = &pwm_led1;
```

2. **Configure second pin in pinctrl:**

```dts
pwm0_default: pwm0_default {
    group1 {
        psels = <NRF_PSEL(PWM_OUT0, 0, 13)>,
                <NRF_PSEL(PWM_OUT1, 0, 14)>;  /* P0.14 for LED2 */
    };
};
```

3. **Update code:**

```c
#define MY_PWM1 DT_ALIAS(pwm_led1)
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(MY_PWM1);

// In main loop:
pwm_set_dt(&pwm_led0, pwm_led0.period, pulse_ns);
pwm_set_dt(&pwm_led1, pwm_led1.period, pwm_led1.period - pulse_ns);  // Inverse
```

**Result**: LED0 gets brighter, LED1 gets dimmer (opposite directions)

---

### Exercise 4: Threshold with Hysteresis

**Goal**: LED OFF below threshold, full brightness above threshold, with hysteresis to prevent flickering

```c
#define THRESHOLD_LOW  1500   /* Turn off threshold (mV) */
#define THRESHOLD_HIGH 1700   /* Turn on threshold (mV) */
static bool led_state = false;

// In loop:
val_mv = buf * VDD_MV / (1 << seq.resolution);

if (val_mv > THRESHOLD_HIGH) {
    led_state = true;
} else if (val_mv < THRESHOLD_LOW) {
    led_state = false;
}

if (led_state) {
    pwm_set_dt(&pwm_led, pwm_led.period, pwm_led.period);  // Full brightness
} else {
    pwm_set_dt(&pwm_led, pwm_led.period, 0);  // Off
}
```

**Hysteresis**: Prevents rapid on/off switching near threshold

---

### Exercise 5: PWM Frequency Experiment

**Goal**: Compare different PWM frequencies and observe effects

1. **Modify `app.overlay` for 100 Hz (10ms period):**

```dts
pwms = <&pwm0 0 PWM_MSEC(10) PWM_POLARITY_INVERTED>;
```

2. **Test frequencies:**
   - 100 Hz (10 ms): May see flicker
   - 1 kHz (1 ms): Smooth (current default)
   - 10 kHz (100 µs): Very smooth, higher CPU load

3. **Measure with oscilloscope** (if available):
   - Connect probe to P0.13
   - Observe pulse width changes

**Observation**: Higher frequency = smoother perception, but may have hardware limitations

---

### Exercise 6: PID-Like Control (Advanced)

**Goal**: Implement proportional control with deadband

```c
#define TARGET_MV 1650      /* Target: 1.65V (half of 3.3V) */
#define DEADBAND_MV 50      /* ±50mV deadband */
#define KP 100              /* Proportional gain */

// In loop:
val_mv = buf * VDD_MV / (1 << seq.resolution);
int error = TARGET_MV - val_mv;

if (abs(error) < DEADBAND_MV) {
    // Within deadband, no change
} else {
    // Proportional response
    int correction = (error * KP) / 1000;
    // Apply to PWM (requires additional hardware, e.g., heater)
    // This is conceptual - you'd need actuator that affects ADC reading
}
```

**Note**: This is conceptual. True closed-loop requires actuator affecting sensor (e.g., heating element affecting temperature sensor).

---

### Exercise 7: Smooth Transitions

**Goal**: Gradually fade LED brightness instead of instant changes

```c
static uint32_t current_pulse = 0;
#define FADE_STEP 10000  /* Change by 10,000 ns per iteration */

// In loop:
uint32_t target_pulse = ((uint64_t)buf * pwm_led.period) / (1 << seq.resolution);

if (current_pulse < target_pulse) {
    current_pulse += FADE_STEP;
    if (current_pulse > target_pulse) current_pulse = target_pulse;
} else if (current_pulse > target_pulse) {
    current_pulse -= FADE_STEP;
    if (current_pulse < target_pulse) current_pulse = target_pulse;
}

pwm_set_dt(&pwm_led, pwm_led.period, current_pulse);
k_msleep(10);  /* Faster loop for smooth fading */
```

**Result**: LED brightness changes smoothly, not instantly

---

## Summary

You've learned:

✅ **Peripheral Integration**: Combining ADC and PWM in one application  
✅ **PWM API**: `PWM_DT_SPEC_GET()`, `pwm_is_ready_dt()`, `pwm_set_dt()`  
✅ **Device Tree Configuration**: Defining PWM LEDs, pin control, multiple peripherals  
✅ **Scaling and Mapping**: Converting ADC values to PWM duty cycle  
✅ **Real-time Control**: Closed-loop system responding to sensor input  
✅ **PWM Concepts**: Period, duty cycle, frequency, polarity  

### Key Concepts

1. **PWM for Analog Output**: PWM creates analog-like output from digital signals
2. **Duty Cycle**: Ratio of on-time to period determines average power/brightness
3. **Scaling Formula**: Map input range to output range with proportional multiplication
4. **Device Tree Integration**: Define multiple peripherals with proper pin routing
5. **Peripheral Interaction**: Read sensor, process data, control actuator

### Practical Applications

- **Motor speed control**: ADC from joystick → PWM to motor driver
- **Temperature control**: ADC from thermistor → PWM to heater/fan
- **Audio volume**: ADC from volume knob → PWM to audio amplifier
- **Servo position**: ADC from position sensor → PWM to servo motor
- **Power supply**: ADC from voltage sensor → PWM for buck/boost converter

### Next Steps

1. **Add feedback control**: Implement PID controller for precise regulation
2. **Multi-channel control**: Control RGB LED (3 PWM channels) with color mixing
3. **Communication**: Send ADC/PWM data via UART, I2C, or Bluetooth
4. **Data logging**: Store sensor readings and PWM settings to flash memory
5. **Safety features**: Add limits, error detection, watchdog timers

---

**Congratulations!** You now understand how to integrate multiple peripherals in Zephyr for real-time control systems. This pattern applies to countless embedded applications requiring sensor-actuator feedback loops.

