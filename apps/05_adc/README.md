# Tutorial: ADC (Analog-to-Digital Converter)

## Table of Contents
- [Introduction](#introduction)
- [What is ADC?](#what-is-adc)
- [ADC in Zephyr](#adc-in-zephyr)
- [Project Structure](#project-structure)
- [Device Tree Configuration](#device-tree-configuration)
- [Kconfig Configuration](#kconfig-configuration)
- [Code Walkthrough](#code-walkthrough)
- [Understanding ADC Parameters](#understanding-adc-parameters)
- [Building and Running](#building-and-running)
- [Troubleshooting](#troubleshooting)
- [Exercises](#exercises)

## Introduction

This tutorial demonstrates how to use the **ADC (Analog-to-Digital Converter)** peripheral in Zephyr RTOS. An ADC converts analog voltage signals (continuous values) into digital values that a microcontroller can process. This is essential for reading sensors like temperature sensors, potentiometers, light sensors, and any other analog input.

## What is ADC?

### Basic Concept

An ADC converts an analog voltage (e.g., 0V to 3.3V) into a digital number:

```
Analog Input (0-3.3V) → ADC → Digital Value (0-4095 for 12-bit)
```

### Key ADC Specifications

1. **Resolution**: Number of bits in the output
   - 12-bit ADC: Values from 0 to 4095 (2^12 - 1)
   - Higher resolution = more precise measurements
   - nRF52840 SAADC (Successive Approximation ADC): up to 12-bit

2. **Reference Voltage**: The voltage that represents the maximum ADC value
   - If reference = 3.3V and value = 4095, input is 3.3V
   - If reference = 3.3V and value = 2048, input is ~1.65V

3. **Gain**: Amplification applied to input signal
   - `ADC_GAIN_1_4`: Input divided by 4 (can measure up to 4× reference voltage)
   - `ADC_GAIN_1`: No amplification (1:1)
   - Allows measuring voltages higher than reference

4. **Acquisition Time**: Time ADC spends sampling the input
   - Longer = more stable but slower readings
   - `ADC_ACQ_TIME_DEFAULT`: Let driver choose optimal value

5. **Sampling Rate**: How often ADC readings are taken
   - Controlled by your application code (e.g., every 100ms)

### Voltage Conversion Formula

```c
// Convert raw ADC value to millivolts
voltage_mv = (raw_value × reference_voltage_mv) / (2^resolution)
```

For our example (12-bit, VDD reference of 3300mV):
```c
voltage_mv = (raw_value × 3300) / 4096
```

## ADC in Zephyr

Zephyr's ADC API provides:
- **Device Tree binding**: Hardware configuration (channels, pins, reference)
- **Driver API**: Functions to configure and read ADC channels
- **Sequence-based reading**: Configure once, read multiple times efficiently

### ADC Workflow

```
1. Define ADC channel in Device Tree (pin, resolution, reference)
2. Get ADC device handle (DEVICE_DT_GET)
3. Configure channel settings (adc_channel_setup)
4. Prepare read sequence (struct adc_sequence)
5. Read ADC value (adc_read)
6. Convert raw value to voltage
```

## Project Structure

```
05_adc/
├── CMakeLists.txt          # Build configuration
├── prj.conf                # Enable ADC driver
├── board/
│   └── app.overlay         # ADC Device Tree configuration
├── src/
│   └── main.c              # ADC reading application
└── README.md               # This file
```

## Device Tree Configuration

### File: `board/app.overlay`

```dts
/ {
    aliases {
        my-adc = &adc;              /* Alias for ADC device */
        my-adc-channel = &adc0_ch0; /* Alias for specific channel */
    };
};

&adc {
    status = "okay";               /* Enable ADC peripheral */
    #address-cells = <1>;
    #size-cells = <0>;

    /* Channel 0 on P0.04 (AIN2) */
    adc0_ch0: channel@0 {
        reg = <0>;                                      /* Channel number */
        zephyr,gain = "ADC_GAIN_1_4";                  /* Input gain (×0.25) */
        zephyr,reference = "ADC_REF_VDD_1_4";          /* Reference: VDD/4 */
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN2>;      /* Physical pin: P0.04 */
        zephyr,resolution = <12>;                       /* 12-bit resolution */
    };
};
```

### Understanding the Configuration

#### 1. **Aliases**
```dts
aliases {
    my-adc = &adc;              /* Reference to ADC peripheral */
    my-adc-channel = &adc0_ch0; /* Reference to channel configuration */
};
```
- Makes code portable (use `MY_ADC` macro instead of hardcoded device name)
- Easier to change hardware without modifying source code

#### 2. **ADC Node Extension (`&adc`)**
```dts
&adc {
    status = "okay";  /* Enable the ADC peripheral */
```
- `&adc`: Reference to ADC peripheral defined in board's base `.dts` file
- `status = "okay"`: Enable the peripheral (disabled by default on some boards)

#### 3. **Channel Configuration**
```dts
adc0_ch0: channel@0 {
    reg = <0>;  /* Channel number (0-7 on nRF52840) */
```
- `channel@0`: Channel identifier (matches `reg` value)
- `reg = <0>`: Hardware channel number

#### 4. **Gain Setting**
```dts
zephyr,gain = "ADC_GAIN_1_4";  /* Input divided by 4 */
```
- **ADC_GAIN_1_4**: Measures up to 4× reference voltage (extended range)
- With VDD/4 reference (~0.825V), can measure: 0.825V × 4 = 3.3V
- Other options: `ADC_GAIN_1`, `ADC_GAIN_1_2`, `ADC_GAIN_2`

#### 5. **Reference Voltage**
```dts
zephyr,reference = "ADC_REF_VDD_1_4";  /* VDD ÷ 4 as reference */
```
- **ADC_REF_VDD_1_4**: VDD voltage divided by 4 (~0.825V if VDD = 3.3V)
- Combined with GAIN_1_4, effective range: 0-3.3V
- Other options: `ADC_REF_INTERNAL` (0.6V), `ADC_REF_VDD_1` (full VDD)

#### 6. **Physical Pin Mapping**
```dts
zephyr,input-positive = <NRF_SAADC_AIN2>;  /* P0.04 */
```
- **NRF_SAADC_AIN2**: Analog input 2 of nRF52840 SAADC
- Maps to physical pin **P0.04** (check nRF52840 pinout)
- AIN0=P0.02, AIN1=P0.03, AIN2=P0.04, AIN3=P0.05, etc.

#### 7. **Resolution**
```dts
zephyr,resolution = <12>;  /* 12-bit ADC (0-4095) */
```
- 12-bit: Values from 0 to 4095
- Higher resolution = better precision

## Kconfig Configuration

### File: `prj.conf`

```properties
CONFIG_ADC=y               # Enable ADC driver
CONFIG_GPIO=y              # Enable GPIO (if using buttons/LEDs)
CONFIG_LOG=y               # Enable logging
CONFIG_LOG_DEFAULT_LEVEL=3 # Info level logging
```

- **CONFIG_ADC=y**: Essential for ADC support
- Enables the ADC driver and API functions

## Code Walkthrough

### File: `src/main.c`

#### 1. **Include Headers and Device Tree Macros**

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

/* Device Tree aliases */
#define MY_ADC_CH DT_ALIAS(my_adc_channel)  /* ADC channel configuration */
#define MY_ADC DT_ALIAS(my_adc)             /* ADC device */

#define VDD_MV 3300  /* VDD voltage in millivolts */
```

- `DT_ALIAS(my_adc_channel)`: Gets node ID from `my-adc-channel` alias
- `DT_ALIAS(my_adc)`: Gets ADC device node ID
- `VDD_MV`: Reference voltage for conversion calculation

#### 2. **Get ADC Device Handle**

```c
static const struct device *adc_dev = DEVICE_DT_GET(MY_ADC);
```

- `DEVICE_DT_GET()`: Retrieves device pointer from Device Tree
- Returns a `struct device*` representing the ADC peripheral

#### 3. **Configure ADC Channel Settings**

```c
static const struct adc_channel_cfg channel_cfg = ADC_CHANNEL_CFG_DT(MY_ADC_CH);
```

- `ADC_CHANNEL_CFG_DT()`: Extracts channel configuration from Device Tree
- Populates structure with:
  - Channel number (`reg = <0>`)
  - Gain setting
  - Reference voltage
  - Acquisition time
  - Differential/single-ended mode

#### 4. **Prepare ADC Read Sequence**

```c
static int16_t buf;  /* Buffer to store ADC reading */

static struct adc_sequence seq = {
    .channels     = BIT(DT_REG_ADDR(MY_ADC_CH)),  /* Bitmask of channels to read */
    .buffer       = &buf,                          /* Where to store result */
    .buffer_size  = sizeof(buf),                   /* Buffer size in bytes */
    .resolution   = DT_PROP(MY_ADC_CH, zephyr_resolution),  /* From DT */
};
```

**Breakdown:**

- **`.channels`**: Bitmask of channels to read
  ```c
  BIT(0) = 0b00000001  /* Read channel 0 */
  BIT(1) = 0b00000010  /* Read channel 1 */
  BIT(0) | BIT(2) = 0b00000101  /* Read channels 0 and 2 */
  ```
  - `DT_REG_ADDR(MY_ADC_CH)` returns channel number from `reg = <0>`

- **`.buffer`**: Pointer to buffer storing ADC result
  - Type: `int16_t` (signed 16-bit) to handle negative values in differential mode

- **`.resolution`**: Number of bits (12 in this example)
  - Must match Device Tree `zephyr,resolution = <12>`

#### 5. **Initialize ADC in Main**

```c
int main(void) {
    int err;

    /* Check if ADC device is ready */
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -1;
    }

    /* Configure the ADC channel */
    err = adc_channel_setup(adc_dev, &channel_cfg);
    if (err) {
        LOG_ERR("Failed to setup ADC channel (err %d)", err);
        return -1;
    }

    LOG_INF("ADC initialized successfully");
```

- **`device_is_ready()`**: Verifies device is initialized and ready
- **`adc_channel_setup()`**: Applies channel configuration
  - Must be called before reading ADC

#### 6. **Read ADC in Loop**

```c
    while (1) {
        /* Read ADC value */
        err = adc_read(adc_dev, &seq);
        if (err) {
            LOG_ERR("ADC read failed (err %d)", err);
        } else {
            /* Convert raw value to millivolts */
            int val_mv = (int)((int64_t)buf * VDD_MV / (1 << seq.resolution));
            
            LOG_INF("ADC raw: %d, voltage: %d mV", buf, val_mv);
        }

        k_msleep(100);  /* Sample every 100ms */
    }
}
```

**ADC Reading:**

```c
err = adc_read(adc_dev, &seq);
```
- Triggers ADC conversion
- Stores result in `seq.buffer` (`buf` variable)
- Returns 0 on success, negative error code on failure

**Voltage Conversion:**

```c
int val_mv = (int)((int64_t)buf * VDD_MV / (1 << seq.resolution));
```

Breaking it down:
```c
// (1 << seq.resolution) = 2^12 = 4096 (for 12-bit)
// Formula: voltage = (raw_value × reference_voltage) / max_adc_value

// Example: buf = 2048 (half of 4096)
val_mv = (2048 × 3300) / 4096 = 1650 mV (1.65V)
```

- `(int64_t)buf`: Cast to 64-bit to prevent overflow during multiplication
- `VDD_MV`: Reference voltage (3300 mV)
- `(1 << seq.resolution)`: Maximum ADC value (4096 for 12-bit)

## Understanding ADC Parameters

### Resolution vs Precision

```
8-bit:  256 steps   → ~13 mV per step (3.3V / 256)
10-bit: 1024 steps  → ~3.2 mV per step (3.3V / 1024)
12-bit: 4096 steps  → ~0.8 mV per step (3.3V / 4096)
```

Higher resolution gives finer voltage measurements but may be slower.

### Gain and Reference Combinations

| Gain | Reference | Effective Range | Use Case |
|------|-----------|-----------------|----------|
| 1/4 | VDD/4 | 0 - VDD (3.3V) | General purpose, full range |
| 1 | VDD/4 | 0 - VDD/4 (~0.825V) | Low voltage, high precision |
| 1 | Internal (0.6V) | 0 - 0.6V | Battery monitoring |

**Current configuration:**
- Gain: 1/4 (ADC_GAIN_1_4)
- Reference: VDD/4 (ADC_REF_VDD_1_4)
- **Effective range: 0 to 3.3V** (full VDD range)

### Acquisition Time

```dts
zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
```

- **Default**: Driver chooses optimal value (typically 10-40 µs)
- **Longer**: More stable readings, better for high-impedance sources
- **Shorter**: Faster sampling, may be noisier

Custom example:
```dts
zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)>;
```

### Sampling Rate in Code

```c
k_msleep(100);  /* 100ms delay = 10 samples/second */
```

- **Faster sampling**: Lower delay (e.g., 10ms = 100 Hz)
- **Slower sampling**: Higher delay (e.g., 1000ms = 1 Hz)
- Balance: CPU usage vs. responsiveness

## Building and Running

### Build Commands

```bash
cd ~/Zephyr_tutorial/apps/05_adc
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
[00:00:00.123,456] <inf> main: ADC initialized successfully
[00:00:00.223,456] <inf> main: ADC raw: 2048, voltage: 1650 mV
[00:00:00.323,456] <inf> main: ADC raw: 2050, voltage: 1651 mV
[00:00:00.423,456] <inf> main: ADC raw: 2047, voltage: 1649 mV
```

### Testing

Connect a potentiometer or voltage source to **P0.04 (AIN2)**:

```
              VDD (3.3V)
                |
                ┴
               ╱ ╲  Potentiometer
              ╱   ╲
             ├─────┤ ← Wiper to P0.04
             |
            GND
```

Rotate potentiometer → voltage changes → ADC readings change

## Troubleshooting

### Problem: "ADC device not ready"

**Cause**: ADC peripheral not enabled in Device Tree

**Solution**: Verify `board/app.overlay`:
```dts
&adc {
    status = "okay";  /* Must be "okay" */
```

---

### Problem: "Failed to setup ADC channel"

**Cause**: Invalid channel configuration or unsupported parameters

**Solution**:
1. Check channel number matches hardware (0-7 for nRF52840)
2. Verify gain/reference combination is valid
3. Ensure `CONFIG_ADC=y` in `prj.conf`

---

### Problem: ADC reads always 0 or 4095

**Cause**: Pin not connected or wrong pin

**Solution**:
1. Verify physical pin mapping:
   ```
   NRF_SAADC_AIN2 = P0.04 (check board schematic)
   ```
2. Check wiring connections
3. Ensure input voltage is within 0-3.3V range

---

### Problem: Noisy/unstable readings

**Causes**:
- High-impedance source
- Electromagnetic interference
- Insufficient acquisition time

**Solutions**:
1. Add capacitor (0.1µF) between input and GND
2. Increase acquisition time:
   ```dts
   zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)>;
   ```
3. Average multiple readings:
   ```c
   int sum = 0;
   for (int i = 0; i < 10; i++) {
       adc_read(adc_dev, &seq);
       sum += buf;
       k_msleep(1);
   }
   int avg = sum / 10;
   ```

---

### Problem: Incorrect voltage readings

**Cause**: Wrong conversion formula or VDD_MV value

**Solution**:
1. Verify VDD voltage (measure with multimeter):
   ```c
   #define VDD_MV 3300  /* Update if different */
   ```
2. Check formula matches gain/reference:
   ```c
   // For GAIN_1_4 + REF_VDD_1_4:
   val_mv = (buf * VDD_MV) / (1 << resolution);
   ```

---

### Problem: Build error "ADC_ACQ_TIME_DEFAULT not defined"

**Cause**: Missing ADC driver includes

**Solution**: Add to `src/main.c`:
```c
#include <zephyr/drivers/adc.h>
```

## Exercises

### Exercise 1: Change Sampling Rate

Modify the code to sample at **1 Hz** (once per second):

```c
k_msleep(1000);  /* 1000ms = 1 second */
```

**Question**: What happens to CPU usage?

---

### Exercise 2: Add Multiple ADC Channels

**Goal**: Read two ADC channels simultaneously

1. Add second channel in `board/app.overlay`:
```dts
adc0_ch1: channel@1 {
    reg = <1>;
    zephyr,gain = "ADC_GAIN_1_4";
    zephyr,reference = "ADC_REF_VDD_1_4";
    zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
    zephyr,input-positive = <NRF_SAADC_AIN3>;  /* P0.05 */
    zephyr,resolution = <12>;
};
```

2. Modify code to read both channels:
```c
static int16_t buf[2];  /* Array for two channels */

static struct adc_sequence seq = {
    .channels = BIT(0) | BIT(1),  /* Read channels 0 and 1 */
    .buffer = buf,
    .buffer_size = sizeof(buf),
    .resolution = 12,
};

// In main loop:
adc_read(adc_dev, &seq);
LOG_INF("CH0: %d mV, CH1: %d mV",
        (buf[0] * VDD_MV) / 4096,
        (buf[1] * VDD_MV) / 4096);
```

---

### Exercise 3: Voltage Threshold Alert

**Goal**: Print alert when voltage exceeds 2.5V

```c
int val_mv = (buf * VDD_MV) / (1 << seq.resolution);

if (val_mv > 2500) {
    LOG_WRN("HIGH VOLTAGE: %d mV", val_mv);
} else {
    LOG_INF("Normal: %d mV", val_mv);
}
```

---

### Exercise 4: Moving Average Filter

**Goal**: Reduce noise by averaging 10 readings

```c
#define AVG_SAMPLES 10

int main(void) {
    /* ... initialization ... */
    
    int16_t samples[AVG_SAMPLES];
    int sample_idx = 0;
    
    while (1) {
        adc_read(adc_dev, &seq);
        samples[sample_idx] = buf;
        sample_idx = (sample_idx + 1) % AVG_SAMPLES;
        
        /* Calculate average */
        int32_t sum = 0;
        for (int i = 0; i < AVG_SAMPLES; i++) {
            sum += samples[i];
        }
        int16_t avg = sum / AVG_SAMPLES;
        
        int val_mv = (avg * VDD_MV) / (1 << seq.resolution);
        LOG_INF("Average voltage: %d mV", val_mv);
        
        k_msleep(100);
    }
}
```

---

### Exercise 5: Change Resolution

**Goal**: Test 8-bit vs 12-bit resolution

1. Modify `board/app.overlay`:
```dts
zephyr,resolution = <8>;  /* Change to 8-bit */
```

2. Update code:
```c
static struct adc_sequence seq = {
    /* ... */
    .resolution = DT_PROP(MY_ADC_CH, zephyr_resolution),
};

// Conversion now uses 256 instead of 4096:
int val_mv = (buf * VDD_MV) / (1 << seq.resolution);
```

**Observations**:
- 8-bit: Values 0-255, ~13mV steps
- 12-bit: Values 0-4095, ~0.8mV steps

---

### Exercise 6: Battery Monitor

**Goal**: Measure battery voltage using internal reference

1. Modify overlay for battery monitoring:
```dts
adc0_ch0: channel@0 {
    reg = <0>;
    zephyr,gain = "ADC_GAIN_1_6";
    zephyr,reference = "ADC_REF_INTERNAL";  /* 0.6V reference */
    zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
    zephyr,input-positive = <NRF_SAADC_VDD>;  /* Measure VDD */
    zephyr,resolution = <12>;
};
```

2. Update conversion formula:
```c
#define REF_MV 600  /* Internal reference: 0.6V */
#define GAIN 6      /* GAIN_1_6 */

// VDD voltage = (raw × reference × gain) / max_value
int vdd_mv = ((int64_t)buf * REF_MV * GAIN) / 4096;
LOG_INF("Battery voltage: %d mV", vdd_mv);
```

---

## Summary

You've learned:

✅ **What ADC is**: Converting analog voltages to digital values  
✅ **ADC parameters**: Resolution, gain, reference voltage, acquisition time  
✅ **Device Tree configuration**: Defining ADC channels with properties  
✅ **ADC API**: `adc_channel_setup()`, `adc_read()`, `adc_sequence`  
✅ **Voltage conversion**: Converting raw ADC values to millivolts  
✅ **Practical usage**: Reading sensors, filtering noise, multiple channels  

### Next Steps

1. **Combine with PWM** (see `06_adc_pwm/`): Control LED brightness based on ADC input
2. **Read real sensors**: Temperature (LM35), light (photoresistor), distance (IR sensor)
3. **Implement PID control**: Use ADC feedback for motor speed control
4. **Data logging**: Store ADC readings to flash memory or SD card

### Key Takeaways

- **Always check `device_is_ready()`** before using ADC
- **Match resolution** between Device Tree and `adc_sequence`
- **Understand gain/reference** to calculate correct voltage range
- **Use averaging** to reduce noise in readings
- **Cast to 64-bit** during conversion to prevent integer overflow

---

**Congratulations!** You now understand ADC peripherals in Zephyr. Try the exercises to deepen your knowledge, then explore the combined ADC+PWM example in `06_adc_pwm/`.
