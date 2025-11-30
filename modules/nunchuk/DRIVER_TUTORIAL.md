# Complete Guide: Writing a Zephyr Driver - Nintendo Nunchuk Example

This tutorial will walk you through creating a complete Zephyr driver using the Nintendo Nunchuk I2C input device as a practical example. By the end of this tutorial, you'll understand the complete process of writing, configuring, and integrating a custom driver into the Zephyr RTOS ecosystem.

## Table of Contents

1. [Overview and Prerequisites](#1-overview-and-prerequisites)
2. [Project Structure and Module Layout](#2-project-structure-and-module-layout)
3. [Device Tree Bindings](#3-device-tree-bindings)
4. [Configuration System (Kconfig)](#4-configuration-system-kconfig)
5. [Driver Implementation](#5-driver-implementation)
6. [Public API Header](#6-public-api-header)
7. [Build System Integration](#7-build-system-integration)
8. [Module Integration](#8-module-integration)
9. [Testing and Debugging](#9-testing-and-debugging)
10. [Best Practices and Advanced Topics](#10-best-practices-and-advanced-topics)

---

## 1. Overview and Prerequisites

### What You'll Learn

- How to structure a Zephyr external module
- Writing device tree bindings for hardware description
- Creating Kconfig options for driver configuration
- Implementing the actual driver code with proper Zephyr APIs
- Integrating with Zephyr's build system
- Creating a public API for applications to use

### Prerequisites

- Basic understanding of C programming
- Familiarity with I2C communication protocol
- Basic knowledge of Zephyr RTOS concepts
- Understanding of device tree concepts
- Experience with CMake build system

### Hardware Requirements

For this tutorial, we're implementing a driver for the Nintendo Wii Nunchuk, which communicates over I2C. The concepts apply to any I2C device.

---

## 2. Project Structure and Module Layout

First, let's understand how to organize a Zephyr external module:

```
modules/nunchuk/
├── module.yaml                 # Module metadata
├── CMakeLists.txt             # Top-level build configuration
├── Kconfig                    # Driver configuration options
├── README.md                  # Documentation
├── zephyr/                    # Zephyr integration files
│   ├── module.yml            # Module integration settings
│   ├── CMakeLists.txt        # Zephyr-specific build config
│   └── Kconfig               # Additional Kconfig if needed
├── dts/                      # Device tree files
│   └── bindings/
│       └── input/
│           └── nintendo,nunchuk.yaml
├── drivers/                  # Driver implementation
│   ├── CMakeLists.txt
│   └── input/
│       ├── CMakeLists.txt
│       └── nunchuk.c
└── include/                  # Public headers
    └── drivers/
        └── nunchuk.h
```

### Step 2.1: Create the Module Metadata

Create `module.yaml` to describe your module:

```yaml
name: nunchuk-driver
version: "1.0.0"
description: "Nintendo Nunchuk I2C driver module for Zephyr RTOS"
author: "Your Name"
license: "Apache-2.0"
keywords:
  - input
  - gamepad
  - i2c
  - sensor
```

### Step 2.2: Set Up Zephyr Integration

Create `zephyr/module.yml` to tell Zephyr how to integrate your module:

```yaml
build:
  cmake: .                    # Points to top-level CMakeLists.txt
  kconfig: Kconfig           # Points to Kconfig file
  settings:
    dts_root: .              # Device tree root directory
```

---

## 3. Device Tree Bindings

Device tree bindings describe the hardware interface and configuration options for your device.

### Step 3.1: Create the Binding File

Create `dts/bindings/input/nintendo,nunchuk.yaml`:

```yaml
# SPDX-License-Identifier: Apache-2.0

description: Nintendo Wii Nunchuk I2C input device with extended features

compatible: "nintendo,nunchuk-extended"

include: i2c-device.yaml

properties:
  poll-interval-ms:
    type: int
    default: 20
    description: |
      Polling interval in milliseconds for reading sensor data.
      Typical range: 10-50ms. Lower values = more responsive but higher CPU usage.

  deadzone:
    type: int
    default: 15
    description: |
      Joystick deadzone radius (0-127). Joystick movements within this distance
      from center (127,127) are ignored to reduce noise and drift.

  nunchuk-type:
    type: string
    default: "white"
    enum:
      - "black"
      - "white"
    description: |
      Nunchuk variant type:
      - "black": Original Nunchuk (init: 0xF0,0x55 then 0xFB,0x00)
      - "white": Newer Nunchuk (init: 0x40,0x00 then 0x00)

  enable-gestures:
    type: boolean
    description: |
      Enable gesture detection using accelerometer data.
      Detects shakes and tilts. Requires FPU support.
```

### Key Concepts:

- **compatible**: Unique identifier that matches with your driver
- **include**: Inherit properties from standard bindings (i2c-device.yaml provides I2C configuration)
- **properties**: Define configurable parameters with types, defaults, and validation

### Step 3.2: Understanding Device Tree Usage

In your application's device tree overlay, you'd use it like this:

```dts
&i2c0 {
    nunchuk: nunchuk@52 {
        compatible = "nintendo,nunchuk-extended";
        reg = <0x52>;
        poll-interval-ms = <20>;
        deadzone = <15>;
        nunchuk-type = "white";
        enable-gestures;
    };
};
```

---

## 4. Configuration System (Kconfig)

Kconfig provides compile-time configuration for your driver.

### Step 4.1: Create the Main Kconfig File

Create `Kconfig`:

```kconfig
# SPDX-License-Identifier: Apache-2.0

# Main driver enable option
config NUNCHUK_DRIVER
	bool "Nintendo Nunchuk I2C driver"
	depends on I2C && INPUT
	help
	  Enable Nintendo Nunchuk I2C input driver support.

if NUNCHUK_DRIVER

config NUNCHUK_INIT_PRIORITY
	int "Nunchuk driver initialization priority"
	default 90
	range 70 99
	help
	  Driver initialization priority. Must be higher than I2C.

config NUNCHUK_THREAD_STACK_SIZE
	int "Stack size for nunchuk work handler"
	default 1536
	range 1024 4096
	help
	  Stack size for the work queue that handles nunchuk polling.

config NUNCHUK_GESTURES
	bool "Enable gesture detection"
	depends on FPU
	default y if FPU
	help
	  Enable accelerometer-based gesture detection.
	  Requires FPU for floating-point calculations.

config NUNCHUK_LOG_LEVEL
	int "Nunchuk driver log level"
	depends on LOG
	default 3
	range 0 4
	help
	  0=OFF, 1=ERROR, 2=WARNING, 3=INFO, 4=DEBUG

endif # NUNCHUK_DRIVER
```

### Key Concepts:

- **bool**: Boolean option (y/n)
- **int**: Integer option with optional range validation
- **depends on**: Prerequisites for this option
- **default**: Default value
- **help**: User documentation
- **if/endif**: Conditional grouping

---

## 5. Driver Implementation

Now for the main driver implementation. This is the heart of your driver.

### Step 5.1: Driver Structure Overview

Create `drivers/input/nunchuk.c` with these key components:

1. **Driver registration and device tree compatibility**
2. **Configuration and runtime data structures**
3. **Hardware initialization functions**
4. **Data reading and processing**
5. **API implementation**
6. **Work queue for periodic polling**

### Step 5.2: Driver Header and Defines

```c
/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Nintendo Nunchuk I2C Driver for Zephyr RTOS
 */

#define DT_DRV_COMPAT nintendo_nunchuk_extended

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <stdlib.h>
#include <drivers/nunchuk.h>

LOG_MODULE_REGISTER(nunchuk, CONFIG_NUNCHUK_LOG_LEVEL);

/* Hardware-specific constants */
#define NUNCHUK_BLACK_CMD1          0xF0
#define NUNCHUK_BLACK_VAL1          0x55
#define NUNCHUK_BLACK_CMD2          0xFB
#define NUNCHUK_BLACK_VAL2          0x00
#define NUNCHUK_WHITE_CMD1          0x40
#define NUNCHUK_WHITE_VAL1          0x00
#define NUNCHUK_WHITE_CMD2          0x00
#define NUNCHUK_READ_CMD            0x00
#define NUNCHUK_DATA_SIZE           6
```

### Step 5.3: Data Structures

Define the configuration and runtime data structures:

```c
/**
 * @brief Device configuration (from device tree)
 */
struct nunchuk_config {
	struct i2c_dt_spec i2c;        // I2C bus configuration
	uint32_t poll_interval_ms;     // Polling interval
	uint8_t deadzone;              // Joystick deadzone
	bool enable_gestures;          // Gesture detection enable
	const char *nunchuk_type;      // Hardware variant
};

/**
 * @brief Device runtime data
 */
struct nunchuk_data_private {
	struct k_work_delayable work;  // Work queue for polling
	const struct device *device;  // Back-reference to device
	struct nunchuk_data sensor_data; // Latest sensor readings
	struct nunchuk_calibration cal;  // Calibration data
	nunchuk_data_callback_t data_callback;     // Data callback
	void *data_callback_user_data;
	nunchuk_gesture_callback_t gesture_callback; // Gesture callback
	void *gesture_callback_user_data;
	bool initialized;
	struct k_mutex data_mutex;     // Thread safety
#ifdef CONFIG_NUNCHUK_GESTURES
	struct nunchuk_data gesture_history[NUNCHUK_GESTURE_HISTORY];
	uint8_t gesture_index;
#endif
};
```

### Step 5.4: Hardware Initialization

Implement device-specific initialization:

```c
/**
 * @brief Initialize the nunchuk hardware
 */
static int nunchuk_device_init(const struct device *dev)
{
	const struct nunchuk_config *cfg = dev->config;
	bool is_white = (strcmp(cfg->nunchuk_type, "white") == 0);
	uint8_t cmd_data[2];
	int ret;

	LOG_INF("Initializing %s nunchuk", is_white ? "white" : "black");

	if (is_white) {
		/* White nunchuk initialization sequence */
		cmd_data[0] = NUNCHUK_WHITE_CMD1;
		cmd_data[1] = NUNCHUK_WHITE_VAL1;
		ret = i2c_write_dt(&cfg->i2c, cmd_data, 2);
		if (ret < 0) {
			LOG_ERR("White nunchuk init cmd1 failed: %d", ret);
			return ret;
		}
		
		k_msleep(NUNCHUK_INIT_DELAY_MS);
		
		cmd_data[0] = NUNCHUK_WHITE_CMD2;
		cmd_data[1] = NUNCHUK_WHITE_CMD2;
		ret = i2c_write_dt(&cfg->i2c, cmd_data, 2);
		if (ret < 0) {
			LOG_ERR("White nunchuk init cmd2 failed: %d", ret);
			return ret;
		}
	} else {
		/* Black nunchuk initialization sequence */
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
	return 0;
}
```

### Step 5.5: Data Reading and Processing

```c
/**
 * @brief Read and decode sensor data
 */
static int nunchuk_read_data(const struct device *dev, struct nunchuk_data *data)
{
	const struct nunchuk_config *cfg = dev->config;
	uint8_t raw_data[NUNCHUK_DATA_SIZE];
	uint8_t read_cmd = NUNCHUK_READ_CMD;
	int ret;

	/* Send read command */
	ret = i2c_write_dt(&cfg->i2c, &read_cmd, 1);
	if (ret < 0) {
		return ret;
	}

	k_usleep(NUNCHUK_READ_DELAY_US);

	/* Read raw data */
	ret = i2c_read_dt(&cfg->i2c, raw_data, NUNCHUK_DATA_SIZE);
	if (ret < 0) {
		return ret;
	}

	/* Decode data (nunchuk uses XOR encoding) */
	for (int i = 0; i < NUNCHUK_DATA_SIZE; i++) {
		raw_data[i] = (raw_data[i] ^ 0x17) + 0x17;
	}

	/* Parse the data */
	data->joystick_x = raw_data[0];
	data->joystick_y = raw_data[1];
	data->accel_x = ((uint16_t)raw_data[2] << 2) | ((raw_data[5] >> 2) & 0x03);
	data->accel_y = ((uint16_t)raw_data[3] << 2) | ((raw_data[5] >> 4) & 0x03);
	data->accel_z = ((uint16_t)raw_data[4] << 2) | ((raw_data[5] >> 6) & 0x03);
	data->button_c = !(raw_data[5] & 0x02);
	data->button_z = !(raw_data[5] & 0x01);
	data->timestamp = k_uptime_get_32();

	return 0;
}
```

### Step 5.6: Work Queue Handler

```c
/**
 * @brief Work queue handler for periodic polling
 */
static void nunchuk_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct nunchuk_data_private *priv = CONTAINER_OF(dwork, 
		struct nunchuk_data_private, work);
	const struct nunchuk_config *cfg = priv->device->config;
	struct nunchuk_data data;
	int ret;

	ret = nunchuk_read_data(priv->device, &data);
	if (ret == 0) {
		k_mutex_lock(&priv->data_mutex, K_FOREVER);
		memcpy(&priv->sensor_data, &data, sizeof(data));
		k_mutex_unlock(&priv->data_mutex);

		/* Call data callback if registered */
		if (priv->data_callback) {
			priv->data_callback(&data, priv->data_callback_user_data);
		}

#ifdef CONFIG_NUNCHUK_GESTURES
		/* Process gestures if enabled */
		if (cfg->enable_gestures) {
			nunchuk_process_gestures(priv->device, &data);
		}
#endif
	} else {
		LOG_WRN("Failed to read nunchuk data: %d", ret);
	}

	/* Schedule next reading */
	k_work_reschedule(&priv->work, K_MSEC(cfg->poll_interval_ms));
}
```

### Step 5.7: Driver Initialization Function

```c
/**
 * @brief Driver initialization function
 */
static int nunchuk_init(const struct device *dev)
{
	const struct nunchuk_config *cfg = dev->config;
	struct nunchuk_data_private *priv = dev->data;
	int ret;

	LOG_INF("Initializing nunchuk driver");

	/* Verify I2C bus is ready */
	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Initialize runtime data */
	priv->device = dev;
	priv->initialized = false;
	k_mutex_init(&priv->data_mutex);

	/* Initialize hardware */
	ret = nunchuk_device_init(dev);
	if (ret < 0) {
		LOG_ERR("Hardware initialization failed: %d", ret);
		return ret;
	}

	/* Initialize work queue */
	k_work_init_delayable(&priv->work, nunchuk_work_handler);

	priv->initialized = true;
	LOG_INF("Nunchuk driver initialized successfully");

	return 0;
}
```

### Step 5.8: Device Registration Macros

```c
/* Extract configuration from device tree */
#define NUNCHUK_CONFIG_INIT(node_id)					\
	{								\
		.i2c = I2C_DT_SPEC_GET(node_id),			\
		.poll_interval_ms = DT_PROP(node_id, poll_interval_ms),	\
		.deadzone = DT_PROP(node_id, deadzone),			\
		.enable_gestures = DT_PROP(node_id, enable_gestures),	\
		.nunchuk_type = DT_PROP(node_id, nunchuk_type),		\
	}

/* Device instance creation macro */
#define NUNCHUK_DEVICE_INIT(node_id)					\
	static const struct nunchuk_config nunchuk_config_##node_id =	\
		NUNCHUK_CONFIG_INIT(node_id);				\
	static struct nunchuk_data_private nunchuk_data_##node_id;	\
	DEVICE_DT_DEFINE(node_id, nunchuk_init, NULL,			\
		&nunchuk_data_##node_id, &nunchuk_config_##node_id,	\
		POST_KERNEL, CONFIG_NUNCHUK_INIT_PRIORITY, NULL);

/* Create device instances for all compatible devices in device tree */
DT_FOREACH_STATUS_OKAY(nintendo_nunchuk_extended, NUNCHUK_DEVICE_INIT)
```

---

## 6. Public API Header

Create `include/drivers/nunchuk.h` to define the public API:

```c
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
 */
struct nunchuk_data {
	uint8_t joystick_x;    /**< Joystick X position (0-255, center=127) */
	uint8_t joystick_y;    /**< Joystick Y position (0-255, center=127) */
	uint16_t accel_x;      /**< Accelerometer X reading (0-1023, ~512=1G) */
	uint16_t accel_y;      /**< Accelerometer Y reading (0-1023, ~512=1G) */
	uint16_t accel_z;      /**< Accelerometer Z reading (0-1023, ~512=1G) */
	bool button_c;         /**< C button state (true=pressed) */
	bool button_z;         /**< Z button state (true=pressed) */
	uint32_t timestamp;    /**< Timestamp of reading in milliseconds */
};

/**
 * @brief Nunchuk gesture types
 */
enum nunchuk_gesture {
	NUNCHUK_GESTURE_NONE,
	NUNCHUK_GESTURE_SHAKE_X,
	NUNCHUK_GESTURE_SHAKE_Y,
	NUNCHUK_GESTURE_SHAKE_Z,
	NUNCHUK_GESTURE_TILT_LEFT,
	NUNCHUK_GESTURE_TILT_RIGHT,
	NUNCHUK_GESTURE_TILT_FORWARD,
	NUNCHUK_GESTURE_TILT_BACKWARD,
};

/**
 * @brief Callback function types
 */
typedef void (*nunchuk_data_callback_t)(const struct nunchuk_data *data, void *user_data);
typedef void (*nunchuk_gesture_callback_t)(enum nunchuk_gesture gesture, void *user_data);

/**
 * @brief Start nunchuk polling
 * @param dev Nunchuk device
 * @return 0 on success, negative error code on failure
 */
int nunchuk_start_polling(const struct device *dev);

/**
 * @brief Stop nunchuk polling
 * @param dev Nunchuk device
 * @return 0 on success, negative error code on failure
 */
int nunchuk_stop_polling(const struct device *dev);

/**
 * @brief Get latest sensor data
 * @param dev Nunchuk device
 * @param data Pointer to data structure to fill
 * @return 0 on success, negative error code on failure
 */
int nunchuk_get_data(const struct device *dev, struct nunchuk_data *data);

/**
 * @brief Set data callback
 * @param dev Nunchuk device
 * @param callback Callback function
 * @param user_data User data pointer
 * @return 0 on success, negative error code on failure
 */
int nunchuk_set_data_callback(const struct device *dev, 
			      nunchuk_data_callback_t callback, 
			      void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_NUNCHUK_H_ */
```

---

## 7. Build System Integration

### Step 7.1: Top-Level CMakeLists.txt

Create `CMakeLists.txt`:

```cmake
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Add include directory for the module
zephyr_include_directories(include)

# Add driver if enabled
if(CONFIG_NUNCHUK_DRIVER)
  add_subdirectory(drivers)
endif()
```

### Step 7.2: Driver CMakeLists.txt

Create `drivers/CMakeLists.txt`:

```cmake
# SPDX-License-Identifier: Apache-2.0

# Add input drivers if enabled
if(CONFIG_INPUT)
  add_subdirectory(input)
endif()
```

Create `drivers/input/CMakeLists.txt`:

```cmake
# SPDX-License-Identifier: Apache-2.0

# Only compile the driver if enabled
zephyr_sources_ifdef(CONFIG_NUNCHUK_DRIVER nunchuk.c)
```

---

## 8. Module Integration

### Step 8.1: Module Registration

Your module needs to be discoverable by Zephyr. This is done through the `zephyr/module.yml` file we created earlier.

### Step 8.2: West Module (Optional)

If you want to distribute your module via west, create a `west.yml`:

```yaml
manifest:
  remotes:
    - name: upstream
      url-base: https://github.com/zephyrproject-rtos
  projects:
    - name: zephyr
      remote: upstream
      revision: main
      import: true
  self:
    path: modules/nunchuk
```

### Step 8.3: Application Integration

In your application's `west.yml` or `CMakeLists.txt`, add:

```cmake
# In your application CMakeLists.txt
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../modules/nunchuk)
```

---

## 9. Testing and Debugging

### Step 9.1: Create a Test Application

Create a simple test application:

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/nunchuk.h>

LOG_MODULE_REGISTER(main);

static const struct device *nunchuk_dev;

void data_callback(const struct nunchuk_data *data, void *user_data)
{
	LOG_INF("Joystick: (%d, %d), Accel: (%d, %d, %d), Buttons: C=%d Z=%d",
		data->joystick_x, data->joystick_y,
		data->accel_x, data->accel_y, data->accel_z,
		data->button_c, data->button_z);
}

int main(void)
{
	LOG_INF("Nunchuk Test Application Starting");

	nunchuk_dev = DEVICE_DT_GET_ANY(nintendo_nunchuk_extended);
	if (!nunchuk_dev) {
		LOG_ERR("No nunchuk device found");
		return -1;
	}

	if (!device_is_ready(nunchuk_dev)) {
		LOG_ERR("Nunchuk device not ready");
		return -1;
	}

	LOG_INF("Nunchuk device found and ready");

	/* Set callback and start polling */
	nunchuk_set_data_callback(nunchuk_dev, data_callback, NULL);
	nunchuk_start_polling(nunchuk_dev);

	/* Application main loop */
	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
```

### Step 9.2: Debugging Tips

1. **Enable logging**: Set `CONFIG_LOG=y` and `CONFIG_NUNCHUK_LOG_LEVEL=4`
2. **Use printk for early debugging**: Before logging is available
3. **Check I2C communication**: Use a logic analyzer or oscilloscope
4. **Verify device tree**: Check that your device is properly instantiated
5. **Memory debugging**: Enable stack canaries and memory protection

### Step 9.3: Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Device not found | Device tree mismatch | Verify compatible string matches |
| I2C communication fails | Wrong I2C address/bus | Check hardware documentation |
| Build errors | Missing dependencies | Verify Kconfig dependencies |
| Initialization fails | Wrong init sequence | Check hardware initialization |
| Data corruption | Timing issues | Adjust delays and polling intervals |

---

## 10. Best Practices and Advanced Topics

### Step 10.1: Error Handling

Always implement proper error handling:

```c
static int nunchuk_api_call(const struct device *dev, void *param)
{
	if (!dev) {
		return -EINVAL;
	}

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	struct nunchuk_data_private *priv = dev->data;
	if (!priv->initialized) {
		return -EAGAIN;
	}

	/* Actual implementation */
	return 0;
}
```

### Step 10.2: Thread Safety

Use mutexes to protect shared data:

```c
int nunchuk_get_data(const struct device *dev, struct nunchuk_data *data)
{
	struct nunchuk_data_private *priv = dev->data;
	
	k_mutex_lock(&priv->data_mutex, K_FOREVER);
	memcpy(data, &priv->sensor_data, sizeof(*data));
	k_mutex_unlock(&priv->data_mutex);
	
	return 0;
}
```

### Step 10.3: Power Management

Implement power management hooks:

```c
#ifdef CONFIG_PM_DEVICE
static int nunchuk_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		return nunchuk_stop_polling(dev);
	case PM_DEVICE_ACTION_RESUME:
		return nunchuk_start_polling(dev);
	default:
		return -ENOTSUP;
	}
}
#endif
```

### Step 10.4: Advanced Features

1. **Calibration**: Store and apply calibration data
2. **Gesture Recognition**: Use accelerometer data for gesture detection
3. **Multiple Instance Support**: Handle multiple devices of the same type
4. **Runtime Configuration**: Allow changing parameters at runtime
5. **Interrupt Support**: Use interrupts instead of polling where possible

### Step 10.5: Documentation

Always provide comprehensive documentation:

```c
/**
 * @brief Start nunchuk polling
 * 
 * This function starts the periodic polling of the nunchuk device.
 * Data will be read at the interval specified in the device tree
 * configuration. If a data callback is registered, it will be called
 * for each successful read.
 * 
 * @param dev Pointer to nunchuk device
 * @retval 0 Success
 * @retval -EINVAL Invalid device pointer
 * @retval -ENODEV Device not ready
 * @retval -EAGAIN Device not initialized
 * @retval -EALREADY Polling already started
 */
int nunchuk_start_polling(const struct device *dev);
```

### Step 10.6: Testing Strategy

1. **Unit Tests**: Test individual functions
2. **Integration Tests**: Test with real hardware
3. **Stress Tests**: Test under load and error conditions
4. **Compliance Tests**: Verify against specifications
5. **Performance Tests**: Measure timing and resource usage

---

## Conclusion

Writing a Zephyr driver involves several interconnected components:

1. **Device Tree Bindings** - Hardware description and configuration
2. **Kconfig** - Compile-time options and dependencies
3. **Driver Implementation** - The actual C code that interfaces with hardware
4. **Public API** - Clean interface for applications
5. **Build System** - CMake integration
6. **Module Structure** - Proper organization and integration

The key to success is understanding each component's role and how they work together. Start with simple functionality and gradually add advanced features like gesture recognition, power management, and multiple device support.

Remember to:
- Follow Zephyr coding standards
- Implement proper error handling
- Use appropriate synchronization primitives
- Provide comprehensive documentation
- Test thoroughly with real hardware

This tutorial provides a complete framework that you can adapt for any I2C device. The patterns and practices shown here apply broadly to Zephyr driver development.

For more information, refer to the [Zephyr Driver Development Documentation](https://docs.zephyrproject.org/latest/kernel/drivers/index.html).