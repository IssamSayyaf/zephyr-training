# Zephyr Blink Tutorial - Your First Zephyr Application

A beginner-friendly guide to understanding your first Zephyr RTOS application that blinks an LED.

## Table of Contents

1. [Overview](#overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Project Structure](#project-structure)
4. [Understanding the Code](#understanding-the-code)
5. [Device Tree Concepts](#device-tree-concepts)
6. [Configuration Files](#configuration-files)
7. [Building and Running](#building-and-running)
8. [Troubleshooting](#troubleshooting)
9. [Exercises](#exercises)
10. [Next Steps](#next-steps)

---

## Overview

This is the "Hello World" of embedded systems - a simple LED blink application. It demonstrates:

- **GPIO (General Purpose Input/Output)** control
- **Device Tree** usage for hardware abstraction
- **Zephyr kernel** timing functions
- **Console output** for debugging
- Basic **error handling**

### What You'll Learn

✅ How to access hardware using Device Tree  
✅ GPIO configuration and control  
✅ Zephyr kernel timing functions  
✅ Basic error handling patterns  
✅ Console/UART output for debugging  

---

## Hardware Requirements

- **Board**: nRF52840 DK (or any Zephyr-supported board with an LED)
- **USB Cable**: For programming and power
- **Computer**: With Zephyr development environment installed

### LED Information

Most development boards have multiple LEDs. This application uses `led0` which is typically:
- **nRF52840 DK**: LED1 (P0.13)
- **Raspberry Pi Pico**: Built-in LED (GPIO 25)
- **STM32 boards**: User LED (varies by board)

---

## Project Structure

```
01_blink/
├── CMakeLists.txt          # Build configuration
├── prj.conf                # Project configuration
├── Kconfig                 # Additional Kconfig (if needed)
├── README.rst              # Project documentation
├── src/
│   └── main.c              # Application source code
├── board/
│   └── app.overlay         # Device tree overlay (optional)
└── build/                  # Generated build files (not in git)
```

### Key Files

| File | Purpose |
|------|---------|
| `main.c` | Your application code |
| `CMakeLists.txt` | Tells CMake how to build the project |
| `prj.conf` | Enables/configures Zephyr features |
| `app.overlay` | Customizes hardware configuration |

---

## Understanding the Code

Let's break down the `main.c` file step by step.

### Step 1: Include Headers

```c
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
```

**What each header does:**

- `<stdio.h>`: Standard C library for `printf()` function
- `<zephyr/kernel.h>`: Core Zephyr kernel functions (timers, threads, etc.)
- `<zephyr/drivers/gpio.h>`: GPIO driver API for controlling pins

> 💡 **Tip**: Zephyr headers start with `zephyr/` prefix. This helps identify Zephyr-specific APIs.

### Step 2: Define Constants

```c
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
```

**Explanation:**

- `SLEEP_TIME_MS`: How long to wait between LED toggles (1 second = 1000 milliseconds)
- `DT_ALIAS(led0)`: Gets the device tree node for the LED labeled as "led0"

> 🔑 **Key Concept**: `DT_ALIAS()` is a Device Tree macro that finds hardware by its alias name. This makes code portable across different boards.

### Step 3: Get LED Specification

```c
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
```

**What this does:**

This line creates a structure containing all information needed to control the LED:
- Which GPIO controller (port) to use
- Which pin number
- Pin flags (active high/low, pull-up/down, etc.)

**Breaking it down:**

- `gpio_dt_spec`: Structure type that holds GPIO configuration
- `GPIO_DT_SPEC_GET()`: Macro that extracts GPIO info from Device Tree
- `LED0_NODE`: The device tree node we defined earlier
- `gpios`: The property name in device tree that contains GPIO information
- `static const`: This variable is constant and local to this file

> 💡 **Why Device Tree?** It separates hardware details from code. The same code works on different boards because hardware specifics are in the device tree, not hardcoded.

### Step 4: Main Function Begins

```c
int main(void)
{
	int ret;
	bool led_state = true;
```

**Variables:**

- `ret`: Stores return values from functions to check for errors
- `led_state`: Tracks whether LED is ON or OFF (for printing messages)

### Step 5: Check if GPIO is Ready

```c
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
```

**What this does:**

Verifies that the GPIO controller for the LED is initialized and ready to use.

**Error handling:**

- If GPIO is NOT ready → returns 0 (exits program)
- The `!` means "not" - so "if not ready"
- `&led` passes the address (pointer) to the led structure

> ⚠️ **Important**: Always check if devices are ready before using them!

### Step 6: Configure GPIO Pin as Output

```c
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
```

**What this does:**

Configures the LED pin as an output in its "active" state (usually HIGH for LEDs).

**Parameters:**

- `&led`: Pointer to our LED specification
- `GPIO_OUTPUT_ACTIVE`: Sets pin as output, initially in active (ON) state

**Error checking:**

- Functions return negative values on error
- `ret < 0` means an error occurred
- We exit the program if configuration fails

> 📝 **Note**: `GPIO_OUTPUT_ACTIVE` respects the device tree configuration. If the LED is active-low (common anode), Zephyr handles this automatically!

### Step 7: Main Loop - Toggle LED

```c
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
```

**The infinite loop:**

`while (1)` creates an infinite loop - the code inside runs forever (typical for embedded systems).

**Toggle the LED:**

```c
ret = gpio_pin_toggle_dt(&led);
```
- Switches LED state: ON → OFF or OFF → ON
- Returns negative on error

**Update and print state:**

```c
led_state = !led_state;
printf("LED state: %s\n", led_state ? "ON" : "OFF");
```
- `!led_state` flips the boolean value
- `printf()` sends message to console/UART
- `? :` is ternary operator: `condition ? if_true : if_false`

**Wait/Sleep:**

```c
k_msleep(SLEEP_TIME_MS);
```
- `k_msleep()` is Zephyr's millisecond sleep function
- Pauses execution for 1000ms (1 second)
- `k_` prefix indicates Zephyr kernel function
- Other threads can run during this time

> 🔑 **Key Concept**: `k_msleep()` is "cooperative" - it yields to other tasks. Don't use busy-wait loops!

---

## Device Tree Concepts

### What is Device Tree?

Device Tree is a data structure that describes hardware to the operating system. It's like a blueprint of your board's components.

### LED Definition Example

In your board's device tree (inside Zephyr), there's something like:

```dts
/ {
    aliases {
        led0 = &led0_node;
    };

    leds {
        compatible = "gpio-leds";
        led0_node: led_0 {
            gpios = <&gpio0 13 GPIO_ACTIVE_LOW>;
            label = "Green LED 0";
        };
    };
};
```

**What this means:**

- `aliases { led0 = &led0_node; }`: Creates shortcut name "led0"
- `gpios = <&gpio0 13 GPIO_ACTIVE_LOW>;`: LED is on GPIO port 0, pin 13, active LOW
- Your code uses `DT_ALIAS(led0)` to find this information

### Custom Device Tree Overlay

If you need to customize hardware, create `board/app.overlay`:

```dts
/ {
    aliases {
        myled = &led1;  // Use LED1 instead of LED0
    };
};

&led1 {
    gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;  // Change to pin 14
};
```

Then in code:

```c
#define LED0_NODE DT_ALIAS(myled)  // Use your custom alias
```

---

## Configuration Files

### prj.conf - Project Configuration

```properties
CONFIG_GPIO=y
```

**What it does:**

Enables GPIO driver support in your build.

**Kconfig system:**

- Configuration options start with `CONFIG_`
- `=y` means "yes, enable this"
- `=n` means "no, disable this"
- Some options take integer values: `CONFIG_MAIN_STACK_SIZE=2048`

### Common Configuration Options

Add these to `prj.conf` as needed:

```properties
# GPIO support (required for this app)
CONFIG_GPIO=y

# Console/UART for printf()
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y

# Logging system (more powerful than printf)
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

# Increase main thread stack if needed
CONFIG_MAIN_STACK_SIZE=2048

# Enable printk (lightweight printf)
CONFIG_PRINTK=y
```

### CMakeLists.txt - Build Configuration

```cmake
cmake_minimum_required(VERSION 3.20.0)

# Set target board
set(BOARD nrf52840dk/nrf52840)

# Find Zephyr
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name
project(ZEPHYR_APP_1)

# Add source files
target_sources(app PRIVATE src/main.c)
```

**Key parts:**

- `set(BOARD ...)`: Specifies target hardware
- `find_package(Zephyr ...)`: Links with Zephyr RTOS
- `target_sources(...)`: Lists source files to compile

---

## Building and Running

### Step 1: Setup Environment

```bash
# Navigate to Zephyr installation
cd ~/zephyrproject

# Activate environment
source zephyr/zephyr-env.sh
```

### Step 2: Navigate to Project

```bash
cd /home/issam/Zephyr_tutorial/apps/01_blink
```

### Step 3: Build

```bash
# Clean build (recommended first time)
west build -b nrf52840dk/nrf52840 -p

# Incremental build (after making changes)
west build
```

**Build options:**

- `-b nrf52840dk/nrf52840`: Target board
- `-p`: Pristine build (clean first)
- `-t menuconfig`: Interactive configuration

### Step 4: Flash to Board

```bash
# Connect board via USB, then:
west flash
```

### Step 5: Monitor Output

```bash
# View serial output
west attach

# Or use minicom
minicom -D /dev/ttyACM0 -b 115200

# Or use screen
screen /dev/ttyACM0 115200
```

**Expected output:**

```
LED state: ON
LED state: OFF
LED state: ON
LED state: OFF
...
```

---

## Troubleshooting

### Problem: Build fails with "No BOARD specified"

**Solution:**
```bash
# Specify board explicitly
west build -b nrf52840dk/nrf52840
```

Or add to `CMakeLists.txt`:
```cmake
set(BOARD nrf52840dk/nrf52840)
```

### Problem: LED doesn't blink

**Check:**

1. **Board connected?** USB cable plugged in
2. **Code flashed?** Run `west flash` after building
3. **Correct board?** Build for the right board type
4. **LED exists?** Check board documentation for LED location
5. **Device tree correct?** Verify `led0` alias exists for your board

**Debug steps:**

```bash
# Check if board is detected
lsusb | grep -i "nordic\|segger"

# View build configuration
cat build/zephyr/.config | grep GPIO

# Check device tree
cat build/zephyr/zephyr.dts | grep -A5 "led0"
```

### Problem: No console output

**Check prj.conf has:**

```properties
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_PRINTK=y
```

**Check serial connection:**

```bash
# Find device
ls /dev/ttyACM* /dev/ttyUSB*

# Check permissions
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### Problem: Build errors

**Common fixes:**

```bash
# Clean build
west build -t clean
west build

# Pristine build
west build -p

# Check environment
echo $ZEPHYR_BASE
west --version
```
