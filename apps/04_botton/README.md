# GPIO Input and Device Tree Tutorial

A comprehensive guide to using GPIO inputs (buttons), understanding Device Tree, and creating custom hardware configurations in Zephyr.

## 📋 Table of Contents

1. [What's New in This Example](#whats-new-in-this-example)
2. [Understanding GPIO Input](#understanding-gpio-input)
3. [What is Device Tree?](#what-is-device-tree)
4. [Device Tree Overlays](#device-tree-overlays)
5. [Step-by-Step: Adding a Button](#step-by-step-adding-a-button)
6. [Understanding the Code](#understanding-the-code)
7. [Device Tree Deep Dive](#device-tree-deep-dive)
8. [GPIO Configuration Options](#gpio-configuration-options)
9. [Debouncing Techniques](#debouncing-techniques)
10. [Best Practices](#best-practices)
11. [Troubleshooting](#troubleshooting)
12. [Exercises](#exercises)

---

## What's New in This Example

### Comparison with Previous Examples

| Feature | 01_blink | 04_button |
|---------|----------|-----------|
| **GPIO Direction** | Output only | Output + Input |
| **Hardware** | LED | LED + Button |
| **Device Tree** | Uses defaults | **Custom overlay** |
| **Pin Config** | OUTPUT | **INPUT with pull-up** |
| **Interaction** | Autonomous | **User-triggered** |
| **Edge Detection** | N/A | **Manual polling** |

### Key Learning Points

✅ **GPIO Input** - Reading button states  
✅ **Device Tree Overlays** - Customizing hardware configuration  
✅ **Aliases** - Creating convenient names for hardware  
✅ **GPIO Flags** - Pull-up/down, active high/low  
✅ **Debouncing** - Handling mechanical button bouncing  
✅ **Pin Configuration** - Input vs Output modes  

---

## Understanding GPIO Input

### GPIO Output vs Input

```
GPIO OUTPUT (LED)              GPIO INPUT (Button)
    ↓                              ↑
CPU → Pin → LED                Button → Pin → CPU
    ↓                              ↑
Set HIGH/LOW                   Read HIGH/LOW
```

### How Buttons Work

**Physical Button:**
```
      Not Pressed              Pressed
        (Open)                 (Closed)
    
    VCC (3.3V)              VCC (3.3V)
        |                       |
       [R]                     [R]
        |                       |
        ├──── Pin              ├──── Pin = GND
        |                       |
       [ ]  Switch             [×]  Switch closed
        |                       |
       GND                     GND
```

**With Pull-up Resistor:**
- Not pressed: Pin reads HIGH (1)
- Pressed: Pin reads LOW (0)
- `GPIO_ACTIVE_LOW`: Logical "pressed" = physical LOW

**With Pull-down Resistor:**
- Not pressed: Pin reads LOW (0)
- Pressed: Pin reads HIGH (1)
- `GPIO_ACTIVE_HIGH`: Logical "pressed" = physical HIGH

---

## What is Device Tree?

Device Tree is a **data structure** that describes hardware to the operating system.

### Why Device Tree?

```
❌ Without Device Tree:
   Code hardcoded for specific board
   Change board → rewrite code
   No portability

✅ With Device Tree:
   Code uses generic aliases
   Change board → update device tree
   Highly portable
```

### Device Tree Benefits

1. **🔄 Portability**: Same code works on different boards
2. **🔧 Flexibility**: Easy hardware reconfiguration
3. **📦 Abstraction**: Hardware details separated from code
4. **🎯 Maintainability**: Clear hardware documentation
5. **⚡ Compile-time**: No runtime overhead

### Device Tree Hierarchy

```
Root (/)
├── chosen             # Boot configuration
├── aliases            # Convenient shortcuts
│   ├── led0           # Points to actual LED node
│   └── button0        # Points to actual button node
├── soc                # System on Chip
│   ├── gpio@50000000  # GPIO port 0
│   └── uart@40002000  # UART controller
└── buttons            # Button definitions
    └── button0        # Individual button
```

---

## Device Tree Overlays

### What is an Overlay?

An **overlay** is a Device Tree fragment that **modifies** or **extends** the base board definition.

```
Base DTS (board)        Overlay (app)         Final DTS
     +                       +                    =
 [Default LED]          [Custom Button]      [LED + Button]
```

### Why Use Overlays?

| Without Overlay | With Overlay |
|----------------|--------------|
| Modify board files | Keep board files unchanged |
| Risk breaking other apps | App-specific changes |
| Hard to maintain | Easy to update |
| Not portable | Portable across boards |

### Overlay File Structure

**File: `board/app.overlay`**

```dts
/ {
    // Root node - top level
    
    buttons {
        // Group related nodes
        compatible = "gpio-keys";
        
        my_button: button_0 {
            // Individual button definition
            gpios = <&gpio0 6 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
            label = "My Button";
        };
    };
};

/ {
    // Separate root section for aliases
    aliases {
        mybutton = &my_button;
    };
};
```

---

## Step-by-Step: Adding a Button

### Step 1: Create Overlay File

```bash
# Navigate to your application
cd apps/04_botton

# Create board directory
mkdir -p board

# Create overlay file
touch board/app.overlay
```

### Step 2: Define Button in Overlay

**File: `board/app.overlay`**

```dts
/ {
    buttons {
        compatible = "gpio-keys";
        
        // Define button on P0.06
        p006_in: p006_in {
            gpios = <&gpio0 6 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
            label = "P0.06_BTN";
        };
    };
};

/ {
    aliases {
        button6 = &p006_in;
    };
};
```

**Understanding the syntax:**

```dts
p006_in: p006_in {              // label: node_name {
    gpios = <&gpio0 6 FLAGS>;   //   property = <controller pin flags>;
    label = "P0.06_BTN";        //   property = "string";
};                              // };
```

**Breaking down the `gpios` property:**
```dts
gpios = <&gpio0 6 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
         │      │  │
         │      │  └─ Flags (pull-up + active-low)
         │      └──── Pin number (6)
         └─────────── GPIO controller reference
```

### Step 3: Configure CMakeLists.txt

**File: `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20.0)

set(BOARD nrf52840dk/nrf52840)

# Tell CMake to use the overlay file
set(DTC_OVERLAY_FILE "board/app.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(button_example)

target_sources(app PRIVATE src/main.c)
```

**Key line:**
```cmake
set(DTC_OVERLAY_FILE "board/app.overlay")
```
This tells the build system to merge your overlay with the base board DTS.

### Step 4: Configure prj.conf

**File: `prj.conf`**

```properties
# GPIO support (required for buttons and LEDs)
CONFIG_GPIO=y

# Console/UART for printf()
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_PRINTK=y

# Optional: Logging
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

# Stack size
CONFIG_MAIN_STACK_SIZE=2048
```

### Step 5: Write Application Code

**File: `src/main.c`**

```c
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define SLEEP_TIME_MS   1000

// Get nodes from device tree using aliases
#define LED0_NODE DT_ALIAS(led0)
#define BUTTON0_NODE DT_ALIAS(button6)

// Create GPIO specifications from device tree
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);

int main(void)
{
    int ret;
    bool led_state = true;

    // Check if LED GPIO is ready
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    // Configure LED as output
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    // Check if button GPIO is ready
    if (!gpio_is_ready_dt(&button)) {
        return 0;
    }

    // Configure button as input
    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret < 0) {
        return 0;
    }

    while (1) {
        // Read button state (0 = pressed because GPIO_ACTIVE_LOW)
        if (gpio_pin_get_dt(&button) == 0) {
            // Debounce delay
            k_msleep(10);
            
            // Check again to confirm
            if (gpio_pin_get_dt(&button) == 0) {
                // Toggle LED state
                led_state = !led_state;
                ret = gpio_pin_set_dt(&led, (int)led_state);
                if (ret < 0) {
                    return 0;
                }
                
                printf("LED state: %s\n", led_state ? "ON" : "OFF");
                
                // Wait for button release
                while (gpio_pin_get_dt(&button) == 0) {
                    k_msleep(10);
                }
            }
        }
    }
    return 0;
}
```

---

## Understanding the Code

### Device Tree Macros

```c
// Get device tree node by alias
#define BUTTON0_NODE DT_ALIAS(button6)
```

**What happens:**
1. Looks for alias `button6` in device tree
2. Finds: `button6 = &p006_in;`
3. Resolves to the actual button node
4. Returns node identifier

### GPIO Specification Structure

```c
struct gpio_dt_spec {
    const struct device *port;    // GPIO controller device
    gpio_pin_t pin;               // Pin number
    gpio_dt_flags_t dt_flags;     // Device tree flags
};
```

**Creating from device tree:**
```c
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
//                                           │              │         │
//                                           │              │         └─ Property name
//                                           │              └─────────── Node
//                                           └──────────────────────────── Macro
```

### GPIO Input Functions

```c
// Configure pin as input
gpio_pin_configure_dt(&button, GPIO_INPUT);

// Read pin state
int value = gpio_pin_get_dt(&button);
// Returns: 0 or 1 (accounting for ACTIVE_LOW/HIGH)

// Alternative: Read raw value
int raw = gpio_pin_get_raw(&button.port, button.pin);
// Returns: actual voltage level (ignores ACTIVE flags)
```

### Button State Logic

```c
if (gpio_pin_get_dt(&button) == 0) {    // Button pressed
    // With GPIO_ACTIVE_LOW:
    // - Physical pin is LOW (GND)
    // - gpio_pin_get_dt returns 0 (logical pressed)
    
    // With GPIO_ACTIVE_HIGH:
    // - Physical pin is HIGH (VCC)
    // - gpio_pin_get_dt returns 1 (logical pressed)
}
```

---

## Device Tree Deep Dive

### DTS Syntax Reference

```dts
// Comments use // or /* */

/ {                              // Root node
    node_label: node_name {      // Label and name
        property = "value";      // String property
        number = <42>;           // Number property
        list = <1 2 3>;          // Array property
        ref = <&other_node>;     // Reference (phandle)
        bytes = [01 02 03];      // Binary data
        flags = <(FLAG1 | FLAG2)>; // Bitwise OR
    };
};
```

### Common Properties

```dts
compatible = "vendor,device";    // Device type identifier
reg = <0x40000000 0x1000>;      // Address and size
gpios = <&gpio0 6 FLAGS>;       // GPIO reference
interrupts = <10 2>;            // IRQ number and priority
status = "okay";                // Enable/disable node
label = "Description";          // Human-readable name
```

### GPIO Controller Reference

```dts
&gpio0 {                        // Reference to gpio0 controller
    status = "okay";            // Ensure it's enabled
};

buttons {
    my_button {
        gpios = <&gpio0 6 ...>; // Use pin 6 from gpio0
    };
};
```

### Aliases

```dts
aliases {
    led0 = &led0_node;          // Create shortcut
    button0 = &button0_node;    // Easy to reference in code
    mybutton = &custom_button;  // Custom names allowed
};
```

**In code:**
```c
#define MY_NODE DT_ALIAS(mybutton)   // Use the alias
```

### Labels and References

```dts
my_label: my_node {             // Define label "my_label"
    property = "value";
};

other_node {
    ref = <&my_label>;          // Reference using label
};
```

---

## GPIO Configuration Options

### GPIO Flags

```dts
// Pull resistors
GPIO_PULL_UP                    // Enable internal pull-up
GPIO_PULL_DOWN                  // Enable internal pull-down

// Active state
GPIO_ACTIVE_HIGH                // Active = HIGH (1)
GPIO_ACTIVE_LOW                 // Active = LOW (0)

// Combining flags
(GPIO_PULL_UP | GPIO_ACTIVE_LOW)   // Pull-up + active-low
```

### Common Configurations

**Button with pull-up (active-low):**
```dts
gpios = <&gpio0 6 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
```
- Not pressed: Pin = HIGH (pulled up), reads as 0 (inactive)
- Pressed: Pin = LOW (grounded), reads as 1 (active)
- ✅ Most common for buttons

**Button with pull-down (active-high):**
```dts
gpios = <&gpio0 6 (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)>;
```
- Not pressed: Pin = LOW (pulled down), reads as 0 (inactive)
- Pressed: Pin = HIGH (connected to VCC), reads as 1 (active)

**External pull-up (active-low):**
```dts
gpios = <&gpio0 6 GPIO_ACTIVE_LOW>;
```
- Hardware has external pull-up resistor
- No internal pull-up needed

### Pin Configuration in Code

```c
// Input modes
GPIO_INPUT                      // Basic input
GPIO_INPUT | GPIO_PULL_UP       // Input with pull-up
GPIO_INPUT | GPIO_PULL_DOWN     // Input with pull-down

// Configure button
gpio_pin_configure_dt(&button, GPIO_INPUT);

// The device tree flags are automatically applied!
// If DT has GPIO_PULL_UP, it's enabled automatically
```

---

## Debouncing Techniques

### What is Bouncing?

When you press a mechanical button, the contacts "bounce" before settling:

```
Physical signal:
Press ──┐ ┌─┐ ┌───┐ ┌─────────
        └─┘ └─┘   └─┘
        │   Bounce  │
        └──────────→│ Stable
           ~50ms
```

### Software Debouncing

**Method 1: Simple Delay**
```c
if (gpio_pin_get_dt(&button) == 0) {
    k_msleep(50);                        // Wait for bounce to settle
    if (gpio_pin_get_dt(&button) == 0) { // Check again
        // Button is truly pressed
    }
}
```

**Method 2: State Confirmation**
```c
// Check button is pressed
if (gpio_pin_get_dt(&button) == 0) {
    k_msleep(10);                        // Short delay
    
    if (gpio_pin_get_dt(&button) == 0) { // Still pressed?
        // Process button press
        
        // Wait for release
        while (gpio_pin_get_dt(&button) == 0) {
            k_msleep(10);
        }
        k_msleep(50);                    // Debounce release too
    }
}
```

**Method 3: Counter-based**
```c
#define DEBOUNCE_COUNT 5
static int press_count = 0;

if (gpio_pin_get_dt(&button) == 0) {
    press_count++;
    if (press_count >= DEBOUNCE_COUNT) {
        // Button confirmed pressed
        press_count = 0;
        // Process action
    }
} else {
    press_count = 0;  // Reset if button released
}
k_msleep(1);  // Check every 1ms
```

**Method 4: Interrupt with Timer (Advanced)**
```c
static struct k_work_delayable button_work;

void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Schedule work after debounce delay
    k_work_schedule(&button_work, K_MSEC(50));
}

void button_work_handler(struct k_work *work)
{
    // Check button state after debounce
    if (gpio_pin_get_dt(&button) == 0) {
        // Button truly pressed
    }
}
```

---

## Best Practices

### 1. Always Check Device Ready

```c
✅ Good:
if (!gpio_is_ready_dt(&button)) {
    printk("Button GPIO not ready\n");
    return -ENODEV;
}

❌ Bad:
// Assume GPIO is ready
gpio_pin_configure_dt(&button, GPIO_INPUT);  // May crash!
```

### 2. Check Return Values

```c
✅ Good:
ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
if (ret < 0) {
    printk("Failed to configure button: %d\n", ret);
    return ret;
}

❌ Bad:
gpio_pin_configure_dt(&button, GPIO_INPUT);
// Ignore errors - may fail silently
```

### 3. Use Device Tree Aliases

```c
✅ Good:
#define BUTTON0_NODE DT_ALIAS(button0)
// Portable across boards

❌ Bad:
#define BUTTON_PIN 6
#define BUTTON_PORT "GPIO_0"
// Hardcoded - breaks on other boards
```

### 4. Implement Debouncing

```c
✅ Good:
if (button_pressed) {
    k_msleep(50);         // Debounce
    if (button_pressed) {
        // Process
    }
}

❌ Bad:
if (button_pressed) {
    // Process immediately - will trigger multiple times
}
```

### 5. Document Your Overlay

```dts
✅ Good:
/ {
    buttons {
        compatible = "gpio-keys";
        
        // User button connected to P0.06
        // Active low with internal pull-up
        user_button: button_0 {
            gpios = <&gpio0 6 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
            label = "User Button 1";
        };
    };
};

❌ Bad:
/ {
    buttons {
        b {
            gpios = <&gpio0 6 3>;  // What does 3 mean?
        };
    };
};
```

### 6. Organize Device Tree Files

```
✅ Good structure:
board/
├── app.overlay           # Main overlay
├── buttons.dtsi          # Button definitions
└── sensors.dtsi          # Sensor definitions

❌ Bad structure:
everything_in_one_overlay.dts
```

---

## Troubleshooting

### Button Not Responding

**Problem:** Button presses not detected

**Solutions:**

1. **Check wiring:**
```
Button → P0.06
Ground → GND
```

2. **Verify device tree:**
```bash
# Check compiled device tree
cat build/zephyr/zephyr.dts | grep -A10 button
```

3. **Check pin number:**
```dts
// Make sure pin number matches hardware
gpios = <&gpio0 6 ...>;  // P0.06
```

4. **Test pin reading:**
```c
printk("Button state: %d\n", gpio_pin_get_dt(&button));
// Should print 1 when not pressed (with pull-up)
```

### Wrong Button State

**Problem:** Button state inverted (pressed reads as not pressed)

**Solutions:**

1. **Check GPIO_ACTIVE flags:**
```dts
// Pull-up with active-low
gpios = <&gpio0 6 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
```

2. **Match hardware configuration:**
```
Pull-up + connects to GND → GPIO_ACTIVE_LOW
Pull-down + connects to VCC → GPIO_ACTIVE_HIGH
```

### Overlay Not Applied

**Error:** Device tree override not working

**Solutions:**

1. **Check CMakeLists.txt:**
```cmake
set(DTC_OVERLAY_FILE "board/app.overlay")  # Before find_package!
```

2. **Verify file path:**
```bash
ls board/app.overlay  # Must exist
```

3. **Check overlay syntax:**
```bash
# Build with verbose to see DTS processing
west build -- -DDTC_WARN_UNIT_ADDRESS_VS_REG=y
```

4. **Clean build:**
```bash
west build -p  # Pristine build
```

### Alias Not Found

**Error:** `DT_ALIAS(button6)` fails to compile

**Solutions:**

1. **Check alias defined:**
```dts
/ {
    aliases {
        button6 = &p006_in;  // Must match!
    };
};
```

2. **Check label exists:**
```dts
p006_in: p006_in {  // Label must be defined
    // ...
};
```

3. **Verify in compiled DTS:**
```bash
cat build/zephyr/zephyr.dts | grep "button6"
```

### Build Errors

**Error:** Device tree compilation fails

**Common fixes:**

```dts
// Missing semicolon
property = <value>  ❌
property = <value>; ✅

// Unclosed brace
node {
    property;
// Missing }

// Wrong reference syntax
ref = &node_label;  ✅
ref = <node_label>; ❌
```

---

## Exercises

### Exercise 1: Add Second Button

Add another button on P0.07:

```dts
/ {
    buttons {
        compatible = "gpio-keys";
        
        button1: p007_in {
            gpios = <&gpio0 7 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
            label = "Button 2";
        };
    };
    
    aliases {
        button7 = &button1;
    };
};
```

Use it to toggle a second LED.

### Exercise 2: Button Press Counter

Count how many times the button is pressed:

```c
static uint32_t press_count = 0;

// In main loop
if (button_pressed) {
    press_count++;
    printk("Button pressed %u times\n", press_count);
}
```

### Exercise 3: Long Press Detection

Detect short vs long press:

```c
#define LONG_PRESS_MS 2000

if (gpio_pin_get_dt(&button) == 0) {
    uint32_t start = k_uptime_get_32();
    
    while (gpio_pin_get_dt(&button) == 0) {
        if ((k_uptime_get_32() - start) > LONG_PRESS_MS) {
            printk("Long press detected!\n");
            break;
        }
        k_msleep(10);
    }
}
```

### Exercise 4: Multiple Buttons with Different Actions

Button 1: Toggle LED
Button 2: Increase blink speed
Button 3: Decrease blink speed
Button 4: Reset to default

### Exercise 5: Add External LED

Define a custom LED in overlay:

```dts
/ {
    leds {
        compatible = "gpio-leds";
        
        my_led: led_custom {
            gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
            label = "My Custom LED";
        };
    };
    
    aliases {
        myled = &my_led;
    };
};
```

---

## Key Takeaways

### Device Tree Essentials

```dts
✅ Overlays extend base board definition
✅ Aliases provide convenient names
✅ Labels allow references
✅ Flags configure pin behavior
✅ Compatible strings identify device types
```

### GPIO Input Pattern

```c
1. Define device tree node
2. Create alias
3. Get spec from DT: GPIO_DT_SPEC_GET()
4. Check ready: gpio_is_ready_dt()
5. Configure: gpio_pin_configure_dt()
6. Read: gpio_pin_get_dt()
```

### Button Best Practices

```c
✅ Always debounce
✅ Use pull-up/pull-down resistors
✅ Check button ready before use
✅ Handle press AND release
✅ Use appropriate ACTIVE flags
```

---

## Additional Resources

- [Zephyr Device Tree Guide](https://docs.zephyrproject.org/latest/build/dts/index.html)
- [GPIO API Reference](https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html)
- [Device Tree Specification](https://www.devicetree.org/)
- [nRF52840 GPIO Documentation](https://infocenter.nordicsemi.com/topic/ps_nrf52840/gpio.html)

---

## Summary

This example demonstrates:

✅ **GPIO Input** - Reading button states  
✅ **Device Tree Overlays** - Customizing hardware configuration  
✅ **Aliases** - Creating convenient hardware names  
✅ **Pull-up/Pull-down** - Proper button configuration  
✅ **Debouncing** - Handling mechanical switch issues  
✅ **Hardware Abstraction** - Portable code across boards  

Understanding Device Tree and GPIO input is fundamental for embedded development. These concepts apply to all hardware interfaces in Zephyr, from simple buttons to complex sensors and peripherals!
