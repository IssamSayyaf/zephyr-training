# Adding a Custom Library to Zephyr Application

This tutorial demonstrates how to create and integrate a custom library into your Zephyr application. We'll build upon the basic blink example by adding a `say_hello` library.

## What's New in This Example

Compared to the basic blink example (`01_blink`), this project adds:

✅ **Custom Library**: A reusable `say_hello` library  
✅ **Library Structure**: Proper organization with separate source and include directories  
✅ **CMake Integration**: How to build and link libraries in Zephyr  
✅ **Modular Code**: Separation of concerns for better code organization  

## Project Structure

```
02_add_lib/
├── CMakeLists.txt              # Main build configuration
├── prj.conf                    # Project Kconfig settings
├── src/
│   └── main.c                  # Application code (uses the library)
└── lib/
    └── say_hello/              # Custom library
        ├── CMakeLists.txt      # Library build configuration
        ├── include/
        │   └── say_hello.h     # Public header file
        └── src/
            └── say_hello.c     # Library implementation
```

### Key Differences from 01_blink

| Aspect | 01_blink | 02_add_lib |
|--------|----------|------------|
| Structure | Single source file | Application + Library |
| Files | main.c only | main.c + library files |
| CMakeLists | Simple | Two-level (app + lib) |
| Code Organization | Everything in main | Modular with library |
| Reusability | None | Library can be reused |

---

## Step-by-Step Guide: Adding a Library

### Step 1: Create Library Directory Structure

```bash
cd apps/02_add_lib

# Create library directory structure
mkdir -p lib/say_hello/src
mkdir -p lib/say_hello/include
```

### Step 2: Create Library Header File

**File: `lib/say_hello/include/say_hello.h`**

```c
#ifndef SAY_HELLO_H_
#define SAY_HELLO_H

// Function declaration - this is the public interface
void say_hello();

#endif
```

**Key Points:**
- Header guards (`#ifndef`, `#define`, `#endif`) prevent multiple inclusion
- Contains function declarations (interface), not implementations
- This is what other code will include to use the library

### Step 3: Create Library Implementation

**File: `lib/say_hello/src/say_hello.c`**

```c
#include <stdio.h>
#include "say_hello.h"

void say_hello()
{
    printf("Hello!!\r\n");
}
```

**Key Points:**
- Includes its own header file for consistency
- Contains the actual implementation (function body)
- Uses `printf()` which requires the standard C library

### Step 4: Create Library CMakeLists.txt

**File: `lib/say_hello/CMakeLists.txt`**

```cmake
# Create a static library target named 'hello_lib'
# STATIC: Creates a .a archive file that gets linked directly into the final binary
# This library contains the compiled object file from say_hello.c
add_library(hello_lib STATIC src/say_hello.c)

# Specify where the library's header files are located
# PUBLIC: Makes these include directories available to:
#   1. The library itself (hello_lib) during its own compilation
#   2. Any target that links against this library (propagates to consumers)
# ${CMAKE_CURRENT_SOURCE_DIR}: Points to this CMakeLists.txt's directory
# So other files can use: #include <say_hello.h>
target_include_directories(
    hello_lib 
    PUBLIC   
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link this library against Zephyr's interface library
# PRIVATE: Only hello_lib needs Zephyr compile options, not its consumers
# zephyr_interface: Provides Zephyr-specific compiler flags, definitions, and settings
#   - Architecture-specific flags (e.g., ARM Cortex-M optimizations)
#   - Zephyr RTOS include paths and definitions
#   - Required for proper compilation in the Zephyr environment
# Without this, the library wouldn't have access to Zephyr headers or proper compiler settings
target_link_libraries(hello_lib PRIVATE zephyr_interface)
```

**Understanding the Commands:**

1. **`add_library(hello_lib STATIC src/say_hello.c)`**
   - Creates a library target called `hello_lib`
   - `STATIC` means it compiles to a static library (.a file)
   - Lists source files to compile into the library

2. **`target_include_directories(hello_lib PUBLIC ...)`**
   - Tells CMake where header files are located
   - `PUBLIC` means anyone linking this library gets these include paths too
   - Essential for `#include "say_hello.h"` to work

3. **`target_link_libraries(hello_lib PRIVATE zephyr_interface)`**
   - Links the library with Zephyr's build system
   - Provides ARM compiler flags, Zephyr headers, and other settings
   - `PRIVATE` means only this library needs it, not consumers

### Step 5: Update Application CMakeLists.txt

**File: `CMakeLists.txt`** (at project root)

```cmake
cmake_minimum_required(VERSION 3.20.0)

set(BOARD nrf52840dk/nrf52840)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(hello_blink)

# Add the library subdirectory - this processes lib/say_hello/CMakeLists.txt
add_subdirectory(lib/say_hello)

# Add application source files
target_sources(app PRIVATE src/main.c)

# Link the application with the library
target_link_libraries(app PRIVATE hello_lib)
```

**New Lines Explained:**

- **`add_subdirectory(lib/say_hello)`**: 
  - Processes the library's CMakeLists.txt
  - Builds the library as part of the project
  - Makes the `hello_lib` target available

- **`target_link_libraries(app PRIVATE hello_lib)`**:
  - Links your application with the library
  - `app` is Zephyr's default executable target
  - Now your app can call functions from `hello_lib`

### Step 6: Use the Library in Your Application

**File: `src/main.c`**

```c
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "say_hello.h"  // Include our library header

#define SLEEP_TIME_MS   1000
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

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

    while (1) {
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;
        }

        led_state = !led_state;
        
        // Call our library function!
        say_hello();
        
        printf("LED state: %s\n", led_state ? "ON" : "OFF");
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
```

**Key Changes:**
- Added `#include "say_hello.h"` to access library functions
- Called `say_hello()` in the main loop
- The library function is now part of your application!

---

## Building and Running

### Build the Application

```bash
# Navigate to project directory
cd apps/02_add_lib

# Make sure Zephyr environment is set up
source ~/zephyrproject/zephyr/zephyr-env.sh

# Build
west build -b nrf52840dk/nrf52840 -p
```

### Expected Output

When you run the application, you should see:

```
Hello!!
LED state: ON
Hello!!
LED state: OFF
Hello!!
LED state: ON
...
```

The "Hello!!" message comes from your custom library!

---

## Understanding CMake Library Integration

### The Build Process

```
┌─────────────────────────────────────────────────────┐
│ 1. CMakeLists.txt (root)                            │
│    - Calls add_subdirectory(lib/say_hello)          │
└────────────────┬────────────────────────────────────┘
                 │
                 v
┌─────────────────────────────────────────────────────┐
│ 2. lib/say_hello/CMakeLists.txt                     │
│    - Compiles say_hello.c                           │
│    - Creates hello_lib.a (static library)           │
│    - Exports include directories                    │
└────────────────┬────────────────────────────────────┘
                 │
                 v
┌─────────────────────────────────────────────────────┐
│ 3. Back to root CMakeLists.txt                      │
│    - Compiles main.c                                │
│    - Links with hello_lib                           │
│    - Creates final executable                       │
└─────────────────────────────────────────────────────┘
```

### Include Path Resolution

When you write `#include "say_hello.h"`:

1. Compiler looks in directories specified by `target_include_directories()`
2. Finds `lib/say_hello/include/say_hello.h`
3. Since we used `PUBLIC`, this path is available to any target linking with `hello_lib`

### Linking Process

```
main.c → main.o ┐
                ├→ Link → app.elf → app.hex
say_hello.c → hello_lib.a ┘
```

---

