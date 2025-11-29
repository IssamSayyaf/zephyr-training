# Creating and Using Zephyr Modules

A comprehensive tutorial on creating custom Zephyr modules that can be shared across multiple applications with Kconfig integration.

## 📋 Table of Contents

1. [What's New in This Example](#whats-new-in-this-example)
2. [What is a Zephyr Module?](#what-is-a-zephyr-module)
3. [Module vs Library - Key Differences](#module-vs-library---key-differences)
4. [Project Structure](#project-structure)
5. [Step-by-Step: Creating a Module](#step-by-step-creating-a-module)
6. [Step-by-Step: Using the Module](#step-by-step-using-the-module)
7. [Understanding Module Files](#understanding-module-files)
8. [Kconfig Integration](#kconfig-integration)
9. [Build System Integration](#build-system-integration)
10. [Testing the Module](#testing-the-module)
11. [Best Practices](#best-practices)
12. [Troubleshooting](#troubleshooting)
13. [Exercises](#exercises)

---

## What's New in This Example

### Evolution from Previous Examples

| Feature | 01_blink | 02_add_lib | 03_Kconfig_demo |
|---------|----------|------------|-----------------|
| **Location** | Inside app | Inside app | `modules/` directory |
| **Scope** | Single app | Single app | **Multiple apps** |
| **Kconfig** | ❌ | ❌ | **✅ Full support** |
| **menuconfig** | ❌ | ❌ | **✅ Appears in menu** |
| **module.yml** | ❌ | ❌ | **✅ Required** |
| **Reusability** | None | Limited | **High** |
| **Configurability** | Hardcoded | Hardcoded | **Runtime config** |

### What You'll Learn

✅ **Creating standalone modules** outside application directory  
✅ **Module directory structure** and required files  
✅ **Using `module.yaml`** for Zephyr integration  
✅ **Adding Kconfig options** for user configuration  
✅ **Conditional compilation** based on Kconfig  
✅ **Linking modules** via `ZEPHYR_EXTRA_MODULES`  
✅ **Making code truly reusable** across projects  

---

## What is a Zephyr Module?

A **Zephyr module** is a self-contained package of code that integrates with Zephyr's build system and can be shared across multiple applications.

### Module Anatomy

```
Module = Code + Configuration + Metadata
         ↓           ↓              ↓
      (src/)    (Kconfig)     (module.yaml)
```

### What Can Modules Contain?

- 🔌 **Drivers**: Hardware interfaces (I2C, SPI, custom sensors)
- 📚 **Libraries**: Reusable code (algorithms, utilities)
- 🎯 **Board Definitions**: Custom hardware support
- 📦 **HAL Code**: Hardware Abstraction Layers
- 🧪 **Tests**: Validation and test code

### Benefits of Modules

| Benefit | Description |
|---------|-------------|
| **🔄 Reusability** | Use the same module in multiple applications |
| **🔧 Configurability** | Enable/disable features via menuconfig |
| **📦 Portability** | Easy to share between projects and teams |
| **🎯 Organization** | Keep related code together |
| **⚡ Versioning** | Version control modules independently |
| **🛠️ Maintainability** | Update once, use everywhere |

---

## Module vs Library - Key Differences

### Local Library (Example 02)

```
apps/02_add_lib/
└── lib/
    └── say_hello/              # 🔒 Only for THIS app
        ├── CMakeLists.txt
        ├── include/
        │   └── say_hello.h
        └── src/
            └── say_hello.c
```

**Characteristics:**
- ❌ Lives inside application directory
- ❌ Cannot be shared with other apps
- ❌ No Kconfig integration
- ❌ Not visible in menuconfig
- ✅ Simple CMake setup
- ✅ Good for app-specific code

### Zephyr Module (Example 03)

```
modules/
└── say_hello/                  # 🌍 Available to ALL apps
    ├── zephyr/
    │   └── module.yaml         # 🔑 Module registration
    ├── Kconfig                 # ⚙️ Configuration options
    ├── CMakeLists.txt          # 🔨 Build rules
    ├── say_hello.h             # 📄 Public interface
    └── say_hello.c             # 💻 Implementation
```

**Characteristics:**
- ✅ Lives outside application directories
- ✅ Shared by multiple applications
- ✅ Full Kconfig support
- ✅ Visible in menuconfig
- ✅ Integrated with Zephyr build system
- ✅ Can be versioned independently

### When to Use Which?

| 📝 Use Local Library When... | 🌐 Use Module When... |
|------------------------------|----------------------|
| Code is application-specific | Code is reusable across apps |
| Quick prototype or test | Production-ready code |
| No configuration needed | Needs user configuration |
| Small helper functions | Complete driver/subsystem |
| Tight coupling with app | Loosely coupled components |
| One-time use | Multiple applications |

---

## Project Structure

### Complete Directory Layout

```
Zephyr_tutorial/
├── apps/
│   ├── 01_blink/               # Example 1: Basic GPIO
│   ├── 02_add_lib/             # Example 2: Local library
│   └── 03_Kconfig_demo/        # Example 3: Using module ⭐
│       ├── CMakeLists.txt      # References module
│       ├── prj.conf            # Enables CONFIG_SAY_HELLO
│       └── src/
│           └── main.c          # Uses module functions
│
└── modules/                     # 🎯 Shared modules directory
    └── say_hello/              # Our custom module ⭐
        ├── zephyr/
        │   └── module.yaml     # Module metadata
        ├── Kconfig             # Configuration options
        ├── CMakeLists.txt      # Build configuration
        ├── say_hello.h         # Public header
        └── say_hello.c         # Implementation
```

### Key Difference: Location Matters!

```
❌ Bad: Module inside app
apps/03_Kconfig_demo/modules/say_hello/

✅ Good: Module at project level
modules/say_hello/
```

---

## Step-by-Step: Creating a Module

### Step 1: Create Module Directory Structure

```bash
# Navigate to your project root
cd ~/Zephyr_tutorial

# Create module directory
mkdir -p modules/say_hello
cd modules/say_hello

# Create zephyr subdirectory
mkdir zephyr
```

**Why this structure?**
- `modules/` is the standard location for external modules
- `say_hello/` contains your module code
- `zephyr/` subdirectory contains integration files

### Step 2: Create module.yaml (Required!)

**File: `modules/say_hello/zephyr/module.yaml`**

```yaml
name: say_hello
build:
  cmake: .
  kconfig: Kconfig
```

**Line-by-line explanation:**

```yaml
name: say_hello              # Module identifier (must be unique)
build:                       # Build configuration section
  cmake: .                   # Path to CMakeLists.txt (. means parent dir)
  kconfig: Kconfig           # Path to Kconfig file (relative to parent dir)
```

**⚠️ Critical:**
- This file **MUST** exist for Zephyr to recognize your module
- Must be named exactly `module.yaml` (or `module.yml`)
- Must be in the `zephyr/` subdirectory

### Step 3: Create Kconfig Options

**File: `modules/say_hello/Kconfig`**

```kconfig
# Create a new option in menuconfig
config SAY_HELLO
    bool "Basic print test to console"
    default n                   # Disabled by default
    depends on PRINTK           # Requires PRINTK to be enabled
    help
        Adds say_hello() function to print a basic message to the console.
        
        When enabled, applications can call say_hello() to print
        a greeting message. This is useful for testing and debugging.
```

**Understanding the Kconfig:**

```kconfig
config SAY_HELLO                # Config option name (use CONFIG_SAY_HELLO in code)
    bool "..."                  # Type: boolean (y/n)
    default n                   # Default value: disabled
    depends on PRINTK           # Only available if PRINTK is enabled
    help                        # Description shown in menuconfig
        ...
```

**Kconfig Types:**

```kconfig
bool "Yes/No option"            # CONFIG_OPTION=y or not set
int "Integer value"             # CONFIG_VALUE=123
string "Text value"             # CONFIG_NAME="text"
hex "Hexadecimal value"         # CONFIG_ADDR=0x1000
```

### Step 4: Create Public Header

**File: `modules/say_hello/say_hello.h`**

```c
#ifndef SAY_HELLO_H_
#define SAY_HELLO_H_

// Public function declaration
void say_hello(void);

#endif /* SAY_HELLO_H_ */
```

**Header Guard Pattern:**
```c
#ifndef SAY_HELLO_H_        // If not defined
#define SAY_HELLO_H_        // Define it
// ... declarations ...
#endif                      // End if
```

This prevents multiple inclusion errors.

### Step 5: Create Implementation

**File: `modules/say_hello/say_hello.c`**

```c
#include <zephyr/kernel.h>
#include "say_hello.h"

void say_hello(void) {
    printk("Hello!\r\n");
}
```

**Key points:**
- Include Zephyr headers: `<zephyr/kernel.h>`
- Include own header: `"say_hello.h"`
- Use `printk()` for console output (lighter than printf)
- Simple implementation for demonstration

### Step 6: Create Module CMakeLists.txt

**File: `modules/say_hello/CMakeLists.txt`**

```cmake
# Check if SAY_HELLO is set in Kconfig
if(CONFIG_SAY_HELLO)

    # Add your include directory
    zephyr_include_directories(.)

    # Add the source file you want to compile
    zephyr_library_sources(say_hello.c)

endif()
```

**Line-by-line explanation:**

```cmake
if(CONFIG_SAY_HELLO)                    # Only build if enabled in Kconfig
    
    zephyr_include_directories(.)        # Add this dir to include path
                                        # So #include "say_hello.h" works
    
    zephyr_library_sources(say_hello.c)  # Add source file to build
                                        # Creates zephyr library
endif()
```

**Why conditional compilation?**
- Only builds code when CONFIG_SAY_HELLO=y
- Saves memory and compilation time
- User controls what gets included

### Step 7: Module Complete! ✅

Your module structure should now look like:

```
modules/say_hello/
├── zephyr/
│   └── module.yaml         ✅ Module registration
├── Kconfig                 ✅ Configuration options
├── CMakeLists.txt          ✅ Build rules
├── say_hello.h             ✅ Public API
└── say_hello.c             ✅ Implementation
```

---

## Step-by-Step: Using the Module

Now let's use the module in an application.

### Step 1: Link Module in Application CMakeLists.txt

**File: `apps/03_Kconfig_demo/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20.0)

# Set target board
set(BOARD nrf52840dk/nrf52840)

# 🔑 CRITICAL: Tell Zephyr where to find the module
set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../../modules/say_hello")

# Find Zephyr package
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name
project(random_number_generator)

# Add application source
target_sources(app PRIVATE src/main.c)
```

**Understanding ZEPHYR_EXTRA_MODULES:**

```cmake
set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../../modules/say_hello")
     │                     │                            │
     │                     │                            └─ Module directory
     │                     └─ Path from CMakeLists.txt location
     └─ Variable name (can be semicolon-separated list)
```

**Path explanation:**
```
Current file: apps/03_Kconfig_demo/CMakeLists.txt
                                   ↓
                                   ../          (up to apps/)
                                   ↓
                                   ../../       (up to project root)
                                   ↓
                                   ../../modules/say_hello
```

**Multiple modules:**
```cmake
set(ZEPHYR_EXTRA_MODULES 
    "${CMAKE_SOURCE_DIR}/../../modules/say_hello"
    "${CMAKE_SOURCE_DIR}/../../modules/my_sensor"
    "${CMAKE_SOURCE_DIR}/../../modules/my_driver"
)
```

### Step 2: Enable Module in prj.conf

**File: `apps/03_Kconfig_demo/prj.conf`**

```properties
# Enable our custom module
CONFIG_SAY_HELLO=y

# Enable floating point printf (for the random example)
CONFIG_PICOLIBC_IO_FLOAT=y
```

**What happens:**
- `CONFIG_SAY_HELLO=y` tells Kconfig to enable the module
- This triggers the `if(CONFIG_SAY_HELLO)` in module's CMakeLists.txt
- Module gets compiled and linked

### Step 3: Use Module in Application Code

**File: `apps/03_Kconfig_demo/src/main.c`**

```c
#include <zephyr/random/random.h>

#ifdef CONFIG_SAY_HELLO
#include "say_hello.h"
#endif

static const int32_t sleep_time_ms = 1000;

int main(void)
{
    uint32_t rnd;
    double rnd_float;
    
    while (1) {
        // Generate random number
        rnd = sys_rand32_get();
        rnd_float = (double)rnd / (UINT32_MAX + 1.0);
        printk("Random number: %.3f\r\n", rnd_float);

#ifdef CONFIG_SAY_HELLO
        // Call module function (only if enabled)
        say_hello();
#endif

        k_msleep(sleep_time_ms);
    }

    return 0;
}
```

**Conditional compilation pattern:**

```c
#ifdef CONFIG_SAY_HELLO          // If module is enabled in Kconfig
#include "say_hello.h"           // Include the header
#endif

// Later in code...
#ifdef CONFIG_SAY_HELLO          // Check again before using
        say_hello();             // Call module function
#endif
```

**Why use `#ifdef`?**
- ✅ Code compiles even if module is disabled
- ✅ No runtime overhead when disabled
- ✅ User can enable/disable via menuconfig
- ✅ Graceful degradation

---
