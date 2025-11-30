# Zephyr Build System Guide

A comprehensive guide to building and compiling Zephyr RTOS applications and drivers.

## Table of Contents

1. [Installation and Setup](#1-installation-and-setup)
2. [Build System Overview](#2-build-system-overview)
3. [Environment Setup](#3-environment-setup)
4. [Basic Build Commands](#4-basic-build-commands)
5. [Build System Architecture](#5-build-system-architecture)
6. [Advanced Build Options](#6-advanced-build-options)
7. [Building with Custom Modules](#7-building-with-custom-modules)
8. [Build Configuration](#8-build-configuration)
9. [Debugging Build Issues](#9-debugging-build-issues)
10. [Board-Specific Builds](#10-board-specific-builds)
11. [Best Practices](#11-best-practices)

---

## 1. Installation and Setup

This section covers installing Zephyr RTOS and SDK from scratch, with a focus on out-of-tree application development (building projects outside the Zephyr source tree).

### 1.1 System Requirements

**Supported Operating Systems:**
- Ubuntu 20.04/22.04/24.04 LTS (recommended)
- Fedora
- macOS
- Windows 10/11 (via WSL2)

**Minimum Hardware:**
- 4GB RAM (8GB+ recommended)
- 10GB free disk space
- USB port for programming boards

### 1.2 Install Dependencies

#### Ubuntu/Debian

```bash
# Update package list
sudo apt update

# Install required dependencies
sudo apt install -y \
    git cmake ninja-build gperf \
    ccache dfu-util device-tree-compiler wget \
    python3-dev python3-pip python3-setuptools python3-tk python3-wheel \
    xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev \
    libmagic1

# Install Python dependencies
pip3 install --user -U west
```

#### Fedora

```bash
sudo dnf install -y \
    git cmake ninja-build gperf ccache dfu-util \
    dtc wget python3-pip python3-tkinter \
    xz file glibc-devel.i686 libstdc++-devel.i686 \
    SDL2-devel python3-devel

pip3 install --user -U west
```

#### macOS

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install cmake ninja gperf python3 ccache qemu dtc wget libmagic

# Install west
pip3 install --user -U west
```

### 1.3 Install Zephyr RTOS

There are two main approaches: workspace application (recommended) and freestanding application.

#### Option A: Workspace Application (Recommended for Learning)

This approach keeps your application alongside Zephyr source, useful for learning and development.

```bash
# Create workspace directory
mkdir ~/zephyrproject
cd ~/zephyrproject

# Initialize west workspace
west init

# Update Zephyr and all modules
west update

# Export Zephyr CMake package (run once after install or update)
west zephyr-export

# Install Python dependencies
pip3 install --user -r ~/zephyrproject/zephyr/scripts/requirements.txt
```

**Directory structure after this:**
```
~/zephyrproject/
├── .west/              # West configuration
├── zephyr/             # Zephyr RTOS source
├── bootloader/         # MCUboot bootloader
├── modules/            # External modules
│   ├── hal/
│   ├── lib/
│   └── ...
└── tools/              # Development tools
```

#### Option B: Freestanding Application (Out-of-Tree Development)

This approach keeps your application separate from Zephyr, ideal for production projects.

```bash
# First, install Zephyr somewhere (this is your "Zephyr installation")
mkdir ~/zephyr-sdk-install
cd ~/zephyr-sdk-install

# Clone Zephyr (or use a release)
git clone https://github.com/zephyrproject-rtos/zephyr.git
cd zephyr

# Install Python dependencies
pip3 install --user -r scripts/requirements.txt

# Export Zephyr CMake package
cmake -P share/zephyr-package/cmake/zephyr_export.cmake

# Now create your application workspace elsewhere
mkdir -p ~/my_projects/my_zephyr_app
cd ~/my_projects/my_zephyr_app

# Your app structure (we'll build this)
```

### 1.4 Install Zephyr SDK

The Zephyr SDK contains toolchains and utilities for building Zephyr applications.

#### Automated Installation (Recommended)

```bash
# Navigate to home directory
cd ~

# Download SDK installer (version 0.16.5-1 - check for latest)
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64.tar.xz

# Extract the archive
tar xvf zephyr-sdk-0.16.5_linux-x86_64.tar.xz

# Run the installer
cd zephyr-sdk-0.16.5
./setup.sh

# Install udev rules (for device permissions)
sudo cp ~/zephyr-sdk-0.16.5/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
sudo udevadm control --reload
```

#### Manual Installation

```bash
# Set SDK installation directory
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5

# Install specific toolchains (if you don't want all)
cd ~/zephyr-sdk-0.16.5
./setup.sh -t arm-zephyr-eabi      # For ARM Cortex-M
./setup.sh -t riscv64-zephyr-elf   # For RISC-V
./setup.sh -t xtensa-espressif_esp32_zephyr-elf  # For ESP32
```

#### Verify SDK Installation

```bash
# Check if SDK is properly installed
ls ~/zephyr-sdk-0.16.5/

# Should see directories like:
# arm-zephyr-eabi/
# cmake/
# riscv64-zephyr-elf/
# sysroots/
# ...
```

### 1.5 Configure Environment Variables

You need to set up environment variables to tell the build system where Zephyr and the SDK are located.

#### Option 1: Temporary (Per Terminal Session)

```bash
# Set Zephyr base directory
export ZEPHYR_BASE=~/zephyrproject/zephyr

# Set SDK installation directory
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5

# Add SDK to PATH
export PATH=$ZEPHYR_SDK_INSTALL_DIR/sysroots/x86_64-pokysdk-linux/usr/bin:$PATH

# Source Zephyr environment script
source ~/zephyrproject/zephyr/zephyr-env.sh
```

#### Option 2: Permanent (Add to Shell Configuration)

Add these lines to your `~/.bashrc` or `~/.zshrc`:

```bash
# Zephyr environment setup
export ZEPHYR_BASE=~/zephyrproject/zephyr
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5
export PATH=$ZEPHYR_SDK_INSTALL_DIR/sysroots/x86_64-pokysdk-linux/usr/bin:$PATH

# Auto-source Zephyr environment (optional, but convenient)
if [ -f "$ZEPHYR_BASE/zephyr-env.sh" ]; then
    source $ZEPHYR_BASE/zephyr-env.sh
fi
```

Then reload your shell:
```bash
source ~/.bashrc  # or source ~/.zshrc for zsh
```

#### Option 3: Using a Virtual Environment (Recommended for Python Isolation)

```bash
# Create Python virtual environment
python3 -m venv ~/zephyrproject/.venv

# Activate it
source ~/zephyrproject/.venv/bin/activate

# Install Python dependencies
pip install west
pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt

# Now set Zephyr environment
export ZEPHYR_BASE=~/zephyrproject/zephyr
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5
source $ZEPHYR_BASE/zephyr-env.sh
```

Create a convenience script `~/zephyrproject/activate.sh`:

```bash
#!/bin/bash
# Zephyr environment activation script

# Activate Python virtual environment
source ~/zephyrproject/.venv/bin/activate

# Set Zephyr environment variables
export ZEPHYR_BASE=~/zephyrproject/zephyr
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk-0.16.5

# Source Zephyr environment
source $ZEPHYR_BASE/zephyr-env.sh

echo "Zephyr environment activated!"
echo "ZEPHYR_BASE: $ZEPHYR_BASE"
echo "SDK: $ZEPHYR_SDK_INSTALL_DIR"
```

Make it executable and use it:
```bash
chmod +x ~/zephyrproject/activate.sh
source ~/zephyrproject/activate.sh
```

### 1.6 Set Up Out-of-Tree Application Structure

This is the recommended structure for developing applications outside the Zephyr tree.

```bash
# Create your project directory (anywhere you want)
mkdir -p ~/Zephyr_tutorial
cd ~/Zephyr_tutorial

# Create application directory
mkdir -p apps/01_blink
cd apps/01_blink

# Create source directory
mkdir src

# Create basic files
touch CMakeLists.txt
touch prj.conf
touch src/main.c
```

#### Create CMakeLists.txt

```cmake
# Specify minimum CMake version
cmake_minimum_required(VERSION 3.20.0)

# Set the target board
set(BOARD nrf52840dk/nrf52840)

# Find Zephyr (must have ZEPHYR_BASE set or sourced zephyr-env.sh)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Define project name
project(blink_app)

# Add source files
target_sources(app PRIVATE src/main.c)
```

#### Create prj.conf

```properties
# Enable GPIO support
CONFIG_GPIO=y

# Enable console/UART
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_PRINTK=y
```

#### Create src/main.c

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    while (1) {
        gpio_pin_toggle_dt(&led);
        k_msleep(1000);
    }
    return 0;
}
```

### 1.7 Build Your First Application

```bash
# Make sure environment is set up
source ~/zephyrproject/activate.sh  # If using the script
# OR
source ~/zephyrproject/zephyr/zephyr-env.sh

# Navigate to your application
cd ~/Zephyr_tutorial/apps/01_blink

# Build the application
west build -b nrf52840dk/nrf52840

# If successful, you'll see:
# -- Build files have been written to: .../build
# [XXX/XXX] Linking C executable zephyr/zephyr.elf
```

### 1.8 Verify Installation

Create a simple verification script `~/zephyrproject/check_install.sh`:

```bash
#!/bin/bash

echo "=== Zephyr Installation Check ==="
echo ""

# Check west
echo -n "West: "
if command -v west &> /dev/null; then
    west --version
else
    echo "❌ NOT FOUND"
fi

# Check CMake
echo -n "CMake: "
if command -v cmake &> /dev/null; then
    cmake --version | head -n1
else
    echo "❌ NOT FOUND"
fi

# Check Ninja
echo -n "Ninja: "
if command -v ninja &> /dev/null; then
    ninja --version
else
    echo "❌ NOT FOUND"
fi

# Check ZEPHYR_BASE
echo -n "ZEPHYR_BASE: "
if [ -n "$ZEPHYR_BASE" ]; then
    echo "✅ $ZEPHYR_BASE"
else
    echo "❌ NOT SET"
fi

# Check SDK
echo -n "Zephyr SDK: "
if [ -n "$ZEPHYR_SDK_INSTALL_DIR" ]; then
    echo "✅ $ZEPHYR_SDK_INSTALL_DIR"
else
    echo "❌ NOT SET"
fi

# Check if Zephyr directory exists
echo -n "Zephyr source: "
if [ -d "$ZEPHYR_BASE" ]; then
    echo "✅ Found"
else
    echo "❌ NOT FOUND"
fi

echo ""
echo "=== Board Support Check ==="
west boards | grep nrf52840dk | head -n3

echo ""
echo "=== Test Build ==="
echo "Run: cd ~/Zephyr_tutorial/apps/01_blink && west build -b nrf52840dk/nrf52840"
```

Run it:
```bash
chmod +x ~/zephyrproject/check_install.sh
~/zephyrproject/check_install.sh
```

### 1.9 Common Installation Issues

#### Issue: "west: command not found"

```bash
# Solution: Add pip user bin to PATH
export PATH=$HOME/.local/bin:$PATH

# Add to ~/.bashrc permanently
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

# Reinstall west if needed
pip3 install --user -U west
```

#### Issue: "Could not find Zephyr"

```bash
# Solution: Make sure ZEPHYR_BASE is set
export ZEPHYR_BASE=~/zephyrproject/zephyr

# And that zephyr-env.sh is sourced
source $ZEPHYR_BASE/zephyr-env.sh
```

#### Issue: "No module named 'elftools'"

```bash
# Solution: Install Python requirements
pip3 install --user -r ~/zephyrproject/zephyr/scripts/requirements.txt
```

#### Issue: SDK toolchain not found

```bash
# Solution: Verify SDK installation
ls $ZEPHYR_SDK_INSTALL_DIR/arm-zephyr-eabi/

# Re-run SDK setup if needed
cd ~/zephyr-sdk-0.16.5
./setup.sh -t arm-zephyr-eabi
```

### 1.10 Multiple Zephyr Versions

You can maintain multiple Zephyr installations:

```bash
# Structure
~/zephyr-v3.5/
├── zephyr/
└── ...

~/zephyr-v3.6/
├── zephyr/
└── ...

# Switch between them by changing ZEPHYR_BASE
export ZEPHYR_BASE=~/zephyr-v3.5/zephyr
source $ZEPHYR_BASE/zephyr-env.sh

# Or use different activation scripts
# activate-v3.5.sh
# activate-v3.6.sh
```

### 1.11 Next Steps After Installation

Once installation is complete:

1. ✅ Verify environment with check script
2. ✅ Build the blink example
3. ✅ Flash to your board: `west flash`
4. ✅ Try samples: `cd ~/zephyrproject/zephyr/samples/basic/blinky`
5. ✅ Explore documentation: https://docs.zephyrproject.org/

---

## 2. Build System Overview

Zephyr uses **CMake** as its build system, combined with the **West** meta-tool for managing repositories and building projects.

> 💡 **Note**: If you haven't installed Zephyr yet, see [Section 1: Installation and Setup](#1-installation-and-setup) first.

### Key Components

- **CMake**: Build system generator
- **West**: Zephyr's meta-tool for building and flashing
- **Ninja/Make**: Build executors
- **Kconfig**: Configuration system
- **Device Tree**: Hardware description

### Build Process Flow

```
Source Code + Config → CMake → Build Files → Ninja/Make → Binary
     ↓                   ↓          ↓            ↓
  prj.conf          Device Tree   Kconfig    Compilation
  CMakeLists.txt    .overlay      .config    Linking
```

---

## 3. Environment Setup

### Prerequisites

Before building, ensure your environment is set up:

```bash
# 1. Source the Zephyr environment (do this in every new terminal)
source ~/zephyrproject/zephyr/zephyr-env.sh

# Or if using virtual environment:
source ~/zephyrproject/.venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh

# 2. Verify west is available
west --version

# 3. Verify Zephyr SDK path is set
echo $ZEPHYR_BASE
echo $ZEPHYR_SDK_INSTALL_DIR
```

### Environment Variables

Key environment variables:

- `ZEPHYR_BASE`: Path to Zephyr source directory
- `ZEPHYR_SDK_INSTALL_DIR`: Path to Zephyr SDK
- `BOARD`: Default board (optional)
- `ZEPHYR_TOOLCHAIN_VARIANT`: Toolchain to use (zephyr, gnuarmemb, etc.)

---

## 4. Basic Build Commands

### Method 1: Using West (Recommended)

West is the recommended way to build Zephyr applications.

#### Build Command Structure

```bash
west build -b <board> [-d <build-dir>] [source-dir] [-- cmake-options]
```

#### Common Build Commands

```bash
# Build for nRF52840 DK board
cd /path/to/your/app
west build -b nrf52840dk/nrf52840

# Build with clean build directory (pristine)
west build -b nrf52840dk/nrf52840 -p

# Build in a specific directory
west build -b nrf52840dk/nrf52840 -d build_custom

# Build with specific source directory
west build -b nrf52840dk/nrf52840 /path/to/source

# Rebuild (incremental build)
west build

# Clean and rebuild
west build -t clean
west build
```

#### Build Options

| Option | Description | Example |
|--------|-------------|---------|
| `-b BOARD` | Specify target board | `-b nrf52840dk/nrf52840` |
| `-d BUILD_DIR` | Build directory | `-d build_debug` |
| `-p` | Pristine build (clean first) | `-p` or `-p auto` or `-p always` |
| `-t TARGET` | Build specific target | `-t menuconfig` |
| `-- CMAKE_ARGS` | Pass CMake arguments | `-- -DCONF_FILE=prj_release.conf` |

### Method 2: Using CMake Directly

You can also use CMake directly, though west is preferred.

```bash
# Create build directory
mkdir build && cd build

# Configure
cmake -B . -GNinja -DBOARD=nrf52840dk/nrf52840 ..

# Build
ninja

# Or use make instead of ninja
cmake -B . -DBOARD=nrf52840dk/nrf52840 ..
make
```

### Build Targets

```bash
# Build the application
west build

# Run menuconfig (interactive configuration)
west build -t menuconfig

# Run guiconfig (GUI configuration)
west build -t guiconfig

# Clean build
west build -t clean

# Generate compile_commands.json
west build -t compile_commands

# Build documentation
west build -t doc

# Show build configuration
west build -t hardenconfig
```

---

## 5. Build System Architecture

### CMakeLists.txt Structure

Your application's `CMakeLists.txt` is the entry point:

```cmake
# Minimum CMake version
cmake_minimum_required(VERSION 3.20.0)

# Set board (optional, can be set via command line)
set(BOARD nrf52840dk/nrf52840)

# Set device tree overlay (optional)
set(DTC_OVERLAY_FILE "board/app.overlay")

# Additional board configurations (optional)
set(CONF_FILE "prj.conf board/nrf_config.conf")

# Extra modules (for custom drivers/libraries)
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/nunchuk
)

# Find Zephyr package
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Define project
project(my_application)

# Add source files
target_sources(app PRIVATE
    src/main.c
    src/helper.c
)

# Add include directories
target_include_directories(app PRIVATE
    include
)

# Add compile definitions
target_compile_definitions(app PRIVATE
    MY_CUSTOM_DEFINE=1
)
```

### Configuration Files

#### prj.conf (Project Configuration)

Main configuration file using Kconfig:

```properties
# Enable GPIO
CONFIG_GPIO=y

# Enable I2C
CONFIG_I2C=y

# Enable logging
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

# Enable your custom driver
CONFIG_NUNCHUK_DRIVER=y

# Stack sizes
CONFIG_MAIN_STACK_SIZE=2048
CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048

# Memory settings
CONFIG_HEAP_MEM_POOL_SIZE=4096
```

#### Board-Specific Configuration

Create board-specific configs:

```
app/
├── CMakeLists.txt
├── prj.conf                  # Base configuration
├── board/
│   ├── nrf52840dk_nrf52840.conf   # Board-specific
│   └── app.overlay           # Device tree overlay
```

#### Device Tree Overlay

Hardware description in `board/app.overlay`:

```dts
/ {
    aliases {
        myled = &led0;
    };
};

&i2c0 {
    status = "okay";
    
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

## 6. Advanced Build Options

### Multiple Configuration Files

```bash
# Use multiple configuration files
west build -b nrf52840dk/nrf52840 -- \
    -DCONF_FILE="prj.conf;board/debug.conf"

# Use different base configuration
west build -b nrf52840dk/nrf52840 -- \
    -DCONF_FILE=prj_release.conf
```

### Device Tree Overlays

```bash
# Use specific overlay
west build -b nrf52840dk/nrf52840 -- \
    -DDTC_OVERLAY_FILE="board/app.overlay"

# Use multiple overlays
west build -b nrf52840dk/nrf52840 -- \
    -DDTC_OVERLAY_FILE="board/base.overlay;board/sensors.overlay"
```

### Shield Support

```bash
# Build with a shield
west build -b nrf52840dk/nrf52840 -- -DSHIELD=adafruit_2_8_tft_touch_v2
```

### Custom Toolchain

```bash
# Use GNU Arm Embedded toolchain
west build -b nrf52840dk/nrf52840 -- \
    -DZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb

# Use LLVM/Clang
west build -b nrf52840dk/nrf52840 -- \
    -DZEPHYR_TOOLCHAIN_VARIANT=llvm
```

### Optimization Levels

```bash
# Debug build (no optimization)
west build -b nrf52840dk/nrf52840 -- \
    -DCMAKE_BUILD_TYPE=Debug

# Release build (optimized)
west build -b nrf52840dk/nrf52840 -- \
    -DCMAKE_BUILD_TYPE=Release

# Minimum size
west build -b nrf52840dk/nrf52840 -- \
    -DCMAKE_BUILD_TYPE=MinSizeRel
```

### Verbose Build

```bash
# Show all build commands
west build -v

# Or
west build -- -v
```

---

## 7. Building with Custom Modules

### Adding External Modules

In your `CMakeLists.txt`:

```cmake
# Method 1: Using ZEPHYR_EXTRA_MODULES
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/nunchuk
    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/custom_lib
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
```

Or via command line:

```bash
# Method 2: Command line
west build -b nrf52840dk/nrf52840 -- \
    -DZEPHYR_EXTRA_MODULES="/path/to/module1;/path/to/module2"
```

Or using west.yml:

```yaml
# Method 3: west.yml
manifest:
  projects:
    - name: my-module
      path: modules/my-module
      revision: main
```

### Module Structure Requirements

Your module must have:

```
modules/mymodule/
├── zephyr/
│   ├── module.yml        # Required
│   ├── CMakeLists.txt    # Required
│   └── Kconfig           # Required
├── CMakeLists.txt
├── Kconfig
└── [source files]
```

---

## 8. Build Configuration

### Interactive Configuration

```bash
# Text-based menu configuration
west build -t menuconfig

# GUI configuration (requires Python tkinter)
west build -t guiconfig

# Save current configuration
west build -t savedefconfig
```

### View Configuration

```bash
# View final .config file
cat build/zephyr/.config

# View device tree configuration
cat build/zephyr/zephyr.dts

# View final device tree
cat build/zephyr/zephyr.dts.pre.tmp
```

### Configuration Debugging

```bash
# Show why a config option has a value
west build -t menuconfig
# Then use '/' to search and '?' to see dependencies

# Generate Kconfig documentation
scripts/kconfig/genconfig.py --help
```

---

## 9. Debugging Build Issues

### Common Build Errors and Solutions

#### Error: "No BOARD was specified"

```bash
# Solution: Specify board explicitly
west build -b nrf52840dk/nrf52840

# Or set in CMakeLists.txt
set(BOARD nrf52840dk/nrf52840)
```

#### Error: "Could not find Zephyr"

```bash
# Solution: Source Zephyr environment
source ~/zephyrproject/zephyr/zephyr-env.sh

# Or set ZEPHYR_BASE
export ZEPHYR_BASE=~/zephyrproject/zephyr
```

#### Error: "Device tree error"

```bash
# View device tree errors in detail
west build -v

# Check device tree output
cat build/zephyr/zephyr.dts

# Validate device tree binding
ls $ZEPHYR_BASE/dts/bindings/
```

#### Error: "Undefined reference to..."

```bash
# Solution: Check if source file is added
# In CMakeLists.txt:
target_sources(app PRIVATE src/missing_file.c)

# Or check if library/driver is enabled in prj.conf
CONFIG_MY_DRIVER=y
```

### Build Verbosity

```bash
# Verbose CMake output
west build -v

# Very verbose (all commands)
west build -- VERBOSE=1

# Show all warnings
west build -- -DWARNING_AS_ERRORS=ON
```

### Clean Build Strategies

```bash
# Clean build directory
west build -t clean

# Pristine build (removes build directory)
west build -p

# Pristine auto (clean if needed)
west build -p auto

# Pristine always (always clean)
west build -p always

# Remove build directory manually
rm -rf build
```

---

## 10. Board-Specific Builds

### Building for Different Boards

```bash
# Nordic nRF52840 DK
west build -b nrf52840dk/nrf52840

# Raspberry Pi Pico
west build -b rpi_pico

# STM32 Discovery
west build -b stm32f4_disco

# QEMU (for testing without hardware)
west build -b qemu_cortex_m3
```

### Custom Board Definition

Create custom board in your project:

```
app/
├── boards/
│   └── arm/
│       └── my_custom_board/
│           ├── my_custom_board.dts
│           ├── my_custom_board.yaml
│           ├── my_custom_board_defconfig
│           └── board.cmake
```

Build with custom board:

```bash
west build -b my_custom_board -- \
    -DBOARD_ROOT=${CMAKE_CURRENT_SOURCE_DIR}
```

---

## 11. Best Practices

### 1. Use West for Building

Always prefer `west build` over direct CMake invocation for consistency.

```bash
# Good
west build -b nrf52840dk/nrf52840

# Works, but not recommended
cmake -B build -DBOARD=nrf52840dk/nrf52840
```

### 2. Pristine Builds for Troubleshooting

When in doubt, do a pristine build:

```bash
west build -b nrf52840dk/nrf52840 -p
```

### 3. Separate Build Directories

Use separate build directories for different configurations:

```bash
# Debug build
west build -b nrf52840dk/nrf52840 -d build_debug -- -DCMAKE_BUILD_TYPE=Debug

# Release build
west build -b nrf52840dk/nrf52840 -d build_release -- -DCMAKE_BUILD_TYPE=Release
```

### 4. Version Control

`.gitignore` for Zephyr projects:

```gitignore
# Build directories
build/
build_*/

# Generated files
*.pyc
__pycache__/
.cache/

# IDE files
.vscode/
.idea/

# Keep these
!.vscode/settings.json
```

### 5. Configuration Management

Keep configurations organized:

```
app/
├── prj.conf              # Base config
├── prj_debug.conf        # Debug config
├── prj_release.conf      # Release config
└── board/
    ├── nrf52840dk_nrf52840.conf
    └── stm32f4_disco.conf
```

### 6. Build Scripts

Create build scripts for common tasks:

```bash
#!/bin/bash
# build.sh

BOARD="${1:-nrf52840dk/nrf52840}"
BUILD_TYPE="${2:-Debug}"

echo "Building for board: $BOARD"
echo "Build type: $BUILD_TYPE"

west build -b $BOARD -p -- \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE

echo "Build complete!"
echo "Binary: build/zephyr/zephyr.hex"
```

### 7. Continuous Integration

Example GitHub Actions workflow:

```yaml
name: Build

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Setup Zephyr
        run: |
          west init -l .
          west update
      - name: Build
        run: |
          west build -b nrf52840dk/nrf52840
```

---

## Quick Reference

### Essential Commands

```bash
# Build
west build -b nrf52840dk/nrf52840

# Clean build
west build -p

# Rebuild
west build

# Configure
west build -t menuconfig

# Flash
west flash

# Debug
west debug

# Clean
west build -t clean
```

### Build Output Files

Important files in `build/zephyr/`:

- `zephyr.elf` - ELF binary with debug symbols
- `zephyr.hex` - Intel HEX format (for flashing)
- `zephyr.bin` - Raw binary
- `zephyr.dts` - Final device tree
- `.config` - Final Kconfig configuration
- `zephyr.map` - Memory map

### Environment Check

```bash
# Verify environment
west --version
echo $ZEPHYR_BASE
cmake --version
ninja --version

# Check board support
west boards | grep nrf52840
```

---

## Example: Complete Build Workflow

Here's a complete example of building the nunchuk driver application:

```bash
# 1. Setup environment
cd ~/zephyrproject
source zephyr/zephyr-env.sh

# 2. Navigate to application
cd /home/issam/Zephyr_tutorial/apps/07_nunchuk_driver

# 3. Clean previous build (optional)
rm -rf build

# 4. Build with custom module
west build -b nrf52840dk/nrf52840 -p

# 5. Check build output
ls -lh build/zephyr/zephyr.hex

# 6. Flash to device
west flash

# 7. Monitor output (if using serial)
west attach
# or
minicom -D /dev/ttyACM0 -b 115200
```

---

## Troubleshooting Checklist

- [ ] Zephyr environment sourced (`source zephyr-env.sh`)
- [ ] Board specified (`-b` flag or in CMakeLists.txt)
- [ ] ZEPHYR_BASE environment variable set
- [ ] Custom modules path correct (ZEPHYR_EXTRA_MODULES)
- [ ] Required Kconfig options enabled in prj.conf
- [ ] Device tree overlay syntax correct
- [ ] All source files added to CMakeLists.txt
- [ ] Clean build attempted (`-p` flag)
- [ ] Build directory writable
- [ ] Sufficient disk space

---

## Additional Resources

- [Zephyr Build Configuration](https://docs.zephyrproject.org/latest/build/index.html)
- [West Build Tool](https://docs.zephyrproject.org/latest/develop/west/build-flash-debug.html)
- [CMake in Zephyr](https://docs.zephyrproject.org/latest/build/cmake/index.html)
- [Kconfig Guide](https://docs.zephyrproject.org/latest/build/kconfig/index.html)
- [Device Tree Guide](https://docs.zephyrproject.org/latest/build/dts/index.html)

---

## Conclusion

The Zephyr build system is powerful but can be complex. Key takeaways:

1. **Use west build** for consistency
2. **Pristine builds** solve most issues
3. **Check environment** variables first
4. **Read build errors** carefully - they usually point to the problem
5. **Use verbose mode** (`-v`) when debugging
6. **Keep configurations** organized and version controlled

With practice, building Zephyr applications becomes straightforward. Start simple, add complexity gradually, and always verify each step works before moving forward.
