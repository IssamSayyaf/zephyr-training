# SPDX-License-Identifier: Apache-2.0

# Specify minimum CMake version required
cmake_minimum_required(VERSION 3.20.0)

# Set the target board to Raspberry Pi Pico
set(BOARD nrf52840dk/nrf52840)
# Find the Zephyr package using the ZEPHYR_BASE environment variable
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Name the project
project(hello_blink)

# Include the library (via directory)
add_subdirectory(lib/say_hello)

# Locate the source code for the application
# Assumes add_executable(app "")
target_sources(app PRIVATE src/main.c)

# Link the say_hello library to the application
target_link_libraries(app PRIVATE hello_lib)