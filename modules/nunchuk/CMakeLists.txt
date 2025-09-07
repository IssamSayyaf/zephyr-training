# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Add include directory for the module
zephyr_include_directories(include)

# Add driver if enabled
if(CONFIG_NUNCHUK_DRIVER)
  add_subdirectory(drivers)
endif()