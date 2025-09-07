# SPDX-License-Identifier: Apache-2.0

if(CONFIG_NUNCHUK_DRIVER)
  # Add the driver subdirectory
  add_subdirectory_ifdef(CONFIG_NUNCHUK_DRIVER ${CMAKE_CURRENT_SOURCE_DIR}/../drivers)
  
  # Add include directories so apps can find our headers
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
endif()