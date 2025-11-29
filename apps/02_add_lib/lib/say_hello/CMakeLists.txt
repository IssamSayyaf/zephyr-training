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