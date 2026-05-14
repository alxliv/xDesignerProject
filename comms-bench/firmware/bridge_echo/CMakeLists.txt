cmake_minimum_required(VERSION 3.13)

# The Pico SDK ships pico_sdk_import.cmake under $PICO_SDK_PATH/external/.
# Copy that file next to this CMakeLists.txt (one-time setup) before
# running cmake — see README.md in the comms-bench root.
include(pico_sdk_import.cmake)

project(bridge_echo C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Default to PICO_BOARD=pico (RP2040). Override on the cmake command
# line with -DPICO_BOARD=pico2 for the Pico 2 / RP2350.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico)
endif()

pico_sdk_init()

add_executable(bridge_echo
    bridge_echo.c
)

target_link_libraries(bridge_echo
    pico_stdlib
    tinyusb_device
)

# USB-CDC stdio on, UART stdio off — we want the USB serial to be the
# only port the laptop talks to.
pico_enable_stdio_usb(bridge_echo 1)
pico_enable_stdio_uart(bridge_echo 0)

# Generate .uf2, .bin, .hex, etc.
pico_add_extra_outputs(bridge_echo)
