# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Copy pico_sdk_import.cmake next to this CMakeLists.txt once, the same
# way you did for bridge_echo. (Source: $PICO_SDK_PATH/external/.)
include(pico_sdk_import.cmake)

project(smoke_nrf24 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Default to the Pico 2 (RP2350). Override with -DPICO_BOARD=pico for
# RP2040 if you're running the smoke test on the original Pico.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

pico_sdk_init()

add_executable(smoke_nrf24
    smoke_nrf24.c
)

target_link_libraries(smoke_nrf24
    pico_stdlib
    hardware_spi
    hardware_gpio
)

# Use the SDK's stdio_usb for printf — this firmware doesn't need the
# low-latency raw TinyUSB path that bridge_echo uses.
pico_enable_stdio_usb(smoke_nrf24 1)
pico_enable_stdio_uart(smoke_nrf24 0)

pico_add_extra_outputs(smoke_nrf24)
