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
    set(PICO_BOARD pico2)
endif()

pico_sdk_init()

add_executable(bridge_echo
    bridge_echo.c
    usb_descriptors.c
)

# Lets tusb.h find our tusb_config.h in this directory.
target_include_directories(bridge_echo PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(bridge_echo
    pico_stdlib
    pico_unique_id
    tinyusb_device
)

# We own the USB stack directly via tinyusb_device — do not let the
# SDK's pico_stdio_usb claim it.
pico_enable_stdio_usb(bridge_echo 0)
pico_enable_stdio_uart(bridge_echo 0)

# Generate .uf2, .bin, .hex, etc.
pico_add_extra_outputs(bridge_echo)
