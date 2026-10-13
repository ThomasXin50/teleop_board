# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
if(NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR "Set PICO_SDK_PATH to your local Raspberry Pi Pico SDK checkout.")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(andre_rp2040_controller C CXX ASM)
pico_sdk_init()

set(CONTROLLER_SOURCES
    main.c
    src/ackermann.c
    src/crc32.c
    src/hal.c
    src/pid.c
    src/protocol.c
    src/rpm.c
    src/stepper.c
    src/vehicle.c
)

# andre_rp2040_controller is the vehicle firmware. The _bench build turns off
# the throttle interlocks and runs steering open loop (BENCH_MODE in config.h);
# it is for bench checks only.
foreach(target andre_rp2040_controller andre_rp2040_controller_bench)
    add_executable(${target} ${CONTROLLER_SOURCES})
    target_include_directories(${target} PRIVATE src)
    target_link_libraries(${target}
        pico_stdlib
        hardware_adc
        hardware_clocks
        hardware_pwm
        hardware_sync
        hardware_watchdog
        m
    )
    target_compile_options(${target} PRIVATE -Wall -Wextra)

    pico_enable_stdio_usb(${target} 1)
    pico_enable_stdio_uart(${target} 0)
    pico_add_extra_outputs(${target})
endforeach()
target_compile_definitions(andre_rp2040_controller_bench PRIVATE BENCH_MODE=1)