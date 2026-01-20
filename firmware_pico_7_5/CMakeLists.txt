# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(mcu_firmware C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(pio_spi_flash)

pico_generate_pio_header(pio_spi_flash ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_sources(pio_spi_flash PRIVATE
        spi_flash.c
        pio_spi.c
        pio_spi.h
        )

target_link_libraries(pio_spi_flash PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(pio_spi_flash)

add_executable(mcu_firmware)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(mcu_firmware 1)
pico_enable_stdio_usb(mcu_firmware 1)

pico_generate_pio_header(mcu_firmware ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_sources(mcu_firmware PRIVATE
        firmware.c
        pio_spi.c
        pio_spi.h
        soft_spi.c
        soft_spi.h
        )

target_link_libraries(mcu_firmware PRIVATE pico_stdlib hardware_pio hardware_watchdog)
pico_add_extra_outputs(mcu_firmware)


