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
set(PICO_BOARD pico2 CACHE STRING "Board type")

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(PWM_Demo C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(PWM_Demo)

# must match with executable name and source file names
target_sources(PWM_Demo PRIVATE
    pwm_demo.c
    mpu6050.c
    VGA/vga16_graphics_v3.c
)

# Assemble the VGA PIO programs and generate their headers.
pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/hsync.pio)
pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/vsync.pio)
pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb.pio)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(PWM_Demo
    pico_stdlib
    pico_multicore
    pico_bootsel_via_double_reset
    hardware_pwm
    hardware_irq
    hardware_dma
    hardware_adc
    hardware_pio
    hardware_i2c
    hardware_clocks
)

# create map/bin/hex file etc.
pico_add_extra_outputs(PWM_Demo)
