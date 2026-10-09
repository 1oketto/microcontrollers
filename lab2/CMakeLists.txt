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
project(VGA_Animation_Demo C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(VGA_Animation_Demo)

target_compile_options(VGA_Animation_Demo PRIVATE -O3)

set(LAB_INITIAL_BALL_COUNT 25000 CACHE STRING "Initial number of balls")
if(NOT LAB_INITIAL_BALL_COUNT MATCHES "^[0-9]+$")
    message(FATAL_ERROR "LAB_INITIAL_BALL_COUNT must be a non-negative integer")
endif()
option(LAB_COLLISION_PROFILE "Report average collision-search work every 120 frames" OFF)
target_compile_definitions(VGA_Animation_Demo PRIVATE
    LAB_INITIAL_BALL_COUNT=${LAB_INITIAL_BALL_COUNT})
if(LAB_COLLISION_PROFILE)
    target_compile_definitions(VGA_Animation_Demo PRIVATE LAB_COLLISION_PROFILE=1)
endif()

# must match with pio filename and executable name from above
pico_generate_pio_header(VGA_Animation_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/hsync.pio)
pico_generate_pio_header(VGA_Animation_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/vsync.pio)
pico_generate_pio_header(VGA_Animation_Demo ${CMAKE_CURRENT_LIST_DIR}/VGA/rgb.pio)

# must match with executable name and source file names
target_sources(VGA_Animation_Demo PRIVATE animation.c VGA/vga16_graphics_v3.c)

# must match with executable name
target_link_libraries(  VGA_Animation_Demo
                        PRIVATE
                        pico_stdlib
                        pico_divider
                        pico_multicore
                        pico_sync
                        pico_bootsel_via_double_reset
                        hardware_pio
                        hardware_dma
                        hardware_spi
                        hardware_adc
                        hardware_irq
                        hardware_clocks
                        hardware_vreg
                        hardware_pll)

# USB serial exposes the BENCH/CAPACITY records without an external UART adapter.
pico_enable_stdio_usb(VGA_Animation_Demo 1)
pico_enable_stdio_uart(VGA_Animation_Demo 1)

# must match with executable name
pico_add_extra_outputs(VGA_Animation_Demo)
