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

# One-variable ablations for on-board measurements. These are comparisons with
# the final build, not cumulative speedups and not substitutes for old firmware.
set(LAB_BENCH_VARIANT "full" CACHE STRING "Benchmark variant")
set_property(CACHE LAB_BENCH_VARIANT PROPERTY STRINGS full full_scan single_core pixel_stamps reference_normal no_draw_cache no_text_cache)
set(LAB_INITIAL_BALL_COUNT 20000 CACHE STRING "Initial number of balls")
if(NOT LAB_INITIAL_BALL_COUNT MATCHES "^[0-9]+$")
    message(FATAL_ERROR "LAB_INITIAL_BALL_COUNT must be a non-negative integer")
endif()
option(LAB_CAPACITY_CALIBRATION "Run the startup capacity search for benchmarks" OFF)
set(LAB_CALIBRATION_WARMUP_FRAMES 600 CACHE STRING "Frames before timing each trial")
set(LAB_CALIBRATION_FRAMES 600 CACHE STRING "Timed frames per capacity trial")
set(LAB_BENCH_DEFINITIONS
    full_scan:LAB_FULL_PEG_SCAN
    single_core:LAB_SINGLE_CORE
    pixel_stamps:LAB_GENERIC_CIRCLE
    reference_normal:LAB_REFERENCE_NORMAL
    no_draw_cache:LAB_NO_DRAW_CACHE
    no_text_cache:LAB_NO_TEXT_CACHE)
set(LAB_VARIANT_FOUND FALSE)
if(LAB_BENCH_VARIANT STREQUAL "full")
    set(LAB_VARIANT_FOUND TRUE)
endif()
foreach(pair IN LISTS LAB_BENCH_DEFINITIONS)
    string(REPLACE ":" ";" parts "${pair}")
    list(GET parts 0 variant)
    list(GET parts 1 flag)
    if(LAB_BENCH_VARIANT STREQUAL variant)
        target_compile_definitions(VGA_Animation_Demo PRIVATE ${flag}=1)
        set(LAB_VARIANT_FOUND TRUE)
    endif()
endforeach()
if(NOT LAB_VARIANT_FOUND)
    message(FATAL_ERROR "Unknown LAB_BENCH_VARIANT: ${LAB_BENCH_VARIANT}")
endif()
target_compile_definitions(VGA_Animation_Demo PRIVATE
    LAB_BENCH_VARIANT="${LAB_BENCH_VARIANT}"
    LAB_INITIAL_BALL_COUNT=${LAB_INITIAL_BALL_COUNT}
    LAB_CALIBRATION_WARMUP_FRAMES=${LAB_CALIBRATION_WARMUP_FRAMES}
    LAB_CALIBRATION_FRAMES=${LAB_CALIBRATION_FRAMES})
if(LAB_CAPACITY_CALIBRATION)
    target_compile_definitions(VGA_Animation_Demo PRIVATE LAB_CAPACITY_CALIBRATION=1)
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
