cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico-stepper-dds C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
add_executable(pico-stepper-dds
    main.c
)

pico_generate_pio_header(pico-stepper-dds ${CMAKE_CURRENT_LIST_DIR}/timer.pio)

target_sources(pico-stepper-dds PRIVATE
        main.c
        )

target_link_libraries(pico-stepper-dds PRIVATE
        pico_stdlib
        hardware_pio
        )

pico_enable_stdio_usb(pico-stepper-dds 1)

pico_add_extra_outputs(pico-stepper-dds)

