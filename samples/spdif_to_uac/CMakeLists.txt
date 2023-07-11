cmake_minimum_required(VERSION 3.12)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

set(project_name "spdif_to_uac" C CXX ASM)
project(${project_name})
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(../.. spdif_rx)
add_subdirectory(../lib/pico_audio_32b pico_audio_32b) # for extended audio.h

# select sample application here (choose only one)
add_executable(${PROJECT_NAME}
    main.c
    usb_descriptors.c
    usb_microphone.c
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${PROJECT_NAME} PRIVATE pico_stdlib pico_rand spdif_rx tinyusb_device tinyusb_board)

# set project resource assignment for PIO and DMA_IRQ to avoid confliction
add_compile_definitions(PICO_SPDIF_RX_PIO=0)
add_compile_definitions(PICO_SPDIF_RX_DMA_IRQ=0)

pico_add_extra_outputs(${PROJECT_NAME}) # create map/bin/hex/uf2 files
