cmake_minimum_required(VERSION 3.13)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake")
    include("${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake")
elseif(DEFINED ENV{PICO_SDK_PATH} AND EXISTS "$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
else()
    message(FATAL_ERROR "Set PICO_SDK_PATH or copy pico_sdk_import.cmake into this project.")
endif()

project(priority_event_telemetry_protocol C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(priority_event_telemetry
    main.c
    protocol.c
)

target_include_directories(priority_event_telemetry PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(priority_event_telemetry
    pico_stdlib
)

pico_enable_stdio_usb(priority_event_telemetry 1)
pico_enable_stdio_uart(priority_event_telemetry 0)
pico_add_extra_outputs(priority_event_telemetry)
