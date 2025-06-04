# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(mqtt_client_dht11 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(mqtt_client_dht11 mqtt_client_dht11.c )

pico_set_program_name(mqtt_client_dht11 "mqtt_client_dht11")
pico_set_program_version(mqtt_client_dht11 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(mqtt_client_dht11 1)
pico_enable_stdio_usb(mqtt_client_dht11 1)

# Add the standard library to the build
target_link_libraries(mqtt_client_dht11
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    pico_mbedtls
    pico_lwip_mbedtls
    )

# Add the standard include files to the build
target_include_directories(mqtt_client_dht11 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/lib
)

pico_add_extra_outputs(mqtt_client_dht11)
