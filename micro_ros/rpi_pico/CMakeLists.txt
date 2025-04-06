cmake_minimum_required(VERSION 3.12)

set(PICO_PLATFORM rp2040)
set(PICO_BOARD pico)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_hackathon C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

option(USE_IMU "Enable IMU functionality" ON)

if(USE_IMU)
    add_definitions(-DUSE_IMU=1)
    set(SOURCES ${SOURCES} bno085_imu.c)
endif()


link_directories(libmicroros)

add_executable(pico_micro_ros_hackathon
    pico_micro_ros_hackathon.c
    pico_uart_transport.c
    motor_controller.c  # Add this line for Motors
    ${SOURCES}
)
target_link_libraries(pico_micro_ros_hackathon
    pico_stdlib
    microros
    hardware_i2c
    hardware_pwm
    hardware_gpio
)

target_include_directories(pico_micro_ros_hackathon PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_micro_ros_hackathon 1)
pico_enable_stdio_uart(pico_micro_ros_hackathon 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_micro_ros_hackathon)
