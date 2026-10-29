cmake_minimum_required(VERSION 3.13)

# Pull in SDK from environment or parent
if (DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
else()
    include(../../pico_sdk_import.cmake OPTIONAL)
endif()

project(subsystem2_standalone C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(test_subsystem2
    src/motor_driver.c
    src/encoder_driver.c
    src/test_main.c
)

target_include_directories(test_subsystem2 PRIVATE
    include
)

target_link_libraries(test_subsystem2
    pico_stdlib
    hardware_pwm
    hardware_sync
    hardware_gpio
    hardware_irq
)

# Route logging strictly to USB CDC (keeps GP1 free from UART pulses)
pico_enable_stdio_usb(test_subsystem2 1)
pico_enable_stdio_uart(test_subsystem2 0)

pico_add_extra_outputs(test_subsystem2)