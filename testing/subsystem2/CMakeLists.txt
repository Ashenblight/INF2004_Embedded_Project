cmake_minimum_required(VERSION 3.13)

# Include Pico SDK tools
include(pico_sdk_import.cmake)

project(inf2004_motion_control C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(motion_subsystem
    src/motor_driver.c
    src/encoder_driver.c
    src/motion_control.c
    # Teammate main & RTOS kernel sources added here
)

target_include_directories(motion_subsystem PRIVATE
    include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include
)

target_link_libraries(motion_subsystem
    pico_stdlib
    hardware_pwm
    hardware_sync
    hardware_gpio
    hardware_irq
)

# Crucial Architectural Mitigation: Route console to USB CDC, NOT UART0 (protects GP1)
pico_enable_stdio_usb(motion_subsystem 1)
pico_enable_stdio_uart(motion_subsystem 0)

pico_add_extra_outputs(motion_subsystem)