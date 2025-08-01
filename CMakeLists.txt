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
include(pico_sdk_import.cmake)
project(SPISDCARD C CXX ASM)
pico_sdk_init()

set(FREERTOS_KERNEL_PATH "C:/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

include_directories( ${CMAKE_SOURCE_DIR}/lib )
add_subdirectory(lib/FatFs_SPI)    

add_executable(${PROJECT_NAME}  
        main.c
        hw_config.c
        lib/ssd1306.c
        lib/sdspicard.c
        lib/mpu6050.c
        )

    

target_link_libraries(${PROJECT_NAME} 
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        pico_stdlib 
        pico_multicore
        FatFs_SPI
        hardware_clocks
        hardware_adc
        hardware_i2c
        hardware_pwm
        hardware_pio
        )

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/lib/pioconfig.pio)
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})



