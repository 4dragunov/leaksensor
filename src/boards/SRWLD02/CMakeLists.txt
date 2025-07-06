##
##   ______                              _
##  / _____)             _              | |
## ( (____  _____ ____ _| |_ _____  ____| |__
##  \____ \| ___ |    (_   _) ___ |/ ___)  _ \
##  _____) ) ____| | | || |_| ____( (___| | | |
## (______/|_____)_|_|_| \__)_____)\____)_| |_|
## (C)2013-2017 Semtech
##  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
##
## License:  Revised BSD License, see LICENSE.TXT file included in the project
## Authors:  Johannes Bruder (STACKFORCE), Miguel Luis (Semtech), Andrey Belyakov ( SombirSoft )
##
project(SRWLD02)
cmake_minimum_required(VERSION 3.6)
enable_language(C ASM CXX)

#---------------------------------------------------------------------------------------
# Target
#---------------------------------------------------------------------------------------
set(MCU STM32WL55xx)
set(BOARD_MCU ${MCU} PARENT_SCOPE)
set(BOARD_MCU ${MCU})
add_compile_definitions(${BOARD_MCU})
add_compile_definitions(STM32WL55xx)
set(SECURE_ELEMENT SOFT_SE PARENT_SCOPE)

include(CMakePrintHelpers)

cmake_print_variables(BOARD_MCU)

list(APPEND ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/adc-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/board.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/delay-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/eeprom-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/gpio-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/i2c-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lpm-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/rtc-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/spi-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/sysIrqHandlers.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/uart-board.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/freertos.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/eeprom.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tickless.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/nonvol.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/sensors-board.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/flash-board.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/eeprom-data.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/modbus_callback.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/stm32_timer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/utilities.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis/arm-gcc/startup_stm32wl55ccux.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis/system_stm32wlxx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/radio_board_if.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_adc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_adc_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_cortex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_dma.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_flash.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_flash_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_i2c.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_pwr.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_pwr_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_rcc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_rcc_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_rtc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_rtc_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_spi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_spi_ex.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_subghz.c"
)

set(MBED_RADIO_SHIELD STM32WL55)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

target_compile_options(${PROJECT_NAME} PUBLIC
	-DUSE_HAL_DRIVER
	-DUSE_BSP_DRIVER
	-DSTM32WL55xx
	-DSTM32_THREAD_SAFE_STRATEGY=4
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=soft
    -fdata-sections
    -ffunction-sections
    -Wall
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-DHARD_FAULT_HANDLER_ENABLED>
)

target_link_options(${PROJECT_NAME} PUBLIC
    -T${CMAKE_SOURCE_DIR}/STM32WL54XX_FLASH_CM4.ld
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=soft
    -specs=nano.specs
    -lc
    -lm
    -lnosys
    -Wl,-Map=${TARGET}.map,--cref
    -Wl,--gc-sections
)


# Add define if debbuger support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)

# Add define if radio debug pins support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/cmsis
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/cmsis/DSP/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/STM32WLxx_HAL_Driver/Inc
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/stm32/cmsis/DSP/Lib/libarm_cortexM4l_math.a)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11 CXX_STANDARD 20)
