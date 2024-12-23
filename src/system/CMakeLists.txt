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
## Authors:  Johannes Bruder (STACKFORCE), Miguel Luis (Semtech)
##
cmake_minimum_required(VERSION 3.10)
project(system)

enable_language(C ASM CXX)

#---------------------------------------------------------------------------------------
# Target
#---------------------------------------------------------------------------------------
set(FREERTOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS")
set(SEGGER_RTT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Segger")
set(SEGGER_SYSVIEW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Segger/SysView")
set(FSM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tinyfsm")

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${FREERTOS_DIR}/Source/*.c"
    "${FREERTOS_DIR}/Source/portable/GCC/ARM_CM3/*.c"
    "${FREERTOS_DIR}/Source/CMSIS_RTOS/*.c"
    "${SEGGER_RTT_DIR}/*.c"
    "${SEGGER_RTT_DIR}/*.s"
    "${SEGGER_SYSVIEW_DIR}/*.c"
)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    
    ${FREERTOS_DIR}
    ${FREERTOS_DIR}/Source/include
    ${FREERTOS_DIR}/Source/portable/GCC/ARM_CM3
    ${FREERTOS_DIR}/Source/CMSIS_RTOS
    ${SEGGER_RTT_DIR}
    ${SEGGER_SYSVIEW_DIR}
    ${FSM_DIR}/include
    $<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
)
