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
project(radio)
cmake_minimum_required(VERSION 3.10)

#---------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------

# Allow switching of radios
set(RADIO_LIST sx1272 sx1276 sx126x lr1110 stm32wl5xx)
set(RADIO sx1272 CACHE STRING "Default radio is sx1272")
set_property(CACHE RADIO PROPERTY STRINGS ${RADIO_LIST})
set_property(CACHE RADIO PROPERTY ADVANCED)
get_filename_component(DIR_ONE_ABOVE ../ ABSOLUTE)

#---------------------------------------------------------------------------------------
# Radio drivers
#---------------------------------------------------------------------------------------
if(${RADIO} STREQUAL lr1110)
    list(APPEND ${PROJECT_NAME}_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/radio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src/lr1110_bootloader.c
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src/lr1110_crypto_engine.c
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src/lr1110_driver_version.c
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src/lr1110_gnss.c
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src/lr1110_radio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src/lr1110_regmem.c
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src/lr1110_system.c
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src/lr1110_wifi.c
    )
elseif(${RADIO} STREQUAL sx126x)
    list(APPEND ${PROJECT_NAME}_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/sx126x/radio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/sx126x/sx126x.c
    )
elseif(${RADIO} STREQUAL sx1272)
    list(APPEND ${PROJECT_NAME}_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/sx1272/sx1272.c
    )
elseif(${RADIO} STREQUAL sx1276)
    list(APPEND ${PROJECT_NAME}_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/sx1276/sx1276.c
    )
elseif(${RADIO} STREQUAL stm32wl5xx)
    list(APPEND ${PROJECT_NAME}_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl5x/stm32_radio_driver/radio_driver.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl5x/stm32_radio_driver/radio_fw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl5x/stm32_radio_driver/radio.c
    )
else()
    message(FATAL_ERROR "Unsupported radio driver selected...")
endif()

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

add_dependencies(${PROJECT_NAME} board)

if(${RADIO} STREQUAL lr1110 )
    list(APPEND ${PROJECT_NAME}_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110
        ${CMAKE_CURRENT_SOURCE_DIR}/lr1110/lr1110_driver/src
    )
elseif(${RADIO} STREQUAL sx126x)
    list(APPEND ${PROJECT_NAME}_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/sx126x
    )
elseif(${RADIO} STREQUAL sx1272)
    list(APPEND ${PROJECT_NAME}_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/sx1272
    )
elseif(${RADIO} STREQUAL sx1276)
    list(APPEND ${PROJECT_NAME}_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/sx1276
    )
elseif(${RADIO} STREQUAL stm32wl5xx)
target_compile_definitions(${PROJECT_NAME} PUBLIC  STM32WL55xx USE_HAL_DRIVER USE_BSP_DRIVER)
message(STATUS "RADIO_HAL ${DIR_ONE_ABOVE}/boards/mcu/stm32/STM32WLxx_HAL_Driver/Inc")
    list(APPEND ${PROJECT_NAME}_INCLUDES
       ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl5x
       ${DIR_ONE_ABOVE}/boards/mcu/stm32/STM32WLxx_HAL_Driver/Inc
    )
else()
    message(FATAL_ERROR "Unsupported radio driver selected...")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${${PROJECT_NAME}_INCLUDES}
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
)

option(USE_RADIO_DEBUG "Enable Radio Debug GPIO's" OFF)
target_compile_definitions(${PROJECT_NAME} PUBLIC  $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)
target_compile_definitions(${PROJECT_NAME} PUBLIC  $<IF:$<STREQUAL:${BOARD_CLOCK_SOURCE},TXCO>,RF_CLOCK_SOURCE_TXCO,RF_CLOCK_SOURCE_XTAL>)
target_include_directories(${PROJECT_NAME} PUBLIC $<TARGET_PROPERTY:${BOARD},INTERFACE_INCLUDE_DIRECTORIES>)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
