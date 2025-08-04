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
project(peripherals)
cmake_minimum_required(VERSION 3.10)

#---------------------------------------------------------------------------------------
# Target
#---------------------------------------------------------------------------------------



if(${SECURE_ELEMENT} MATCHES SOFT_SE)
	message(STATUS "SOFT_SE - SECURE ELEMENT")
    file(GLOB SE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/soft-se/*.c")
    list(APPEND ${PROJECT_NAME}_SOURCES ${SE_SOURCES})
else()
    message(FATAL_ERROR "No secure-element selected.")
endif()

if(${BOARD_OWI} MATCHES OWI_UART)
	message(STATUS "Onewire uart driver added")	
    file(GLOB OWI_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/onewire/*.c" "${CMAKE_CURRENT_SOURCE_DIR}/onewire/*.cpp")
    list(APPEND ${PROJECT_NAME}_SOURCES ${OWI_SOURCES})
endif()

if(${BOARD_GAUGE} MATCHES BQ35100)
	message(STATUS "Bq35100 battery gauge driver added")
    list(APPEND ${PROJECT_NAME}_SOURCES  "${CMAKE_CURRENT_SOURCE_DIR}/gauge/BQ35100.cpp")
endif()

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})
message(STATUS ${${PROJECT_NAME}_SOURCES})	

if(${SECURE_ELEMENT_PRE_PROVISIONED} MATCHES ON)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DSECURE_ELEMENT_PRE_PROVISIONED)
endif()

if(${SECURE_ELEMENT} MATCHES SOFT_SE)
    target_include_directories( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/soft-se)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DSOFT_SE)
endif()

if(${BOARD_OWI} MATCHES OWI_UART)
	target_compile_definitions(${PROJECT_NAME} PRIVATE -DOWIUART)
	target_include_directories( ${PROJECT_NAME} PUBLIC	 ${CMAKE_CURRENT_SOURCE_DIR}/onewire )
endif()

if(${BOARD_GAUGE} MATCHES BQ35100)
	target_compile_definitions(${PROJECT_NAME} PRIVATE -DBQ35100)
	target_include_directories( ${PROJECT_NAME} PUBLIC	 ${CMAKE_CURRENT_SOURCE_DIR}/gauge )
endif()

target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:mac,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
