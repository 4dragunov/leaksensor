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
## Authors:  Johannes Bruder (STACKFORCE), Miguel Luis (Semtech), Andrey Belyakov (SimbirSoft)
##

#---------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------

# Allow switching of target platform
set(BOARD_LIST SRWLD01)
set(BOARD SRWLD01 CACHE STRING "Default target platform is SRWLD01")
set_property(CACHE BOARD PROPERTY STRINGS ${BOARD_LIST})

# Allow switching of board radios
set(BOARD_RADIO_LIST SX1276MB1LAS SX1276MB1MAS )
set(BOARD_RADIO SX1261MBXBAS CACHE STRING "Default board radio is SX1261MBXBAS")
set_property(CACHE BOARD_RADIO PROPERTY STRINGS ${BOARD_RADIO_LIST})

# Allow switching of secure-elements
set(SECURE_ELEMENT_LIST SOFT_SE LR1110_SE ATECC608A_TNGLORA_SE)
set(SECURE_ELEMENT SOFT_SE CACHE STRING "Default secure element is SOFT_SE")
set_property(CACHE SECURE_ELEMENT PROPERTY STRINGS ${SECURE_ELEMENT_LIST})

# Allow switching of Applications
set(APPLICATION_LIST LeakDetector )
set(APPLICATION LeakDetector CACHE STRING "Default Application is LeakDetector")
set_property(CACHE APPLICATION PROPERTY STRINGS ${APPLICATION_LIST})

# Switch for USB-Uart support, enable it for some Applications who needs it.
option(USE_USB_CDC "Use USB-Uart" OFF)

# Switch for debugger support.
option(USE_DEBUGGER "Use Debugger" ON)

# Switch for Class B support of LoRaMac.
option(CLASSB_ENABLED "Class B support of LoRaMac" OFF)

#---------------------------------------------------------------------------------------
# Target Boards
#---------------------------------------------------------------------------------------

if(BOARD STREQUAL SRWLD01)
    # Configure toolchain for stm32f103
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/boards/SRWLD01/cmsis/arm-gcc/stm32f103xe_flash.ld)
    include(stm32f1)

    # Build platform specific board implementation
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/SRWLD01)

    # Configure radio
    if(BOARD_RADIO STREQUAL SX1272MB2DAS)
        set(RADIO sx1272 CACHE INTERNAL "Radio sx1272 selected")
    elseif(BOARD_RADIO STREQUAL SX1276MB1LAS OR BOARD_RADIO STREQUAL SX1276MB1MAS)
        set(RADIO sx1276 CACHE INTERNAL "Radio sx1276 selected")
    elseif(BOARD_RADIO STREQUAL SX1261MBXBAS OR BOARD_RADIO STREQUAL SX1262MBXCAS OR BOARD_RADIO STREQUAL SX1262MBXDAS)
        set(RADIO sx126x CACHE INTERNAL "Radio sx126x selected")
    elseif(BOARD_RADIO STREQUAL LR1110MB1XXS)
        set(RADIO lr1110 CACHE INTERNAL "Radio lr1110 selected")
    else()
        message(STATUS "Please specify the BOARD_RADIO!\nPossible values are: SX1276MB1LAS, SX1276MB1MAS")
    endif()

endif()

#---------------------------------------------------------------------------------------
# General Components
#---------------------------------------------------------------------------------------

add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")

set(BOARD_MCU "")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards)

add_compile_definitions(${BOARD_MCU})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/radio)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/system)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/mac)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/peripherals)

#---------------------------------------------------------------------------------------
# Applications
#---------------------------------------------------------------------------------------

if(APPLICATION STREQUAL LeakDetector)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps/SmartRoof.LoRa.Node)

endif()
