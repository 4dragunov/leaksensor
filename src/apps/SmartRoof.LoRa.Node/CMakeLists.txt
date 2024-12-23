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
project(SmartRoof.LoRa.Node)

#---------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------
enable_language(C ASM CXX)

# Allow switching of sub projects
set(SUB_PROJECT_LIST LeakDetector)
set(SUB_PROJECT LeakDetector CACHE STRING "Default sub project is LeakDetector")
set_property(CACHE SUB_PROJECT PROPERTY STRINGS ${SUB_PROJECT_LIST})

# Allow LORAWAN_DEFAULT_CLASS overriding for LeakDetector project
if(SUB_PROJECT STREQUAL LeakDetector)
    set(LORAWAN_DEFAULT_CLASS_LIST CLASS_A CLASS_B CLASS_C)
    set(LORAWAN_DEFAULT_CLASS CLASS_A CACHE STRING "Default LoRaWAN class is ClassA")
    set_property(CACHE LORAWAN_DEFAULT_CLASS PROPERTY STRINGS ${LORAWAN_DEFAULT_CLASS_LIST})
endif()

# Allow switching of active region
set(ACTIVE_REGION_LIST   LORAMAC_REGION_EU868 LORAMAC_REGION_US915 LORAMAC_REGION_CN779
    LORAMAC_REGION_EU433 LORAMAC_REGION_AU915 LORAMAC_REGION_AS923 LORAMAC_REGION_CN470
    LORAMAC_REGION_KR920 LORAMAC_REGION_IN865 LORAMAC_REGION_RU864
)
set(ACTIVE_REGION LORAMAC_REGION_EU868 CACHE STRING "Default active region is EU868")
set_property(CACHE ACTIVE_REGION PROPERTY STRINGS ${ACTIVE_REGION_LIST})

if((SUB_PROJECT STREQUAL LeakDetector) AND NOT CLASSB_ENABLED )
    message(FATAL_ERROR "Please turn on Class B support of LoRaMac ( CLASSB_ENABLED=ON ) to use LeakDetector sub projects")
endif()

# Allow selection of secure-element provisioning method
option(SECURE_ELEMENT_PRE_PROVISIONED "Secure-element pre-provisioning" ON)

if(SUB_PROJECT STREQUAL LeakDetector)

    #---------------------------------------------------------------------------------------
    # Application common features handling
    #---------------------------------------------------------------------------------------
    list(APPEND ${PROJECT_NAME}_COMMON
        "${CMAKE_CURRENT_LIST_DIR}/common/CayenneLpp.c"
        "${CMAKE_CURRENT_LIST_DIR}/common/cli.c"
        "${CMAKE_CURRENT_LIST_DIR}/common/LmHandlerMsgDisplay.c"
        "${CMAKE_CURRENT_LIST_DIR}/common/NvmDataMgmt.c"
    )

    #---------------------------------------------------------------------------------------
    # Application LoRaMac handler
    #---------------------------------------------------------------------------------------
    list(APPEND ${PROJECT_NAME}_LMH
        "${CMAKE_CURRENT_LIST_DIR}/common/LmHandler/LmHandler.c"
    )

    #---------------------------------------------------------------------------------------
    # LoRaMac handler applicative packages
    #---------------------------------------------------------------------------------------
    list(APPEND ${PROJECT_NAME}_LMHP
        "${CMAKE_CURRENT_LIST_DIR}/common/LmHandler/packages/FragDecoder.c"
        "${CMAKE_CURRENT_LIST_DIR}/common/LmHandler/packages/LmhpClockSync.c"
        "${CMAKE_CURRENT_LIST_DIR}/common/LmHandler/packages/LmhpCompliance.c"
        "${CMAKE_CURRENT_LIST_DIR}/common/LmHandler/packages/LmhpFragmentation.c"
        "${CMAKE_CURRENT_LIST_DIR}/common/LmHandler/packages/LmhpRemoteMcastSetup.c"
    )


else()
    message(FATAL_ERROR "Unknown SUB_PROJECT")
endif()

#---------------------------------------------------------------------------------------
# Application
#---------------------------------------------------------------------------------------
file(GLOB ${PROJECT_NAME}_SOURCES 
	"${CMAKE_CURRENT_LIST_DIR}/${SUB_PROJECT}/${BOARD}/*.c"
	"${CMAKE_CURRENT_LIST_DIR}/${SUB_PROJECT}/${BOARD}/*.cpp"
	)

add_executable(${PROJECT_NAME}-${SUB_PROJECT}
                            ${${PROJECT_NAME}_COMMON}
                            ${${PROJECT_NAME}_LMH}
                            ${${PROJECT_NAME}_LMHP}
                            ${${PROJECT_NAME}_SOURCES}
                            $<TARGET_OBJECTS:mac>
                            $<TARGET_OBJECTS:system>
                            $<TARGET_OBJECTS:radio>
                            $<TARGET_OBJECTS:peripherals>
                            $<TARGET_OBJECTS:${BOARD}>
)

target_compile_definitions(${PROJECT_NAME}-${SUB_PROJECT} PRIVATE $<$<BOOL:${CLASSB_ENABLED}>:LORAMAC_CLASSB_ENABLED>)
target_compile_definitions(${PROJECT_NAME}-${SUB_PROJECT} PRIVATE ACTIVE_REGION=${ACTIVE_REGION})
target_compile_definitions(${PROJECT_NAME}-${SUB_PROJECT} PRIVATE $<$<CONFIG:Debug>:DEBUG>)

if(SUB_PROJECT STREQUAL LeakDetector)
    target_compile_definitions(${PROJECT_NAME}-${SUB_PROJECT} PRIVATE LORAWAN_DEFAULT_CLASS=${LORAWAN_DEFAULT_CLASS}
    $<$<CONFIG:Debug>:DEBUG>
#	-D_USE_32BIT_TIME_T 
)

endif()
if(${SECURE_ELEMENT_PRE_PROVISIONED} MATCHES ON)
    target_compile_definitions(${PROJECT_NAME}-${SUB_PROJECT} PRIVATE -DSECURE_ELEMENT_PRE_PROVISIONED)
endif()
if(${SECURE_ELEMENT} MATCHES SOFT_SE)
    target_compile_definitions(${PROJECT_NAME}-${SUB_PROJECT} PRIVATE -DSOFT_SE)
endif()
target_compile_definitions(${PROJECT_NAME}-${SUB_PROJECT}  PUBLIC
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:mac,INTERFACE_COMPILE_DEFINITIONS>>
)

target_include_directories(${PROJECT_NAME}-${SUB_PROJECT} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/common/LmHandler
    ${CMAKE_CURRENT_SOURCE_DIR}/common/LmHandler/packages
    ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_PROJECT}/${BOARD}
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:mac,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:${BOARD},INTERFACE_INCLUDE_DIRECTORIES>>
)

set_property(TARGET ${PROJECT_NAME}-${SUB_PROJECT} PROPERTY C_STANDARD 11 CXX_STANDARD 17)

target_link_libraries(${PROJECT_NAME}-${SUB_PROJECT} m)

#---------------------------------------------------------------------------------------
# Debugging and Binutils
#---------------------------------------------------------------------------------------

include(gdb-helper)
include(binutils-arm-none-eabi)

# Generate debugger configurations
generate_run_gdb_stlink(${PROJECT_NAME}-${SUB_PROJECT})
generate_run_gdb_openocd(${PROJECT_NAME}-${SUB_PROJECT})
generate_vscode_launch_openocd(${PROJECT_NAME}-${SUB_PROJECT})

# Print section sizes of target
print_section_sizes(${PROJECT_NAME}-${SUB_PROJECT})

# Create output in hex and binary format
create_bin_output(${PROJECT_NAME}-${SUB_PROJECT})
create_hex_output(${PROJECT_NAME}-${SUB_PROJECT})
