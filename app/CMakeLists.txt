cmake_minimum_required(VERSION 3.10)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("Build type: "              ${CMAKE_BUILD_TYPE})

#
# Core project settings
#
project(PowerController C CXX ASM)
enable_language(C CXX ASM)

#
# Core MCU flags, CPU, instruction set and FPU setup
# Needs to be set properly for your MCU
#
set(CPU_PARAMETERS
    -mthumb
    -mthumb-interwork 
    -ffunction-sections 
    -fdata-sections 
    -g 
    -fno-common 
    -fmessage-length=0
    -mcpu=cortex-m0
    -mfloat-abi=soft
)

# Set linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F030C8TX_FLASH.ld)
#
# List of source files to compile
#
file(GLOB_RECURSE SOURCES "Core/*.*" "Drivers/*.*" "Middlewares/*.*")
#
# Include directories
#
set(INCLUDE_DIRS
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3
    Drivers/CMSIS/Device/ST/STM32F0xx/Include
    Drivers/CMSIS/Include
    Core/ThreadSafe
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/Third_Party/FreeRTOS/c++/include
    Middlewares/Third_Party/FreeRTOS/c/include
    Middlewares/Third_Party/segger
    Middlewares/Third_Party/tinyfsm/include
    )
#
# Symbols definition
#
set(DEFINES
$<$<CONFIG:Debug>:DEBUG>
USE_HAL_DRIVER
STM32F030x8
CPP_FREERTOS_NO_EXCEPTIONS
STM32_THREAD_SAFE_STRATEGY=2
)

set(EXECUTABLE ${CMAKE_PROJECT_NAME})
# Executable files
add_executable(${EXECUTABLE} ${SOURCES} ${LINKER_SCRIPT})
# Include paths
target_include_directories(${EXECUTABLE} PRIVATE ${INCLUDE_DIRS})
# Project symbols
target_compile_definitions(${EXECUTABLE} PRIVATE ${DEFINES})

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    -fstack-usage 
    
    # Debug configuration
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    # Release configuration
    $<$<CONFIG:Release>:-Os>
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    -T${LINKER_SCRIPT}
    ${CPU_PARAMETERS}
    -Wl,--cref
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nosys.specs
    -Wl,--gc-sections -static
    -Wl,--print-memory-usage
)

# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)
