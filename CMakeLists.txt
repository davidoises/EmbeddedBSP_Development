cmake_minimum_required(VERSION 3.15)

# Toolchain parameters.
# CACHE INTERNAL FORCE - this means the variable will persist accross runs and will be forced to be
# the specified value.
set(CMAKE_TOOLCHAIN_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/toolchain_conf.cmake
    CACHE INTERNAL "" FORCE
)

##### 1. Set variables for the board and MCU to be used
set(target_board "samv71-evalkit" CACHE STRING "Target board name")
if(target_board STREQUAL "samv71-evalkit")
    set(target_mcu "ATSAMV71Q21B")
    # set(target_mcu_family "SAMV71")
elseif(target_board STREQUAL "samd21-evalkit")
    set(target_mcu "ATSAMD21G18A")
    # set(target_mcu_family "SAMD21")
elseif(target_board STREQUAL "NUCLEO-F446RE")
    set(target_mcu "STM32F446RE")
    # set(target_mcu_family "STM32F4")
else()
    message(FATAL_ERROR "Unknown target_board: ${target_board}")
endif()
message(STATUS "Target Board is ${target_board}")
message(STATUS "Target MCU is ${target_mcu}")

##### 2. Create the top level project and global variables for compiling.

# Project name
project(embedded_app C ASM)

# Define export compile commands for debugging
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define the executable name as the project name
set(EXECUTABLE ${CMAKE_PROJECT_NAME})

set(MACRO_DEFINES "")

##### 2.1 Selection of common include directories and source files

set(INCS
    ${CMAKE_CURRENT_SOURCE_DIR}/app
)

# Define source files 
set(SRCS  
    ${CMAKE_CURRENT_SOURCE_DIR}/app/main.c
    
)

##### 3. Include board and mcu specific compilation files.

# Include the board cmake (which includes the mcu cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/board_${target_board}.cmake)

##### 4. Use board and MCU specifics to build the whole project.

set(BUILD_LD_FLAGS
    ${CPU_PARAMETERS}
)
list(APPEND BUILD_LD_FLAGS
    -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
)
set(BUILD_LD_SPECS
    nosys
    nano
)

# Executable files 
add_executable(${EXECUTABLE}  
    ${SRCS}
)

# Add header directories  
target_include_directories(${EXECUTABLE} PRIVATE  
    ${INCS}
)

# Macro defines used in code
foreach(macro_define ${MACRO_DEFINES})
    target_compile_definitions(${EXECUTABLE} PRIVATE ${macro_define})
endforeach()

# Target-specific compilation options 
target_compile_options(${EXECUTABLE} PRIVATE  
    ${CPU_PARAMETERS})   

# Target-specific linking options 
target_link_options(${EXECUTABLE} PRIVATE  
    -T ${LINKER_SCRIPT}  
    ${BUILD_LD_FLAGS}
)

foreach(specs ${BUILD_LD_SPECS})
    target_link_options(${EXECUTABLE} PRIVATE --specs=${specs}.specs)
endforeach()

##### 5. Extra outputs to generate
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            ${CMAKE_PROJECT_NAME}.elf
            ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generate hex file"
    VERBATIM
)

add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            ${CMAKE_PROJECT_NAME}.elf
            ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generate binary file"
    VERBATIM
)