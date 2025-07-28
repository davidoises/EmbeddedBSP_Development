# Define processor-specific parameters
set(CPU_PARAMETERS
    -mcpu=cortex-m4
    -mthumb
)

# set(target_mcu_family "STM32F4")

list(APPEND MACRO_DEFINES
    STM32F446xx
)

# Define include directories
list(APPEND INCS
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cmsis-device-f4/Include
)

# Define source files 
list(APPEND SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/app/config/${target_board}/startup.c
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cmsis-device-f4/Source/system_stm32f4xx.c
)