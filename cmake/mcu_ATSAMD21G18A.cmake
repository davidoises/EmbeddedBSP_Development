# Define processor-specific parameters
set(CPU_PARAMETERS
    -mcpu=cortex-m0plus
    -mthumb
)

set(target_mcu_family "samd21")

list(APPEND MACRO_DEFINES
    __SAMD21G18A__
)

# Define include directories
list(APPEND INCS
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/interfaces
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/src/${target_mcu_family}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/common
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/common/interrupt
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/${target_mcu_family}
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/${target_mcu_family}/preprocessor
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/${target_mcu_family}/header_files
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/${target_mcu_family}/cmsis/${target_mcu_family}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/${target_mcu_family}/cmsis/${target_mcu_family}/source
)

# Define source files 
list(APPEND SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/src/${target_mcu_family}/drivers/adc_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/src/${target_mcu_family}/drivers/pio_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/src/${target_mcu_family}/drivers/pmc_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/src/${target_mcu_family}/drivers/uart_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/${target_mcu_family}/cmsis/${target_mcu_family}/source/startup_${target_mcu_family}.c
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/${target_mcu_family}/syscalls/gcc/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/xdk-asf/common/interrupt/interrupt_sam_nvic.c
)