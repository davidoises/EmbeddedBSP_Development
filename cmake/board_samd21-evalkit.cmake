# Specify the location of the linker script
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker_scripts/samd21g18a_flash.ld)

list(APPEND MACRO_DEFINES
    SAMD21_EVALKIT
)

# Define include directories 
list(APPEND INCS
    ${CMAKE_CURRENT_SOURCE_DIR}/app/config/${target_board}
)

# Define source files. No source files for this board
# list(APPEND SRCS
#     ${CMAKE_CURRENT_SOURCE_DIR}/app/config/${target_board}/pio_config.c
# )

# Include MCU/CPU specifics now
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/mcu_${target_mcu}.cmake)