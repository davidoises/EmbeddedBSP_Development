################################################################################
# Automatically-generated file. Do not edit!
################################################################################

ifdef SystemRoot
	SHELL = cmd.exe
	MK_DIR = mkdir
else
	ifeq ($(shell uname), Linux)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), CYGWIN)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), MINGW32)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), MINGW64)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), DARWIN)
		MK_DIR = mkdir -p
	endif
endif

# List the subdirectories for creating object files
SUB_DIRS +=  \
.. \
../../xdk-asf/samv71/cmsis/samv71/source \
../../xdk-asf/samv71/syscalls/gcc

# SUB_DIRS +=  \
#  \
# hpl/uart \
# hpl/afec \
# hpl/pmc \
# samv71b/gcc/gcc \
# hal/src \
# hpl/xdmac \
# samv71b/gcc \
# hal/utils/src \
# examples \
# hpl/core

# List the object files
OBJS +=  \
../../xdk-asf/samv71/cmsis/samv71/source/startup_samv71.o \
../main.o \
../../xdk-asf/samv71/syscalls/gcc/syscalls.o

# OBJS +=  \
# hal/src/hal_io.o \
# samv71b/gcc/gcc/startup_samv71q21b.o \
# atmel_start.o \
# hpl/uart/hpl_uart.o \
# hal/utils/src/utils_syscalls.o \
# hal/src/hal_delay.o \
# hpl/core/hpl_init.o \
# hal/utils/src/utils_list.o \
# hal/utils/src/utils_assert.o \
# hpl/afec/hpl_afec.o \
# hal/src/hal_usart_sync.o \
# hpl/pmc/hpl_pmc.o \
# samv71b/gcc/system_samv71q21b.o \
# hal/src/hal_init.o \
# main.o \
# examples/driver_examples.o \
# driver_init.o \
# hal/src/hal_adc_sync.o \
# hal/utils/src/utils_ringbuffer.o \
# hal/src/hal_gpio.o \
# hal/utils/src/utils_event.o \
# hal/src/hal_sleep.o \
# hpl/pmc/hpl_sleep.o \
# hpl/core/hpl_core_m7_base.o \
# hal/src/hal_atomic.o \
# hpl/xdmac/hpl_xdmac.o

OBJS_AS_ARGS +=  \
"../../xdk-asf/samv71/cmsis/samv71/source/startup_samv71.o" \
"../main.o" \
"../../xdk-asf/samv71/syscalls/gcc/syscalls.o"

# OBJS_AS_ARGS +=  \
# "hal/src/hal_io.o" \
# "samv71b/gcc/gcc/startup_samv71q21b.o" \
# "atmel_start.o" \
# "hpl/uart/hpl_uart.o" \
# "hal/utils/src/utils_syscalls.o" \
# "hal/src/hal_delay.o" \
# "hpl/core/hpl_init.o" \
# "hal/utils/src/utils_list.o" \
# "hal/utils/src/utils_assert.o" \
# "hpl/afec/hpl_afec.o" \
# "hal/src/hal_usart_sync.o" \
# "hpl/pmc/hpl_pmc.o" \
# "samv71b/gcc/system_samv71q21b.o" \
# "hal/src/hal_init.o" \
# "main.o" \
# "examples/driver_examples.o" \
# "driver_init.o" \
# "hal/src/hal_adc_sync.o" \
# "hal/utils/src/utils_ringbuffer.o" \
# "hal/src/hal_gpio.o" \
# "hal/utils/src/utils_event.o" \
# "hal/src/hal_sleep.o" \
# "hpl/pmc/hpl_sleep.o" \
# "hpl/core/hpl_core_m7_base.o" \
# "hal/src/hal_atomic.o" \
# "hpl/xdmac/hpl_xdmac.o"

# List the directories containing header files
DIR_INCLUDES +=  \
-I"." \
-I"../../xdk-asf/common" \
-I"../../xdk-asf/common/interrupt" \
-I"../../xdk-asf/samv71" \
-I"../../xdk-asf/samv71/fpu" \
-I"../../xdk-asf/samv71/preprocessor" \
-I"../../xdk-asf/samv71/header_files" \
-I"../../xdk-asf/samv71/cmsis/samv71/include" \
-I"../../xdk-asf/samv71/cmsis/samv71/source" \
-I"../../xdk-asf/thirdparty/cmsis/include"

# DIR_INCLUDES +=  \
# -I"../" \
# -I"../config" \
# -I"../examples" \
# -I"../hal/include" \
# -I"../hal/utils/include" \
# -I"../hpl/afec" \
# -I"../hpl/core" \
# -I"../hpl/pio" \
# -I"../hpl/pmc" \
# -I"../hpl/uart" \
# -I"../hpl/xdmac" \
# -I"../hri" \
# -I"../" \
# -I"../CMSIS/Core/Include" \
# -I"../samv71b/include"

# List the dependency files
DEPS := $(OBJS:%.o=%.d)

DEPS_AS_ARGS +=  \
"../../xdk-asf/samv71/cmsis/samv71/source/startup_samv71.d" \
"../main.d" \
"../../xdk-asf/samv71/syscalls/gcc/syscalls.d"

# DEPS_AS_ARGS +=  \
# "hal/src/hal_gpio.d" \
# "hal/src/hal_io.d" \
# "samv71b/gcc/gcc/startup_samv71q21b.d" \
# "hpl/uart/hpl_uart.d" \
# "hal/utils/src/utils_syscalls.d" \
# "hpl/xdmac/hpl_xdmac.d" \
# "hal/utils/src/utils_list.d" \
# "hal/utils/src/utils_assert.d" \
# "hal/src/hal_delay.d" \
# "hpl/core/hpl_init.d" \
# "samv71b/gcc/system_samv71q21b.d" \
# "hal/src/hal_init.d" \
# "hal/src/hal_usart_sync.d" \
# "hpl/afec/hpl_afec.d" \
# "hpl/pmc/hpl_pmc.d" \
# "driver_init.d" \
# "hal/src/hal_adc_sync.d" \
# "main.d" \
# "examples/driver_examples.d" \
# "hal/src/hal_sleep.d" \
# "hal/utils/src/utils_ringbuffer.d" \
# "hal/utils/src/utils_event.d" \
# "atmel_start.d" \
# "hal/src/hal_atomic.d" \
# "hpl/core/hpl_core_m7_base.d" \
# "hpl/pmc/hpl_sleep.d"

OUTPUT_FILE_NAME :=application_samv71
QUOTE := "
OUTPUT_FILE_PATH +=$(OUTPUT_FILE_NAME).elf
OUTPUT_FILE_PATH_AS_ARGS +=$(OUTPUT_FILE_NAME).elf

vpath %.c ../
vpath %.s ../
vpath %.S ../

# All Target
all: $(SUB_DIRS) $(OUTPUT_FILE_PATH)

# Linker target

$(OUTPUT_FILE_PATH): $(OBJS)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker
	$(QUOTE)arm-none-eabi-gcc$(QUOTE) -o $(OUTPUT_FILE_NAME).elf $(OBJS_AS_ARGS) -Wl,--start-group -lm -Wl,--end-group -mthumb \
-Wl,-Map="$(OUTPUT_FILE_NAME).map" --specs=nano.specs -Wl,--gc-sections -mcpu=cortex-m7 \
 \
-T"../../xdk-asf/samd21/linker_scripts/samv71q21b_flash.ld" \
-L"../../xdk-asf/samd21/linker_scripts"
	@echo Finished building target: $@

	"arm-none-eabi-objcopy" -O binary "$(OUTPUT_FILE_NAME).elf" "$(OUTPUT_FILE_NAME).bin"
	"arm-none-eabi-objcopy" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  \
        "$(OUTPUT_FILE_NAME).elf" "$(OUTPUT_FILE_NAME).hex"
	"arm-none-eabi-objcopy" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma \
        .eeprom=0 --no-change-warnings -O binary "$(OUTPUT_FILE_NAME).elf" \
        "$(OUTPUT_FILE_NAME).eep" || exit 0
	"arm-none-eabi-objdump" -h -S "$(OUTPUT_FILE_NAME).elf" > "$(OUTPUT_FILE_NAME).lss"
	"arm-none-eabi-size" "$(OUTPUT_FILE_NAME).elf"



# Compiler targets




%.o: %.c
	@echo Building file: $<
	@echo ARM/GNU C Compiler
	$(QUOTE)arm-none-eabi-gcc$(QUOTE) -x c -mthumb -DDEBUG -Os -ffunction-sections -mlong-calls -g3 -Wall -c -std=gnu99 \
-D__SAMV71Q21B__ -mcpu=cortex-m7 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
$(DIR_INCLUDES) \
-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -o "$@" "$<"
	@echo Finished building: $<

%.o: %.s
	@echo Building file: $<
	@echo ARM/GNU Assembler
	$(QUOTE)arm-none-eabi-as$(QUOTE) -x c -mthumb -DDEBUG -Os -ffunction-sections -mlong-calls -g3 -Wall -c -std=gnu99 \
-D__SAMV71Q21B__ -mcpu=cortex-m7 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
$(DIR_INCLUDES) \
-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -o "$@" "$<"
	@echo Finished building: $<

%.o: %.S
	@echo Building file: $<
	@echo ARM/GNU Preprocessing Assembler
	$(QUOTE)arm-none-eabi-gcc$(QUOTE) -x c -mthumb -DDEBUG -Os -ffunction-sections -mlong-calls -g3 -Wall -c -std=gnu99 \
-D__SAMV71Q21B__ -mcpu=cortex-m7 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
$(DIR_INCLUDES) \
-MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -o "$@" "$<"
	@echo Finished building: $<

# Detect changes in the dependent files and recompile the respective object files.
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(DEPS)),)
-include $(DEPS)
endif
endif

$(SUB_DIRS):
	$(MK_DIR) "$@"

clean:
	rm -f $(OBJS_AS_ARGS)
	rm -f $(OUTPUT_FILE_PATH)
	rm -f $(DEPS_AS_ARGS)
	rm -f $(OUTPUT_FILE_NAME).a $(OUTPUT_FILE_NAME).hex $(OUTPUT_FILE_NAME).bin \
        $(OUTPUT_FILE_NAME).lss $(OUTPUT_FILE_NAME).eep $(OUTPUT_FILE_NAME).map \
        $(OUTPUT_FILE_NAME).srec
