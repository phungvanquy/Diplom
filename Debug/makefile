################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Diplom.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Diplom.list \

OBJCOPY_BIN += \
Diplom.bin \


# All Target
all: Diplom.elf secondary-outputs

# Tool invocations
Diplom.elf: $(OBJS) $(USER_OBJS) C:\Users\phung\OneDrive\Diplom\Code_Proteus\Code\Diplom\STM32F411RETX_FLASH.ld
	arm-none-eabi-gcc -o "Diplom.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\phung\OneDrive\Diplom\Code_Proteus\Code\Diplom\STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Diplom.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Diplom.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Diplom.list"
	@echo 'Finished building: $@'
	@echo ' '

Diplom.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Diplom.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
