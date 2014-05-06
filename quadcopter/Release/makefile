################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include dev/src/servo/subdir.mk
-include dev/src/quad/subdir.mk
-include dev/src/motor/subdir.mk
-include dev/src/hal/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
quadcopter.lss \

FLASH_IMAGE += \
quadcopter.hex \

EEPROM_IMAGE += \
quadcopter.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: quadcopter.elf secondary-outputs

# Tool invocations
quadcopter.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,quadcopter.map -mmcu=atmega2560 -o "quadcopter.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

quadcopter.lss: quadcopter.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S quadcopter.elf  >"quadcopter.lss"
	@echo 'Finished building: $@'
	@echo ' '

quadcopter.hex: quadcopter.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex quadcopter.elf  "quadcopter.hex"
	@echo 'Finished building: $@'
	@echo ' '

quadcopter.eep: quadcopter.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex quadcopter.elf  "quadcopter.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: quadcopter.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega2560 quadcopter.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) quadcopter.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
