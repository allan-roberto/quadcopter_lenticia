################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../dev/src/quad/main.c 

OBJS += \
./dev/src/quad/main.o 

C_DEPS += \
./dev/src/quad/main.d 


# Each subdirectory must supply rules for building sources it contributes
dev/src/quad/%.o: ../dev/src/quad/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/opt/quadcopter_lenticia/quadcopter/dev/include" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


