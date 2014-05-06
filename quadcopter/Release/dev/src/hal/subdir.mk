################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../dev/src/hal/accelerometer.c \
../dev/src/hal/gyro.c \
../dev/src/hal/i2c.c \
../dev/src/hal/input.c \
../dev/src/hal/kalman.c \
../dev/src/hal/pwm.c \
../dev/src/hal/servo.c \
../dev/src/hal/time.c \
../dev/src/hal/timer.c \
../dev/src/hal/uart.c \
../dev/src/hal/wiring.c 

OBJS += \
./dev/src/hal/accelerometer.o \
./dev/src/hal/gyro.o \
./dev/src/hal/i2c.o \
./dev/src/hal/input.o \
./dev/src/hal/kalman.o \
./dev/src/hal/pwm.o \
./dev/src/hal/servo.o \
./dev/src/hal/time.o \
./dev/src/hal/timer.o \
./dev/src/hal/uart.o \
./dev/src/hal/wiring.o 

C_DEPS += \
./dev/src/hal/accelerometer.d \
./dev/src/hal/gyro.d \
./dev/src/hal/i2c.d \
./dev/src/hal/input.d \
./dev/src/hal/kalman.d \
./dev/src/hal/pwm.d \
./dev/src/hal/servo.d \
./dev/src/hal/time.d \
./dev/src/hal/timer.d \
./dev/src/hal/uart.d \
./dev/src/hal/wiring.d 


# Each subdirectory must supply rules for building sources it contributes
dev/src/hal/%.o: ../dev/src/hal/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/opt/quadcopter_lenticia/quadcopter/dev/include" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega2560 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


