################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../drivers/Startup/startup_stm32f446retx.s 

OBJS += \
./drivers/Startup/startup_stm32f446retx.o 


# Each subdirectory must supply rules for building sources it contributes
drivers/Startup/%.o: ../drivers/Startup/%.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

