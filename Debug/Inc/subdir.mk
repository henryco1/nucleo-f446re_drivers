################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/spi_txonly_arduino_test.c 

OBJS += \
./Inc/spi_txonly_arduino_test.o 

C_DEPS += \
./Inc/spi_txonly_arduino_test.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/spi_txonly_arduino_test.o: ../Inc/spi_txonly_arduino_test.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"E:/Coding/Coding/embedded_c/mcu1-course/mcu1/nucleo-f446re_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Inc/spi_txonly_arduino_test.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

