################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/i2c_master_rx_testIT.c \
../Src/sysmem.c 

OBJS += \
./Src/i2c_master_rx_testIT.o \
./Src/sysmem.o 

C_DEPS += \
./Src/i2c_master_rx_testIT.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/i2c_master_rx_testIT.o: ../Src/i2c_master_rx_testIT.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"E:/Coding/Coding/embedded_c/mcu1-course/mcu1/nucleo-f446re_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/i2c_master_rx_testIT.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"E:/Coding/Coding/embedded_c/mcu1-course/mcu1/nucleo-f446re_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

