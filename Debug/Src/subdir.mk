################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/cli.c \
../Src/dma.c \
../Src/gpio.c \
../Src/main.c \
../Src/serial.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/tim.c \
../Src/timer.c \
../Src/usart.c 

OBJS += \
./Src/adc.o \
./Src/cli.o \
./Src/dma.o \
./Src/gpio.o \
./Src/main.o \
./Src/serial.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/tim.o \
./Src/timer.o \
./Src/usart.o 

C_DEPS += \
./Src/adc.d \
./Src/cli.d \
./Src/dma.d \
./Src/gpio.d \
./Src/main.d \
./Src/serial.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/tim.d \
./Src/timer.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_FULL_LL_DRIVER '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DSTM32F407xx '-DHSE_VALUE=8000000' '-DHSE_STARTUP_TIMEOUT=100' '-DLSE_STARTUP_TIMEOUT=5000' '-DLSE_VALUE=32768' '-DEXTERNAL_CLOCK_VALUE=12288000' '-DHSI_VALUE=16000000' '-DLSI_VALUE=32000' '-DVDD_VALUE=3300' '-DPREFETCH_ENABLE=1' '-DINSTRUCTION_CACHE_ENABLE=1' '-DDATA_CACHE_ENABLE=1' -I"/Users/sanyv/code/AC6/407-LL/Inc" -I"/Users/sanyv/code/AC6/407-LL/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/Users/sanyv/code/AC6/407-LL/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/sanyv/code/AC6/407-LL/Drivers/CMSIS/Include"  -O2 -Wall -fmessage-length=0 -ffunction-sections -fdata-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


