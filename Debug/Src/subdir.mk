################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/board.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/sx1272.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/board.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/sx1272.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/board.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/sx1272.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DSTM32 -DARM_MATH_CM4 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -I"/Users/rad/Dropbox/Research/sensing_board/CMSIS/core" -I"/Users/rad/Dropbox/Research/sensing_board/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/Users/rad/Dropbox/Research/sensing_board/Utilities/STM32F4xx-Nucleo" -I"/Users/rad/Dropbox/Research/sensing_board/Inc" -I"/Users/rad/Dropbox/Research/sensing_board/Drivers/CMSIS/Include" -I"/Users/rad/Dropbox/Research/sensing_board/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/rad/Dropbox/Research/sensing_board/CMSIS/Device" -I"/Users/rad/Dropbox/Research/sensing_board/Drivers/STM32F4xx_HAL_Driver/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


