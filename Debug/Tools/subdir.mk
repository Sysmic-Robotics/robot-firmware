################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Tools/MAX5814.c \
../Tools/MAX581x.c \
../Tools/PID.c \
../Tools/board.c \
../Tools/encoder.c \
../Tools/motor.c \
../Tools/nrf24.c \
../Tools/vl6180x.c 

OBJS += \
./Tools/MAX5814.o \
./Tools/MAX581x.o \
./Tools/PID.o \
./Tools/board.o \
./Tools/encoder.o \
./Tools/motor.o \
./Tools/nrf24.o \
./Tools/vl6180x.o 

C_DEPS += \
./Tools/MAX5814.d \
./Tools/MAX581x.d \
./Tools/PID.d \
./Tools/board.d \
./Tools/encoder.d \
./Tools/motor.d \
./Tools/nrf24.d \
./Tools/vl6180x.d 


# Each subdirectory must supply rules for building sources it contributes
Tools/%.o Tools/%.su Tools/%.cyclo: ../Tools/%.c Tools/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Pablo/Documents/Sysmic/robot-firmware/Tools" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Tools

clean-Tools:
	-$(RM) ./Tools/MAX5814.cyclo ./Tools/MAX5814.d ./Tools/MAX5814.o ./Tools/MAX5814.su ./Tools/MAX581x.cyclo ./Tools/MAX581x.d ./Tools/MAX581x.o ./Tools/MAX581x.su ./Tools/PID.cyclo ./Tools/PID.d ./Tools/PID.o ./Tools/PID.su ./Tools/board.cyclo ./Tools/board.d ./Tools/board.o ./Tools/board.su ./Tools/encoder.cyclo ./Tools/encoder.d ./Tools/encoder.o ./Tools/encoder.su ./Tools/motor.cyclo ./Tools/motor.d ./Tools/motor.o ./Tools/motor.su ./Tools/nrf24.cyclo ./Tools/nrf24.d ./Tools/nrf24.o ./Tools/nrf24.su ./Tools/vl6180x.cyclo ./Tools/vl6180x.d ./Tools/vl6180x.o ./Tools/vl6180x.su

.PHONY: clean-Tools

