################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU ARM Embedded (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/modbus/functions/mbfunccoils.c \
../Drivers/modbus/functions/mbfuncdiag.c \
../Drivers/modbus/functions/mbfuncdisc.c \
../Drivers/modbus/functions/mbfuncholding.c \
../Drivers/modbus/functions/mbfuncinput.c \
../Drivers/modbus/functions/mbfuncother.c \
../Drivers/modbus/functions/mbutils.c 

C_DEPS += \
./Drivers/modbus/functions/mbfunccoils.d \
./Drivers/modbus/functions/mbfuncdiag.d \
./Drivers/modbus/functions/mbfuncdisc.d \
./Drivers/modbus/functions/mbfuncholding.d \
./Drivers/modbus/functions/mbfuncinput.d \
./Drivers/modbus/functions/mbfuncother.d \
./Drivers/modbus/functions/mbutils.d 

OBJS += \
./Drivers/modbus/functions/mbfunccoils.o \
./Drivers/modbus/functions/mbfuncdiag.o \
./Drivers/modbus/functions/mbfuncdisc.o \
./Drivers/modbus/functions/mbfuncholding.o \
./Drivers/modbus/functions/mbfuncinput.o \
./Drivers/modbus/functions/mbfuncother.o \
./Drivers/modbus/functions/mbutils.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/modbus/functions/%.o Drivers/modbus/functions/%.su: ../Drivers/modbus/functions/%.c Drivers/modbus/functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/modbus/include -I../Drivers/modbus/port -I../Drivers/modbus/rtu -I../Drivers/modbus/ascii -I"E:/stm32_repositary/STM32Cube_FW_G4_V1.5.0/Drivers/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-modbus-2f-functions

clean-Drivers-2f-modbus-2f-functions:
	-$(RM) ./Drivers/modbus/functions/mbfunccoils.d ./Drivers/modbus/functions/mbfunccoils.o ./Drivers/modbus/functions/mbfunccoils.su ./Drivers/modbus/functions/mbfuncdiag.d ./Drivers/modbus/functions/mbfuncdiag.o ./Drivers/modbus/functions/mbfuncdiag.su ./Drivers/modbus/functions/mbfuncdisc.d ./Drivers/modbus/functions/mbfuncdisc.o ./Drivers/modbus/functions/mbfuncdisc.su ./Drivers/modbus/functions/mbfuncholding.d ./Drivers/modbus/functions/mbfuncholding.o ./Drivers/modbus/functions/mbfuncholding.su ./Drivers/modbus/functions/mbfuncinput.d ./Drivers/modbus/functions/mbfuncinput.o ./Drivers/modbus/functions/mbfuncinput.su ./Drivers/modbus/functions/mbfuncother.d ./Drivers/modbus/functions/mbfuncother.o ./Drivers/modbus/functions/mbfuncother.su ./Drivers/modbus/functions/mbutils.d ./Drivers/modbus/functions/mbutils.o ./Drivers/modbus/functions/mbutils.su

.PHONY: clean-Drivers-2f-modbus-2f-functions

