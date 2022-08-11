################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU ARM Embedded (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/modbus/mb.c 

C_DEPS += \
./Drivers/modbus/mb.d 

OBJS += \
./Drivers/modbus/mb.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/modbus/%.o Drivers/modbus/%.su: ../Drivers/modbus/%.c Drivers/modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/modbus/include -I../Drivers/modbus/port -I../Drivers/modbus/rtu -I../Drivers/modbus/ascii -I"E:/stm32_repositary/STM32Cube_FW_G4_V1.5.0/Drivers/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-modbus

clean-Drivers-2f-modbus:
	-$(RM) ./Drivers/modbus/mb.d ./Drivers/modbus/mb.o ./Drivers/modbus/mb.su

.PHONY: clean-Drivers-2f-modbus

