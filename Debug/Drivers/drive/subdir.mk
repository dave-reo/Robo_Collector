################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/drive/motor_driver.c 

OBJS += \
./Drivers/drive/motor_driver.o 

C_DEPS += \
./Drivers/drive/motor_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/drive/%.o Drivers/drive/%.su Drivers/drive/%.cyclo: ../Drivers/drive/%.c Drivers/drive/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Mechatronics_Labs/ME-507/TP/Github_Repo/Robo_Collector/Drivers/drive" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-drive

clean-Drivers-2f-drive:
	-$(RM) ./Drivers/drive/motor_driver.cyclo ./Drivers/drive/motor_driver.d ./Drivers/drive/motor_driver.o ./Drivers/drive/motor_driver.su

.PHONY: clean-Drivers-2f-drive

