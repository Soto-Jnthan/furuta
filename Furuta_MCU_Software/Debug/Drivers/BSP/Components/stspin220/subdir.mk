################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/stspin220/stspin220.c 

OBJS += \
./Drivers/BSP/Components/stspin220/stspin220.o 

C_DEPS += \
./Drivers/BSP/Components/stspin220/stspin220.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/stspin220/%.o Drivers/BSP/Components/stspin220/%.su Drivers/BSP/Components/stspin220/%.cyclo: ../Drivers/BSP/Components/stspin220/%.c Drivers/BSP/Components/stspin220/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/Components/Common" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/Components/stspin220" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/STM32L4xx_Nucleo" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/X-NUCLEO-IHMxx" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-stspin220

clean-Drivers-2f-BSP-2f-Components-2f-stspin220:
	-$(RM) ./Drivers/BSP/Components/stspin220/stspin220.cyclo ./Drivers/BSP/Components/stspin220/stspin220.d ./Drivers/BSP/Components/stspin220/stspin220.o ./Drivers/BSP/Components/stspin220/stspin220.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-stspin220

