################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l476rgtx.s 

OBJS += \
./Core/Startup/startup_stm32l476rgtx.o 

S_DEPS += \
./Core/Startup/startup_stm32l476rgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/Components/Common" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/Components/stspin220" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/STM32L4xx_Nucleo" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/X-NUCLEO-IHMxx" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l476rgtx.d ./Core/Startup/startup_stm32l476rgtx.o

.PHONY: clean-Core-2f-Startup

