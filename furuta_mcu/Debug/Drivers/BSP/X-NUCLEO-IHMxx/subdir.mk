################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm06a1_stm32l4xx.c \
../Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.c 

OBJS += \
./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm06a1_stm32l4xx.o \
./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.o 

C_DEPS += \
./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm06a1_stm32l4xx.d \
./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/X-NUCLEO-IHMxx/%.o Drivers/BSP/X-NUCLEO-IHMxx/%.su Drivers/BSP/X-NUCLEO-IHMxx/%.cyclo: ../Drivers/BSP/X-NUCLEO-IHMxx/%.c Drivers/BSP/X-NUCLEO-IHMxx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/Components/Common" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/Components/stspin220" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/STM32L4xx_Nucleo" -I"C:/Users/JnthnS/STM32CubeIDE/workspace_1.12.0/Furuta_MCU_Software/Drivers/BSP/X-NUCLEO-IHMxx" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-IHMxx

clean-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-IHMxx:
	-$(RM) ./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm06a1_stm32l4xx.cyclo ./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm06a1_stm32l4xx.d ./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm06a1_stm32l4xx.o ./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihm06a1_stm32l4xx.su ./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.cyclo ./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.d ./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.o ./Drivers/BSP/X-NUCLEO-IHMxx/x_nucleo_ihmxx.su

.PHONY: clean-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-IHMxx

