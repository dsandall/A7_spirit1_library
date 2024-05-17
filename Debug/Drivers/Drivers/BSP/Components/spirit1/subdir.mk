################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Drivers/BSP/Components/spirit1/radio_target.c 

OBJS += \
./Drivers/Drivers/BSP/Components/spirit1/radio_target.o 

C_DEPS += \
./Drivers/Drivers/BSP/Components/spirit1/radio_target.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Drivers/BSP/Components/spirit1/%.o Drivers/Drivers/BSP/Components/spirit1/%.su Drivers/Drivers/BSP/Components/spirit1/%.cyclo: ../Drivers/Drivers/BSP/Components/spirit1/%.c Drivers/Drivers/BSP/Components/spirit1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/thebu/STM32CubeIDE/workspace_anotherNewRTOS/A7_spirit1_library/Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Inc" -I"C:/Users/thebu/STM32CubeIDE/workspace_anotherNewRTOS/A7_spirit1_library/Drivers/Drivers/BSP/Components/spirit1" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Drivers-2f-BSP-2f-Components-2f-spirit1

clean-Drivers-2f-Drivers-2f-BSP-2f-Components-2f-spirit1:
	-$(RM) ./Drivers/Drivers/BSP/Components/spirit1/radio_target.cyclo ./Drivers/Drivers/BSP/Components/spirit1/radio_target.d ./Drivers/Drivers/BSP/Components/spirit1/radio_target.o ./Drivers/Drivers/BSP/Components/spirit1/radio_target.su

.PHONY: clean-Drivers-2f-Drivers-2f-BSP-2f-Components-2f-spirit1

