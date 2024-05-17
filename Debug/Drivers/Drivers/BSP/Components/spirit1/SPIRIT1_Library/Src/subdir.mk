################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Aes.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Calibration.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Commands.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Csma.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_DirectRF.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_General.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Gpio.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Irq.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_LinearFifo.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Management.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktBasic.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktCommon.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktMbus.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktStack.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Qi.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Radio.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Timer.c \
../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Types.c 

OBJS += \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Aes.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Calibration.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Commands.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Csma.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_DirectRF.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_General.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Gpio.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Irq.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_LinearFifo.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Management.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktBasic.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktCommon.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktMbus.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktStack.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Qi.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Radio.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Timer.o \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Types.o 

C_DEPS += \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Aes.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Calibration.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Commands.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Csma.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_DirectRF.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_General.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Gpio.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Irq.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_LinearFifo.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Management.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktBasic.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktCommon.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktMbus.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktStack.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Qi.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Radio.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Timer.d \
./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Types.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/%.o Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/%.su Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/%.cyclo: ../Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/%.c Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/thebu/STM32CubeIDE/workspace_anotherNewRTOS/A7_spirit1_library/Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Inc" -I"C:/Users/thebu/STM32CubeIDE/workspace_anotherNewRTOS/A7_spirit1_library/Drivers/Drivers/BSP/Components/spirit1" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Drivers-2f-BSP-2f-Components-2f-spirit1-2f-SPIRIT1_Library-2f-Src

clean-Drivers-2f-Drivers-2f-BSP-2f-Components-2f-spirit1-2f-SPIRIT1_Library-2f-Src:
	-$(RM) ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Aes.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Aes.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Aes.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Aes.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Calibration.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Calibration.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Calibration.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Calibration.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Commands.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Commands.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Commands.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Commands.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Csma.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Csma.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Csma.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Csma.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_DirectRF.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_DirectRF.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_DirectRF.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_DirectRF.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_General.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_General.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_General.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_General.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Gpio.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Gpio.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Gpio.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Gpio.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Irq.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Irq.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Irq.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Irq.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_LinearFifo.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_LinearFifo.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_LinearFifo.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_LinearFifo.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Management.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Management.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Management.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Management.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktBasic.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktBasic.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktBasic.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktBasic.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktCommon.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktCommon.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktCommon.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktCommon.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktMbus.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktMbus.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktMbus.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktMbus.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktStack.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktStack.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktStack.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_PktStack.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Qi.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Qi.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Qi.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Qi.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Radio.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Radio.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Radio.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Radio.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Timer.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Timer.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Timer.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Timer.su ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Types.cyclo ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Types.d ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Types.o ./Drivers/Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/SPIRIT_Types.su

.PHONY: clean-Drivers-2f-Drivers-2f-BSP-2f-Components-2f-spirit1-2f-SPIRIT1_Library-2f-Src

