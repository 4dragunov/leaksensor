################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_hw.c \
../Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_slave.c \
../Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_utils.c 

C_DEPS += \
./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_hw.d \
./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_slave.d \
./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_utils.d 

OBJS += \
./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_hw.o \
./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_slave.o \
./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_utils.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/%.o Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/%.su Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/%.cyclo: ../Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/%.c Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DVECT_TAB_OFFSET=0x4000 -DVECT_TAB_BASE_ADDRESS=FLASH_BASE -DUSE_HAL_DRIVER -DSTM32F103xE -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Core/ThreadSafe -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/SX1278/driver" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/inc" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/segger/SysView" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/segger" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/tinyfsm/include" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/Wire" -Og -ffunction-sections -fdata-sections -mslow-flash-data -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-STM32F3xx_FreeRTOS_ModbusRTU_RS485-2f-src

clean-Middlewares-2f-Third_Party-2f-STM32F3xx_FreeRTOS_ModbusRTU_RS485-2f-src:
	-$(RM) ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_hw.cyclo ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_hw.d ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_hw.o ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_hw.su ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_slave.cyclo ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_slave.d ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_slave.o ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_slave.su ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_utils.cyclo ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_utils.d ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_utils.o ./Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/src/ModbusRTU_utils.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-STM32F3xx_FreeRTOS_ModbusRTU_RS485-2f-src

