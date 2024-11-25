################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Middlewares/Third_Party/segger/SEGGER_RTT_ASM_ARMv7M.S 

C_SRCS += \
../Middlewares/Third_Party/segger/SEGGER_RTT.c \
../Middlewares/Third_Party/segger/SEGGER_RTT_printf.c 

C_DEPS += \
./Middlewares/Third_Party/segger/SEGGER_RTT.d \
./Middlewares/Third_Party/segger/SEGGER_RTT_printf.d 

OBJS += \
./Middlewares/Third_Party/segger/SEGGER_RTT.o \
./Middlewares/Third_Party/segger/SEGGER_RTT_ASM_ARMv7M.o \
./Middlewares/Third_Party/segger/SEGGER_RTT_printf.o 

S_UPPER_DEPS += \
./Middlewares/Third_Party/segger/SEGGER_RTT_ASM_ARMv7M.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/segger/%.o Middlewares/Third_Party/segger/%.su Middlewares/Third_Party/segger/%.cyclo: ../Middlewares/Third_Party/segger/%.c Middlewares/Third_Party/segger/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DVECT_TAB_OFFSET=0x4000 -DVECT_TAB_BASE_ADDRESS=FLASH_BASE -DUSE_HAL_DRIVER -DSTM32F103xE -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Core/ThreadSafe -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/SX1278/driver" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/inc" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/segger/SysView" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/segger" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/tinyfsm/include" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/Wire" -Og -ffunction-sections -fdata-sections -mslow-flash-data -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/Third_Party/segger/%.o: ../Middlewares/Third_Party/segger/%.S Middlewares/Third_Party/segger/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/SX1278/driver" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/inc" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/segger/SysView" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/segger" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/tinyfsm/include" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/Wire" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Middlewares-2f-Third_Party-2f-segger

clean-Middlewares-2f-Third_Party-2f-segger:
	-$(RM) ./Middlewares/Third_Party/segger/SEGGER_RTT.cyclo ./Middlewares/Third_Party/segger/SEGGER_RTT.d ./Middlewares/Third_Party/segger/SEGGER_RTT.o ./Middlewares/Third_Party/segger/SEGGER_RTT.su ./Middlewares/Third_Party/segger/SEGGER_RTT_ASM_ARMv7M.d ./Middlewares/Third_Party/segger/SEGGER_RTT_ASM_ARMv7M.o ./Middlewares/Third_Party/segger/SEGGER_RTT_printf.cyclo ./Middlewares/Third_Party/segger/SEGGER_RTT_printf.d ./Middlewares/Third_Party/segger/SEGGER_RTT_printf.o ./Middlewares/Third_Party/segger/SEGGER_RTT_printf.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-segger

