################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/ThreadSafe/newlib_lock_glue.c 

C_DEPS += \
./Core/ThreadSafe/newlib_lock_glue.d 

OBJS += \
./Core/ThreadSafe/newlib_lock_glue.o 


# Each subdirectory must supply rules for building sources it contributes
Core/ThreadSafe/%.o Core/ThreadSafe/%.su Core/ThreadSafe/%.cyclo: ../Core/ThreadSafe/%.c Core/ThreadSafe/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSER_VECT_TAB_ADDRESS -DVECT_TAB_OFFSET=0x4000 -DVECT_TAB_BASE_ADDRESS=FLASH_BASE -DUSE_HAL_DRIVER -DSTM32F103xE -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Core/ThreadSafe -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/SX1278/driver" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/STM32F3xx_FreeRTOS_ModbusRTU_RS485/inc" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/segger/SysView" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/segger" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/tinyfsm/include" -I"C:/Users/Andrey/git/smart.roof/leaksensor/app/Middlewares/Third_Party/Wire" -Og -ffunction-sections -fdata-sections -mslow-flash-data -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-ThreadSafe

clean-Core-2f-ThreadSafe:
	-$(RM) ./Core/ThreadSafe/newlib_lock_glue.cyclo ./Core/ThreadSafe/newlib_lock_glue.d ./Core/ThreadSafe/newlib_lock_glue.o ./Core/ThreadSafe/newlib_lock_glue.su

.PHONY: clean-Core-2f-ThreadSafe

