################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.o Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.su Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.cyclo: ../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/%.c Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L412xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/RingBuffer/config" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/RingBuffer/include" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/config" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/include" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/ARM_ITM" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/File" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/Jlink_RTT" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/STM32_USB_CDC" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/TCPIP" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/TCPIP_Win32" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/UDP" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/XMOS_xScope" -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang

clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang:
	-$(RM) ./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.cyclo ./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.d ./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o ./Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang

