################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l412kbux.s 

OBJS += \
./Core/Startup/startup_stm32l412kbux.o 

S_DEPS += \
./Core/Startup/startup_stm32l412kbux.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/RingBuffer/config" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/RingBuffer/include" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/config" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/include" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/ARM_ITM" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/File" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/Jlink_RTT" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/STM32_USB_CDC" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/TCPIP" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/TCPIP_Win32" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/UDP" -I"D:/ANASTASIIA/WorkIDE/KProj/Middlewares/TraceRecorder/streamports/XMOS_xScope" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l412kbux.d ./Core/Startup/startup_stm32l412kbux.o

.PHONY: clean-Core-2f-Startup

