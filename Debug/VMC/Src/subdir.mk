################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VMC/Src/debug.c \
../VMC/Src/pid_dc.c \
../VMC/Src/pid_et.c \
../VMC/Src/pid_op.c \
../VMC/Src/pid_pool_mgr.c \
../VMC/Src/pid_state.c \
../VMC/Src/pid_struct.c \
../VMC/Src/pid_synthesizer.c \
../VMC/Src/pid_wheel_mgr.c \
../VMC/Src/rtos_timer_mgr.c 

OBJS += \
./VMC/Src/debug.o \
./VMC/Src/pid_dc.o \
./VMC/Src/pid_et.o \
./VMC/Src/pid_op.o \
./VMC/Src/pid_pool_mgr.o \
./VMC/Src/pid_state.o \
./VMC/Src/pid_struct.o \
./VMC/Src/pid_synthesizer.o \
./VMC/Src/pid_wheel_mgr.o \
./VMC/Src/rtos_timer_mgr.o 

C_DEPS += \
./VMC/Src/debug.d \
./VMC/Src/pid_dc.d \
./VMC/Src/pid_et.d \
./VMC/Src/pid_op.d \
./VMC/Src/pid_pool_mgr.d \
./VMC/Src/pid_state.d \
./VMC/Src/pid_struct.d \
./VMC/Src/pid_synthesizer.d \
./VMC/Src/pid_wheel_mgr.d \
./VMC/Src/rtos_timer_mgr.d 


# Each subdirectory must supply rules for building sources it contributes
VMC/Src/%.o VMC/Src/%.su VMC/Src/%.cyclo: ../VMC/Src/%.c VMC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I"D:/WorkSpace/STM32CUBEIDE/VMC/VMC/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-VMC-2f-Src

clean-VMC-2f-Src:
	-$(RM) ./VMC/Src/debug.cyclo ./VMC/Src/debug.d ./VMC/Src/debug.o ./VMC/Src/debug.su ./VMC/Src/pid_dc.cyclo ./VMC/Src/pid_dc.d ./VMC/Src/pid_dc.o ./VMC/Src/pid_dc.su ./VMC/Src/pid_et.cyclo ./VMC/Src/pid_et.d ./VMC/Src/pid_et.o ./VMC/Src/pid_et.su ./VMC/Src/pid_op.cyclo ./VMC/Src/pid_op.d ./VMC/Src/pid_op.o ./VMC/Src/pid_op.su ./VMC/Src/pid_pool_mgr.cyclo ./VMC/Src/pid_pool_mgr.d ./VMC/Src/pid_pool_mgr.o ./VMC/Src/pid_pool_mgr.su ./VMC/Src/pid_state.cyclo ./VMC/Src/pid_state.d ./VMC/Src/pid_state.o ./VMC/Src/pid_state.su ./VMC/Src/pid_struct.cyclo ./VMC/Src/pid_struct.d ./VMC/Src/pid_struct.o ./VMC/Src/pid_struct.su ./VMC/Src/pid_synthesizer.cyclo ./VMC/Src/pid_synthesizer.d ./VMC/Src/pid_synthesizer.o ./VMC/Src/pid_synthesizer.su ./VMC/Src/pid_wheel_mgr.cyclo ./VMC/Src/pid_wheel_mgr.d ./VMC/Src/pid_wheel_mgr.o ./VMC/Src/pid_wheel_mgr.su ./VMC/Src/rtos_timer_mgr.cyclo ./VMC/Src/rtos_timer_mgr.d ./VMC/Src/rtos_timer_mgr.o ./VMC/Src/rtos_timer_mgr.su

.PHONY: clean-VMC-2f-Src

