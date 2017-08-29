################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_it.c \
../Src/system_stm32f7xx.c 

CPP_SRCS += \
../Src/PID.cpp \
../Src/Wheel.cpp \
../Src/WheelSide.cpp \
../Src/main.cpp 

OBJS += \
./Src/PID.o \
./Src/Wheel.o \
./Src/WheelSide.o \
./Src/main.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_it.o \
./Src/system_stm32f7xx.o 

C_DEPS += \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_it.d \
./Src/system_stm32f7xx.d 

CPP_DEPS += \
./Src/PID.d \
./Src/Wheel.d \
./Src/WheelSide.d \
./Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Inc" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Drivers/STM32F7xx_HAL_Driver/Inc" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F746xx -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Inc" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Drivers/STM32F7xx_HAL_Driver/Inc" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Drivers/CMSIS/Include" -I"D:/CaptainRobot/Controle_moteurs/Solution_STM32/cube/test_motorControl_cpp/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


