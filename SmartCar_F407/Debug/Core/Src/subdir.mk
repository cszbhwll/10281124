################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app.c \
../Core/Src/arm.c \
../Core/Src/bus_servo.c \
../Core/Src/chassis.c \
../Core/Src/console.c \
../Core/Src/display.c \
../Core/Src/field.c \
../Core/Src/imu_mpu6050.c \
../Core/Src/main.c \
../Core/Src/mission.c \
../Core/Src/motor_drv.c \
../Core/Src/nav.c \
../Core/Src/obstacle.c \
../Core/Src/port_stm32.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/sys.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/task_code.c \
../Core/Src/tof_vl53l0x.c \
../Core/Src/vision.c 

OBJS += \
./Core/Src/app.o \
./Core/Src/arm.o \
./Core/Src/bus_servo.o \
./Core/Src/chassis.o \
./Core/Src/console.o \
./Core/Src/display.o \
./Core/Src/field.o \
./Core/Src/imu_mpu6050.o \
./Core/Src/main.o \
./Core/Src/mission.o \
./Core/Src/motor_drv.o \
./Core/Src/nav.o \
./Core/Src/obstacle.o \
./Core/Src/port_stm32.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/sys.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/task_code.o \
./Core/Src/tof_vl53l0x.o \
./Core/Src/vision.o 

C_DEPS += \
./Core/Src/app.d \
./Core/Src/arm.d \
./Core/Src/bus_servo.d \
./Core/Src/chassis.d \
./Core/Src/console.d \
./Core/Src/display.d \
./Core/Src/field.d \
./Core/Src/imu_mpu6050.d \
./Core/Src/main.d \
./Core/Src/mission.d \
./Core/Src/motor_drv.d \
./Core/Src/nav.d \
./Core/Src/obstacle.d \
./Core/Src/port_stm32.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/sys.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/task_code.d \
./Core/Src/tof_vl53l0x.d \
./Core/Src/vision.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app.cyclo ./Core/Src/app.d ./Core/Src/app.o ./Core/Src/app.su ./Core/Src/arm.cyclo ./Core/Src/arm.d ./Core/Src/arm.o ./Core/Src/arm.su ./Core/Src/bus_servo.cyclo ./Core/Src/bus_servo.d ./Core/Src/bus_servo.o ./Core/Src/bus_servo.su ./Core/Src/chassis.cyclo ./Core/Src/chassis.d ./Core/Src/chassis.o ./Core/Src/chassis.su ./Core/Src/console.cyclo ./Core/Src/console.d ./Core/Src/console.o ./Core/Src/console.su ./Core/Src/display.cyclo ./Core/Src/display.d ./Core/Src/display.o ./Core/Src/display.su ./Core/Src/field.cyclo ./Core/Src/field.d ./Core/Src/field.o ./Core/Src/field.su ./Core/Src/imu_mpu6050.cyclo ./Core/Src/imu_mpu6050.d ./Core/Src/imu_mpu6050.o ./Core/Src/imu_mpu6050.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mission.cyclo ./Core/Src/mission.d ./Core/Src/mission.o ./Core/Src/mission.su ./Core/Src/motor_drv.cyclo ./Core/Src/motor_drv.d ./Core/Src/motor_drv.o ./Core/Src/motor_drv.su ./Core/Src/nav.cyclo ./Core/Src/nav.d ./Core/Src/nav.o ./Core/Src/nav.su ./Core/Src/obstacle.cyclo ./Core/Src/obstacle.d ./Core/Src/obstacle.o ./Core/Src/obstacle.su ./Core/Src/port_stm32.cyclo ./Core/Src/port_stm32.d ./Core/Src/port_stm32.o ./Core/Src/port_stm32.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/sys.cyclo ./Core/Src/sys.d ./Core/Src/sys.o ./Core/Src/sys.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/task_code.cyclo ./Core/Src/task_code.d ./Core/Src/task_code.o ./Core/Src/task_code.su ./Core/Src/tof_vl53l0x.cyclo ./Core/Src/tof_vl53l0x.d ./Core/Src/tof_vl53l0x.o ./Core/Src/tof_vl53l0x.su ./Core/Src/vision.cyclo ./Core/Src/vision.d ./Core/Src/vision.o ./Core/Src/vision.su

.PHONY: clean-Core-2f-Src

