################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/custom_memory_manager.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/hrtim.c \
../Core/Src/i2c.c \
../Core/Src/it_transport.c \
../Core/Src/iwdg.c \
../Core/Src/ld_sensors.c \
../Core/Src/main.c \
../Core/Src/microros_allocators.c \
../Core/Src/microros_main.c \
../Core/Src/microros_time.c \
../Core/Src/motor_control.c \
../Core/Src/oled_battery.c \
../Core/Src/sensor_polling.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_hal_timebase_tim.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/custom_memory_manager.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/hrtim.o \
./Core/Src/i2c.o \
./Core/Src/it_transport.o \
./Core/Src/iwdg.o \
./Core/Src/ld_sensors.o \
./Core/Src/main.o \
./Core/Src/microros_allocators.o \
./Core/Src/microros_main.o \
./Core/Src/microros_time.o \
./Core/Src/motor_control.o \
./Core/Src/oled_battery.o \
./Core/Src/sensor_polling.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_hal_timebase_tim.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/custom_memory_manager.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/hrtim.d \
./Core/Src/i2c.d \
./Core/Src/it_transport.d \
./Core/Src/iwdg.d \
./Core/Src/ld_sensors.d \
./Core/Src/main.d \
./Core/Src/microros_allocators.d \
./Core/Src/microros_main.d \
./Core/Src/microros_time.d \
./Core/Src/motor_control.d \
./Core/Src/oled_battery.d \
./Core/Src/sensor_polling.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_hal_timebase_tim.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -c -I../Core/Inc -I"/home/teerth/STM32Projects/motherboard_kratos/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/custom_memory_manager.cyclo ./Core/Src/custom_memory_manager.d ./Core/Src/custom_memory_manager.o ./Core/Src/custom_memory_manager.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hrtim.cyclo ./Core/Src/hrtim.d ./Core/Src/hrtim.o ./Core/Src/hrtim.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/it_transport.cyclo ./Core/Src/it_transport.d ./Core/Src/it_transport.o ./Core/Src/it_transport.su ./Core/Src/iwdg.cyclo ./Core/Src/iwdg.d ./Core/Src/iwdg.o ./Core/Src/iwdg.su ./Core/Src/ld_sensors.cyclo ./Core/Src/ld_sensors.d ./Core/Src/ld_sensors.o ./Core/Src/ld_sensors.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/microros_allocators.cyclo ./Core/Src/microros_allocators.d ./Core/Src/microros_allocators.o ./Core/Src/microros_allocators.su ./Core/Src/microros_main.cyclo ./Core/Src/microros_main.d ./Core/Src/microros_main.o ./Core/Src/microros_main.su ./Core/Src/microros_time.cyclo ./Core/Src/microros_time.d ./Core/Src/microros_time.o ./Core/Src/microros_time.su ./Core/Src/motor_control.cyclo ./Core/Src/motor_control.d ./Core/Src/motor_control.o ./Core/Src/motor_control.su ./Core/Src/oled_battery.cyclo ./Core/Src/oled_battery.d ./Core/Src/oled_battery.o ./Core/Src/oled_battery.su ./Core/Src/sensor_polling.cyclo ./Core/Src/sensor_polling.d ./Core/Src/sensor_polling.o ./Core/Src/sensor_polling.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_hal_timebase_tim.cyclo ./Core/Src/stm32h7xx_hal_timebase_tim.d ./Core/Src/stm32h7xx_hal_timebase_tim.o ./Core/Src/stm32h7xx_hal_timebase_tim.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

