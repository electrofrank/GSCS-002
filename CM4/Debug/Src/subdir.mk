################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/app_ethernet.c \
../Src/ethernetif.c \
../Src/freertos.c \
../Src/hx711.c \
../Src/lwip.c \
../Src/main.c \
../Src/retarget.c \
../Src/spp.c \
../Src/stm32h7xx_hal_msp.c \
../Src/stm32h7xx_hal_timebase_tim.c \
../Src/stm32h7xx_it.c \
../Src/sysmem.c 

OBJS += \
./Src/app_ethernet.o \
./Src/ethernetif.o \
./Src/freertos.o \
./Src/hx711.o \
./Src/lwip.o \
./Src/main.o \
./Src/retarget.o \
./Src/spp.o \
./Src/stm32h7xx_hal_msp.o \
./Src/stm32h7xx_hal_timebase_tim.o \
./Src/stm32h7xx_it.o \
./Src/sysmem.o 

C_DEPS += \
./Src/app_ethernet.d \
./Src/ethernetif.d \
./Src/freertos.d \
./Src/hx711.d \
./Src/lwip.d \
./Src/main.d \
./Src/retarget.d \
./Src/spp.d \
./Src/stm32h7xx_hal_msp.d \
./Src/stm32h7xx_hal_timebase_tim.d \
./Src/stm32h7xx_it.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Drivers/BSP/Components/lan8742 -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/app_ethernet.d ./Src/app_ethernet.o ./Src/ethernetif.d ./Src/ethernetif.o ./Src/freertos.d ./Src/freertos.o ./Src/hx711.d ./Src/hx711.o ./Src/lwip.d ./Src/lwip.o ./Src/main.d ./Src/main.o ./Src/retarget.d ./Src/retarget.o ./Src/spp.d ./Src/spp.o ./Src/stm32h7xx_hal_msp.d ./Src/stm32h7xx_hal_msp.o ./Src/stm32h7xx_hal_timebase_tim.d ./Src/stm32h7xx_hal_timebase_tim.o ./Src/stm32h7xx_it.d ./Src/stm32h7xx_it.o ./Src/sysmem.d ./Src/sysmem.o

.PHONY: clean-Src

