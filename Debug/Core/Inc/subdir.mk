################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/matrix.cpp \
../Core/Inc/order1_soc_estimation.cpp \
../Core/Inc/order2_soc_estimation.cpp \
../Core/Inc/rint_soc_estimation.cpp \
../Core/Inc/voc_soc.cpp 

OBJS += \
./Core/Inc/matrix.o \
./Core/Inc/order1_soc_estimation.o \
./Core/Inc/order2_soc_estimation.o \
./Core/Inc/rint_soc_estimation.o \
./Core/Inc/voc_soc.o 

CPP_DEPS += \
./Core/Inc/matrix.d \
./Core/Inc/order1_soc_estimation.d \
./Core/Inc/order2_soc_estimation.d \
./Core/Inc/rint_soc_estimation.d \
./Core/Inc/voc_soc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.cpp Core/Inc/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/matrix.cyclo ./Core/Inc/matrix.d ./Core/Inc/matrix.o ./Core/Inc/matrix.su ./Core/Inc/order1_soc_estimation.cyclo ./Core/Inc/order1_soc_estimation.d ./Core/Inc/order1_soc_estimation.o ./Core/Inc/order1_soc_estimation.su ./Core/Inc/order2_soc_estimation.cyclo ./Core/Inc/order2_soc_estimation.d ./Core/Inc/order2_soc_estimation.o ./Core/Inc/order2_soc_estimation.su ./Core/Inc/rint_soc_estimation.cyclo ./Core/Inc/rint_soc_estimation.d ./Core/Inc/rint_soc_estimation.o ./Core/Inc/rint_soc_estimation.su ./Core/Inc/voc_soc.cyclo ./Core/Inc/voc_soc.d ./Core/Inc/voc_soc.o ./Core/Inc/voc_soc.su

.PHONY: clean-Core-2f-Inc

