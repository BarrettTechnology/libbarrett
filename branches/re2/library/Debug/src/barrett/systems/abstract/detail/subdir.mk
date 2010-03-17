################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/barrett/systems/abstract/detail/execution_manager.cpp \
../src/barrett/systems/abstract/detail/system.cpp 

OBJS += \
./src/barrett/systems/abstract/detail/execution_manager.o \
./src/barrett/systems/abstract/detail/system.o 

CPP_DEPS += \
./src/barrett/systems/abstract/detail/execution_manager.d \
./src/barrett/systems/abstract/detail/system.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/systems/abstract/detail/%.o: ../src/barrett/systems/abstract/detail/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/root/libbarrett/trunk/cdlbt/src -I/usr/local/include/eigen2 -I/usr/xenomai/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


