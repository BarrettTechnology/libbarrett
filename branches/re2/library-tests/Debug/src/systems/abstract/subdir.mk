################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/systems/abstract/controller.cpp \
../src/systems/abstract/execution_manager.cpp \
../src/systems/abstract/single_io.cpp \
../src/systems/abstract/system.cpp 

OBJS += \
./src/systems/abstract/controller.o \
./src/systems/abstract/execution_manager.o \
./src/systems/abstract/single_io.o \
./src/systems/abstract/system.o 

CPP_DEPS += \
./src/systems/abstract/controller.d \
./src/systems/abstract/execution_manager.d \
./src/systems/abstract/single_io.d \
./src/systems/abstract/system.d 


# Each subdirectory must supply rules for building sources it contributes
src/systems/abstract/%.o: ../src/systems/abstract/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/dc/svn/tmp/re2/library/src" -I/usr/local/include/eigen2 -I"/home/dc/svn/tmp/re2/cdlbt/src" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


