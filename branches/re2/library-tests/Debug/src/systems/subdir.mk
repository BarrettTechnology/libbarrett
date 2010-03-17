################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/systems/callback.cpp \
../src/systems/constant.cpp \
../src/systems/converter.cpp \
../src/systems/gain.cpp \
../src/systems/helpers.cpp \
../src/systems/io_conversion.cpp \
../src/systems/manual_execution_manager.cpp \
../src/systems/pid_controller.cpp \
../src/systems/print_to_stream.cpp \
../src/systems/summer.cpp \
../src/systems/tool_orientation.cpp 

OBJS += \
./src/systems/callback.o \
./src/systems/constant.o \
./src/systems/converter.o \
./src/systems/gain.o \
./src/systems/helpers.o \
./src/systems/io_conversion.o \
./src/systems/manual_execution_manager.o \
./src/systems/pid_controller.o \
./src/systems/print_to_stream.o \
./src/systems/summer.o \
./src/systems/tool_orientation.o 

CPP_DEPS += \
./src/systems/callback.d \
./src/systems/constant.d \
./src/systems/converter.d \
./src/systems/gain.d \
./src/systems/helpers.d \
./src/systems/io_conversion.d \
./src/systems/manual_execution_manager.d \
./src/systems/pid_controller.d \
./src/systems/print_to_stream.d \
./src/systems/summer.d \
./src/systems/tool_orientation.d 


# Each subdirectory must supply rules for building sources it contributes
src/systems/%.o: ../src/systems/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/dc/svn/tmp/re2/library/src" -I/usr/local/include/eigen2 -I"/home/dc/svn/tmp/re2/cdlbt/src" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


