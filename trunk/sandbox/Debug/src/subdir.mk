################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/cartesian_hold.cpp \
../src/teach.cpp 

OBJS += \
./src/cartesian_hold.o \
./src/teach.o 

CPP_DEPS += \
./src/cartesian_hold.d \
./src/teach.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/root/libbarrett/trunk/library/src -I/usr/local/include/eigen2 -I/root/libbarrett/trunk/cdlbt/src -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


