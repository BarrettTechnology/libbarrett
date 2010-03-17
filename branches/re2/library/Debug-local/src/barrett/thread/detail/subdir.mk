################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/barrett/thread/detail/null_mutex.cpp 

OBJS += \
./src/barrett/thread/detail/null_mutex.o 

CPP_DEPS += \
./src/barrett/thread/detail/null_mutex.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/thread/detail/%.o: ../src/barrett/thread/detail/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/dc/svn/tmp/re2/cdlbt/src" -I/usr/local/include/eigen2 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


