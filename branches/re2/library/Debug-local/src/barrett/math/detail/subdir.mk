################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/barrett/math/detail/trapezoidal_velocity_profile.cpp 

OBJS += \
./src/barrett/math/detail/trapezoidal_velocity_profile.o 

CPP_DEPS += \
./src/barrett/math/detail/trapezoidal_velocity_profile.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/math/detail/%.o: ../src/barrett/math/detail/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/dc/svn/tmp/re2/cdlbt/src" -I/usr/local/include/eigen2 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


