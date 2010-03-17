################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/math/kinematics.cpp \
../src/math/matrix.cpp \
../src/math/spline.cpp \
../src/math/utils.cpp 

OBJS += \
./src/math/kinematics.o \
./src/math/matrix.o \
./src/math/spline.o \
./src/math/utils.o 

CPP_DEPS += \
./src/math/kinematics.d \
./src/math/matrix.d \
./src/math/spline.d \
./src/math/utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/math/%.o: ../src/math/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/dc/svn/tmp/re2/library/src" -I/usr/local/include/eigen2 -I"/home/dc/svn/tmp/re2/cdlbt/src" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


