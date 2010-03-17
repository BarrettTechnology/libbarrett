################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/log/reader.cpp \
../src/log/real_time_writer.cpp \
../src/log/verify_file_contents.cpp \
../src/log/writer.cpp 

OBJS += \
./src/log/reader.o \
./src/log/real_time_writer.o \
./src/log/verify_file_contents.o \
./src/log/writer.o 

CPP_DEPS += \
./src/log/reader.d \
./src/log/real_time_writer.d \
./src/log/verify_file_contents.d \
./src/log/writer.d 


# Each subdirectory must supply rules for building sources it contributes
src/log/%.o: ../src/log/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/dc/svn/tmp/re2/library/src" -I/usr/local/include/eigen2 -I"/home/dc/svn/tmp/re2/cdlbt/src" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


