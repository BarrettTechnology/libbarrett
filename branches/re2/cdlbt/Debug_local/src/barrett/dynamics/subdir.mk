################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/barrett/dynamics/dynamics.c 

OBJS += \
./src/barrett/dynamics/dynamics.o 

C_DEPS += \
./src/barrett/dynamics/dynamics.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/dynamics/%.o: ../src/barrett/dynamics/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -DCANTYPE_PEAKISA -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI -I/home/dc/svn/libbarrett/dependencies/xenomai/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


