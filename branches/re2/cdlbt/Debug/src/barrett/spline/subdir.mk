################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/barrett/spline/spline.c 

OBJS += \
./src/barrett/spline/spline.o 

C_DEPS += \
./src/barrett/spline/spline.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/spline/%.o: ../src/barrett/spline/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -DCANTYPE_PEAKISA -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI -I/usr/xenomai/include -I/root/libbarrett/trunk/library/src -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


