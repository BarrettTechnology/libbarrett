################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/barrett/calgrav/calgrav.c 

OBJS += \
./src/barrett/calgrav/calgrav.o 

C_DEPS += \
./src/barrett/calgrav/calgrav.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/calgrav/%.o: ../src/barrett/calgrav/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -DCANTYPE_PEAKISA -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI -I/usr/xenomai/include -I/root/libbarrett/trunk/library/src -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


