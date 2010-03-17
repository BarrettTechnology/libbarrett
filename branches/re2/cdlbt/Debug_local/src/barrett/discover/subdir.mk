################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/barrett/discover/discover.c 

OBJS += \
./src/barrett/discover/discover.o 

C_DEPS += \
./src/barrett/discover/discover.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/discover/%.o: ../src/barrett/discover/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -DCANTYPE_PEAKISA -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI -I/home/dc/svn/libbarrett/dependencies/xenomai/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


