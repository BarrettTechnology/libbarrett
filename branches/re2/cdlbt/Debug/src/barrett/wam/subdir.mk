################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/barrett/wam/wam.c \
../src/barrett/wam/wam_list.c \
../src/barrett/wam/wam_local.c \
../src/barrett/wam/wam_rpc.c \
../src/barrett/wam/wam_thread.c 

OBJS += \
./src/barrett/wam/wam.o \
./src/barrett/wam/wam_list.o \
./src/barrett/wam/wam_local.o \
./src/barrett/wam/wam_rpc.o \
./src/barrett/wam/wam_thread.o 

C_DEPS += \
./src/barrett/wam/wam.d \
./src/barrett/wam/wam_list.d \
./src/barrett/wam/wam_local.d \
./src/barrett/wam/wam_rpc.d \
./src/barrett/wam/wam_thread.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/wam/%.o: ../src/barrett/wam/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -DCANTYPE_PEAKISA -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI -I/usr/xenomai/include -I/root/libbarrett/trunk/library/src -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


