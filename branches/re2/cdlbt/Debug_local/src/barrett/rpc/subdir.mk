################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/barrett/rpc/rpc.c \
../src/barrett/rpc/rpc_tcpjson.c 

OBJS += \
./src/barrett/rpc/rpc.o \
./src/barrett/rpc/rpc_tcpjson.o 

C_DEPS += \
./src/barrett/rpc/rpc.d \
./src/barrett/rpc/rpc_tcpjson.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/rpc/%.o: ../src/barrett/rpc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -DCANTYPE_PEAKISA -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI -I/home/dc/svn/libbarrett/dependencies/xenomai/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


