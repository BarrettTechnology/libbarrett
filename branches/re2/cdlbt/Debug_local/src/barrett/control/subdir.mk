################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/barrett/control/control_cartesian_xyz.c \
../src/barrett/control/control_cartesian_xyz_q.c \
../src/barrett/control/control_joint.c \
../src/barrett/control/control_joint_legacy.c 

OBJS += \
./src/barrett/control/control_cartesian_xyz.o \
./src/barrett/control/control_cartesian_xyz_q.o \
./src/barrett/control/control_joint.o \
./src/barrett/control/control_joint_legacy.o 

C_DEPS += \
./src/barrett/control/control_cartesian_xyz.d \
./src/barrett/control/control_cartesian_xyz_q.d \
./src/barrett/control/control_joint.d \
./src/barrett/control/control_joint_legacy.d 


# Each subdirectory must supply rules for building sources it contributes
src/barrett/control/%.o: ../src/barrett/control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -DCANTYPE_PEAKISA -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI -I/home/dc/svn/libbarrett/dependencies/xenomai/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


