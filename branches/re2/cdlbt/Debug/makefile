################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/barrett/wambot/subdir.mk
-include src/barrett/wam/subdir.mk
-include src/barrett/spline/subdir.mk
-include src/barrett/rpc/subdir.mk
-include src/barrett/refgen/subdir.mk
-include src/barrett/profile/subdir.mk
-include src/barrett/os/subdir.mk
-include src/barrett/log/subdir.mk
-include src/barrett/kinematics/subdir.mk
-include src/barrett/interp/subdir.mk
-include src/barrett/gsl/subdir.mk
-include src/barrett/dynamics/subdir.mk
-include src/barrett/discover/subdir.mk
-include src/barrett/control/subdir.mk
-include src/barrett/calgrav/subdir.mk
-include src/barrett/bus/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: libcdlbt.a

# Tool invocations
libcdlbt.a: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC Archiver'
	ar -r "libcdlbt.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ARCHIVES) libcdlbt.a
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
