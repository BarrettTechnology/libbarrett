################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/subdir.mk
-include src/systems/subdir.mk
-include src/systems/abstract/subdir.mk
-include src/math/subdir.mk
-include src/log/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: libbarrett-tests

dependents:
	-cd /home/dc/svn/tmp/re2/library/Debug-local && $(MAKE) clean all

# Tool invocations
libbarrett-tests: $(OBJS) $(USER_OBJS) /home/dc/svn/tmp/re2/library/Debug-local/libbarrett.so
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -L"/home/dc/svn/tmp/re2/library/Debug-local" -L"/home/dc/svn/tmp/re2/cdlbt/Debug_local" -Xlinker -rpath "/home/dc/svn/tmp/re2/library/Debug-local" -o"libbarrett-tests" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C++_DEPS)$(C_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(CXX_DEPS)$(C_UPPER_DEPS) libbarrett-tests
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
/home/dc/svn/tmp/re2/library/Debug-local/libbarrett.so:

-include ../makefile.targets
