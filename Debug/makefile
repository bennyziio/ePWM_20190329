################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/c2000_6.4.11

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./source/DSP2833x_ADC_cal.obj" \
"./source/DSP2833x_CSMPasswords.obj" \
"./source/DSP2833x_CodeStartBranch.obj" \
"./source/DSP2833x_CpuTimers.obj" \
"./source/DSP2833x_DefaultIsr.obj" \
"./source/DSP2833x_EPwm.obj" \
"./source/DSP2833x_EQep.obj" \
"./source/DSP2833x_GlobalVariableDefs.obj" \
"./source/DSP2833x_Gpio.obj" \
"./source/DSP2833x_MemCopy.obj" \
"./source/DSP2833x_PieCtrl.obj" \
"./source/DSP2833x_PieVect.obj" \
"./source/DSP2833x_SysCtrl.obj" \
"./source/DSP2833x_usDelay.obj" \
"./source/EQEP_posspeed.obj" \
"./source/main.obj" \
"../cmd/DSP2833x_Headers_nonBIOS.cmd" \
"../cmd/F28335.cmd" \
"../C2000_Lib/IQmath_fpu32.lib" \
"../C2000_Lib/rts2800_fpu32.lib" \
$(GEN_CMDS__FLAG) \
-l"rts2800_fpu32.lib" \
-l"IQmath_fpu32.lib" \
-l"libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include source/subdir_vars.mk
-include cmd/subdir_vars.mk
-include C2000_Lib/subdir_vars.mk
-include source/subdir_rules.mk
-include cmd/subdir_rules.mk
-include C2000_Lib/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
ePWM_20190329.out \

EXE_OUTPUTS__QUOTED += \
"ePWM_20190329.out" \

BIN_OUTPUTS += \
ePWM_20190329.hex \

BIN_OUTPUTS__QUOTED += \
"ePWM_20190329.hex" \


# All Target
all: ePWM_20190329.out

# Tool invocations
ePWM_20190329.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv6/tools/compiler/c2000_6.4.11/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --advice:performance=all -g --diag_wrap=off --diag_warning=225 --display_error_number -z -m"ePWM_20190329.map" --stack_size=0x300 --warn_sections -i"C:/ti/ccsv6/tools/compiler/c2000_6.4.11/lib" -i"F:/Work_CCS/tutorial/ePWM_20190329/C2000_Lib" -i"C:/ti/ccsv6/tools/compiler/c2000_6.4.11/include" -i"F:/Work_CCS/tutorial/ePWM_20190329" -i"F:/Work_CCS/lib" --priority --reread_libs --diag_wrap=off --display_error_number --xml_link_info="ePWM_20190329_linkInfo.xml" --rom_model -o "ePWM_20190329.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

ePWM_20190329.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C2000 Hex Utility'
	"C:/ti/ccsv6/tools/compiler/c2000_6.4.11/bin/hex2000"  -o "ePWM_20190329.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "source\DSP2833x_CpuTimers.pp" "source\DSP2833x_DefaultIsr.pp" "source\DSP2833x_EPwm.pp" "source\DSP2833x_EQep.pp" "source\DSP2833x_GlobalVariableDefs.pp" "source\DSP2833x_Gpio.pp" "source\DSP2833x_MemCopy.pp" "source\DSP2833x_PieCtrl.pp" "source\DSP2833x_PieVect.pp" "source\DSP2833x_SysCtrl.pp" "source\EQEP_posspeed.pp" "source\main.pp" 
	-$(RM) "source\DSP2833x_ADC_cal.obj" "source\DSP2833x_CSMPasswords.obj" "source\DSP2833x_CodeStartBranch.obj" "source\DSP2833x_CpuTimers.obj" "source\DSP2833x_DefaultIsr.obj" "source\DSP2833x_EPwm.obj" "source\DSP2833x_EQep.obj" "source\DSP2833x_GlobalVariableDefs.obj" "source\DSP2833x_Gpio.obj" "source\DSP2833x_MemCopy.obj" "source\DSP2833x_PieCtrl.obj" "source\DSP2833x_PieVect.obj" "source\DSP2833x_SysCtrl.obj" "source\DSP2833x_usDelay.obj" "source\EQEP_posspeed.obj" "source\main.obj" 
	-$(RM) "source\DSP2833x_ADC_cal.pp" "source\DSP2833x_CSMPasswords.pp" "source\DSP2833x_CodeStartBranch.pp" "source\DSP2833x_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

