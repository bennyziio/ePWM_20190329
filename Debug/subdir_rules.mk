################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_6.4.11/bin/cl2000" -v28 -mt -ml --float_support=fpu32 --include_path="C:/ti/ccsv6/tools/compiler/c2000_6.4.11/include" -g --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


