################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
roverKernel/HAL/tm4c1294_hal.obj: ../roverKernel/HAL/tm4c1294_hal.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/home/v125/ProgIDEs/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="/home/v125/Documents/git/roverRPi3" --include_path="/home/v125/Documents/git" --include_path="/home/v125/ProgIDEs/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.0.LTS/include" --include_path="/home/v125/ProgIDEs/ti/ccsv7/workspace1" --include_path="/home/v125/ProgIDEs/ti/TivaWare_C_Series-2.1.0.12573" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="roverKernel/HAL/tm4c1294_hal.d" --obj_directory="roverKernel/HAL" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


