################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include service/subdir.mk
-include ports/source/gnu/subdir.mk
-include kernel/source/subdir.mk
-include emlib/subdir.mk
-include cpu/source/subdir.mk
-include common/source/rtos/subdir.mk
-include common/source/platform_mgr/subdir.mk
-include common/source/lib/subdir.mk
-include common/source/kal/subdir.mk
-include common/source/common/subdir.mk
-include common/source/collections/subdir.mk
-include common/source/auth/subdir.mk
-include common/subdir.mk
-include SystemView/subdir.mk
-include Graphics/subdir.mk
-include Drivers/subdir.mk
-include CMSIS/EFM32PG12B/subdir.mk
-include BSP/siliconlabs/generic/source/subdir.mk
-include BSP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: RTOS_Lab7.axf

# Tool invocations
RTOS_Lab7.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "RTOS_Lab7.ld" -Xlinker --gc-sections -Xlinker -Map="RTOS_Lab7.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o RTOS_Lab7.axf "./BSP/bsp_bcc.o" "./BSP/bsp_stk.o" "./BSP/bsp_stk_leds.o" "./BSP/bsp_trace.o" "./BSP/siliconlabs/generic/source/bsp_cpu.o" "./BSP/siliconlabs/generic/source/bsp_os.o" "./CMSIS/EFM32PG12B/startup_efm32pg12b.o" "./CMSIS/EFM32PG12B/system_efm32pg12b.o" "./Drivers/capsense.o" "./Drivers/display.o" "./Drivers/displayls013b7dh03.o" "./Drivers/displaypalemlib.o" "./Drivers/retargetio.o" "./Drivers/retargettextdisplay.o" "./Drivers/textdisplay.o" "./Drivers/udelay.o" "./Graphics/dmd_display.o" "./Graphics/glib.o" "./Graphics/glib_bitmap.o" "./Graphics/glib_circle.o" "./Graphics/glib_font_narrow_6x8.o" "./Graphics/glib_font_number_16x20.o" "./Graphics/glib_line.o" "./Graphics/glib_rectangle.o" "./Graphics/glib_string.o" "./SystemView/SEGGER_RTT.o" "./SystemView/SEGGER_RTT_ASM_ARMv7M.o" "./SystemView/SEGGER_SYSVIEW.o" "./SystemView/SEGGER_SYSVIEW_Config_MicriumOSKernel.o" "./SystemView/SEGGER_SYSVIEW_MicriumOSKernel.o" "./common/sl_slist.o" "./common/source/auth/auth.o" "./common/source/collections/bitmap.o" "./common/source/collections/map.o" "./common/source/collections/slist.o" "./common/source/common/common.o" "./common/source/kal/kal_kernel.o" "./common/source/lib/lib_ascii.o" "./common/source/lib/lib_math.o" "./common/source/lib/lib_mem.o" "./common/source/lib/lib_str.o" "./common/source/platform_mgr/platform_mgr.o" "./common/source/rtos/rtos_err_str.o" "./cpu/source/cpu_core.o" "./emlib/em_acmp.o" "./emlib/em_adc.o" "./emlib/em_aes.o" "./emlib/em_assert.o" "./emlib/em_burtc.o" "./emlib/em_can.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_cryotimer.o" "./emlib/em_crypto.o" "./emlib/em_csen.o" "./emlib/em_dac.o" "./emlib/em_dbg.o" "./emlib/em_dma.o" "./emlib/em_ebi.o" "./emlib/em_emu.o" "./emlib/em_gpcrc.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_iadc.o" "./emlib/em_idac.o" "./emlib/em_int.o" "./emlib/em_lcd.o" "./emlib/em_ldma.o" "./emlib/em_lesense.o" "./emlib/em_letimer.o" "./emlib/em_leuart.o" "./emlib/em_mpu.o" "./emlib/em_msc.o" "./emlib/em_opamp.o" "./emlib/em_pcnt.o" "./emlib/em_pdm.o" "./emlib/em_prs.o" "./emlib/em_qspi.o" "./emlib/em_rmu.o" "./emlib/em_rtc.o" "./emlib/em_rtcc.o" "./emlib/em_se.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./emlib/em_vcmp.o" "./emlib/em_vdac.o" "./emlib/em_wdog.o" "./kernel/source/os_cfg_app.o" "./kernel/source/os_core.o" "./kernel/source/os_dbg.o" "./kernel/source/os_flag.o" "./kernel/source/os_mon.o" "./kernel/source/os_msg.o" "./kernel/source/os_mutex.o" "./kernel/source/os_prio.o" "./kernel/source/os_q.o" "./kernel/source/os_sem.o" "./kernel/source/os_stat.o" "./kernel/source/os_task.o" "./kernel/source/os_time.o" "./kernel/source/os_tmr.o" "./kernel/source/os_var.o" "./ports/source/gnu/armv7m_cpu_a.o" "./ports/source/gnu/armv7m_os_cpu_a.o" "./ports/source/gnu/armv7m_os_cpu_c.o" "./service/sl_sleeptimer.o" "./service/sl_sleeptimer_hal_rtcc.o" "./src/app.o" "./src/buttons.o" "./src/cmu.o" "./src/ex_main.o" "./src/fifo.o" "./src/gpio.o" "./src/slider.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: RTOS_Lab7.hex'
	arm-none-eabi-objcopy -O ihex "RTOS_Lab7.axf" "RTOS_Lab7.hex"
	@echo ' '

	@echo 'Building bin file: RTOS_Lab7.bin'
	arm-none-eabi-objcopy -O binary "RTOS_Lab7.axf" "RTOS_Lab7.bin"
	@echo ' '

	@echo 'Building s37 file: RTOS_Lab7.s37'
	arm-none-eabi-objcopy -O srec "RTOS_Lab7.axf" "RTOS_Lab7.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "RTOS_Lab7.axf" -A
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) RTOS_Lab7.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
