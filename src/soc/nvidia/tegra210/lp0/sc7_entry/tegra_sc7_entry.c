/*
 * Copyright (c) 2019 CTCaer
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tegra_sc7_entry.h"
#include "i2c.h"

void __attribute__((noreturn)) sc7_entry();

void usleep(u32 us)
{
	u32 start = TMR(TIMERUS_CNTR_1US);

	while ((u32)(TMR(TIMERUS_CNTR_1US) - start) <= us) // Casting to u32 is important!
		;
}

void __attribute__((noreturn)) _halt()
{
	while (true)
		;
}

static void pinmux_config_i2c(u32 idx)
{
	PINMUX_AUX(PINMUX_AUX_X_I2C_SCL(idx)) = PINMUX_INPUT_ENABLE;
	PINMUX_AUX(PINMUX_AUX_X_I2C_SDA(idx)) = PINMUX_INPUT_ENABLE;
}

static int _emc_update_timing(bool ch1_enabled)
{
	if (ch1_enabled)
		EMC(EMC_TIMING_CONTROL) = 1;
	else
		EMC_CH0(EMC_TIMING_CONTROL) = 1;

	u32 timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
	while (true)
	{
		u32 timing_update_stalled = ch1_enabled ? EMC(EMC_EMC_STATUS) : EMC_CH0(EMC_EMC_STATUS);

		timing_update_stalled &= 0x800000;
		if (!timing_update_stalled)
			return RET_SUCCESS;

		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			return RET_ERROR;
	}
}

static int _config_io_pad_dpd()
{
	// Set warmboot DDR/WEAK brick configuration to PMC. 
	u32 emc_lp0_pmc_ctrl = EMC(EMC_PMC_SCRATCH3);
	PMC(APBDEV_PMC_DDR_CNTRL) = emc_lp0_pmc_ctrl & 0x7FFFF;
	if (emc_lp0_pmc_ctrl & 0x40000000) // Dll weak bias enabled.
		PMC(APBDEV_PMC_WEAK_BIAS) = 0x7FFF0000;

	// Put DDR3 reset pads into deep power down.
	PMC(APBDEV_PMC_IO_DPD3_REQ) = 0x8FFFFFFF;
	u32 timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
	while (!(PMC(APBDEV_PMC_IO_DPD3_STATUS) == 0xFFFFFFF))
		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			return RET_ERROR;
	usleep(19);

	// Put DDR4 reset pads into deep power down.
	PMC(APBDEV_PMC_IO_DPD4_REQ) = 0x8FFFFFFF;
	timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
	while (!(PMC(APBDEV_PMC_IO_DPD4_STATUS) == 0xFFF1FFF))
		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			return RET_ERROR;
	usleep(19);

	return RET_SUCCESS;
}

static int _sdram_disable_digital_dll(bool ch1_enabled)
{
	u32 timeout;
	u32 cfg_dll_en = EMC_CH0(EMC_CFG_DIG_DLL) & 1;

	// Disable digital dll.
	if (cfg_dll_en)
	{
		if (ch1_enabled)
			EMC(EMC_CFG_DIG_DLL) &= 0xFFFFFFFE;
		else
			EMC_CH0(EMC_CFG_DIG_DLL) &= 0xFFFFFFFE;
	}

	if (_emc_update_timing(ch1_enabled))
		return RET_ERROR;

	// Wait for dll to be disabled.
	if (cfg_dll_en)
	{
		timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
		while (EMC_CH0(EMC_CFG_DIG_DLL) & 1)
			if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
				return RET_ERROR;

		if (ch1_enabled)
		{
			timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
			while (EMC_CH1(EMC_CFG_DIG_DLL) & 1)
				if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
					return RET_ERROR;
		}
	}

	return RET_SUCCESS;
}

static int _config_sdram_dpd()
{
	u32 timeout;
	bool ch1_enabled = (EMC(EMC_FBIO_CFG7) >> 2) & 1;

	// If LPDDR4 save Frequency-Set-Point Operation Mode and Write Enable.
	if ((EMC(EMC_FBIO_CFG5) & 3) == 1)
	{
		u32 mr13_fspop = EMC(EMC_MRW3) & 0xC0; // bit6-7: WR, OP.
		PMC(APBDEV_PMC_SCRATCH18) = (PMC(APBDEV_PMC_SCRATCH18) & 0xFFFFFF3F) | mr13_fspop;

		u32 mr13_fspwr = mr13_fspop | (mr13_fspop << 8);
		PMC(APBDEV_PMC_SCRATCH12) = mr13_fspwr | (PMC(APBDEV_PMC_SCRATCH12) & 0xFFFF3F3F);
		PMC(APBDEV_PMC_SCRATCH13) = mr13_fspwr | (PMC(APBDEV_PMC_SCRATCH13) & 0xFFFF3F3F);
	}

	// Disable linked timing update.
	EMC(EMC_CFG) = EMC(EMC_CFG) & 0xCFFFFFFF;
	if (_emc_update_timing(ch1_enabled))
		return RET_ERROR;
	usleep(5);

	// Disable ZQ calibration.
	if (ch1_enabled)
		EMC(EMC_ZCAL_INTERVAL) = 0;
	else
		EMC_CH0(EMC_ZCAL_INTERVAL) = 0;

	// Set DQS clock.
	EMC(EMC_AUTO_CAL_CONFIG) = (EMC(EMC_AUTO_CAL_CONFIG) & 0x7FFFFFFF) | 0x600;

	// Disable digital dll if enabled.
	if (_sdram_disable_digital_dll(ch1_enabled))
		return RET_ERROR;

	// Stall all reads/writes.
	if (ch1_enabled)
		EMC(EMC_REQ_CTRL) = 3;
	else
		EMC_CH0(EMC_REQ_CTRL) = 3;

	// Wait for all non-stalled requests to finish.
	timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
	while (!(EMC_CH0(EMC_EMC_STATUS) & 4))
		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			return RET_ERROR;

	if (ch1_enabled)
		while (!(EMC_CH1(EMC_EMC_STATUS) & 4))
			if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
				return RET_ERROR;

	// Enable self refresh.
	if (ch1_enabled)
		EMC(EMC_SELF_REF) = 1;
	else
		EMC_CH0(EMC_SELF_REF) = 1;

	// Wait per dram device for self refresh.
	u32 dev_self_rfrs_mask;
	if (EMC(EMC_ADR_CFG)) // dram_dev_num.
		dev_self_rfrs_mask = 0x300;
	else
		dev_self_rfrs_mask = 0x100;

	timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
	while ((EMC_CH0(EMC_EMC_STATUS) & dev_self_rfrs_mask) != dev_self_rfrs_mask )
		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			return RET_ERROR;

	if (ch1_enabled)
		while ((EMC_CH1(EMC_EMC_STATUS) & dev_self_rfrs_mask) != dev_self_rfrs_mask)
			if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
				return RET_ERROR;

	// Enable partial array self refresh to all segments.
	EMC(EMC_MRW) = 0x88110000;     // Send request to 1st dram device.
	if (EMC(EMC_ADR_CFG) & 1)
		EMC(EMC_MRW) = 0x48110000; // Send request to 2nd dram device.

	return RET_SUCCESS;
}

static int _powergate_cpu_rail()
{
	// Powergate CRAIL.
	u32 timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
	while (PMC(APBDEV_PMC_PWRGATE_STATUS) & 1)
		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			return RET_ERROR;

	PMC(APBDEV_PMC_SET_SW_CLAMP) = 1;

	timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
	while (!(PMC(APBDEV_PMC_CLAMP_STATUS) & 1))
		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			return RET_ERROR;
	usleep(9);

	pinmux_config_i2c(I2C_5);

	// Enable I2C5 clock.
	if (!(CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_H) & 0x8000))
	{
		CLOCK(CLK_RST_CONTROLLER_CLK_ENB_H_SET) = 0x8000;
		CLOCK(CLK_RST_CONTROLLER_RST_DEV_H_SET) = 0x8000;
		usleep(2);
		CLOCK(CLK_RST_CONTROLLER_RST_DEV_H_CLR) = 0x8000;
	}

	i2c_init(I2C_5);

	// Disable power to CPU.
	if (PMC(APBDEV_PMC_SCRATCH201) & 2)
		i2c_send_byte(I2C_5, 0x1B, 0x00, 0x00); // Disable pmic_cpu.
	else
		i2c_send_byte(I2C_5, 0x3C, 0x3B, 0x3B); // Disable OVR2.

	// Disable I2C5.
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_H_CLR) = 0x8000;
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_H_SET) = 0x8000;
	usleep(700);

	return RET_SUCCESS;
}

static void _soc_therm_invalidate_cores()
{
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_U_SET) = 0x4000; // Enable clock to SOC_THERM.
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_U_SET) = 0x4000; // Set reset to SOC_THERM.
	usleep(2);
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_U_CLR) = 0x4000; // Clear reset to SOC_THERM.

	SOC_THERM(SOC_THERM_TSENSOR_VALID) |= 0xF;        // Invalidate CPU core TSOSCs.
}

static void _config_exception_vectors()
{
	EXCP_VEC(EVP_COP_RESET_VECTOR) = (u32)&sc7_entry;
	EXCP_VEC(EVP_COP_UNDEF_VECTOR) = (u32)&_halt;
	EXCP_VEC(EVP_COP_PREFETCH_ABORT_VECTOR) = (u32)&_halt;
	EXCP_VEC(EVP_COP_DATA_ABORT_VECTOR) = (u32)&_halt;
	EXCP_VEC(EVP_COP_RSVD_VECTOR) = (u32)&_halt;
	EXCP_VEC(EVP_COP_IRQ_VECTOR) = (u32)&_halt;
	EXCP_VEC(EVP_COP_FIQ_VECTOR) = (u32)&_halt;
	EXCP_VEC(EVP_COP_SWI_VECTOR) = (u32)&_halt;
}

void __attribute__((noreturn)) sc7_entry()
{
	_config_exception_vectors();

	// Clear interrupt enable for BPMP.
	for (u32 i = 0; i < 6; i++)
		IRQ_CTLR(IRQ_CONTROLLER(i) | ICTLR_COP_IER_CLR) = 0xFFFFFFFF;

	FLOW_CTLR(FLOW_CTLR_FC_SEQUENCE_INTERCEPT) = 0xFF; // Clear pending IRQs for CPUs.
	FLOW_CTLR(FLOW_CTLR_FC_SEQUENCE_INTERCEPT) = 0x20000; // Enable IRQ to BPMP for CRAIL gating.
	(void)FLOW_CTLR(FLOW_CTLR_FC_SEQUENCE_INTERCEPT);

	BPMP_CACHE_CTRL(BPMP_CACHE_CONFIG) |= CFG_DISABLE_WRITE_BUFFER | CFG_DISABLE_READ_BUFFER; 

	// Notify TZ that we are alive.
	RES_SEMA(RES_SEMA_SHRD_SMP_SET) = SIGN_OF_LIFE;

	// Wait for Flow Controller to give the signal to start.
	u32 timeout = (u32)TMR(TIMERUS_CNTR_1US) + 1000000;
	while (!(IRQ_CTLR(IRQ_CONTROLLER(0) | ICTLR_ISR) & PRI_ICTLR_FC_INT))
		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			goto halt_bpmp;

	// Check if a pending IRQ for CRAIL is ready to be served.
	if (!(FLOW_CTLR(FLOW_CTLR_FC_SEQUENCE_INTERCEPT) & INTERRUPT_PENDING_CRAIL))
		goto halt_bpmp;

	// Invalidate TSOSC for all CPU cores.
	_soc_therm_invalidate_cores();

	// Clear pending CRAIL irq.
	FLOW_CTLR(FLOW_CTLR_FC_SEQUENCE_INTERCEPT) =
		(FLOW_CTLR(FLOW_CTLR_FC_SEQUENCE_INTERCEPT) & 0xFFFDFF00) | INTERRUPT_PENDING_CRAIL;
	(void)FLOW_CTLR(FLOW_CTLR_FC_SEQUENCE_INTERCEPT);

	if (_powergate_cpu_rail())
		goto halt_bpmp;

	if (_config_sdram_dpd())
		goto halt_bpmp;

	if (_config_io_pad_dpd())
		goto halt_bpmp;

	// Enable warmboot path.
	PMC(APBDEV_PMC_SCRATCH0) |= 1;

	// Set sampling of pads when in deep power down.
	PMC(APBDEV_PMC_DPD_SAMPLE) |= 1;
	(void)PMC(APBDEV_PMC_DPD_SAMPLE);
	usleep(127);

	// Enter deep power down.
	PMC(APBDEV_PMC_DPD_ENABLE) |= 1;

halt_bpmp:
	_halt();
}
