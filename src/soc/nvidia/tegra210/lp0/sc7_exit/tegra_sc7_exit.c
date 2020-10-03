/*
 * Copyright (c) 2018-2019 CTCaer
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

#include "tegra_sc7_exit.h"
#include "i2c.h"
#include "uart.h"

#define UARTB_PRINT
//#define COREBOOT_EXTRA
//#define FUSE_BYPASS_ENABLE

extern void _start();

// LP0 Header
extern u8 blob_data;
extern u8 blob_data_size;
extern u8 blob_total_size;

struct lp0_header
{
	u32 length_insecure;	  // Insecure total length.
	u32 reserved[3];
	u8  rsa_modulus[256];	  // RSA key modulus.
	u8  aes_signature[16];	  // AES signature.
	u8  rsa_signature[256];	  // RSA-PSS signature.
	u8  random_aes_block[16]; // Random data, may be zero.
	u32 length_secure;		  // Secure total length.
	u32 destination;		  // Where to load the blob in iRAM.
	u32 entry_point;		  // Entry point for the blob.
	u32 code_length;		  // Length of just the data.
} __packed;

struct lp0_header header __attribute__((section(".header"))) =
{
	.length_insecure = (u32)&blob_total_size,
	.length_secure   = (u32)&blob_total_size,
	.destination     = (u32)&blob_data,
	.entry_point     = (u32)&_start,
	.code_length     = (u32)&blob_data_size
};

char uart_cldvfs_pmic[] = "SC7 Exit: CLDVFS and PMIC...\r\n";
char uart_cpu0_bringup[] = "SC7 Exit: CPU0 power bringup...\r\n";
char uart_done[] = "SC7 Exit: done!\r\n";

fuse_storage_t fuse_bypass_storage[] = {};

void usleep(u32 us)
{
	u32 start = TMR(TIMERUS_CNTR_1US);

	while ((u32)(TMR(TIMERUS_CNTR_1US) - start) <= us) // Casting to u32 is important!
		;
}

void _config_uart_b()
{
	PINMUX_AUX(PINMUX_AUX_UARTX_TX(UART_B)) = 0;
	PINMUX_AUX(PINMUX_AUX_UARTX_RX(UART_B)) = PINMUX_INPUT_ENABLE | PINMUX_PULL_UP;
	PINMUX_AUX(PINMUX_AUX_UARTX_RTS(UART_B)) = 0;
	PINMUX_AUX(PINMUX_AUX_UARTX_CTS(UART_B)) = PINMUX_INPUT_ENABLE | PINMUX_PULL_DOWN;

	CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_UARTB) = 0x2;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_L_SET) = 0x80;
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_L_SET) = 0x80;
	usleep(2);
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_L_CLR) = 0x80;

	uart_init(UART_B, 115200);
}

int _cluster_pmc_enable_partition(u32 part)
{
	u32 part_mask = 1 << part;

	// Check if the partition has the state we want.
	if (PMC(APBDEV_PMC_PWRGATE_STATUS) & part_mask)
		return 1;

	// Wait for any pending power gate toggles.
	u32 timeout = (u32)TMR(TIMERUS_CNTR_1US) + 10000;
	while (PMC(APBDEV_PMC_PWRGATE_TOGGLE) & 0x100)
		if ((u32)TMR(TIMERUS_CNTR_1US) > timeout)
			return 0;

	// Toggle power gating.
	PMC(APBDEV_PMC_PWRGATE_TOGGLE) = part | 0x100;

	while (!(PMC(APBDEV_PMC_PWRGATE_STATUS) & part_mask))
		;

	// Remove clamping.
	PMC(APBDEV_PMC_REMOVE_CLAMPING_CMD) = part_mask;
	while (PMC(APBDEV_PMC_CLAMP_STATUS) & part_mask)
		;

	return 1;
}

void _config_security()
{
	// Disable access to secure registers.
	if (FUSE(FUSE_SECURITY_MODE))
	{
		u8 secure_feature_cfg = 0;
		u8 cfg_control = 0;
		
		if (PMC(APBDEV_PMC_STICKY_BITS) & 0x40) // Jtag disable.
		{
			secure_feature_cfg = 4;
			cfg_control = 0;
		}
		else
		{
			secure_feature_cfg = 0xD;
			cfg_control = 0x40; // Jtag enable.
		}
		SB(SB_PFCFG) = (SB(SB_PFCFG) & 0xFFFFFFF0) | secure_feature_cfg;
		APB_MISC(APB_MISC_PP_CONFIG_CTL) |= cfg_control;
	}

	PMC(APBDEV_PMC_STICKY_BITS) |= FUSE(FUSE_SECURITY_MODE) & 1; // Disable HDA codec loopback if secure mode.
}

void _fuse_bypass()
{
	if (!(FUSE(FUSE_DISABLEREGPROGRAM) & 1))
	{
		// Enable write access,write status and bypass.
		FUSE(FUSE_WRITE_ACCESS_SW) = (FUSE(FUSE_WRITE_ACCESS_SW) & 0xFFFFFFFE) | 0x10000;
		FUSE(FUSE_FUSEBYPASS) = 1;

		// Write fuses to cached registers.
		fuse_storage_t *fuses = fuse_bypass_storage;
		u32 fuses_size = sizeof(fuse_bypass_storage) / sizeof(fuse_storage_t);
		while (fuses_size)
		{
			FUSE(fuses->address) = fuses->value;
			fuses++;
			fuses_size--;
		}

		// Disable write access and bypass.
		FUSE(FUSE_WRITE_ACCESS_SW) |= 1;
		FUSE(FUSE_FUSEBYPASS) = 0;

		// Disable programming and set TZ_STICKY_BIT.
		FUSE(FUSE_DISABLEREGPROGRAM) = 1;
		FUSE(FUSE_PRIVATEKEYDISABLE) = (PMC(APBDEV_PMC_SECURE_SCRATCH21) & 0x10) | FUSE(FUSE_PRIVATEKEYDISABLE);

		// Relock APBDEV_PMC_SECURE_SCRATCH21 and read/writes to PMC_SECURE_SCRATCH 0-7.
		PMC(APBDEV_PMC_SEC_DISABLE2) |= 0x4000000;
		PMC(APBDEV_PMC_SEC_DISABLE)  |= 0xFFFF0;
	}
}

void _config_oscillators()
{
	CLOCK(CLK_RST_CONTROLLER_OSC_CTRL) = (CLOCK(CLK_RST_CONTROLLER_OSC_CTRL) & 0xFFFFFC0E) | (16 * ((PMC(APBDEV_PMC_OSC_EDPD_OVER) >> 1) & 0x3F)) | 1;
	CLOCK(CLK_RST_CONTROLLER_SPARE_REG0) = 4; // Set CLK_M_DIVISOR to 2.
	(void)CLOCK(CLK_RST_CONTROLLER_SPARE_REG0);
	TMR(TIMERUS_USEC_CFG) = 0x45F; // For 19.2MHz clk_m.
}

void _restore_ram()
{
	APB_MISC(APB_MISC_GP_ASDBGREG) = (APB_MISC(APB_MISC_GP_ASDBGREG) & 0xFCFFFFFF) | 0x2000000; // Restore RAM SVOP.
	EMC(EMC_PMACRO_CFG_PM_GLOBAL_0) = 0xFF0000; // Enable configuration to all BYTE pad macros.
	EMC(EMC_PMACRO_TRAINING_CTRL_0) = 8; // Set training control for channel 0.
	EMC(EMC_PMACRO_TRAINING_CTRL_1) = 8; // Set training control for channel 1.
	EMC(EMC_PMACRO_CFG_PM_GLOBAL_0) = 0; // Disable configuration to all BYTE pad macros.
}

void _sdmmc23_set_low_power()
{
	// Disable SDMMC2/3 for power save.
	CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_L) |= 0x200; // SDMMC2
	usleep(2);
	CLOCK(CLK_RST_CONTROLLER_RST_DEVICES_L) &= 0xFFFFFDFF;
	SDMMC2(SDMMC_VENDOR_IO_TRIM_CNTRL) &= 0xFFFFFFFB;
	SDMMC2(SDMMC_SDMEMCOMPPADCTRL) &= 0x7FFFFFFF;
	(void)SDMMC2(SDMMC_SDMEMCOMPPADCTRL);
	CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_L) &= 0xFFFFFDFF;

	CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_U) |= 0x20u; // SDMMC3
	usleep(2);
	CLOCK(CLK_RST_CONTROLLER_RST_DEVICES_U) &= 0xFFFFFFDF;
	SDMMC3(SDMMC_VENDOR_IO_TRIM_CNTRL) &= 0xFFFFFFFB;
	SDMMC3(SDMMC_SDMEMCOMPPADCTRL) &= 0x7FFFFFFF;
	(void)SDMMC3(SDMMC_SDMEMCOMPPADCTRL);
	CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_U) &= 0xFFFFFFDF;
}

void _mbist_workaround()
{
#ifdef COREBOOT_EXTRA
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_Y_CLR) = 0x40; // Clear reset on APE clock.
	usleep(2);

	// I2S channels to master and disable SLCG.
	I2S(I2S1_CTRL) |= I2S_CTRL_MASTER_EN;
	I2S(I2S1_CG)   &= ~I2S_CG_SLCG_ENABLE;
	I2S(I2S2_CTRL) |= I2S_CTRL_MASTER_EN;
	I2S(I2S2_CG)   &= ~I2S_CG_SLCG_ENABLE;
	I2S(I2S3_CTRL) |= I2S_CTRL_MASTER_EN;
	I2S(I2S3_CG)   &= ~I2S_CG_SLCG_ENABLE;
	I2S(I2S4_CTRL) |= I2S_CTRL_MASTER_EN;
	I2S(I2S4_CG)   &= ~I2S_CG_SLCG_ENABLE;
	I2S(I2S5_CTRL) |= I2S_CTRL_MASTER_EN;
	I2S(I2S5_CG)   &= ~I2S_CG_SLCG_ENABLE;
	usleep(2);
#endif

	// Disable clock gate overrides.
	CLOCK(CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRA) = 0;
	CLOCK(CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB) = 0;
#ifdef COREBOOT_EXTRA
	// Enable clock gate overrides.
	CLOCK(CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRC) = (1 << 1);  // AHUB.
	CLOCK(CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD) = (1 << 24); // QSPI.
	CLOCK(CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRE) = (3 << 10); // ADSP, APE.
#else
	CLOCK(CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRC) = 0;
	CLOCK(CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD) = 0;
	CLOCK(CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRE) = 0;
#endif

	// Clear reset for all enabled modules.
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_L_CLR) = CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_L) & 0x7FFFFECF;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_H_CLR) = CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_H) & 0xFDFFFF3E;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_U_CLR) = CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_U) & 0xFE0FFDFF;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_V_CLR) = CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_V) & 0x7FBFFFF7;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_W_CLR) = CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_W) & 0xFFDFFF03;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_X_CLR) = CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_X) & 0xDCFFB87F;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_Y_CLR) = CLOCK(CLK_RST_CONTROLLER_CLK_OUT_ENB_Y) & 0xFFFFFCFF;

	bool channel1_enabled = (EMC(EMC_FBIO_CFG7) >> 2) & 1;
	if (channel1_enabled)
		CLOCK(CLK_RST_CONTROLLER_CLK_ENB_W_SET) = 0x40000000;
}

void _cluster_cpu0_clock_bringup()
{
	// Set reset for CoreSight.
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_U_SET) = 0x200;

	// CAR2PMC_CPU_ACK_WIDTH should be set to 0.
	CLOCK(CLK_RST_CONTROLLER_CPU_SOFTRST_CTRL2) &= 0xFFFFF000;

	 // Set Active cluster to FAST.
	FLOW_CTLR(FLOW_CTLR_BPMP_CLUSTER_CONTROL) &= 0xFFFFFFFE;

	// Set reset vector.
	SB(SB_AA64_RESET_LOW) = PMC(APBDEV_PMC_SECURE_SCRATCH34) | SB_AA64_RST_AARCH64_MODE_EN;
	SB(SB_AA64_RESET_HIGH) = PMC(APBDEV_PMC_SECURE_SCRATCH35);

	// Set core dividers to both clusters.
	CLOCK(CLK_RST_CONTROLLER_SUPER_CCLKG_DIVIDER) = 0x80000000;
	CLOCK(CLK_RST_CONTROLLER_SUPER_CCLKLP_DIVIDER) = 0x80000000;

	// Enable CoreSight clock.
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_U_SET) = 0x200;
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_U_CLR) = 0x200;

	// Configure MSELECT source and enable clock.
	CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_MSELECT) = 6;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_V_SET) = 8;
	usleep(2);
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_V_CLR) = 8;

	// Enable WRAP type and its conversion to INCR type.
	MSELECT(MSELECT_CONFIG) = (MSELECT(MSELECT_CONFIG) & 0xFCFFFFFF) | 0x38000000;

	// Disable PLLX. It's not used as CPU parent clock.
	CLOCK(CLK_RST_CONTROLLER_PLLX_BASE) &= 0xBFFFFFFF;
}

void _config_cldvfs_and_pmic()
{
	// Set CLDVFS and i2c to PMU pins
	PINMUX_AUX(PINMUX_AUX_DVFS_PWM) = 0x11;
	(void)PINMUX_AUX(PINMUX_AUX_DVFS_PWM);

	PINMUX_AUX(PINMUX_AUX_X_I2C_SCL(I2C_5)) = PINMUX_INPUT_ENABLE;
	PINMUX_AUX(PINMUX_AUX_X_I2C_SDA(I2C_5)) = PINMUX_INPUT_ENABLE;

	// Enable clock to CLDVFS.
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_W_SET) = 0x8000000;
	CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_DVFS_REF) = 14;
	CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_DVFS_SOC) = 14;

	// Enable clock to I2C5
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_H_SET) = 0x8000;
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_H_SET) = 0x8000;
	usleep(5);
	CLOCK(CLK_RST_CONTROLLER_CLK_SOURCE_I2C5) = 4;
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_H_CLR) = 0x8000;

	i2c_init(I2C_5);

	if (PMC(APBDEV_PMC_SCRATCH201) & 2) // add cpu disable on top. in case we have that bs here
		i2c_send_byte(I2C_5, 0x1B, 0x00, 0x80);
	else
		i2c_send_byte(I2C_5, 0x3C, 0x3B, 0x9);
	usleep(2000);
}

void _disable_cldvfs_and_i2c5()
{
	// Disable I2C5 clock.
	CLOCK(CLK_RST_CONTROLLER_RST_DEV_H_SET) = 0x8000;
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_H_CLR) = 0x8000;

	// Disable clock to CLDVFS.
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_W_CLR) = 0x8000000;
}

void _cluster_cpu0_power_bringup()
{
#ifdef COREBOOT_EXTRA
	// Reprogram PMC_CPUPWRGOOD_TIMER based on 204Mhz pclk and restore it later.
	u32 orig_timer = PMC(APBDEV_PMC_CPUPWRGOOD_TIMER);
	PMC(APBDEV_PMC_CPUPWRGOOD_TIMER) = orig_timer * (204000000 / 32768);
#endif

	// Enable CPU rail.
	_cluster_pmc_enable_partition(0);

	// Remove clamping from CPU rail.
	PMC(APBDEV_PMC_SET_SW_CLAMP) = 0;
	PMC(APBDEV_PMC_REMOVE_CLAMPING_CMD) = 1;
	while (PMC(APBDEV_PMC_CLAMP_STATUS) & 1)
		;
	usleep(8);

	_disable_cldvfs_and_i2c5();
	
	// Request and wait for RAM repair.
	FLOW_CTLR(FLOW_CTLR_RAM_REPAIR) = 1;
	while (!(FLOW_CTLR(FLOW_CTLR_RAM_REPAIR) & 2))
		;

	// Enable cluster 0 non-CPU.
	_cluster_pmc_enable_partition(15);

	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_Y_SET) = 0x80000000; // Enable PLLP branches to CPU.
	usleep(2);

	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_L_SET) = 1; // Enable CPU.
	CLOCK(CLK_RST_CONTROLLER_CLK_ENB_V_SET) = 1; // Enable CPUG.
	usleep(10);

	// Set burst dividers policies to both clusters.
	CLOCK(CLK_RST_CONTROLLER_CCLKG_BURST_POLICY) = 0x20004444;
	CLOCK(CLK_RST_CONTROLLER_CCLKLP_BURST_POLICY) = 0x20004444;
	usleep(10);

	// Clear reset to all non CPU region of the CPU complex.
	CLOCK(CLK_RST_CONTROLLER_RST_CPUG_CMPLX_CLR) = 0x20000000;

	// Enable CE0.
	_cluster_pmc_enable_partition(14);

#ifdef COREBOOT_EXTRA
	// Restore the original PMC_CPUPWRGOOD_TIMER.
	PMC(APBDEV_PMC_CPUPWRGOOD_TIMER) = orig_timer;
#endif
}

void __attribute__((noreturn)) lp0_exit()
{
#ifdef COREBOOT_EXTRA
	APB_MISC(APB_MISC_PP_CONFIG_CTL) = 0x40; // Enable JTAG.
#else
	APB_MISC(APB_MISC_PP_CONFIG_CTL) = 0x80; // Enable RTCK Daisy chaining.
#endif

	_config_security();

	PINMUX_AUX(PINMUX_AUX_GPIO_PA6) |= PINMUX_INPUT_ENABLE;

#ifdef FUSE_BYPASS_ENABLE
	// Make fuses visible.
	CLOCK(CLK_RST_CONTROLLER_MISC_CLK_ENB) = (CLOCK(CLK_RST_CONTROLLER_MISC_CLK_ENB) & 0xEFFFFFFF) | (1 << 28);
	_fuse_bypass();
#endif

	_config_oscillators();

	_restore_ram();

	_mbist_workaround();

	_cluster_cpu0_clock_bringup();

	PMC(APBDEV_PMC_SCRATCH190) &= 0xFFFFFFFE; // Clear PMC Scratch 190 bit0.
	PMC(APBDEV_PMC_DPD_SAMPLE) = 0; // Disable deep power down sampling of pads.
	usleep(10);

#ifdef UARTB_PRINT
	_config_uart_b();
#endif

#ifdef COREBOOT_EXTRA
	_sdmmc23_set_low_power();
#endif

#ifdef COREBOOT_EXTRA
	// Clear MC interrupts.
	if (!MC(MC_INTMASK))
	{
		u32 mc_int = MC(MC_INTSTATUS);
		if (mc_int)
			MC(MC_INTSTATUS) = mc_int;
	}

	//! WTF
	{
		// Set both _ACCESS bits so that kernel/secure code can reconfig VPR careveout as needed from the TrustZone.
		MC(MC_VIDEO_PROTECT_SIZE_MB) = 0; // Set size to 0MB.
		MC(MC_VIDEO_PROTECT_REG_CTRL) = 3; // VPR_WR_ACCESS_DISABLE | VPR_ALLOW_TZ_WR_ACCESS.
	}
#endif

#ifdef UARTB_PRINT
	uart_send(UART_B, uart_cldvfs_pmic, sizeof(uart_cldvfs_pmic));
#endif

	_config_cldvfs_and_pmic();

#ifdef UARTB_PRINT
	uart_send(UART_B, uart_cpu0_bringup, sizeof(uart_cpu0_bringup));
#endif

	_cluster_cpu0_power_bringup();

#ifdef UARTB_PRINT
	uart_send(UART_B, uart_done, sizeof(uart_done));
#endif

	// Clear CPU0 CORE and POR resets.
	CLOCK(CLK_RST_CONTROLLER_RST_CPUG_CMPLX_CLR) = 0x10001;

	__asm__ ("" : : "" (header)); //! Important: Preserve LP0 header!

	// Halt BPMP.
	while (true)
		FLOW_CTLR(FLOW_CTLR_HALT_COP_EVENTS) = HALT_COP_JTAG | HALT_COP_WAIT_EVENT;
}
