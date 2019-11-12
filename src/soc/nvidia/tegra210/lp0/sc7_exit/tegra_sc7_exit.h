/*
 * Copyright (c) 2018 naehrwert
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

#ifndef _LP0_EXIT_H_
#define _LP0_EXIT_H_

#include <compiler.h>

typedef signed char s8;
typedef short s16;
typedef short SHORT;
typedef int s32;
typedef int INT;
typedef long LONG;
typedef long long int s64;
typedef unsigned char u8;
typedef unsigned char BYTE;
typedef unsigned short u16;
typedef unsigned short WORD;
typedef unsigned short WCHAR;
typedef unsigned int u32;
typedef unsigned int UINT;
typedef unsigned long DWORD;
typedef unsigned long long QWORD;
typedef unsigned long long int u64;
typedef volatile unsigned char vu8;
typedef volatile unsigned short vu16;
typedef volatile unsigned int vu32;

typedef int bool;
#define true  1
#define false 0

#define IRAM_BASE 0x40000000
#define BPMP_CACHE_BASE 0x50040000
#define MSELECT_BASE 0x50060000
#define TMR_BASE 0x60005000
#define CLOCK_BASE 0x60006000
#define FLOW_CTLR_BASE 0x60007000
#define SB_BASE (0x6000C200)
#define GPIO_BASE 0x6000D000
#define EXCP_VEC_BASE 0x6000F000
#define APB_MISC_BASE 0x70000000
#define PINMUX_AUX_BASE 0x70003000
#define UART_BASE 0x70006000
#define RTC_BASE 0x7000E000
#define PMC_BASE 0x7000E400
#define FUSE_BASE 0x7000F800
#define MC_BASE 0x70019000
#define SDMMC2_BASE 0x700B0200
#define SDMMC3_BASE 0x700B0400
#define EMC_BASE 0x7001B000
#define CL_DVFS_BASE 0x70110000
#define I2S_BASE 0x702D1000

#define _REG(base, off) *(vu32 *)((base) + (off))

#define IRAM(off) _REG(IRAM_BASE, off)
#define BPMP_CACHE_CTRL(off) _REG(BPMP_CACHE_BASE, off)
#define MSELECT(off) _REG(MSELECT_BASE, off)
#define TMR(off) _REG(TMR_BASE, off)
#define CLOCK(off) _REG(CLOCK_BASE, off)
#define FLOW_CTLR(off) _REG(FLOW_CTLR_BASE, off)
#define SB(off) _REG(SB_BASE, off)
#define GPIO(off) _REG(GPIO_BASE, off)
#define EXCP_VEC(off) _REG(EXCP_VEC_BASE, off)
#define APB_MISC(off) _REG(APB_MISC_BASE, off)
#define PINMUX_AUX(off) _REG(PINMUX_AUX_BASE, off)
#define RTC(off) _REG(RTC_BASE, off)
#define PMC(off) _REG(PMC_BASE, off)
#define FUSE(off) _REG(FUSE_BASE, off)
#define MC(off) _REG(MC_BASE, off)
#define SDMMC2(off) _REG(SDMMC2_BASE, off)
#define SDMMC3(off) _REG(SDMMC3_BASE, off)
#define EMC(off) _REG(EMC_BASE, off)
#define CL_DVFS(off) _REG(CL_DVFS_BASE, off)
#define I2S(off) _REG(I2S_BASE, off)

/*! MSELECT registers. */
#define MSELECT_CONFIG 0x0

/*! EVP registers. */
#define EVP_CPU_RESET_VECTOR 0x100

/*! Misc registers. */
#define APB_MISC_PP_STRAPPING_OPT_A 0x08
#define APB_MISC_PP_CONFIG_CTL 0x24
#define APB_MISC_PP_PINMUX_GLOBAL 0x40
#define APB_MISC_GP_HIDREV 0x804
#define APB_MISC_GP_ASDBGREG 0x810
#define APB_MISC_GP_LCD_BL_PWM_CFGPADCTRL 0xA34
#define APB_MISC_GP_SDMMC1_PAD_CFGPADCTRL 0xA98
#define APB_MISC_GP_EMMC4_PAD_CFGPADCTRL 0xAB4
#define APB_MISC_GP_EMMC4_PAD_PUPD_CFGPADCTRL 0xABC
#define APB_MISC_GP_WIFI_EN_CFGPADCTRL 0xB64
#define APB_MISC_GP_WIFI_RST_CFGPADCTRL 0xB68

/*! System registers. */
#define AHB_ARBITRATION_XBAR_CTRL 0xE0
#define AHB_AHB_SPARE_REG 0x110

/*! Secure boot registers. */
#define SB_CSR 0x0
#define  SB_CSR_NS_RST_VEC_WR_DIS    (1 << 1)
#define  SB_CSR_PIROM_DISABLE        (1 << 4)
#define SB_PFCFG 0x8
#define SB_AA64_RESET_LOW  0x30
#define  SB_AA64_RST_AARCH64_MODE_EN (1 << 0)
#define SB_AA64_RESET_HIGH 0x34

/*! TMR registers. */
#define TIMERUS_CNTR_1US          (0x10 + 0x0)
#define TIMERUS_USEC_CFG          (0x10 + 0x4)
#define TIMER_TMR9_TMR_PTV        0x80
#define  TIMER_EN     (1 << 31)
#define  TIMER_PER_EN (1 << 30)
#define TIMER_WDT4_CONFIG         (0x100 + 0x80)
#define  TIMER_SRC(TMR) (TMR & 0xF)
#define  TIMER_PER(PER) ((PER & 0xFF) << 4)
#define  TIMER_SYSRESET_EN (1 << 14)
#define  TIMER_PMCRESET_EN (1 << 15)
#define TIMER_WDT4_COMMAND        (0x108 + 0x80)
#define  TIMER_START_CNT   (1 << 0)
#define  TIMER_CNT_DISABLE (1 << 1)
#define TIMER_WDT4_UNLOCK_PATTERN (0x10C + 0x80)
#define  TIMER_MAGIC_PTRN 0xC45A

/*! I2S registers. */
#define I2S1_CG   0x88
#define I2S1_CTRL 0xA0
#define I2S2_CG   0x188
#define I2S2_CTRL 0x1A0
#define I2S3_CG   0x288
#define I2S3_CTRL 0x2A0
#define I2S4_CG   0x388
#define I2S4_CTRL 0x3A0
#define I2S5_CG   0x488
#define I2S5_CTRL 0x4A0
#define  I2S_CG_SLCG_ENABLE (1 << 0)
#define  I2S_CTRL_MASTER_EN (1 << 10)

/*! Flow controller registers. */
#define FLOW_CTLR_HALT_COP_EVENTS  0x4
#define  HALT_COP_SEC        (1 << 23)
#define  HALT_COP_MSEC       (1 << 24)
#define  HALT_COP_USEC       (1 << 25)
#define  HALT_COP_JTAG       (1 << 28)
#define  HALT_COP_WAIT_EVENT (1 << 30)
#define  HALT_COP_WAIT_IRQ   (1 << 31)
#define  HALT_COP_MAX_CNT        0xFF
#define FLOW_CTLR_RAM_REPAIR 0x40
#define FLOW_CTLR_BPMP_CLUSTER_CONTROL 0x98

/*! Pinmux registers. */
#define PINMUX_AUX_DVFS_PWM 0x184
#define PINMUX_AUX_GPIO_PA6 0x244
#define  PINMUX_PULL_MASK    (3 << 2)
#define  PINMUX_PULL_NONE    (0 << 2)
#define  PINMUX_PULL_DOWN    (1 << 2)
#define  PINMUX_PULL_UP      (2 << 2)
#define  PINMUX_TRISTATE     (1 << 4)
#define  PINMUX_PARKED       (1 << 5)
#define  PINMUX_INPUT_ENABLE (1 << 6)
#define  PINMUX_LOCK         (1 << 7)
#define  PINMUX_LPDR         (1 << 8)
#define  PINMUX_HSM          (1 << 9)
#define  PINMUX_IO_HV        (1 << 10)
#define  PINMUX_OPEN_DRAIN   (1 << 11)
#define  PINMUX_SCHMT        (1 << 12)
#define  PINMUX_DRIVE_1X     (0 << 13)
#define  PINMUX_DRIVE_2X     (1 << 13)
#define  PINMUX_DRIVE_3X     (2 << 13)
#define  PINMUX_DRIVE_4X     (3 << 13)
/*! 0:UART-A, 1:UART-B, 3:UART-C, 3:UART-D */
#define PINMUX_AUX_UARTX_TX(x)  (0xE4 + 0x10 * (x))
#define PINMUX_AUX_UARTX_RX(x)  (0xE8 + 0x10 * (x))
#define PINMUX_AUX_UARTX_RTS(x) (0xEC + 0x10 * (x))
#define PINMUX_AUX_UARTX_CTS(x) (0xF0 + 0x10 * (x))
/*! 0:GEN1, 1:GEN2, 2:GEN3, 3:CAM, 4:PWR */
#define PINMUX_AUX_X_I2C_SCL(x) (0xBC + 8 * (x))
#define PINMUX_AUX_X_I2C_SDA(x) (0xC0 + 8 * (x))

/*! PMC registers. */
#define APBDEV_PMC_SEC_DISABLE 0x4
#define APBDEV_PMC_DPD_SAMPLE 0x20
#define APBDEV_PMC_CLAMP_STATUS 0x2C
#define APBDEV_PMC_PWRGATE_TOGGLE 0x30
#define APBDEV_PMC_REMOVE_CLAMPING_CMD 0x34
#define APBDEV_PMC_PWRGATE_STATUS 0x38
#define APBDEV_PMC_CPUPWRGOOD_TIMER 0xC8
#define APBDEV_PMC_OSC_EDPD_OVER 0x1A4
#define APBDEV_PMC_STICKY_BITS 0x2C0
#define APBDEV_PMC_SEC_DISABLE2 0x2C4
#define APBDEV_PMC_SECURE_SCRATCH21 0x334
#define APBDEV_PMC_SECURE_SCRATCH34 0x368
#define APBDEV_PMC_SECURE_SCRATCH35 0x36C
#define APBDEV_PMC_SET_SW_CLAMP 0x47C
#define APBDEV_PMC_SCRATCH190 0x818
#define APBDEV_PMC_SCRATCH201 0x844

/*! Clock registers. */
#define CLK_RST_CONTROLLER_RST_DEVICES_L 0x4
#define CLK_RST_CONTROLLER_RST_DEVICES_U 0xC
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_L 0x10
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_H 0x14
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_U 0x18
#define CLK_RST_CONTROLLER_MISC_CLK_ENB 0x48
#define CLK_RST_CONTROLLER_OSC_CTRL 0x50
#define CLK_RST_CONTROLLER_PLLX_BASE 0xE0
#define CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRA 0xF8
#define CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRB 0xFC
#define CLK_RST_CONTROLLER_CLK_SOURCE_I2C5 0x128
#define CLK_RST_CONTROLLER_CLK_SOURCE_UARTB 0x17C
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_X 0x280
#define CLK_RST_CONTROLLER_CLK_ENB_X_CLR 0x288
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_Y 0x298
#define CLK_RST_CONTROLLER_CLK_ENB_Y_SET 0x29C
#define CLK_RST_CONTROLLER_CLK_ENB_Y_CLR 0x2A0
#define CLK_RST_CONTROLLER_RST_DEV_Y_CLR 0x2AC
#define CLK_RST_CONTROLLER_RST_DEV_L_SET 0x300
#define CLK_RST_CONTROLLER_RST_DEV_L_CLR 0x304
#define CLK_RST_CONTROLLER_RST_DEV_H_SET 0x308
#define CLK_RST_CONTROLLER_RST_DEV_H_CLR 0x30C
#define CLK_RST_CONTROLLER_RST_DEV_U_SET 0x310
#define CLK_RST_CONTROLLER_RST_DEV_U_CLR 0x314
#define CLK_RST_CONTROLLER_CLK_ENB_L_SET 0x320
#define CLK_RST_CONTROLLER_CLK_ENB_L_CLR 0x324
#define CLK_RST_CONTROLLER_CLK_ENB_H_SET 0x328
#define CLK_RST_CONTROLLER_CLK_ENB_H_CLR 0x32C
#define CLK_RST_CONTROLLER_CLK_ENB_U_SET 0x330
#define CLK_RST_CONTROLLER_CLK_ENB_U_CLR 0x334
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_V 0x360
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_W 0x364
#define CLK_RST_CONTROLLER_CCLKG_BURST_POLICY 0x368
#define CLK_RST_CONTROLLER_SUPER_CCLKG_DIVIDER 0x36C
#define CLK_RST_CONTROLLER_CCLKLP_BURST_POLICY 0x370
#define CLK_RST_CONTROLLER_SUPER_CCLKLP_DIVIDER 0x374
#define CLK_RST_CONTROLLER_CPU_SOFTRST_CTRL2 0x388
#define CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRC 0x3A0
#define CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRD 0x3A4
#define CLK_RST_CONTROLLER_CLK_SOURCE_MSELECT 0x3B4
#define CLK_RST_CONTROLLER_RST_DEV_V_CLR 0x434
#define CLK_RST_CONTROLLER_CLK_ENB_V_SET 0x440
#define CLK_RST_CONTROLLER_CLK_ENB_V_CLR 0x444
#define CLK_RST_CONTROLLER_CLK_ENB_W_SET 0x448
#define CLK_RST_CONTROLLER_CLK_ENB_W_CLR 0x44C
#define CLK_RST_CONTROLLER_RST_CPUG_CMPLX_CLR 0x454
#define CLK_RST_CONTROLLER_LVL2_CLK_GATE_OVRE 0x554
#define CLK_RST_CONTROLLER_SPARE_REG0 0x55C
#define CLK_RST_CONTROLLER_CLK_SOURCE_DVFS_REF 0x62C
#define CLK_RST_CONTROLLER_CLK_SOURCE_DVFS_SOC 0x630

/*! Fuse registers. */
#define FUSE_FUSEBYPASS 0x24
#define FUSE_PRIVATEKEYDISABLE 0x28
#define FUSE_DISABLEREGPROGRAM 0x2C
#define FUSE_WRITE_ACCESS_SW 0x30
#define FUSE_SECURITY_MODE 0x1A0

/*! MC registers. */
#define MC_INTSTATUS 0x0
#define MC_INTMASK 0x4
#define MC_VIDEO_PROTECT_SIZE_MB 0x64C
#define MC_VIDEO_PROTECT_REG_CTRL 0x650

/*! EMC registers. */
#define EMC_FBIO_CFG7 0x584
#define EMC_PMACRO_CFG_PM_GLOBAL_0 0xC30
#define EMC_PMACRO_TRAINING_CTRL_0 0xCF8
#define EMC_PMACRO_TRAINING_CTRL_1 0xCFC

/*! SDMMC registers. */
#define SDMMC_VENDOR_IO_TRIM_CNTRL 0x1AC
#define SDMMC_SDMEMCOMPPADCTRL 0x1E0

/*! GPIO registers. */

typedef struct _fuse_storage_t
{
	u32 address;
	u32 value;
} fuse_storage_t;

/*! Generic clock descriptor. */
typedef struct _clock_t
{
	u32 reset;
	u32 enable;
	u32 source;
	u8 index;
	u8 clk_src;
	u8 clk_div;
} clock_t;

void usleep(u32 us);

#endif
