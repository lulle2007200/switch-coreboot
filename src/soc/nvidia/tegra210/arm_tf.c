/*
 * This file is part of the coreboot project.
 *
 * Copyright 2015 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <arch/cache.h>
#include <arm_tf.h>
#include <assert.h>
#include <cbmem.h>
#include <cbfs.h>
#include <arch/io.h>
#include <soc/addressmap.h>
#include <soc/console_uart.h>
#include <soc/mtc.h>
#include <stdlib.h>
#include <string.h>
#include <symbols.h>

static const uint8_t retail_pkc_modulus[0x100] = {
    0xF7, 0x86, 0x47, 0xAB, 0x71, 0x89, 0x81, 0xB5, 0xCF, 0x0C, 0xB0, 0xE8, 0x48, 0xA7, 0xFD, 0xAD,
    0xCB, 0x4E, 0x4A, 0x52, 0x0B, 0x1A, 0x8E, 0xDE, 0x41, 0x87, 0x6F, 0xB7, 0x31, 0x05, 0x5F, 0xAA,
    0xEA, 0x97, 0x76, 0x21, 0x20, 0x2B, 0x40, 0x48, 0x76, 0x55, 0x35, 0x03, 0xFE, 0x7F, 0x67, 0x62,
    0xFD, 0x4E, 0xE1, 0x22, 0xF8, 0xF0, 0x97, 0x39, 0xEF, 0xEA, 0x47, 0x89, 0x3C, 0xDB, 0xF0, 0x02,
    0xAD, 0x0C, 0x96, 0xCA, 0x82, 0xAB, 0xB3, 0xCB, 0x98, 0xC8, 0xDC, 0xC6, 0xAC, 0x5C, 0x93, 0x3B,
    0x84, 0x3D, 0x51, 0x91, 0x9E, 0xC1, 0x29, 0x22, 0x95, 0xF0, 0xA1, 0x51, 0xBA, 0xAF, 0x5D, 0xC3,
    0xAB, 0x04, 0x1B, 0x43, 0x61, 0x7D, 0xEA, 0x65, 0x95, 0x24, 0x3C, 0x51, 0x3E, 0x8F, 0xDB, 0xDB,
    0xC1, 0xC4, 0x2D, 0x04, 0x29, 0x5A, 0xD7, 0x34, 0x6B, 0xCC, 0xF1, 0x06, 0xF9, 0xC9, 0xE1, 0xF9,
    0x61, 0x52, 0xE2, 0x05, 0x51, 0xB1, 0x3D, 0x88, 0xF9, 0xA9, 0x27, 0xA5, 0x6F, 0x4D, 0xE7, 0x22,
    0x48, 0xA5, 0xF8, 0x12, 0xA2, 0xC2, 0x5A, 0xA0, 0xBF, 0xC8, 0x76, 0x4B, 0x66, 0xFE, 0x1C, 0x73,
    0x00, 0x29, 0x26, 0xCD, 0x18, 0x4F, 0xC2, 0xB0, 0x51, 0x77, 0x2E, 0x91, 0x09, 0x1B, 0x41, 0x5D,
    0x89, 0x5E, 0xEE, 0x24, 0x22, 0x47, 0xE5, 0xE5, 0xF1, 0x86, 0x99, 0x67, 0x08, 0x28, 0x42, 0xF0,
    0x58, 0x62, 0x54, 0xC6, 0x5B, 0xDC, 0xE6, 0x80, 0x85, 0x6F, 0xE2, 0x72, 0xB9, 0x7E, 0x36, 0x64,
    0x48, 0x85, 0x10, 0xA4, 0x75, 0x38, 0x79, 0x76, 0x8B, 0x51, 0xD5, 0x87, 0xC3, 0x02, 0xC9, 0x1B,
    0x93, 0x22, 0x49, 0xEA, 0xAB, 0xA0, 0xB5, 0xB1, 0x3C, 0x10, 0xC4, 0x71, 0xF0, 0xF1, 0x81, 0x1A,
    0x3A, 0x9C, 0xFC, 0x51, 0x61, 0xB1, 0x4B, 0x18, 0xB2, 0x3D, 0xAA, 0xD6, 0xAC, 0x72, 0x26, 0xB7
};

typedef struct bl31_plat_params {
	/* TZ memory size */
	uint64_t tzdram_size;
	/* TZ memory base */
	uint64_t tzdram_base;
	/* UART port ID */
	int uart_id;
	/* L2 ECC parity protection disable flag */
	int32_t l2_ecc_parity_prot_dis;
	/* SHMEM base address for storing the boot logs */
	uint64_t boot_profiler_shmem_base;
	/* System Suspend Entry Firmware size */
	uint64_t sc7entry_fw_size;
	/* System Suspend Entry Firmware base address */
	uint64_t sc7entry_fw_base;
	/* System Suspend Wakeup Firmware size */
	uint64_t warmboot_fw_size;
	/* System Suspend Wakeup Firmware base address */
	uint64_t warmboot_fw_base;
	/* EMC Table size */
	uint64_t emc_table_size;
	/* EMC Table base address */
	uint64_t emc_table_base;
	/* Rebootstub size */
	uint64_t rebootstub_size;
	/* Rebootstub base address */
	uint64_t rebootstub_base;
} bl31_plat_params_t;

static bl31_plat_params_t t210_plat_params;

void *soc_get_bl31_plat_params(bl31_params_t *params)
{
	uintptr_t tz_base_mib;
	size_t tz_size_mib;
	int uart_id = 0;
	struct cbfsf sc7entry_fw, warmboot_fw, rebootstub;
	struct region_device fh;

	carveout_range(CARVEOUT_TZ, &tz_base_mib, &tz_size_mib);

	assert(tz_size_mib < 4096);

	switch (console_uart_get_id()) {
	case UART_ID_NONE:
		break;
	case UART_ID_A:
		uart_id = 1;
		break;
	case UART_ID_B:
		uart_id = 2;
		break;
	case UART_ID_C:
		uart_id = 3;
		break;
	case UART_ID_D:
		uart_id = 4;
		break;
	case UART_ID_E:
		uart_id = 5;
		break;
	}

	t210_plat_params.tzdram_size = (tz_size_mib - 1) * MiB;
	t210_plat_params.tzdram_base = (tz_base_mib + 1) * MiB;
	t210_plat_params.uart_id = uart_id;
	t210_plat_params.sc7entry_fw_base = (tz_base_mib * MiB);
	t210_plat_params.warmboot_fw_base = (tz_base_mib * MiB) + (MiB / 2); // warmboot fw is 512kb before tzdram base
	t210_plat_params.rebootstub_base = (tz_base_mib * MiB) + (MiB - 0x2000); // rebootstub is probably < 1 page anyway

	if (cbfs_boot_locate(&sc7entry_fw, CONFIG_SC7ENTRY_FILE, NULL))
		die("ERROR: Cannot locate sc7entry firmware");
			
	cbfs_file_data(&fh, &sc7entry_fw);
	t210_plat_params.sc7entry_fw_size = (region_device_sz(&fh) + 4 * KiB - 1) & ~(4 * KiB - 1); // 4KB aligned
	memset((void *)(uintptr_t)t210_plat_params.sc7entry_fw_base, 0, t210_plat_params.sc7entry_fw_size);
	rdev_readat(&fh, (void *)(uintptr_t)(tz_base_mib * MiB), 0, region_device_sz(&fh));
	
	if (cbfs_boot_locate(&warmboot_fw, CONFIG_WARMBOOT_FILE, NULL))
		die("ERROR: Cannot locate warmboot firmware");

	cbfs_file_data(&fh, &warmboot_fw);
	t210_plat_params.warmboot_fw_size = (region_device_sz(&fh) + 4 * KiB -1) & ~(4 * KiB -1); // 4KB aligned
	memset((void *)(uintptr_t)t210_plat_params.warmboot_fw_base, 0, t210_plat_params.warmboot_fw_size);
	rdev_readat(&fh, (void *)(uintptr_t)t210_plat_params.warmboot_fw_base, 0, region_device_sz(&fh));

	if (cbfs_boot_locate(&rebootstub, CONFIG_REBOOTSTUB_FILE, NULL))
		die("ERROR: Cannot locate rebootstub");

	cbfs_file_data(&fh, &rebootstub);
	t210_plat_params.rebootstub_size = (region_device_sz(&fh) + 4 * KiB -1) & ~(4 * KiB -1); // 4KB aligned
	memset((void *)(uintptr_t)t210_plat_params.rebootstub_base, 0, t210_plat_params.rebootstub_size);
	rdev_readat(&fh, (void *)(uintptr_t)t210_plat_params.rebootstub_base, 0, region_device_sz(&fh));

	/* Switch: Apply N's warmboot RSA modulus */
	memcpy((void *)(uintptr_t)(t210_plat_params.warmboot_fw_base + 0x10), retail_pkc_modulus, 0x100);

	t210_plat_params.emc_table_size = get_mtc_size();
	t210_plat_params.emc_table_base = (uint64_t)(uintptr_t)cbmem_find(CBMEM_ID_MTC);
	
	dcache_clean_by_mva(&t210_plat_params, sizeof(t210_plat_params));
	return &t210_plat_params;
}
