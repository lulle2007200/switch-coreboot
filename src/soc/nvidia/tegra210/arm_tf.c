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
#include <stdlib.h>
#include <string.h>
#include <symbols.h>

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
} bl31_plat_params_t;

static bl31_plat_params_t t210_plat_params;

void *soc_get_bl31_plat_params(bl31_params_t *params)
{
	uintptr_t tz_base_mib;
	size_t tz_size_mib;
	int uart_id = 0;
	struct cbfsf sc7entry_fw, warmboot_fw;
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

	dcache_clean_by_mva(&t210_plat_params, sizeof(t210_plat_params));
	return &t210_plat_params;
}
