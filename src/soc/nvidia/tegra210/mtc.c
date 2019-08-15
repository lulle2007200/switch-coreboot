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

#include <arch/io.h>
#include <arch/lib_helpers.h>
#include <cbfs.h>
#include <cbmem.h>
#include <console/console.h>
#include <soc/mtc.h>
#include <soc/addressmap.h>
#include <soc/clock.h>
#include <boardid.h>
#include <string.h>
#include "mtc/mtc.h"
#include "mtc/mtc_table.h"
#include "mtc/mtc_switch.h"

#define MAX_MTC_TABLE_ENTRIES	20
#define MTC_TABLE_ENTRY_SIZE	4880
#define MTC_TABLE_MAX_SIZE	(MAX_MTC_TABLE_ENTRIES * MTC_TABLE_ENTRY_SIZE)

#define OP_SWITCH 0
#define OP_TRAIN  1

int tegra210_run_mtc(void)
{
	mtc_config_t mtc_cfg;
	
	mtc_cfg.emc_2X_clk_src_is_pllmb = false;
	mtc_cfg.fsp_for_src_freq = false;
	mtc_cfg.train_ram_patterns = true;
	raw_write_cptr_el3(0);
	raw_write_cpacr_el1(3 << 20);

	mtc_cfg.table_entries = EMC_TABLE_SIZE_R7 / sizeof(emc_table_t);

	void *cbmem_tab = cbmem_add(CBMEM_ID_MTC, EMC_TABLE_SIZE_R7);
	if (cbmem_tab == NULL) {
		printk(BIOS_ERR, "MTC table allocation in cbmem failed!\n");
		return -1;
	}

	printk(BIOS_INFO, "MTC: table is at %p\n", cbmem_tab);

	switch (ram_code())
	{
	case 1:
		memcpy(cbmem_tab, nx_abca2_2_10NoCfgVersion_V9_8_7_V1_6, EMC_TABLE_SIZE_R7);
		break;
	case 0:
	case 2:
	case 3:
	case 4:
	default:
		memcpy(cbmem_tab, nx_abca2_0_3_10NoCfgVersion_V9_8_7_V1_6, EMC_TABLE_SIZE_R7);
		break;
	}

	mtc_cfg.mtc_table = (emc_table_t *)cbmem_tab;

	int boot_index = 0;
	u32 reg = read32(CLK_RST_REG(clk_src_emc));
	printk(BIOS_INFO, "MTC: clk_src_emc=0x%08x\n", reg);
	for (boot_index = 0; boot_index < mtc_cfg.table_entries; boot_index++) {
		if (reg == mtc_cfg.mtc_table[boot_index].clk_src_emc)
			break;
	}

	if (boot_index >= mtc_cfg.table_entries) {
		printk(BIOS_ERR, "MTC: failed to find boot entry\n");
		goto cleanup;
	}

	printk(BIOS_INFO, "MTC: running training\n");
	mtc_cfg.rate_from = mtc_cfg.mtc_table[boot_index].rate_khz;
	mtc_cfg.train_mode = OP_TRAIN;

	for (int i = 0; i < mtc_cfg.table_entries; i++) {
		mtc_cfg.rate_to = mtc_cfg.mtc_table[i].rate_khz;
		printk(BIOS_INFO, "MTC: training %d kHz -> %d kHz\n",
			mtc_cfg.rate_from, mtc_cfg.rate_to);

		minerva_main(&mtc_cfg);
	}

	printk(BIOS_INFO, "MTC: increasing memory clock\n");
	mtc_cfg.train_mode = OP_SWITCH;

	int target_index = 0;
	for (target_index = boot_index + 1; target_index < mtc_cfg.table_entries; target_index++) {
		if (mtc_cfg.mtc_table[target_index].periodic_training) {
		    target_index--;
			break;
		}
	}
	
	if (target_index >= mtc_cfg.table_entries)
	    target_index = mtc_cfg.table_entries - 1;
	
	if (!mtc_cfg.fsp_for_src_freq) {
		printk(BIOS_INFO, "MTC: activating FSP WAR\n");

		target_index--;
		mtc_cfg.rate_to = mtc_cfg.mtc_table[target_index].rate_khz;
		printk(BIOS_INFO, "MTC: switching %d kHz -> %d kHz\n",
			mtc_cfg.rate_from, mtc_cfg.rate_to);
		minerva_main(&mtc_cfg);

		target_index++;
		mtc_cfg.rate_to = mtc_cfg.mtc_table[target_index].rate_khz;
		printk(BIOS_INFO, "MTC: switching %d kHz -> %d kHz\n",
			mtc_cfg.rate_from, mtc_cfg.rate_to);
		minerva_main(&mtc_cfg);
	}
	else {
		mtc_cfg.rate_to = mtc_cfg.mtc_table[target_index].rate_khz;
		printk(BIOS_INFO, "MTC: switching %d kHz -> %d kHz\n",
			mtc_cfg.rate_from, mtc_cfg.rate_to);

		minerva_main(&mtc_cfg);
	}

	printk(BIOS_INFO, "MTC: successful\n");
	return 0;

cleanup:
	cbmem_entry_remove(cbmem_entry_find(CBMEM_ID_MTC));
	return -1;
}


void soc_add_mtc(struct lb_header *header)
{
	struct lb_range *mtc;
	mtc = (struct lb_range *)lb_new_record(header);
	mtc->tag = LB_TAG_MTC;
	mtc->size = sizeof(*mtc);

	mtc->range_start = (uintptr_t)cbmem_find(CBMEM_ID_MTC);
	mtc->range_size = EMC_TABLE_SIZE_R7;
}

size_t get_mtc_size()
{
	return EMC_TABLE_SIZE_R7;
}
