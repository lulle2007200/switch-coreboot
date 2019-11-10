/*
 * Copyright 2019 CTCaer
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

/* We use ELF as output format. So that we can debug the code in some form. */
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

ENTRY(_start)
SECTIONS
{
	. = 0x40000000;

	/* The actual lp0 blob code. */
	.data_start = .;
	.data . : {
		*(.text._start);
		*(.text);
		*(.text.*);
		*(.rodata);
		*(.rodata.*);
		*(.data);
		*(.data.*);
		*(.bss);
		*(.bss.*);
		*(.sbss);
		*(.sbss.*);
		. = ALIGN(16);
	}
	.data_end = .;

	/DISCARD/ : {
		*(.comment)
		*(.note)
		*(.comment.*)
		*(.note.*)
		*(.ARM.*)
	}
}
