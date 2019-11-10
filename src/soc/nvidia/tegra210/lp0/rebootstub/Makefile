################################################################################
##
## Copyright 2019 CTCaer
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; version 2 of the License.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
################################################################################

-include ../../../../../../.xcompile
GCC_PREFIX?=$(CROSS_COMPILE_arm)

CC = $(GCC_PREFIX)gcc
NM = $(GCC_PREFIX)nm
OBJCOPY = $(GCC_PREFIX)objcopy

DD = dd
CP = cp
MV = mv
RM = rm

.PHONY: all
all: tegra_rebootstub.fw

tegra_rebootstub.elf: tegra_rebootstub.ld start.S i2c.c shutdown.c
	$(CC) -march=armv4t -mtune=arm7tdmi -marm \
		-mno-unaligned-access -nostdlib -static -O2 -g -ffunction-sections \
		-fdata-sections -fno-inline -fomit-frame-pointer -Wall \
		-Wl,--nmagic,--gc-sections,--build-id=none -T tegra_rebootstub.ld \
		-I ../../../../../include -I ../../../../../arch/arm/include \
		-o $@ $(filter %.S,$+) $(filter %.c,$+)

tegra_rebootstub.fw: tegra_rebootstub.elf
	@# Get rid of any files we're about to create.
	$(RM) -f $@.fw
	@# Convert the ELF image into a binary image.
	$(OBJCOPY) -S -O binary $< $@

clean:
	$(RM) -f tegra_rebootstub.fw tegra_rebootstub.elf
