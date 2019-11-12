/*
 * Copyright (c) 2018 naehrwert
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

#include "i2c.h"
#include "tegra_sc7_exit.h"

static u32 i2c_addrs[] = {
	0x7000C000, 0x7000C400, 0x7000C500,
	0x7000C700, 0x7000D000, 0x7000D100
};

static void _i2c_wait(vu32 *base)
{
	base[I2C_CONFIG_LOAD] = 0x25;
	for (u32 i = 0; i < 20; i++)
	{
		usleep(1);
		if (!(base[I2C_CONFIG_LOAD] & 1))
			break;
	}
}

int i2c_send_byte(u32 idx, u32 x, u32 y, u8 b)
{
	vu32 *base = (vu32 *)i2c_addrs[idx];
	base[I2C_CMD_ADDR0] = x << 1;       //Set x (send mode).

	base[I2C_CMD_DATA1] = y | (b <<8);  //Set value.

	base[I2C_CNFG] = (1 << 1) | 0x2800; //Set size and send mode.
	_i2c_wait(base);                    //Kick transaction.

	base[I2C_CNFG] = (base[I2C_CNFG] & 0xFFFFFDFF) | 0x200;

	while (base[I2C_STATUS] & 0x100)
		;

	if (base[I2C_STATUS] << 28)
		return 0;

	return 1;
}

void i2c_init(u32 idx)
{
	vu32 *base = (vu32 *)i2c_addrs[idx];

	base[I2C_CLK_DIVISOR_REGISTER] = 0x50001;
	base[I2C_BUS_CLEAR_CONFIG] = 0x90003;
	_i2c_wait(base);

	for (u32 i = 0; i < 10; i++)
	{
		usleep(20000);
		if (base[INTERRUPT_STATUS_REGISTER] & 0x800)
			break;
	}

	(vu32)base[I2C_BUS_CLEAR_STATUS];
	base[INTERRUPT_STATUS_REGISTER] = base[INTERRUPT_STATUS_REGISTER];
}
