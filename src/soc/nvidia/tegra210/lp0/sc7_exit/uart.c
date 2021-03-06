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

#include "uart.h"
#include "tegra_sc7_exit.h"

/* UART A, B, C, D and E. */
static const u32 uart_baseoff[5] = { 0, 0x40, 0x200, 0x300, 0x400 };

void uart_init(u32 idx, u32 baud)
{
	uart_t *uart = (uart_t *)(UART_BASE + uart_baseoff[idx]);

	// Make sure no data is being sent.
	uart_wait_idle(idx, UART_TX_IDLE);

	// Misc settings.
	u32 div = 221; // 115200.
	uart->UART_IER_DLAB = 0; // Disable interrupts.
	uart->UART_LCR = UART_LCR_DLAB | UART_LCR_WORD_LENGTH_8; // Enable DLAB & set 8n1 mode.
	uart->UART_THR_DLAB = (u8)div; // Divisor latch LSB.
	uart->UART_IER_DLAB = (u8)(div >> 8); // Divisor latch MSB.
	uart->UART_LCR = UART_LCR_WORD_LENGTH_8; // Disable DLAB.
	(void)uart->UART_SPR;

	// Setup and flush fifo.
	uart->UART_IIR_FCR = UART_IIR_FCR_EN_FIFO;
	(void)uart->UART_SPR;
	usleep(20);
	uart->UART_MCR = 0; // Disable hardware flow control.
	usleep(96);
	uart->UART_IIR_FCR = UART_IIR_FCR_EN_FIFO | UART_IIR_FCR_TX_CLR | UART_IIR_FCR_RX_CLR;

	// Wait 3 symbols for baudrate change.
	usleep(29); // 115200
	uart_wait_idle(idx, UART_TX_IDLE | UART_RX_IDLE);
}

void uart_wait_idle(u32 idx, u32 which)
{
	uart_t *uart = (uart_t *)(UART_BASE + uart_baseoff[idx]);
	if (UART_TX_IDLE & which)
	{
		while (!(uart->UART_LSR & UART_LSR_TMTY))
			;
	}
	if (UART_RX_IDLE & which)
	{
		while (uart->UART_LSR & UART_LSR_RDR)
			;
	}
}

void uart_send(u32 idx, const char *buf, u32 len)
{
	uart_t *uart = (uart_t *)(UART_BASE + uart_baseoff[idx]);

	for (u32 i = 0; i != len; i++)
	{
		while (!(uart->UART_LSR & UART_LSR_THRE))
			;
		uart->UART_THR_DLAB = buf[i];
	};
}
