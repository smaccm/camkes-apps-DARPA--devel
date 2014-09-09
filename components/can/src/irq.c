/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <stdint.h>
#include <stdio.h>

#include "mcp2515.h"

#include "can.h"

#define TXIF_MASK (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF)
#define RXIF_MASK (CANINTF_RX0IF | CANINTF_RX1IF)
#define TXIF_SHF  2

static void message_error_recovery(void)
{
	printf("%s\n", __func__);
}

static void error_recovery(void)
{
	printf("%s\n", __func__);
}

static void transmit_message(uint8_t tx)
{
	printf("%s: %x\n", __func__, tx);
}

static void receive_message(uint8_t rx)
{
	printf("%s: %x\n", __func__, rx);
}

static void irq_handler(void *arg)
{
	uint8_t flags;
	uint8_t mask = 0;

	/* Read interrupt flags */
	flags = mcp2515_read_reg(CANINTF);
	printf("%s: %x\n", __func__, flags);

	if (flags & CANINTF_MERRF) {
		message_error_recovery();
		mask |= CANINTF_MERRF;
	}

	if (flags & CANINTF_WAKIF) {
		/* XXX: Not implemented */
		mask |= CANINTF_WAKIF;
	}

	if (flags & CANINTF_ERRIF) {
		error_recovery();
		mask |= CANINTF_ERRIF;
	}

	if (flags & TXIF_MASK) {
		transmit_message((flags & TXIF_MASK) >> TXIF_SHF);
		mask |= (flags & TXIF_MASK);
	}

	if (flags & RXIF_MASK) {
		receive_message(flags & RXIF_MASK);
		mask |= (flags & RXIF_MASK);
	}

	/* Clear interrupt flags */
	mcp2515_bit_modify(CANINTF, mask, 0);

	Int_reg_callback(irq_handler, NULL);
}

void enable_intrrupt(void)
{
	uint8_t val;

	/* Enable RX full interrupts. */
	val = CANINTE_RX0IE | CANINTE_RX1IE;

	/* Enable TX empty interrupts. */
	val |= CANINTE_TX0IE | CANINTE_TX1IE | CANINTE_TX2IE;

	/* Enable message error interrupt. */
	val |= CANINTE_MERRE;

	/* Enable error interrupt. */
	val |= CANINTE_ERRIE;

	mcp2515_write_reg(CANINTE, val);

	Int_reg_callback(irq_handler, NULL);
}
