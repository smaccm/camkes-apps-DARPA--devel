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

	mcp2515_bit_modify(CANINTE, 0xF, val);
}
