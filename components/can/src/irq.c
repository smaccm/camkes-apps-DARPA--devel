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
#include <queue.h>

#include "can.h"

#define TXIF_MASK (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF)
#define RXIF_MASK (CANINTF_RX0IF | CANINTF_RX1IF)
#define TXIF_SHF  2

#define TXB_NUM  3 //Number of TX buffers on the controller.

static void message_error_recovery(void)
{
	printf("%s\n", __func__);
	/* Clear interrupt flags */
	mcp2515_bit_modify(CANINTF, CANINTE_MERRE, 0);
}

static void error_recovery(void)
{
	printf("%s\n", __func__);
	/* Clear interrupt flags */
	mcp2515_bit_modify(CANINTF, CANINTF_ERRIF, 0);
}

/**
 * Put message queue into TXB.
 *
 * @tx: TXB bits(Bit 0 -- TXB0, Bit 1 -- TXB1, Bit 2 -- TXB2)
 */
static void transmit_message(uint8_t tx)
{
	struct can_frame frame;
	uint8_t rts = 0;

	/* Load messages into empty TX buffers. */
	for (int i = 0; i < TXB_NUM; i++) {
		if (tx & BIT(i)) {
			if (tx_queue_pop(&frame)) {
				load_txb(i, &frame, 0);
				rts |= BIT(i);
			} else {
				/* Nothing to send */
				break;
			}
		}
	}

	/* Clear interrupt flags */
	mcp2515_bit_modify(CANINTF, rts << TXIF_SHF, 0);

	/* Initiating transmission */
	mcp2515_rts(rts);
}

/**
 * Put RXB into message queue.
 *
 * @rx: RXB bits(0 -- None, 1 -- RXB0, 2 -- RXB1, 3 -- Both)
 */
static void receive_message(uint8_t rx)
{
	struct can_frame frame;
	int ret = 0;

	if (rx & BIT(0)) {
		recv_rxb(0, &frame);
		ret += rx_queue_push(&frame);
	}

	if (rx & BIT(1)) {
		recv_rxb(1, &frame);
		ret += rx_queue_push(&frame);
	}

	/* If the RX queue is full, warn user. */
	if (ret) {
		printf("CAN: Drop %d message(s).\n", -ret);
	}

	/* Clear interrupt flags */
	mcp2515_bit_modify(CANINTF, rx, 0);
}

/**
 * IRQ handler
 *
 * NOTE: The interrupt flags are cleared individually because
 *       some of the operations may change the interrupt flags on the
 *       controller while we are still in the interrupt handler. Hence
 *       it will cause wrong interrupt acknowledgement. Note that, disabling
 *       the interrupts won't help, CANINTF is independent to CANINTE,
 *       it will change anyway.
 */
static void irq_handler(void *arg UNUSED)
{
	uint8_t flags;

	/* Read interrupt flags */
	flags = mcp2515_read_reg(CANINTF);

	if (flags & CANINTF_MERRF) {
		message_error_recovery();
	}

	if (flags & CANINTF_WAKIF) {
		/* 
		 * XXX: Not implemented
		 * Clear interrupt flags
		 */
		mcp2515_bit_modify(CANINTF, CANINTF_WAKIF, 0);
	}

	if (flags & CANINTF_ERRIF) {
		error_recovery();
	}

	if (flags & TXIF_MASK) {
		transmit_message((flags & TXIF_MASK) >> TXIF_SHF);
	}

	if (flags & RXIF_MASK) {
		receive_message(flags & RXIF_MASK);
	}

	Int_reg_callback(irq_handler, NULL);
}

/**
 * Enable all interrupts except "wake up".
 */
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
