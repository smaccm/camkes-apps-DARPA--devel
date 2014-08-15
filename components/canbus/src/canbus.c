/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/* standard */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "utils.h"

#include "canbus.h"

/* application common */
#include "can_inf.h"
#include "spi_inf.h"
#include "common.h"


int run(void)
{
	int i = 0;
	int len = 12;
	int error = 0;
	int checksum = 0;
	CAN_msg_ptr tx, rx;
	CAN_data * buf;

	buf = (CAN_data_ptr) can_buf;
	tx = &(buf->tx);
	rx = &(buf->rx);

	/* Initialize CAN controller. */
	while(can_mcp2515_init(125000));

	printf("Start CAN Loop-back Test\n");

	/* Prepare CAN frame. */
	tx->id = 0x123;
	tx->exide = 0;
	tx->rtr = 0;
	tx->length = 8;
	tx->data[0] = 0x08;
	tx->data[1] = 0x07;
	tx->data[2] = 0x06;
	tx->data[3] = 0x05;
	tx->data[4] = 0x04;
	tx->data[5] = 0x03;

	while (1) {
		/* Send message */
		error = can_mcp2515_send_message(0);
		printf("Send: error(%d), id(%x), data(%x, %x, %x, %x, %x, %x, %x, %x)\n",
			error, tx->id,
			tx->data[0], tx->data[1], tx->data[2], tx->data[3],
			tx->data[4], tx->data[5], tx->data[6], tx->data[7]);
		if(error){
			while(can_mcp2515_init(125000)){
				printf("Re-initialising MCP2515, Send error %d\n", error);
			}
		}
		udelay(10000);

		/* Receive message */
		error = can_mcp2515_read_message(0, 100000);
		printf("Recv: error(%d), id(%x), data(%x, %x, %x, %x, %x, %x, %x, %x)\n",
			error, rx->id,
			rx->data[0], rx->data[1], rx->data[2], rx->data[3],
			rx->data[4], rx->data[5], rx->data[6], rx->data[7]);
		if(error){
			while(can_mcp2515_init(125000)){
				printf("Re-initialising MCP2515, Send error %d\n", error);
			}
		}
		udelay(10000);
	}

	return 0;
}

