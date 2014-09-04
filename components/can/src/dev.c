/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <can_inf.h>

#include <mcp2515.h>
#include <can.h>

void can__init(void)
{
	printf("CAN device started...\n");
}

int can_setup(int baudrate)
{
	set_mode(REQOP_CONFIG);

	set_baudrate(125000);

	set_mode(REQOP_NORMAL);
	return 0;
}

void can_send(struct can_frame frame)
{
	transmit_frame(0, &frame);
}

void can_recv(struct can_frame *frame)
{
	receive_frame(0, frame);
}

void can_set_filter(unsigned int can_id, unsigned int mask)
{
}
