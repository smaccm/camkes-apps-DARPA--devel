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


void can_send_with_priority(can_frame_t frame, unsigned int prio)
{
	load_frame(0, &frame, prio);
}

void can_send(struct can_frame frame)
{
	load_frame(0, &frame, 0);
}

void can_recv(struct can_frame *frame)
{
	receive_frame(1, frame);
}

int can_set_filter(struct can_id id, unsigned int mask)
{
	int ret;

	set_mode(REQOP_CONFIG);

	ret = set_rx_filter(id, mask);

	set_mode(REQOP_NORMAL);

	return ret;
}

void can_clear_filter(int filter_id)
{
	clear_rx_filter(filter_id);
}

void can_disable_filtering(void)
{
	clear_filter_mask(2);
}
