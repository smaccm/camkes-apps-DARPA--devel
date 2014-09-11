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
#include <queue.h>

#include <can.h>

#define MSG_QUEUE_SIZE  10

void can__init(void)
{
	printf("CAN device started...\n");
	mq_init(MSG_QUEUE_SIZE);
}

int can_setup(int baudrate)
{
	mcp2515_reset();

	/* Wait until reset finishes. */
	while (1) {
		if (get_mode() == REQOP_CONFIG) {
			break;
		}
	}

	set_baudrate(125000);
	enable_intrrupt();

	set_mode(REQOP_NORMAL);

	mcp2515_bit_modify(CANINTF, CANINTF_TX0IF, 0xFF);

	return 0;
}

void can_send_with_priority(can_frame_t frame, unsigned int prio)
{
	/* FIXME: Not compatible with message buffering.
	 *        Disable interrupt send message and re-enable.
	 */
	load_txb(0, &frame, prio);
}

void can_send(struct can_frame frame)
{
	while (tx_queue_push(&frame) < 0);
}

void can_recv(struct can_frame *frame)
{
	while (rx_queue_pop(frame) == NULL);
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
	set_mode(REQOP_CONFIG);

	clear_rx_filter(filter_id);

	set_mode(REQOP_NORMAL);
}

void can_disable_filtering(void)
{
	set_mode(REQOP_CONFIG);

	clear_filter_mask(2);

	set_mode(REQOP_NORMAL);
}
