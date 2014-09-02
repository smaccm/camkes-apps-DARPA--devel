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

#include <can.h>

int setup(int bitrate)
{
	return 0;
}

void send(struct can_frame frame)
{
}

void recv(struct can_frame *frame)
{
}

void set_filter(unsigned int can_id, unsigned int mask)
{
}
