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

#include "client.h"

/* application common */
#include "common.h"

void timer_update_callback(void *arg)
{
	printf("Time up... %u!\n", *(unsigned int*)arg);
	(*(unsigned int*)arg)++;

	if (*(unsigned int*)arg == 10) {
		printf("Enough, no more timer message...\n");
	} else {
		timer_update_reg_callback(timer_update_callback, arg);
	}
}

int run(void)
{
	char str[] = "UART Test\n";
	int i = 0;
	int len = 12;
	int error = 0;

	/*
	 * Timer test
	 */
	unsigned int cnt = 1;
	timer_update_reg_callback(timer_update_callback, &cnt);


	printf("Start UART Test\n");
	char* buf = (char*) uart_buf;
	int rcount = 0;
	int wcount = 0;
	strcpy(buf,str);
	wcount = uart_write(0, 15);
	while(1){
		uart_read(0,10);
		printf("read: %s\n",buf);
		uart_write(0,10);
	}

	printf("finish\n");
}

