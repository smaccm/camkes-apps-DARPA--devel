/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <stdio.h>

#include <platsupport/chardev.h>
#include <platsupport/serial.h>

#include <uart.h>

#ifdef CONFIG_PLAT_EXYNOS5410
#define DEV_ID  EXYNOS5_UART1
#elif CONFIG_PLAT_IMX31
#define DEV_ID  IMX31_UART1
#else
#error
#endif
static struct ps_chardevice *dev = NULL;

static void *uart_io_map(void* cookie, uintptr_t paddr, size_t size, int cached, ps_mem_flags_t flags)
{
	return (void*)uart0base;
}

static const ps_io_ops_t uart_ops = {
	.io_mapper = {.io_map_fn = uart_io_map},
	.io_port_ops = NULL,
	.dma_manager = NULL,
	.clock_sys = NULL,
	.mux_sys = NULL,
};

void uart__init(void)
{
	printf("UART driver init\n");

	dev = malloc(sizeof(struct ps_chardevice));
	dev = ps_cdev_init(DEV_ID, &uart_ops, dev);
	if (!dev) {
		printf("error!\n");
	}
	
	uart_configure(dev, 57600, 8, PARITY_NONE, 1);
}

int uart_read(int uart_num, int rsize)
{
	int c;
	char *buf = (char*)client_buf;

	if (uart_num != 0) {
		printf("Only support UART0!\n");
		return -1;
	}
	
	for (int i = 0; i < rsize; i++) {
		do {
			c = ps_cdev_getchar(dev);
		} while (c < 0);
		buf[i] = (char)c;
	}
	
	return rsize;
}

int uart_write(int uart_num, int wsize)
{
	char *buf = (char*)client_buf;

	if (uart_num != 0) {
		printf("Only support UART0!\n");
		return -1;
	}
	
	for (int i = 0; i < wsize; i++) {
		ps_cdev_putchar(dev, buf[i]);
	}
	
	return wsize;
}

