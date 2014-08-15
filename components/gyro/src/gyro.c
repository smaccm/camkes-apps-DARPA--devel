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

#include "gyro.h"

/* application common */
#include "common.h"
#include <periph.h>
#include <utils.h>

#define WHO_AM_I 0xD4
#define READ_MASK 0x80
#define WRITE_MASK 0x0

// Device registers
#define WHO_AM_I_REG 0xF
#define CTRL_REG1 0x20
#define CTRL_REG3 0x22
#define OUT_X_L 0B00101000
#define OUT_X_H 0B00101001
#define OUT_Y_L 0B00101010
#define OUT_Y_H 0B00101011
#define OUT_Z_L 0B00101100
#define OUT_Z_H 0B00101101

static int read_register(int address)
{
	int ret = 0;

	spi_chip_select(PERIPH_ID_GYRO);

	ret = spi_transfer_byte(PERIPH_ID_GYRO, READ_MASK | address);
	spi_transfer_byte(PERIPH_ID_GYRO, 0x00);

	spi_chip_unselect(PERIPH_ID_GYRO);

	return ret;
}

static void write_register(int address, unsigned char value)
{
	spi_chip_select(PERIPH_ID_GYRO);

	spi_transfer_byte(PERIPH_ID_GYRO, WRITE_MASK | address);
	spi_transfer_byte(PERIPH_ID_GYRO, value);

	spi_chip_unselect(PERIPH_ID_GYRO);
}

static int begin(void)
{
	write_register(CTRL_REG1, 0xF);
	write_register(CTRL_REG3, 1 << 3);

	udelay(1000);

	return read_register(WHO_AM_I_REG);
}

static int read_axis(int axis)
{
	int low, high;
	int addr_low, addr_high;

	/* 0 -- X; 1 -- Y; 2 -- Z */
	switch (axis) {
		case 0:
			addr_low = OUT_X_L;
			addr_high = OUT_X_H;
			break;
		case 1:
			addr_low = OUT_Y_L;
			addr_high = OUT_Y_H;
			break;
		case 2:
			addr_low = OUT_Z_L;
			addr_high = OUT_Z_H;
			break;
		default:
			addr_low = OUT_X_L;
			addr_high = OUT_X_H;
			break;
	}
	
	low = read_register(addr_low);
	high = read_register(addr_high);

	return high << 8 | low;
}

int run(void)
{
	int ret;

	printf("Start Gyro...\n");

	spi_register_slave("Gyro", PERIPH_ID_GYRO);

	ret = begin();

	if (ret != WHO_AM_I) {
		printf("Wrong Gyro...%d!\n", ret);
	} else {
		printf("I am Gyro %d...!\n", ret);
	}
	
	printf("X(%d), Y(%d), Z(%d)\n", read_axis(0), read_axis(1), read_axis(2));

	spi_unregister_slave(PERIPH_ID_GYRO);
	return 0;
}

