/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/*
 * MCP2515 - SPI interface instruction set.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "can.h"

#include "mcp2515.h"

#include "can_inf.h"
#include "spi_inf.h"
#include "common.h"
#include "utils.h"

#define DEV_ID CAN_APP_ID
static spi_dev_port *spi_dev = NULL;

void spi__init(void)
{
	spi_dev = (spi_dev_port*)spi_can;
}

/*
 * Soft reset - put MCP2515 into default state.
 */
void mcp2515_reset(void)
{
	spi_dev->txbuf[0] = CMD_RESET;
	spi_transfer(DEV_ID, 1, 0);
}

/* Read register */
uint8_t mcp2515_read_reg(int reg)
{
	spi_dev->txbuf[0] = CMD_READ;
	spi_dev->txbuf[1] = reg;

	spi_transfer(DEV_ID, 2, 1);

	return spi_dev->rxbuf[2];
}

/* Read registers in series */
void mcp2515_read_nregs(uint8_t reg, int count, uint8_t *buf)
{
	if (!buf) {
		printf("Empty buffer!\n");
		return;
	}

	spi_dev->txbuf[0] = CMD_READ;
	spi_dev->txbuf[1] = reg;

	spi_transfer(DEV_ID, 2, count);

	memcpy(buf, &spi_dev->rxbuf[2], count);
}

/* Write to register. */
void mcp2515_write_reg(uint8_t reg, uint8_t val)
{
	spi_dev->txbuf[0] = CMD_WRITE;
	spi_dev->txbuf[1] = reg;
	spi_dev->txbuf[2] = val;

	spi_transfer(DEV_ID, 3, 0);
}

/* Write to registers. */
void mcp2515_write_nregs(uint8_t reg, uint8_t *buf, int count)
{
	if (!buf) {
		printf("Empty buffer!\n");
		return;
	}

	spi_dev->txbuf[0] = CMD_WRITE;
	spi_dev->txbuf[1] = reg;

	memcpy(&spi_dev->txbuf[2], buf, count);

	spi_transfer(DEV_ID, 2 + count, 0);
}

/* Bit Modify Instruction:
 * provides a means for settig or clearing individual bits in specific status
 * and control registers. Not available for all registers.
 * The following listed registers allow the user to use this instruction:
 * BFPCTRL, TXRTSCTRL, CANCTRL, CNF1, CNF2, CNF3, CANINTE, CANINTF, EFLG,
 * TXB0CTRL, TXB1CTRL, TXB2CTRL, RXB0CTRL, RXB1CTRL
 */
void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t val)
{
	spi_dev->txbuf[0] = CMD_BIT_MODIFY;
	spi_dev->txbuf[1] = reg;
	spi_dev->txbuf[2] = mask;
	spi_dev->txbuf[3] = val;

	spi_transfer(DEV_ID, 4, 0);
}

/*
 * Read Status Instruction -
 * Allows single instruction access to some of the often used status bits
 * for message reception and transmission.
 * 		Bit		Descriptions
 * 	--------------------------------
 * 		0 		CANNINTF.RX0IF		// RXB0 Full Interrupt Flag bit
 *		1 		CANNINTFL.RX1IF		// RXB1 Full Interrupt Flag bit
 *		2		TXB0CNTRL.TXREQ		// TXB0 Message Transmit Request bit
 *		3		CANINTF.TX0IF		// TXB0 Empty Interrupt Flag bit
 *		4		TXB1CNTRL.TXREQ		// TXB1 Message Transmit Request bit
 *		5		CANINTF.TX1IF		// TXB1 Empty Interrupt Flag bit
 *		6		TXB2CNTRL.TXREQ		// TXB2 Message Transmit Request bit
 *		7		CANINTF.TX2IF		// TXB2 Empty Interrupt Flag bit
 *	---------------------------------
 */
uint8_t mcp2515_read_status(void)
{
	spi_dev->txbuf[0] = CMD_READ_STATUS;

	spi_transfer(DEV_ID, 1, 1);

	return spi_dev->rxbuf[1];
}

/*
 * Read RX Status Instruction -
 * Used to quickly determine which filter matched the message and message type (standard, extended, remote).
 * After the command byte is sent, the controller will return 8 bits of data contain the status data.
 * 	Bit	7 6		Received Message
 * 	--------------------------------
 * 		0 0 	No Rx message
 * 		0 1		Message in RXB0
 * 		1 0		Message in RXB1
 * 		1 1 	Message in both buffers
 *	---------------------------------
 *	Bit	4 3		Msg Type Received
 *	---------------------------------
 *		0 0		Standard data frame
 *		0 1		Standard remote frame
 *		1 0		Extended data frame
 *		1 1		Extended remote frame
 *	---------------------------------
 *	Bit	2 1	0	Filter Match
 *	---------------------------------
 *		0 0	0	RXF0
 *		0 0	1	RXF1
 *		0 1	0	RXF2
 *		0 1	1	RXF3
 *		1 0 0 	RXF4
 *		1 0 1	RXF5
 *		1 1 0	RXF0 (Rollover to RXB1)
 *		1 1 1	RXF1 (Rollover to RXB1)
 *	---------------------------------
 */
uint8_t mcp2515_rx_status(void)
{
	spi_dev->txbuf[0] = CMD_RX_STATUS;

	spi_transfer(DEV_ID, 1, 1);

	return spi_dev->rxbuf[1];
}
