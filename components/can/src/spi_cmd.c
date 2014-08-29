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
#include <string.h>
#include <stdbool.h>

#include "can.h"

#include "mcp2515.h"

#include "can_inf.h"
#include "spi_inf.h"
#include "common.h"
#include "utils.h"

spi_dev_port_p 	mcp2515_spi_dev;	//ptr to SPI buffer
/*
 * MCP2515 - driver function definitions
 */
/* Soft reset - put MCP2515 into default state */
void _mcp2515_reset(void){

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

	mcp2515_spi_dev->txbuf[0] = MCP_RESET;
    mcp2515_spi_transfer(mcp2515_spi_dev, 1, 0);

    _release_spin_lock(&(mcp2515_spi_dev->lock));

}

/* Read registers in series and return the first byte*/
uint8_t _mcp2515_read_reg(int reg, int rcount){
	uint8_t data = 0;

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

    mcp2515_spi_dev->txbuf[0] = MCP_READ;
    mcp2515_spi_dev->txbuf[1] = reg;
    mcp2515_spi_transfer(mcp2515_spi_dev, 2, rcount);
    data = mcp2515_spi_dev->rxbuf[2];

    _release_spin_lock(&(mcp2515_spi_dev->lock));

    return data;
}

/* Write the value v into register r and return */
void _mcp2515_write_reg(int r, unsigned char v){

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

    mcp2515_spi_dev->txbuf[0] = MCP_WRITE;
    mcp2515_spi_dev->txbuf[1] = r;
    mcp2515_spi_dev->txbuf[2] = v;
    mcp2515_spi_transfer(mcp2515_spi_dev, 3,0);

	_release_spin_lock(&(mcp2515_spi_dev->lock));

}

/* Write multiple bytes from the buffer to regs starting from reg v */
int _mcp2515_write_mult_regs(uint8_t reg, uint8_t * buf, int byte_len){

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

    mcp2515_spi_dev->txbuf[0] = MCP_WRITE;
    mcp2515_spi_dev->txbuf[1] = reg;
    memcpy(&(mcp2515_spi_dev->txbuf[2]), buf, byte_len);
	mcp2515_spi_transfer(mcp2515_spi_dev, 2 + byte_len, 0);

	_release_spin_lock(&(mcp2515_spi_dev->lock));

	return 0;
}

/* Bit Modify Instruction:
 * provides a means for settig or clearing individual bits in specific status
 * and control registers. Not available for all registers.
 * The following listed registers allow the user to use this instruction:
 * BFPCTRL, TXRTSCTRL, CANCTRL, CNF1, CNF2, CNF3, CANINTE, CANINTF, EFLG,
 * TXB0CTRL, TXB1CTRL, TXB2CTRL, RXB0CTRL, RXB1CTRL
 */
void _mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data){

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

	mcp2515_spi_dev->txbuf[0] = MCP_BIT_MOD;
	mcp2515_spi_dev->txbuf[1] = adress;
	mcp2515_spi_dev->txbuf[2] = mask;
	mcp2515_spi_dev->txbuf[3] = data;
	mcp2515_spi_transfer(mcp2515_spi_dev, 4,0);

	_release_spin_lock(&(mcp2515_spi_dev->lock));
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
int _mcp2515_read_status(void){
	int status = 0;

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

    mcp2515_spi_dev->txbuf[0] = MCP_READ_STATUS;
    mcp2515_spi_transfer(mcp2515_spi_dev, 1, 1);
    status = mcp2515_spi_dev->rxbuf[1];

	_release_spin_lock(&(mcp2515_spi_dev->lock));

    return status;
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
int _mcp2515_rx_status(void){
	int status = 0;

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

    mcp2515_spi_dev->txbuf[0] = MCP_RX_STATUS;
    mcp2515_spi_transfer(mcp2515_spi_dev, 1, 1);
    status = mcp2515_spi_dev->rxbuf[1];

	_release_spin_lock(&(mcp2515_spi_dev->lock));

    return status;
}

