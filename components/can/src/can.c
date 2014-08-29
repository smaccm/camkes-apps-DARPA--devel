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
 * 	can.c
 *  Created on: Mar 15, 2013
 *      Author: Jiawei Xie
 */

/*
 * Device driver for MCP2515
 * Microchip Technology MCP2515 is a stand-alone
 * Controller Area Network (CAN) controller that
 * implements the CAN specification, version 2.0B.
 * It is capable of transmitting and receiving both standard
 * and extended data and remote frames. The MCP2515
 * has two acceptance masks and six acceptance filters
 * that are used to filter out unwanted messages, thereby
 * reducing the host MCUs overhead. The MCP2515
 * interfaces with microcontrollers (MCUs) via an industry
 * standard Serial Peripheral Interface (SPI).
 *
 * CAN characteristics:
 *  Standard data frame:
 *    up to 8 bytes payload per message
 *    11 bit standard id
 *    18 bit extended id
 *     4 bit data length code
 *
 * Extended:
 *  32 bit s arbitration:  11 bits most significant address
 *                         18 bits less significant address
 *                          3 bits control
 *  6 bits control: data length code
 *
 * This driver uses standard and extended frames.
 * the hardware can listen on up to three addresses at a time.
 * It can buffer two receive frames, and three transmit frames,
 *
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

enum{
	INTERRUPT,
	POLL,
	NOT_INITIALISED,
	INITALISED
};

/* Local CAN registers copy */
static mcp2515_regs local;

/* ------------------------
 * 		SPI Interface
 * ------------------------ */

//shared buffer
extern spi_dev_port_p 	mcp2515_spi_dev;	//ptr to SPI buffer
CAN_data_ptr	msg;				//ptr to application buffer


/* Circular buffer */
#define MAX_QUEUE_SIZE			9			//number of messages hold on queue
typedef struct {
	CAN_msg 	data[MAX_QUEUE_SIZE];			//data array
	int			valids;							//size of the valid data in buffer
	int			head;							//head index, update upon read
	int			tail;							//tail index, update upon write
	volatile 	bool		lock;				//lock to the buffer
}circ_buf;

circ_buf	rx_queue;			//received message queue
circ_buf	tx_queue;			//transmit message queue

/* DEVICE SPIN LOCKS */
volatile 	bool		spi_lock;


static void
append_msg_to_queue(circ_buf* buf, CAN_msg * msg)
{
	CAN_msg_ptr tmp;

	_acquire_spin_lock(&(buf->lock));
	//if the tail is aligned with the head, advances the head
	if((buf->valids == MAX_QUEUE_SIZE) && (buf->head == buf->tail)){
		buf->head ++;
		if(buf->head >= MAX_QUEUE_SIZE){
			buf->head = buf->head % MAX_QUEUE_SIZE;
		}
	}
	tmp = &(buf->data[buf->tail]);
	memcpy((void *)tmp, (void *)msg, sizeof(CAN_msg));
	//buf->data[buf->tail] = in;
	buf->valids ++;
	buf->tail ++;
	if(buf->tail >= MAX_QUEUE_SIZE){
		buf->tail = buf->tail % MAX_QUEUE_SIZE;
	}
	if(buf->valids > MAX_QUEUE_SIZE){
		buf->valids = MAX_QUEUE_SIZE;
	}
	_release_spin_lock(&(buf->lock));
}

static int
remove_msg_from_queue(circ_buf* buf, CAN_msg * out)
{
	CAN_msg_ptr tmp;
	int ret = 0;
	_acquire_spin_lock(&(buf->lock));
	if(buf->valids > 0){
		tmp = &(buf->data[buf->head]);
		memcpy(out, tmp, sizeof(CAN_msg));
		//*out = buf->data[buf->head];
		buf->valids --;
		buf->head ++;
		if(buf->head >= MAX_QUEUE_SIZE){
			buf->head = buf->head % MAX_QUEUE_SIZE;
		}
		ret = 1;
	}else{
		//no valid data for read
		ret = 0;
	}
	_release_spin_lock(&(buf->lock));
	return ret;
}


void can__init(void) {
	spi_lock = 0;
	mcp2515_spi_dev = (spi_dev_port_p) spi1_can;
	msg = (CAN_data_ptr) can_buf;
	local.state = NOT_INITIALISED;
	tx_queue.head = 0;
	tx_queue.lock = 0;
	tx_queue.tail = 0;
	tx_queue.valids = 0;

	rx_queue.head = 0;
	rx_queue.lock = 0;
	rx_queue.tail = 0;
	rx_queue.valids = 0;
}

int mcp2515_spi_transfer(spi_dev_port* dev, int wcount, int rcount){
	(void) dev;
	int ret = 0;
	ret = spi_transfer(CAN_APP_ID, wcount, rcount);
	return ret;
}

void _mcp2515_rts(int txb_id){

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

	if(txb_id == TXB0ID) txb_id = 1;
    mcp2515_spi_dev->txbuf[0] = MCP_RTS|(txb_id);

    mcp2515_spi_transfer(mcp2515_spi_dev, 1,0);

	_release_spin_lock(&(mcp2515_spi_dev->lock));

}


int mcp2515_check_free_buffer(void)
{
  int status = _mcp2515_read_status();
  if ((status & 0x54) == 0x54)
  {
	// all buffers used
	return FALSE;
  }
  return TRUE;
}

/*
 * This function read the CANSTAT and return the opMode
 */
int _mcp2515_get_opMode(void){
	int mode = 0;
	//read the mode from CANCTRL
	mode = _mcp2515_read_reg(CANSTAT, 1);
	return (CAN_EXT_VAL(mode,OPMOD));
}

void _mcp2515_set_opMode(REQOP_MODE mode){
	//REQOP = 0:0:0 (Set Normal Operation mode)
	CAN_INST_VAL(local.canctrl,REQOP,mode);
	_mcp2515_bit_modify(CANCTRL, 0xE0, local.canctrl);
}


int can_mcp2515_check_message(void){
  int temp;
  temp = _mcp2515_read_reg(CANINTF, 1);
  // Verify RX0,RX1 Frame Receive
  return (temp & (CAN_RX0IF_MASK|CAN_RX1IF_MASK));
}

/*
 * Load the message into tx buffer with tx_id
 */
void _mcp2515_load_tx_buffer(int txb_id, CAN_msg * message){
	//mcp2515_spi_dev->txbuf[0] = MCP_LOAD_TX_BUFFER|(txb*2);
	uint8_t sidh,sidl, eid8, eid0, dlc = 0;
	int buf_len = 0;

	if(message->exide == TRUE){
		sidh = (message->id >> 21) & CAN_SIDH8_MASK;
		sidl = (message->id >> 18) & CAN_SIDL3_MASK;
		//ExID Enable bit
		CAN_INST_VAL(sidl,EXIDE,1);
		CAN_INST_VAL(sidl,EID1716,(message->id >> 16));
		//Extend ID
		eid8 = (message->id >> 8) & CAN_EID8_MASK;
		eid0 = (message->id) & CAN_EID0_MASK;

	}else{
		sidh = (message->id >> 3) & CAN_SIDH8_MASK;
		sidl = (message->id << 5) & CAN_SIDL3_MASK;
		//Extend ID
		eid8 = 0;
		eid0 = 0;
	}
	if (message->rtr){
		// a rtr-frame has a length, but contains no data
		CAN_INST_VAL(dlc, RTR, message->rtr);
		// set message length
		CAN_INST_VAL(dlc, DLC, message->length);
		//set SPI tx buffer length
		buf_len = 6;
	}else{
		// set message length
		CAN_INST_VAL(dlc, DLC, message->length);
		// copy in data
		memcpy(&(mcp2515_spi_dev->txbuf[6]), message->data, message->length);
		//set SPI tx buffer length
		buf_len = 6 + message->length;
	}


	//store in tx buffers
	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

	mcp2515_spi_dev->txbuf[0] = MCP_LOAD_TX_BUFFER | txb_id;
	mcp2515_spi_dev->txbuf[1] = sidh;
	mcp2515_spi_dev->txbuf[2] = sidl;
	mcp2515_spi_dev->txbuf[3] = eid8;
	mcp2515_spi_dev->txbuf[4] = eid0;
	mcp2515_spi_dev->txbuf[5] = dlc;
    mcp2515_spi_transfer(mcp2515_spi_dev, buf_len, 0);

	_release_spin_lock(&(mcp2515_spi_dev->lock));

}

int _mcp2515_read_rx_buffer(uint8_t rxb_id, CAN_msg * message, uint8_t rx_status){

	uint8_t length = 0;
	uint8_t exidf, srr = 0;
	uint8_t buf[MAX_CAN_MSG_SIZE];

	memset(buf, 0, MAX_CAN_MSG_SIZE);

	_acquire_spin_lock(&(mcp2515_spi_dev->lock));

	mcp2515_spi_dev->txbuf[0] = (MCP_READ_RX_BUFFER | rxb_id);

	mcp2515_spi_transfer(mcp2515_spi_dev, 1, MAX_CAN_MSG_SIZE);

	memcpy(buf, &(mcp2515_spi_dev->rxbuf[1]), MAX_CAN_MSG_SIZE);

	_release_spin_lock(&(mcp2515_spi_dev->lock));


	memset(message, 0, sizeof(CAN_msg));
	//Extended ID Flag
	exidf = CAN_EXT_VAL(buf[1], IDE);
	//SRR, valid if IDE = 0
	srr = CAN_EXT_VAL(buf[1],SRR);

	switch(CAN_EXT_VAL(rx_status, MSG_TYPE)){

	case STANDARD_DATA_FRAME:
		message->exide = 0;
		message->rtr = 0;
		if(exidf || srr)
			return ERR_CAN_MSG_TYPE;	//received Exted/RTR frame
		break;

	case EXTENDED_DATA_FRAME:
		message->exide = 1;
		message->rtr = 0;
		if(!exidf)
			return ERR_CAN_MSG_TYPE;	//received Std frame
		break;

	case STANDARD_REMOTE_FRAME:
		message->exide = 0;
		message->rtr = 1;
		if(exidf || !srr)
			return ERR_CAN_MSG_TYPE;	//received Exted/Data frame
		break;

	case EXTENDED_REMOTE_FRAME:
		message->exide = 1;
		message->rtr = 1;
		if(!exidf)
			return ERR_CAN_MSG_TYPE;	//received Std frame
		break;

	default: //error
	  break;
	}
	///printf("2, EXIDE %d, RTR %d\n", message->exide, message->rtr);
	if(message->exide){
		// read id
		message->id  = (uint32_t) (buf[0] << 21)
								| ((buf[1] & CAN_SIDL3_MASK) << 18)
								| ((buf[1] & CAN_EID1716_MASK) << 16)
								| (buf[2] << 8)
								| buf[3];

	}else{
		// read id
		message->id  = (uint32_t) buf[0] << 3 | buf[1] >> 5;
	}
	//printf("3, buf: %d, %d, %d\n",buf[0],buf[1],message->id);
	// read DLC
	length = buf[4] & 0x0f;
	message->length = length;

	if(message->rtr){
		// do nothing
	}else{
		// Data Frame
		if(length > 8){
		  //message error, need to handle
		}else{
		  // read data
		  memcpy(message->data, &buf[5], length);
		}
	}

	return 0; //errors
}


/*
 * This function sends CAN message to CAN bus and return the message buffer ID (1/2/3)
 * it used.
 */
int can_mcp2515_send_message (unsigned int app_id){
	uint8_t txbuf_id;
	int opMode = 0;
	int status = 0;
	int error = 0;

	CAN_msg_ptr message = &(msg->tx);

	(void) app_id;
	//printf("CAN send: enter\n");

	_acquire_spin_lock(&(local.can_lock));

	int local_mode = CAN_EXT_VAL(local.canctrl,REQOP);
	//make sure we are in the correct mode for send
	if(local_mode == NORMAL_MODE || local_mode == LOOPBACK_MODE){
		// allow for sending, but need to verify with the actual hardware
		opMode = _mcp2515_get_opMode();
		if(opMode != local_mode){
			printf("CAN send: RECOVING\n");
			//opMode does not match, CAN registers need to be recovery
			error = mcp2515_recovery();
			if(error){
				printf("CAN send: RECOVING ERROR! \n");
				_release_spin_lock(&(local.can_lock));
				return error;
			}
		}
	}else{
		// in the wrong mode - not allow for send!
		_release_spin_lock(&(local.can_lock));
		return ERR_INVALID_CAN_opMODE;
	}
//	printf("CAN send: sending\n");
	//Performs send
	status = _mcp2515_read_status();
	//determine which buffer we should use
	if (!CAN_EXT_VAL(status, TXB0CNTRL_TXREQ)){
		txbuf_id = TXB0ID;
	}else if (!CAN_EXT_VAL(status, TXB1CNTRL_TXREQ)){
		txbuf_id = TXB1ID;
	}else if (!CAN_EXT_VAL(status, TXB2CNTRL_TXREQ)){
		txbuf_id = TXB2ID;
	}else{
		_release_spin_lock(&(local.can_lock));
		// put request into job queues.
		if(local.mode == INTERRUPT){
			printf("all buffer used, status: %d\n",status);
			append_msg_to_queue(&tx_queue, message);
			return error;
		}else{
			return ERR_ALL_TXBUF_FULL;
		}
	}

	_mcp2515_load_tx_buffer(txbuf_id, message);
	udelay(1);
	_mcp2515_rts(txbuf_id);

	_release_spin_lock(&(local.can_lock));
//	printf("CAN send: leave\n");

	return error;
}

/*
 * Read a message from the message buffer.
 * Returns the number of messages ready for collection,
 * including the one read.
 * Returns 0 if there are no messages ready, unless wait is set to
 * force waiting for a message.
 */
int can_mcp2515_read_message(unsigned int app_id, unsigned int timeout){
	uint8_t status = 0;
	uint8_t addr = 0;
	int error = 0;
	int timeout_cnt = 0;

	CAN_msg_ptr message = &(msg->rx);

	(void) app_id;

	int opMode = 0;

	_acquire_spin_lock(&(local.can_lock));

	int local_mode = CAN_EXT_VAL(local.canctrl,REQOP);
	//make sure we are in the correct mode for read
	if(local_mode == NORMAL_MODE || local_mode == LOOPBACK_MODE){
		// allow for read, but need to verify with the actual hardware
		opMode = _mcp2515_get_opMode();
		if(opMode != local_mode){
			printf("CAN read: RECOVING\n");
			//opMode does not match, CAN registers need to be recovery
			error = mcp2515_recovery();
			if(error){
				printf("CAN read: RECOVING ERROR! \n");
				_release_spin_lock(&(local.can_lock));
				return error;
			}
		}
	}else{
		// in the wrong mode - not allow for read!
		_release_spin_lock(&(local.can_lock));
		return ERR_INVALID_CAN_opMODE;
	}

	_release_spin_lock(&(local.can_lock));

	if(local.mode == INTERRUPT){
		printf("can read: interrupt\n");
		//poll from rx queue
		if(timeout){
			timeout_cnt = timeout;
		}else{
			timeout_cnt = TIMEOUT_CONST;
		}

		while(timeout_cnt && !remove_msg_from_queue(&rx_queue, message)){
			if(timeout){
				timeout_cnt --;
			}
			//
			//seL4_Yield();
		}

		if(timeout && (timeout_cnt == 0)){
			printf("time out!\n");
			return ERR_TIMEOUT;
		}

		//printf("can read: leave\n");
	}else if(local.mode == POLL){

		_acquire_spin_lock(&(local.can_lock));
		//Performs read
		status = _mcp2515_rx_status();

		switch(CAN_EXT_VAL(status, RECEIVED_MSG)){
		case NO_RX_MSG:
		  _release_spin_lock(&(local.can_lock));
		  return 0;
		  break;
		case MSG_IN_RXB0:	addr = RXB0ID;
		  break;
		case MSG_IN_RXB1:	addr = RXB1ID;
		  break;
		case MSG_IN_BOTH:	addr = RXB0ID;
		  break;
		}
		error = _mcp2515_read_rx_buffer(addr,message,status);
		// clear interrupt flag
		if (addr == 0){
			_mcp2515_bit_modify(CANINTF, CAN_RX0IF_MASK, 0);
		}else{
			_mcp2515_bit_modify(CANINTF, CAN_RX1IF_MASK, 0);
		}
		_release_spin_lock(&(local.can_lock));
	}

	return error;	//(status & 0x07) + 1; Filter Match
}


void can_mcp2515_tx_priority(int txb, int x){
	switch(txb){
	case TXB0ID:	CAN_INST_VAL(local.txb0ctrl, TXP, x);
			_mcp2515_write_reg(TXBnCTRL(0),local.txb0ctrl);
		break;
	case TXB1ID:	CAN_INST_VAL(local.txb1ctrl, TXP, x);
			_mcp2515_write_reg(TXBnCTRL(1),local.txb1ctrl);
		break;
	case TXB2ID: CAN_INST_VAL(local.txb2ctrl, TXP, x);
			_mcp2515_write_reg(TXBnCTRL(2),local.txb2ctrl);
	}
}


void can_mcp2515_abort_tx(int txb){
	switch(txb){
	case TXB0ID:	CAN_INST_VAL(local.txb0ctrl, TXREQ, 0);
			_mcp2515_write_reg(TXBnCTRL(0),local.txb0ctrl);
		break;
	case TXB1ID:	CAN_INST_VAL(local.txb1ctrl, TXREQ, 0);
			_mcp2515_write_reg(TXBnCTRL(1),local.txb1ctrl);
		break;
	case TXB2ID: CAN_INST_VAL(local.txb2ctrl, TXREQ, 0);
			_mcp2515_write_reg(TXBnCTRL(2),local.txb2ctrl);
	}
}

void can_mcp2515_abort_all_tx(void){
	CAN_INST_VAL(local.canctrl, ABAT, 1);
	_mcp2515_write_reg(CANCTRL,local.canctrl);
}

/*
 * With One-shot mode enabled, a message will only attempt to transmit
 * one time, regardless of arbitration loss or error frame.
 * One-shot mode is required to maintain time slots in
 * deterministic systems, such as TTCAN
 */
void can_mcp2515_one_shot_mode(int op){
	CAN_INST_VAL(local.canctrl,OSM,op);
	_mcp2515_bit_modify(CANCTRL, CAN_OSM_MASK, local.canctrl);
}


/*
 * Error States Detection:
 * This function can be used to determine:
 */
int can_mcp2515_error_states(void){
	local.rec = _mcp2515_read_reg(REC,1);
	local.tec = _mcp2515_read_reg(TEC,1);
	if(local.rec < ERROR_PASSIVE_LIMIT && local.tec < ERROR_PASSIVE_LIMIT){
		return ERR_ACTIVE;
	}
	if(local.tec >= BUS_OFF_LIMIT){
		return ERR_BUS_OFF;
	}
	return ERR_PASSIVE;
}



/*
 * Set RXBn Operation Mode
 * options:
 * 		RECEIVE_ALL_MESSAGE
 * 		RECEIVE_VALID_STD_ID
 * 		RECEIVE_VALID_EXT_ID
 * 		RECEIVE_ALL_VALID_ID
 */
int can_mcp2515_rxb_opMode(int rxbuf_id, int rxbmode){

	switch(rxbuf_id){

	case RXB0ID:
		// Buffer 0
		CAN_INST_VAL(local.rxb0ctrl, RXM, rxbmode);
		_mcp2515_write_reg(RXB0CTRL,local.rxb0ctrl);
		break;
	case RXB1ID:
		// Buffer 1
		CAN_INST_VAL(local.rxb1ctrl, RXM, rxbmode);
		_mcp2515_write_reg(RXB1CTRL,local.rxb0ctrl);
		break;
	default:
		return ERR_INVALID_BUF_ID;
	}
	return 0;
}

/*
 * This function check the CANINITF and return according to
 * interrupt priority
 */
int mcp2515_check_Int(void){
	local.canintf = _mcp2515_read_reg(CANINTF,1);
	if(CAN_EXT_VAL(local.canintf,MERRF))	return MERR_INT;
	if(CAN_EXT_VAL(local.canintf,ERRIF))	return ERR_INT;
	if(CAN_EXT_VAL(local.canintf,WAKIF))	return WAKE_INT;
	if(CAN_EXT_VAL(local.canintf,TX0IF))	return TX0_INT;
	if(CAN_EXT_VAL(local.canintf,TX1IF))	return TX1_INT;
	if(CAN_EXT_VAL(local.canintf,TX2IF))	return TX2_INT;
	if(CAN_EXT_VAL(local.canintf,RX0IF))	return RX0_INT;
	if(CAN_EXT_VAL(local.canintf,RX1IF))	return RX1_INT;
	return NO_INT;
}

enum int_req_type{
	SEND_REQUEST,
	READ_REQUEST,
	ERROR_REQUEST,
	NO_REQUEST
};

static __inline__ int
handle_send_request(int id){
	CAN_msg outgoing;
	if(remove_msg_from_queue( &tx_queue, &outgoing)){
		//Performs send
		_mcp2515_load_tx_buffer(id, &outgoing);
		udelay(1);
		_mcp2515_rts(id);
		return 1;
	}
	return 0;
}

static __inline__ void
handle_read_request(int id){
	CAN_msg incoming;
	int status =0;
	//Performs read
	status = _mcp2515_rx_status();
	_mcp2515_read_rx_buffer(id, &incoming,status);
	//printf("appending imcoming\n");
	append_msg_to_queue( &rx_queue, &incoming);
	//printf("finished appending imcoming\n");
}

/*
 * If an error interrupt is generated, this function will be called
 * to handle the error state.
 */
int can_mcp2515_error_handling(void){
	uint8_t eflg = 0;
	CAN_msg temp;
	//read error flag EFLG
	eflg = _mcp2515_read_reg(EFLG,1);
	//Receive Buffer 1 Overflow
	if(CAN_EXT_VAL(eflg,RX1OVR)){
		//printf("Receive Buffer 1 Overflow\n");
		//set when a valid message is received for RXB1 and CANINTF.RX1IF = 1
		//read in message
		handle_read_request(RXB1ID);
		//_mcp2515_read_rx_buffer(RXB1ID, &temp, _mcp2515_rx_status());
		//reset to clear
		_mcp2515_bit_modify(EFLG,CAN_RX1OVR_MASK,0xff);
	}
	if(CAN_EXT_VAL(eflg, RX0OVR)){
		//printf("Receive Buffer 0 Overflow\n");
		//set when a valid message is received for RXB0 and CANINTF.RX0IF = 1
		//read in message
		handle_read_request(RXB0ID);
		//_mcp2515_read_rx_buffer(RXB0ID, &temp, _mcp2515_rx_status());
		//reset to clear
		_mcp2515_bit_modify(EFLG,CAN_RX0OVR_MASK,0xff);
	}
	if(CAN_EXT_VAL(eflg, TXB0)){
		printf("Bus-off \n");
		//Bus-off error, tec >= 255
		//reset or wait until a successful bus recovery sequence
	}
	if(CAN_EXT_VAL(eflg, TXEP)){
		printf("Transmit-Error-passive \n");
		//Transmit Error-passive, TEC >= 128, will be reset when is less
	}
	if(CAN_EXT_VAL(eflg, TXWAR)){
		printf("Transmit Error Warning\n");
		//Transmit Error Warning, TEC >= 96, will be reset when is less
	}
	if(CAN_EXT_VAL(eflg, RXWAR)){
		printf("Receive Error Warning\n");
		//Receive Error Warning, REC >= 96, will be reset when is less
	}
	if(CAN_EXT_VAL(eflg, EWARN)){
		printf("EC/REC >= 96\n");
		//set when TEC/REC >= 96 (TXWAR/RXWAR = 1), reset when both are less
	}
	return 0;
}

void can_mcp2515_handle_interrupts(void* data UNUSED){
	int mask = 0;

	if(local.mode == POLL){
		//Int_reg_callback(&can_mcp2515_handle_interrupts);
		IntAck_emit();	//release the gpio thread from blocking
		return;
	}

	if(local.state != INITALISED){
		Int_reg_callback(&can_mcp2515_handle_interrupts, NULL);
		IntAck_emit();	//release the gpio thread from blocking
		return;
	}
	/*
	 * Interrupt handling routines
	 */
	_acquire_spin_lock(&(local.can_lock));

	local.canintf = _mcp2515_read_reg(CANINTF,1);

	//message error
	if(CAN_EXT_VAL(local.canintf,MERRF)){
		printf("message error\n");
		CAN_INST_VAL(local.canintf, MERRF, 0);
		mask |= CAN_MERRF_MASK;
	}

	//errors
	if(CAN_EXT_VAL(local.canintf,ERRIF)){
		//printf("error: ");
		CAN_INST_VAL(local.canintf, ERRIF, 0);
		mask |= CAN_ERRIF_MASK;
		can_mcp2515_error_handling();
	}

	//wake up
	if(CAN_EXT_VAL(local.canintf,WAKIF)){
		printf("Wake up\n");
		CAN_INST_VAL(local.canintf, WAKIF, 0);
		//wake-up
		mask |= CAN_WAKIF_MASK;
	}

	//TXB0
	if(CAN_EXT_VAL(local.canintf,TX0IF)){
		//printf("TXB0 interrupt\n");
		CAN_INST_VAL(local.canintf, TX0IF, 0);
		mask |= CAN_TX0IF_MASK;
		handle_send_request(TXB0ID);
	}

	//TXB1
	if(CAN_EXT_VAL(local.canintf,TX1IF)){
		//printf("TXB1 interrupt\n");
		CAN_INST_VAL(local.canintf, TX1IF, 0);
		mask |= CAN_TX1IF_MASK;
		handle_send_request(TXB1ID);
	}

	//TXB2
	if(CAN_EXT_VAL(local.canintf,TX2IF)){
		//printf("TXB2 interrupt\n");
		CAN_INST_VAL(local.canintf, TX2IF, 0);
		mask |= CAN_TX2IF_MASK;
		handle_send_request(TXB2ID);
	}

	//RxB0
	if(CAN_EXT_VAL(local.canintf,RX0IF)){
		//printf("RXB0 interrupt\n");
		CAN_INST_VAL(local.canintf, RX0IF, 0);
		mask |= CAN_RX0IF_MASK;
		handle_read_request(RXB0ID);
	}

	//RxB1
	if(CAN_EXT_VAL(local.canintf,RX1IF)){
		//printf("RXB1 interrupt\n");
		CAN_INST_VAL(local.canintf, RX1IF, 0);
		mask |= CAN_RX1IF_MASK;
		handle_read_request(RXB1ID);
	}

	//acknowledge all interrupts
	_mcp2515_bit_modify(CANINTF, mask, local.canintf);
	_release_spin_lock(&(local.can_lock));

	Int_reg_callback(&can_mcp2515_handle_interrupts, NULL);
	IntAck_emit();	//acknowledge to GPIO interrupt
}

/*
 * MCP2515 Interrupt Configuration
 */
int can_mcp2515_config_Int(int irq, int op){
	uint8_t mask = 0;
	// Clear Interrupts
	if(CLEAR == op){
		switch(irq){
		case MERR_INT: CAN_INST_VAL(local.canintf, MERRF, 0);
			mask = CAN_MERRF_MASK;
			break;
		case WAKE_INT: CAN_INST_VAL(local.canintf, WAKIF, 0);
			mask = CAN_WAKIF_MASK;
			break;
		case ERR_INT: CAN_INST_VAL(local.canintf, ERRIF, 0);
			mask = CAN_ERRIF_MASK;
			break;
		case TX2_INT: CAN_INST_VAL(local.canintf, TX2IF, 0);
			mask = CAN_TX2IF_MASK;
			break;
		case TX1_INT: CAN_INST_VAL(local.canintf, TX1IF, 0);
			mask = CAN_TX1IF_MASK;
			break;
		case TX0_INT: CAN_INST_VAL(local.canintf, TX0IF, 0);
			mask = CAN_TX0IF_MASK;
			break;
		case RX1_INT: CAN_INST_VAL(local.canintf, RX1IF, 0);
			mask = CAN_RX1IF_MASK;
			break;
		case RX0_INT: CAN_INST_VAL(local.canintf, RX0IF, 0);
			mask = CAN_RX0IF_MASK;
			break;
		case NO_INT:
			return 0;
		}
		_mcp2515_bit_modify(CANINTF, mask, local.canintf);
		return 0;
	}
	// Enable/disable Interrupts
	switch(irq){
	case MERR_INT: 	CAN_INST_VAL(local.caninte, MERRE, op);
		break;
	case WAKE_INT: 	CAN_INST_VAL(local.caninte, WAKIE, op);
		break;
	case ERR_INT: 	CAN_INST_VAL(local.caninte, ERRIE, op);
		break;
	case TX2_INT: 	CAN_INST_VAL(local.caninte, TX2IE, op);
		break;
	case TX1_INT: 	CAN_INST_VAL(local.caninte, TX1IE, op);
		break;
	case TX0_INT: 	CAN_INST_VAL(local.caninte, TX0IE, op);
		break;
	case RX1_INT:	CAN_INST_VAL(local.caninte, RX1IE, op);
		break;
	case RX0_INT: 	CAN_INST_VAL(local.caninte, RX0IE, op);
		break;
	case NO_INT:
		return 0;
	}
	_mcp2515_write_reg(CANINTE,local.caninte);
	return 0;
}


//Configure Baud Rate Prescaler
int can_mcp2515_config_baudrate(int speed){

	int pre_mode = _mcp2515_get_opMode();

	if(pre_mode > CONFIGURATION_MODE || pre_mode < NORMAL_MODE){
		//SPI bus error
		return ERR_CAN_ACCESS_FAILED;
	}

	_mcp2515_set_opMode(CONFIGURATION_MODE);

	switch(speed){
					    //Set to 62.5 kbps
						//SJW=00(Length=1 TQ), BRP = 15(TQ = 2 x (15+1)/FOSC)
						//CNF1 = 0:0 + 0:0:1:1:1:1 -> 62.5 Kbps
	 case CAN_62_5kbps:
		 	 	 	 	CAN_INST_VAL(local.cnf1,BRP,CAN_62_5kbps);
						break;

					   //Set to 125 kbps
						//SJW=00(Length=1 TQ), BRP = 7(TQ = 2 x (7+1)/FOSC)
						//CNF1 = 0:0 + 0:0:0:1:1:1 -> 125  Kbps
	 case CAN_125kbps:  CAN_INST_VAL(local.cnf1,BRP,CAN_125kbps);
						break;

						//Set to 250 kbps
						//SJW=00(Length=1 TQ), BRP = 3(TQ = 2 x (3+1)/FOSC)
					   //CNF1 = 0:0 + 0:0:0:0:1:1 -> 250  Kbps
	 case CAN_250kbps:  CAN_INST_VAL(local.cnf1,BRP,CAN_250kbps);
						break;

						//Set to 500 kbps
					   //SJW=00(Length=1 TQ), BRP = 1(TQ = 2 x (1+1)/FOSC)
						//CNF1 = 0:0 + 0:0:0:0:0:1 -> 500  Kbps
	 case CAN_500kbps:  CAN_INST_VAL(local.cnf1,BRP,CAN_500kbps);
						break;

						//Set to 1000 kbps
						//SJW=00(Length=1 TQ), BRP = 0(TQ = 2 x (0+1)/FOSC)
						//CNF1 = 0:0 + 0:0:0:0:0:0 -> 1000 Kbps
	 case CAN_1000kbps: CAN_INST_VAL(local.cnf1,BRP,CAN_1000kbps);
						break;

						//unrecognized or null command
	 default:           return 0;
	}

	//BTLMODE=1,PHSEG1=3TQ(0:1:0),PRSEG=3TQ(0:1:0)
	//CNF2 = 1+0+0:1:0+0:1:0
	CAN_INST_VAL(local.cnf2, BTLMODE, 1);	//1 bit value
	CAN_INST_VAL(local.cnf2, PHSEG1, 2);	//3 bit value
	CAN_INST_VAL(local.cnf2, PRSEG, 2);	//3 bit value
	//PHSEG2=3TQ(0:1:0)
	//CNF3 = 0+0+x:x:x:0:1:0
	CAN_INST_VAL(local.cnf3, PHSEG2, 2);	//3 bit value

	//update registers
	_mcp2515_write_reg(CNF1,local.cnf1);
	_mcp2515_write_reg(CNF2,local.cnf2);
	_mcp2515_write_reg(CNF3,local.cnf3);

	//put the CAN device into previous operation mode
	_mcp2515_set_opMode(pre_mode);
	return 0;
}


/*
 * This function is used to customize the CAN baud rate setting
 * SOF = 0
 * WAKFIL = 0
 * BTLMODE = 1 (PS2 determined by CNF3)
 * SAM: Triple_sample(1) or Single_sample(0)
 */
int can_mcp2515_set_baudrate(unsigned int baudrate, unsigned int sjw, unsigned int sam, unsigned int sample_point_percent){
	int pre_mode = -1;
	// all in TQ unit
	uint8_t ps1, ps2, syncseg, propseg = 0;
	uint8_t nTQ,nTQrest = 0;	//num of TQs within 1 CAN bit time = 1/baudrate
	uint8_t brp = 0;	//minimum baud rate prescaler

	/* Calculate baud rate prescaler(BRP),
	 * provide the maximum number of TQs within 1 CAN bit time
	 *
	 * 			2 * BRP							   1
	 * TQ = -----------------;  nTQ * TQ = ------------------- = 1 CAN bit time
	 * 			CAN_XTAL						baudrate
	 *
	 *					1					CAN_XTAL
	 * baudrate = ----------------- =  -----------------
	 *				nTQ * TQ			nTQ * 2 * BRP
	 */
	do{
		brp++; // update BRP
		nTQ = (uint8_t)( CAN_XTAL / ((baudrate * 2 * (uint32_t)brp)) );

	} while( brp <= (CAN_MAX_BRP + 1) && nTQ > CAN_MAX_TQ );


	// 75% sampling point, thus PS2 = nTQ / 4
	//nTQrest = nTQ - (nTQ >> 2)

	if(sample_point_percent >= CAN_MAX_SAMPLE_POINT){
		return ERR_SET_BAUDRATE_FAILED;
	}

	//calculate ps2 according to the user-provided sampling point
	nTQrest = (uint8_t)((uint32_t) nTQ * sample_point_percent / 100);
	ps2 = nTQ - nTQrest;
	//for PS1 = (rest of TQ) / 2 - 1
	//ps1 = (nTQrest >> 1);
	//for propseg,  (rest of TQ) / 2 - syncseg (1) - 1
	syncseg = 1; //fixed
	propseg = 3; //(nTQrest >> 1) - syncseg;
	ps1 = nTQrest - propseg - syncseg;
	if((propseg+ps1) < ps2 || (propseg + ps1) < 2 || ps2 < sjw){
		return ERR_SET_BAUDRATE_FAILED;
	}

	CAN_INST_VAL(local.cnf1, BRP, 	 brp - 1);
	CAN_INST_VAL(local.cnf3, PHSEG2, ps2 - 1);
	CAN_INST_VAL(local.cnf2, PHSEG1, ps1 - 1);
	CAN_INST_VAL(local.cnf2, PRSEG,  propseg - 1);

	if(sam){
		CAN_INST_VAL(local.cnf2, SAM, sam);
	}
	//set SJW, default 0
	if( sjw > 0 && (sjw < CAN_MAX_SJW)){
		CAN_INST_VAL(local.cnf1, SJW, (sjw-1));
	}
	printf("baudrate: BRP %d, propseg %d, ps1 %d, ps2 %d, SJW %d, SAM %d\n",\
			brp, propseg, ps1, ps2, sjw, sam);

	pre_mode = _mcp2515_get_opMode();
	if(pre_mode > CONFIGURATION_MODE || pre_mode < NORMAL_MODE){
		//SPI bus error
		return ERR_CAN_ACCESS_FAILED;
	}

	_mcp2515_set_opMode(CONFIGURATION_MODE);
	//update registers
	_mcp2515_write_reg(CNF1,local.cnf1);
	_mcp2515_write_reg(CNF2,local.cnf2);
	_mcp2515_write_reg(CNF3,local.cnf3);
	//put the CAN device into previous operation mode
	_mcp2515_set_opMode(pre_mode);

	return 0;
}

/*
 * For auto-baud detection, it is necessary that at least two other nodes are communicating
 * with each other. The filters and masks can be used to allow only particular messages to be loaded
 * into the receive registers, or the masks can be set to all zeros to allow a message with
 * any identifier to pass. The controller will be placed in Listen-only mode.
 */
int can_mcp2515_auto_baud_rate_detection(void){
	int baudrate = 0;
	uint8_t backup[3];	//for backup

	int pre_mode = _mcp2515_get_opMode();

	if(pre_mode > CONFIGURATION_MODE || pre_mode < NORMAL_MODE){
		//SPI bus error
		return ERR_CAN_ACCESS_FAILED;
	}

	backup[0] = local.rxb0ctrl;
	backup[1] = local.rxb1ctrl;
	backup[2] = local.caninte;

	//set to receive all messages
	can_mcp2515_rxb_opMode(RXB0ID, RECEIVE_ALL_MESSAGE);
	can_mcp2515_rxb_opMode(RXB1ID, RECEIVE_ALL_MESSAGE);
	//enable message error interrupt
	can_mcp2515_config_Int(MERR_INT, ENABLE);
	_mcp2515_set_opMode(LISTENONLY_MODE);

	//check interrupt
	do{
		//baud rate is not correct
		baudrate += 10000; // next baud rate, every 10kbist/s
		can_mcp2515_set_baudrate(baudrate,1,1,60);
		udelay(1);
		//clear interrupt
		can_mcp2515_config_Int(MERR_INT, CLEAR);
		//delay arbitrary length of time to allow message being received
		udelay(10);
	}while(MERR_INT == mcp2515_check_Int());
	//now the device should locked up to the desired baud rate
	//Finished, restore to previous states and return
	can_mcp2515_rxb_opMode(RXB0ID,	backup[0]);
	can_mcp2515_rxb_opMode(RXB1ID,	backup[1]);
	_mcp2515_write_reg(CANINTE,	backup[2]);
	_mcp2515_set_opMode(pre_mode);

	return baudrate;
}

/*
 * This function is used to recovery the CAN controller upon power interruption
 */
int mcp2515_recovery(void){
	uint8_t mode = 0;

	//reset the chip
	_mcp2515_reset();
	udelay(10);

	mode = _mcp2515_get_opMode();
	// Now the device should be in configuration mode
	//mode = _mcp2515_read_reg(CANSTAT, 1);
	if(mode != CONFIGURATION_MODE){
		return ERR_CAN_RESET_FAILD;
	}
	//recover the CNF registers
	_mcp2515_write_reg(CNF1, local.cnf1);
	//test accessibility
	if(_mcp2515_read_reg(CNF1,1)!= local.cnf1){
		return ERR_CAN_ACCESS_FAILED;
	}
	_mcp2515_write_reg(CNF2, 		local.cnf2);
	_mcp2515_write_reg(CNF3, 		local.cnf3);
	_mcp2515_write_reg(CANINTE, 	local.caninte);
	_mcp2515_write_reg(TXRTSCTRL, 	local.txrtsctrl);
	_mcp2515_write_reg(BFPCTRL, 	local.bfpctrl);
	_mcp2515_write_reg(TXBnCTRL(0),	local.txb0ctrl);
	_mcp2515_write_reg(TXBnCTRL(1),	local.txb1ctrl);
	_mcp2515_write_reg(TXBnCTRL(2),	local.txb2ctrl);
	_mcp2515_write_reg(RXB0CTRL,	local.rxb0ctrl);
	_mcp2515_write_reg(RXB1CTRL,	local.rxb1ctrl);
	// write from 0x00 ~ 0x0B (RXF0SIDH ~ RXF2EID0)
	_mcp2515_write_mult_regs(RXF0SIDH, &(local.rxf0sidh), 12);
	// write from 0x10 ~ 0x1B (RXF3SIDH ~ RXF5EID0)
	_mcp2515_write_mult_regs(RXF3SIDH, &(local.rxf3sidh), 12);
	// write from 0x20 ~ 0x27 (RXM0SIDH ~ RXM1EID0)
	_mcp2515_write_mult_regs(RXM0SIDH, &(local.rxm0sidh), 8);
	// all error counters are cleared
	local.tec = local.rec = 0;
	//put into previous opMode
	_mcp2515_bit_modify(CANCTRL, 0xE0, local.canctrl);
	//now the CAN should be up and running
	return 0;
}

/*
 * This function is used to enable/disable RXB0 message
 * rollover to RXB1 if RXB0 is full
 */
void can_mcp2515_rxb_rollover(int op){
	CAN_INST_VAL(local.rxb0ctrl, RXB0_BUKT, op);
	_mcp2515_write_reg(RXB0CTRL, local.rxb0ctrl);
}


/* _mcp2515_local_init:
 * Initialize the local can registers to the default states to match up with mcp2515
 * default states
 */
void _mcp2515_local_init(void){
	memset(&local, 0, sizeof(struct mcp2515_reg_t));
	/* CAN Control: (default) Configuration mode, CLKEN = 1, CLKPRE = 0b11*/
	local.canctrl = 0x87;
}

/*
 * Reset and initialise the CAN controller
 */
int can_mcp2515_init(int speed){

	int v,mode = 0;

	local.can_lock = 0;

	_acquire_spin_lock(&(local.can_lock));
	//reset the local can register copy
	_mcp2515_local_init();
	/*
	 * Reset, set CNF1, CNF2, CNF3
	 * Set TXRTSCTRL
	 * Set filter and mask registers
	 * Configure receive regs as FIFO.
	 */
	_mcp2515_reset();
	/*
	 * delay a bit till MCP2515 has restart
	 */
	udelay(10);

	_mcp2515_set_opMode(CONFIGURATION_MODE);
	mode = _mcp2515_get_opMode();
	// Now the device should be in configuration mode
	//mode = _mcp2515_read_reg(CANSTAT, 1);
	printf("mode: %u\n", mode);
	if(mode != CONFIGURATION_MODE){
		printf("MCP2515 is not connected!\n");
		_release_spin_lock(&(local.can_lock));
		return ERR_CAN_RESET_FAILD;
	}
	//can_mcp2515_config_baudrate(speed);
	v = can_mcp2515_set_baudrate(speed, 1, 1, 60);
	if(v ){
		printf("baud set error %d\n",v);
		_release_spin_lock(&(local.can_lock));
		return v;
	}

	//Enable RX1,RX0 all Interrupts
//	can_mcp2515_config_Int(RX0_INT, ENABLE);
//	can_mcp2515_config_Int(RX1_INT, ENABLE);
//	can_mcp2515_config_Int(TX0_INT, ENABLE);
//	can_mcp2515_config_Int(TX1_INT, ENABLE);
//	can_mcp2515_config_Int(TX2_INT, ENABLE);
//	can_mcp2515_config_Int(ERR_INT, ENABLE);
//	can_mcp2515_config_Int(MERR_INT, ENABLE);

	local.mode	= POLL;


	can_mcp2515_one_shot_mode(DISABLE);
	//disable rollover
	can_mcp2515_rxb_rollover(DISABLE);
	// Buffer 0 : Receive All Message
	can_mcp2515_rxb_opMode(RXB0ID, RECEIVE_VALID_STD_ID);
	//Buffer 1 : Receive All Message
	can_mcp2515_rxb_opMode(RXB1ID, RECEIVE_VALID_STD_ID);
	// Configure Filter RXF0
	//can_mcp2515_set_FILTER_MASK(RXF0, 0x345, 0);
	//Mask RXM0
	//can_mcp2515_set_FILTER_MASK(RXM0, 0xFFFF, 0);
	//can_mcp2515_set_FILTER_MASK(RXM1, 0xFFFF, 0);

	//Enable RX1BF,RX0BF Output Pins
	local.bfpctrl = 0;
	CAN_INST_VAL(local.bfpctrl, B1BFE, ENABLE);
	CAN_INST_VAL(local.bfpctrl, B0BFE, ENABLE);
	CAN_INST_VAL(local.bfpctrl, B1BFM, ENABLE);
	CAN_INST_VAL(local.bfpctrl, B0BFM, ENABLE);
	_mcp2515_write_reg(BFPCTRL,local.bfpctrl);
	//Disable TX0RTS,TX1RTS,TX2RTS Input Pin
	local.txrtsctrl = 0;
	_mcp2515_write_reg(TXRTSCTRL, local.txrtsctrl);
	//put CAN device into normal operation mode
	_mcp2515_set_opMode(NORMAL_MODE);

	v = _mcp2515_read_reg(CANCTRL, 1);
	printf("MCP2515 init, CANCTRL %d\n",v);
	local.state = INITALISED;

	if(local.mode == INTERRUPT){
		Int_reg_callback(&can_mcp2515_handle_interrupts, NULL);
	}

	_release_spin_lock(&(local.can_lock));

	return 0;
}

/*
 * Read a message from the message buffer.
 * Returns the number of messages ready for collection,
 * including the one read.
 * Returns 0 if there are no messages ready, unless wait is set to
 * force waiting for a message.
 */
/*
int mcp2515_read_can_message(struct can_message *msg, int wait)
{
    int x = _mcp2515_rx_status();
    uint8_t buf[14];

    if (!(x & (CANSTAT_RXB0|CANSTAT_RXB1))) {
        if (!wait)
            return 0;
        while (!(x & (CANSTAT_RXB0|CANSTAT_RXB1))) {
            x = _mcp2515_rx_status();
        }
    }

    //mcp2515_spi_dev->txbuf[0] = MCP_READ_RX_BUFFER(x & CANSTAT_RXB1 ? 2 : 0);


    mcp2515_spi_transfer(&mcp2515_spi_dev, 1, 12);


    msg->can_flags = 0;

    msg->can_addr = buf[0] << 3 | buf[1] >> 5;
    if (buf[1] & 0x04) // Extended identifier frame
	{
        msg->can_addr <<= 18;
        msg->can_addr |= (buf[1] & 3) << 16 | (buf[2] << 8) | buf[3];
        msg->can_flags |= CAN_IDE;
    }

    msg->can_len = buf[4] & 0xf;
    if (buf[4] & (1<<6))
        msg->can_flags |= CAN_RTR;

    memset(msg->can_payload, 0, sizeof msg->can_payload);
    memcpy(msg->can_payload, &buf[5], msg->can_len);


    return !!(x & CANSTAT_RXB0) + !!(x & CANSTAT_RXB1);
}
*/


/*
 * Set filters or masks
 */
int can_mcp2515_set_FILTER_MASK(int filtermask, unsigned int val, unsigned int exide){
	uint8_t sidh,sidl, eid8, eid0 = 0;
	uint8_t * localdes = NULL;
	uint8_t reg = 0;

	int pre_mode = _mcp2515_get_opMode();

	if(pre_mode > CONFIGURATION_MODE || pre_mode < NORMAL_MODE){
		//SPI bus error
		return ERR_CAN_ACCESS_FAILED;
	}
	_mcp2515_set_opMode(CONFIGURATION_MODE);

	if(exide == TRUE){
		sidh = (val >> 21) & CAN_SIDH8_MASK;
		sidl = (val >> 18) & CAN_SIDL3_MASK;
		//ExID Enable bit
		CAN_INST_VAL(sidl,EXIDE,1);
		CAN_INST_VAL(sidl,EID1716,(val >> 16));
		//Extend ID
		eid8 = (val >> 8) & CAN_EID8_MASK;
		eid0 = (val) & CAN_EID0_MASK;

	}else{
		sidh = (val >> 3) & CAN_SIDH8_MASK;
		sidl = (val << 5) & CAN_SIDL3_MASK;
		//Extend ID
		eid8 = 0;
		eid0 = 0;
	}
	switch(filtermask){
	case RXF0: 	localdes = &local.rxf0sidh;
				reg = RXF0SIDH;
				break;
	case RXF1: 	localdes = &local.rxf1sidh;
				reg = RXF1SIDH;
				break;
	case RXF2: 	localdes = &local.rxf2sidh;
				reg = RXF2SIDH;
				break;
	case RXF3: 	localdes = &local.rxf3sidh;
				reg = RXF3SIDH;
				break;
	case RXF4: 	localdes = &local.rxf4sidh;
				reg = RXF4SIDH;
				break;
	case RXF5: 	localdes = &local.rxf5sidh;
				reg = RXF5SIDH;
				break;
	case RXM0: 	localdes = &local.rxm0sidh;
				reg = RXM0SIDH;
				break;
	case RXM1: 	localdes = &local.rxm1sidh;
				reg = RXM1SIDH;
				break;
	}
	localdes[0] = sidh;
	localdes[1] = sidl;
	localdes[2] = eid8;
	localdes[4] = eid0;
	_mcp2515_write_mult_regs(reg, localdes, 4);
	//put the CAN device into previous operation mode
	_mcp2515_set_opMode(pre_mode);
	return 0;
}


int can_mcp2515_rxb_filter_hit(int rxid){
	int rxbctrl, RXFhit = 0;
	switch(rxid){
	case RXB0ID:
		rxbctrl = _mcp2515_read_reg(RXB0CTRL,1);
		RXFhit = CAN_EXT_VAL(rxbctrl,RXB0_FILHIT);
		break;
	case RXB1ID:
		rxbctrl = _mcp2515_read_reg(RXB1CTRL,1);
		RXFhit = CAN_EXT_VAL(rxbctrl,RXB1_FILHIT);
		break;
	}
	return RXFhit;
}


