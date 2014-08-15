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
 * can_inf.h
 *  Created on: Jun 6, 2013
 *      Author: Jiawei Xie
 */

#ifndef CAN_INF_H_
#define CAN_INF_H_


typedef struct{
	uint32_t id;
	uint8_t exide:1;
	uint8_t rtr:1;
	uint8_t length:4;
	uint8_t data[8];
} CAN_msg, *CAN_msg_ptr;

typedef struct{
	CAN_msg rx;
	CAN_msg tx;
}CAN_data, *CAN_data_ptr;


#define CAN_IDE 				1 			/* Extended address */
#define CAN_RTR 				2 			/* Remote frame Request */
/*
 * Basic CAN Bus Baud rate Setting
 */
#define CAN_1000kbps  			0       							// CAN speed at 1000 kbps
#define CAN_500kbps   			1       							// CAN speed at 500  kbps
#define CAN_250kbps   			3   								// CAN speed at 250  kbps
#define CAN_125kbps   			7									// CAN speed at 125  kbps
#define CAN_62_5kbps   			15 									// CAN speed at 62.5 kbps

#define TXB0ID					0
#define TXB1ID					0x02
#define TXB2ID					0x04
#define RXB0ID					0
#define RXB1ID					0x04

#define ERROR_PASSIVE_LIMIT		128
#define BUS_OFF_LIMIT			255

#define TIMEOUT_CONST			500000000000000000

/*
 * CAN device commands
 */
//un-used
typedef enum{
	CAN_ctl_SET_BAUDRATE = 128,
	CAN_ctl_SENG,
	CAN_ctl_READ,
	CAN_ctl_ABORT_TX,
	CAN_ctl_ABORT_ALL_TX,
	CAN_ctl_UPDATE_RXB_OPMODE,
	CAN_ctl_REFRESH_ERROR
}CAN_ctl;

typedef struct {
	/* Operation Command */
	CAN_ctl cmd;// = 0;

	/* CAN default settings */
	int baudRate;//	 = CAN_125kbps;
	int rxRollOver;//	 = ENABLE;
	int oneShortMode;// = DISABLE;
	int errorState;//	 = 0;

	/* CAN message buffer */
	CAN_msg rxbuf;
	CAN_msg txbuf;

}CAN_dev, *CAN_dev_p;


/*
 * Error Code
 */
typedef enum error_mode_t{

	ERR_CAN_MSG_TYPE		=	-1,

	ERR_ACTIVE				= 	-2,
	ERR_PASSIVE				=	-3,
	ERR_BUS_OFF				=	-4,

	ERR_SPI_READ			=	-5,
	ERR_SPI_WRITE			=	-6,

	ERR_CAN_RESET_FAILD		=	-7,
	ERR_CAN_ACCESS_FAILED	=	-8,

	ERR_INVALID_BUF_ID		=	-9,
	ERR_INVALID_CAN_opMODE	= 	-10,
	ERR_SET_BAUDRATE_FAILED	= 	-11,

	ERR_ALL_TXBUF_FULL		= 	-12,

	ERR_TIMEOUT				= 	-13

}CAN_ERROR_CODE;

typedef enum{
	NO_INT	 	= 0,
	MERR_INT 	= 1,
	WAKE_INT 	= 2,
	ERR_INT 	= 3,
	TX2_INT		= 4,
	TX1_INT		= 5,
	TX0_INT		= 6,
	RX1_INT		= 7,
	RX0_INT		= 8
}MCP_INTERRUPT;

typedef enum{
	NO_RX_MSG 	= 0,
	MSG_IN_RXB0 = 1,
	MSG_IN_RXB1 = 2,
	MSG_IN_BOTH = 3
}RECEIVED_MSG;

typedef enum msg_type_t{
	STANDARD_DATA_FRAME 	= 0,
	STANDARD_REMOTE_FRAME 	= 1,
	EXTENDED_DATA_FRAME 	= 2,
	EXTENDED_REMOTE_FRAME 	= 3
}MSG_TYPE;

/* Request Operation Modes
 *  */
typedef enum MCP_opMode_t{
	NORMAL_MODE 		= 0,
	SLEEP_MODE 			= 1,
	LOOPBACK_MODE 		= 2,
	LISTENONLY_MODE 	= 3,
	CONFIGURATION_MODE 	= 4
}REQOP_MODE;

typedef enum clkpre_t{
	SYSCLK_DIV_1 = 0,
	SYSCLK_DIV_2 = 1,
	SYSCLK_DIV_4 = 2,
	SYSCLK_DIV_8 = 3
}FCLKOUT;

typedef enum txb_priority_t{
	HIGHEST_MESSAGE_PRIORITY 			= 3,
	HIGH_INTERMEDIATE_MESSAGE_PRIORITY	= 2,
	LOW_INTERMEDIATE_MESSAGE_PRIORITY 	= 1,
	LOWEST_MESSAGE_PRIORITY 			= 0
}TXB_PRIORITY;

typedef enum RXB_opMode_t{
	RECEIVE_ALL_MESSAGE 	= 3,
	RECEIVE_VALID_EXT_ID 	= 2,
	RECEIVE_VALID_STD_ID 	= 1,
	RECEIVE_ALL_VALID_ID 	= 0
}RXB_opMode;

typedef enum filter_hit_bits_t{
	//Acceptance Filter x
	RXF5 = 5,
	RXF4 = 4,
	RXF3 = 3,
	RXF2 = 2,
	RXF1 = 1,
	RXF0 = 0,
	//Acceptance Mask x
	RXM1 = 7,
	RXM0 = 6
}RXB_FILTER_HIT;


#endif /* CAN_INF_H_ */
