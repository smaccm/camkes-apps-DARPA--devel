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
 * MCP2515 instructions and registers
 *
 */

#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include "can_inf.h"
#include "common.h"

/* Transmit Buffers */
#define TXBCTRL(n)  ((((n) + 0x3) << 4)) //Control register
#define TXBSIDH(n)  (TXBCTRL(n) + 0x1)   //Standard identifier high
#define TXBSIDL(n)  (TXBCTRL(n) + 0x2)   //Standard identifier low
#define TXBEID8(n)  (TXBCTRL(n) + 0x3)   //Extended identifier high
#define TXBEID0(n)  (TXBCTRL(n) + 0x4)   //Extended identifier low
#define TXBDLC(n)   (TXBCTRL(n) + 0x5)   //Data length code

/* Transmit Buffer Control Bits */
#define TXBCTRL_ABTF     BIT(6) //Message aborted flag
#define TXBCTRL_MLOA     BIT(5) //Message lost arbitration
#define TXBCTRL_TXERR    BIT(4) //Transmission error detected
#define TXBCTRL_TXREQ    BIT(3) //Message transmit request
#define TXBCTRL_TXP_SHF  0      //Transmit buffer priority

/* Receive Buffers */
#define RXBCTRL(n)  ((((n) + 0x6) << 4)) //Control register
#define RXBSIDH(n)  (RXBCTRL(n) + 0x1)   //Standard identifier high
#define RXBSIDL(n)  (RXBCTRL(n) + 0x2)   //Standard identifier low
#define RXBEID8(n)  (RXBCTRL(n) + 0x3)   //Extended identifier high
#define RXBEID0(n)  (RXBCTRL(n) + 0x4)   //Extended identifier low
#define RXBDLC(n)   (RXBCTRL(n) + 0x5)   //Data length code

/* Receive Buffer Control Bits */
#define RXBCTRL_RXM_SHF     BIT(5) //Operating mode
#define RXBCTRL_RTR         BIT(3) //Received remote transfer request
#define RXBCTRL_BUKT_SHF    BIT(1) //Rollover enable
#define RXBCTRL_FILHIT_SHF  BIT(0) //Filter bit

/* Control Registers */
#define CANSTAT  0xE //CAN controller state
#define CANCTRL  0xF //CAN controller control
#define CANINTE  0x2B //CAN interrupt enable
#define CANINTF  0x2C //CAN interrupt flag

#define CANCTRL_REQOP_MASK  0xE0   //Request operation mode bit mask
#define CANCTRL_ABAT        BIT(4) //Abort all pending transmissions
#define CANCTRL_OSM         BIT(3) //One shot mode
#define CANCTRL_CLKEN       BIT(2) //CLKOUT pin enable
#define CANCTRL_CLKPRE_SHF  0      //CLKOUT pin prescaler

#define CANINTE_MERRE  BIT(7) //Message error interrupt enable
#define CANINTE_WAKIE  BIT(6) //Wakeup interrupt enable
#define CANINTE_ERRIE  BIT(5) //Error interrupt enable
#define CANINTE_TX2IE  BIT(4) //Transmit buffer 2 empty interrupt enable
#define CANINTE_TX1IE  BIT(3) //Transmit buffer 1 empty interrupt enable
#define CANINTE_TX0IE  BIT(2) //Transmit buffer 0 empty interrupt enable
#define CANINTE_RX1IE  BIT(1) //Receive buffer 1 full interrupt enable
#define CANINTE_RX0IE  BIT(0) //Receive buffer 0 full interrupt enable

#define CANINTF_MERRF  BIT(7) //Message error interrupt flag
#define CANINTF_WAKIF  BIT(6) //Wakeup interrupt flag
#define CANINTF_ERRIF  BIT(5) //Error interrupt flag
#define CANINTF_TX2IF  BIT(4) //Transmit buffer 2 empty interrupt flag
#define CANINTF_TX1IF  BIT(3) //Transmit buffer 1 empty interrupt flag
#define CANINTF_TX0IF  BIT(2) //Transmit buffer 0 empty interrupt flag
#define CANINTF_RX1IF  BIT(1) //Receive buffer 1 full interrupt flag
#define CANINTF_RX0IF  BIT(0) //Receive buffer 0 full interrupt flag

/* Bit timing registers */
#define CNF1  0x2A //Bit timing configuration 1
#define CNF2  0x29 //Bit timing configuration 2
#define CNF3  0x28 //Bit timing configuration 3

#define CNF1_SJW_SHF  6 //Synchronization jump width length
#define CNF1_BRP_SHF  0 //Baud rate prescaler

#define CNF2_BTLMODE     BIT(7) //PS2 bit time length
#define CNF2_SAM         BIT(6) //Sample point configuration
#define CNF2_PHSEG1_SHF  3      //PS1 length
#define CNF2_PRSEG_SHF   0      //Propagation segment length

#define CNF3_SOF         BIT(7) //Start of frame signal
#define CNF3_WAKFIL      BIT(6) //Wake up filter
#define CNF3_PHSEG2_SHF  0      //PS2 length

/* SPI Interface Instruction set */
#define CMD_RESET         0xC0 //Resets internal registers to default state
#define CMD_READ          0x03 //Read data from register
#define CMD_READ_RXB      0x90 //Read RX buffer
#define CMD_WRITE         0x02 //Write data to register
#define CMD_LOAD_TXB      0x40 //Load TX buffer
#define CMD_RTS           0x80 //Request to send
#define CMD_READ_STATUS   0xA0 //Read status
#define CMD_RX_STATUS     0xB0 //RX status
#define CMD_BIT_MODIFY    0x05 //Set or clear individual bits in a particular register

/*
 * Modes of Operation
 * Note: Do *NOT* change the order of those modes.
 */
enum op_mode {
	REQOP_NORMAL = 0,
	REQOP_SLEEP,
	REQOP_LOOP,
	REQOP_LISTEN,
	REQOP_CONFIG
};

/* SPI Command Functions */
void mcp2515_reset(void);
uint8_t mcp2515_read_reg(uint8_t reg);
void mcp2515_read_nregs(uint8_t reg, int count, uint8_t *buf);
void mcp2515_write_reg(uint8_t reg, uint8_t val);
void mcp2515_write_nregs(uint8_t reg, uint8_t *buf, int count);
void mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t val);
uint8_t mcp2515_read_status(void);
uint8_t mcp2515_rx_status(void);
void mcp2515_rts(uint8_t mask);
void mcp2515_load_txb(uint8_t *buf, uint8_t len, uint8_t idx, uint8_t flag);
void mcp2515_read_rxb(uint8_t *buf, uint8_t len, uint8_t idx, uint8_t flag);

/* MCP2515 functions */
void set_baudrate(int speed);
void set_mode(enum op_mode mode);
void transmit_frame(int txb_idx, struct can_frame *frame);
void receive_frame(int rxb_idx, struct can_frame *frame);

/****************************************************************************/
/* the set macro */
#define CAN_SET_VAL(REG_FIELD, val)    (((val) << CAN_##REG_FIELD##_SHIFT) & CAN_##REG_FIELD##_MASK)
/* the extract macro */
#define CAN_EXT_VAL(reg, REG_FIELD)    (((reg) & CAN_##REG_FIELD##_MASK) >> CAN_##REG_FIELD##_SHIFT)
/* the insert macro */
#define CAN_INST_VAL(reg, REG_FIELD, val)   ((reg) = ((reg) & ~CAN_##REG_FIELD##_MASK) | CAN_SET_VAL(REG_FIELD, val))


#define MAX_CAN_MSG_SIZE		14		//bytes
/*
 * Our boards use 20MHz clocks
 */
#define CAN_XTAL 					20000000

/*
 * Read Status SPI command
 * Status Bits
 */
#define CAN_CANINTF_RX0IF_SHIFT 			0
#define CAN_CANINTF_RX0IF_MASK 				(1<<CAN_CANINTF_RX0IF_SHIFT)

#define CAN_CANINTF_RX1IF_SHIFT 			1
#define CAN_CANINTF_RX1IF_MASK 				(1<<CAN_CANINTF_RX1IF_SHIFT)

#define CAN_TXB0CNTRL_TXREQ_SHIFT 			2
#define CAN_TXB0CNTRL_TXREQ_MASK 			(1<<CAN_TXB0CNTRL_TXREQ_SHIFT)

#define CAN_CANINTF_TX0IF_SHIFT				3
#define CAN_CANINTF_TX0IF_MASK				(1<<CAN_CANINTF_TX0IF_SHIFT)

#define CAN_TXB1CNTRL_TXREQ_SHIFT 			4
#define CAN_TXB1CNTRL_TXREQ_MASK 			(1<<CAN_TXB1CNTRL_TXREQ_SHIFT)

#define CAN_CANINTF_TX1IF_SHIFT				5
#define CAN_CANINTF_TX1IF_MASK 				(1<<CAN_CANINTF_TX1IF_SHIFT)

#define CAN_TXB2CNTRL_TXREQ_SHIFT			6
#define CAN_TXB2CNTRL_TXREQ_MASK			(1<<CAN_TXB2CNTRL_TXREQ_SHIFT)

#define CAN_CANINTF_TX2IF_SHIFT 			7
#define CAN_CANINTF_TX2IF_MASK 				(1<<CAN_CANINTF_TX2IF_SHIFT)

/*
 * Read RX status SPI command
 * RX Status bits
 */
#define CAN_RECEIVED_MSG_MASK				0xC0
#define CAN_RECEIVED_MSG_SHIFT				6
#define CAN_MSG_TYPE_MASK					0x18
#define CAN_MSG_TYPE_SHIFT					3
#define CAN_FILTER_MATCH_MASK				0x07
#define CAN_FILTER_MATCH_SHIFT				0

#define CANSTAT_FILTER_MASK 				7
#define CANSTAT_MSG_TYPE_MASK 				3
#define CANSTAT_MSG_TYPE_BIT  				3
#define CANSTAT_STD_FRAME 		 			0
#define CANSTAT_STD_REMOTE 					1
#define CANSTAT_EXT_FRAME  					2
#define CANSTAT_EXT_REMOTE 					3
#define CANSTAT_RXB0 						(1<<6)
#define CANSTAT_RXB1 						(1<<7)

/*
 * Local CAN registers copy
 */
typedef struct mcp2515_reg_t{
	volatile 	int can_lock;
	uint8_t		state;
	uint8_t		mode;
	uint8_t		txb0ctrl;
	uint8_t		txb1ctrl;
	uint8_t		txb2ctrl;
	uint8_t		rxb0ctrl;
	uint8_t		rxb1ctrl;
	/* CAN Control */
	uint8_t		canctrl;
	/* CAN Interrupt */
	uint8_t		caninte;
	uint8_t		canintf;
	uint8_t		bfpctrl;
	/* ERROR */
	uint8_t		tec;
	uint8_t		rec;
	uint8_t		eflg;
	/* The following registers are modifiable when in configuration mode */
	uint8_t		txrtsctrl;
	/* Bit Timing */
	uint8_t		cnf1;
	uint8_t		cnf2;
	uint8_t		cnf3;
	/* RXF0 */
	uint8_t		rxf0sidh;
	uint8_t		rxf0sidl;
	uint8_t		rxf0eid8;
	uint8_t		rxf0eid0;
	/* RXF1 */
	uint8_t		rxf1sidh;
	uint8_t		rxf1sidl;
	uint8_t		rxf1eid8;
	uint8_t		rxf1eid0;
	/* RXF2 */
	uint8_t		rxf2sidh;
	uint8_t		rxf2sidl;
	uint8_t		rxf2eid8;
	uint8_t		rxf2eid0;
	/* RXF3 */
	uint8_t		rxf3sidh;
	uint8_t		rxf3sidl;
	uint8_t		rxf3eid8;
	uint8_t		rxf3eid0;
	/* RXF4 */
	uint8_t		rxf4sidh;
	uint8_t		rxf4sidl;
	uint8_t		rxf4eid8;
	uint8_t		rxf4eid0;
	/* RXF5 */
	uint8_t		rxf5sidh;
	uint8_t		rxf5sidl;
	uint8_t		rxf5eid8;
	uint8_t		rxf5eid0;
	/* RXM0 */
	uint8_t		rxm0sidh;
	uint8_t		rxm0sidl;
	uint8_t		rxm0eid8;
	uint8_t		rxm0eid0;
	/* RXM1 */
	uint8_t		rxm1sidh;
	uint8_t		rxm1sidl;
	uint8_t		rxm1eid8;
	uint8_t		rxm1eid0;
}mcp2515_regs;

/* -----------------------------
 *     Register addresses
 *----------------------------*/
/* TXB */
#define TXBnCTRL(n) 	(0x30 + (n*0x10))			// Transmit buffer 0,1,2 control
/* Transmit buffer standard identifier high: 0x31, 0x41, 0x51 */
#define TXBnSIDH(n) 	(0x31 + 0x10*(n)) 			// bits [10:3]
/* Transmit buffer standard identifier low: 0x32, 0x42, 0x52 */
#define TXBnSIDL(n) 	(0x32 + 0x10*(n)) 			//(7:6)SID[2:0] - (1:0)EID[17:16]
#define TXBnEID8(n) 	(0x33 + 0x10*(n))			//Extended Identifier bits[15:8]
#define TXBnEID0(n) 	(0x34 + 0x10*(n))			//Extended Identifier bits[7:0]
#define TXBnDLC(n) 		(0x35 + 0x10*(n))			//Transmit buffer n data length code
#define TXBnDm(n, m) 	(0x36 + 0x10 * (n) + (m))	//Transmit buffer n data byte m: 0x36 - 3D, 0x46 - 4D, 0x56 - 5D
/* RXB */
#define RXB0CTRL 		0x60						//Receive buffer 0 control
#define RXBnSIDH(n) 	(0x61 + 0x10*(n))			//Receive Buffer n Standard Identifier High: 0x61, 0x71
#define RXBnSIDL(n) 	(0x62 + 0x10*(n))			//Receive buffer n Standard identifier Low: 0x62, 0x72
#define RXBnEID8(n)		(0x63 + 0x10*(n))			//RXBn Extended Identifier bits [15:8]
#define RXBnEID0(n)		(0x64 + 0x10*(n))			//RXBn Extended Identifier bits [7:0]
#define RXBnDLC(n)		(0x65 + 0x10*(n))			//Receive Buffer n Data Length Code
#define RXBnDM(n, m) 	(0x66 + (m) + 0x10*(n))		//Receive buffer n data byte m: 0x66 - 6D, 0x76 - 7D
#define RXB1CTRL 		0x70						//Receive buffer 1 control
/* Control Registers */
#define BFPCTRL 		0x0C		//RXnBF pin control and status
#define TXRTSCTRL 		0x0D		// TXnRTS Pin control and status
/* ERROR */
#define TEC				0x1C		//Transmit Error Counter
#define REC				0x1D		//Receiver Error Counter
#define EFLG			0x2D		//Error Flag

/* Message Acceptance Filters and Masks
 *
 * Association:
 * RXB0: RXF0, RXF1, RXM0
 * RXB1: RXF2,3,4, RXM1
*/
/* RXF0 */
#define RXF0SIDH		0x00
#define RXF0SIDL		0x01
#define RXF0EID8		0x02
#define RXF0EID0		0x03
/* RXF1 */
#define RXF1SIDH		0x04
#define RXF1SIDL		0x05
#define RXF1EID8		0x06
#define RXF1EID0		0x07
/* RXF2 */
#define RXF2SIDH		0x08
#define RXF2SIDL		0x09
#define RXF2EID8		0x0A
#define RXF2EID0		0x0B

/* RXF3 */
#define RXF3SIDH		0x10
#define RXF3SIDL		0x11
#define RXF3EID8		0x12
#define RXF3EID0		0x13
/* RXF4 */
#define RXF4SIDH		0x14
#define RXF4SIDL		0x15
#define RXF4EID8		0x16
#define RXF4EID0		0x17
/* RXF5 */
#define RXF5SIDH		0x18	/* RXFnSIDH: Filter n Standard Identifier High: SID[10:3]  */
#define RXF5SIDL		0x19	/* RXFnSIDL: Filter n Standard Identifier Low: (7:6)SID[2:0] - (1:0)EID[17:16] */
#define RXF5EID8		0x1A	/* RXFnEID8: Filter n Extended Identifier High: EID[15:8] */
#define RXF5EID0		0x1B	/* RXFnEID0: Filter n Extended Identifier Low: EID[7:0] */
/* RMX0 */
#define RXM0SIDH		0x20	/* RXMnSIDH: Mask n Standard Identifier High: SID[10:3] */
#define RXM0SIDL		0x21	/* RXMnSIDL: Mask n Standard Identifier Low: (7:6)SID[2:0] - (1:0)EID[17:16]  */
#define RXM0EID8		0x22	/* RXMnEID8: Mask n Extended Identifier High: EID[15:8] */
#define RXM0EID0		0x23	/* RXMnEID0: Filter n Extended Identifier Low: EID[7:0] */
/* RMX1 */
#define RXM1SIDH		0x24
#define RXM1SIDL		0x25
#define RXM1EID8		0x26
#define RXM1EID0		0x27

/* Configurations */
#define CNF1 			0x2A
#define CNF2 			0x29
#define CNF3 			0x28


/* -----------------------------
 *   BIT FIELDS DEFINITIONS
 *----------------------------*/


/*
 * CANCTRL: CAN Control Register
 */
#define CAN_REQOP_SHIFT						5
#define CAN_REQOP_MASK						0xE0

//Request Operation Mode bits
#define CAN_REQOP2_SHIFT					7
#define CAN_REQOP2_MASK						(1<<CAN_REQOP2_SHIFT)

#define CAN_REQOP1_SHIFT					6
#define CAN_REQOP1_MASK						(1<<CAN_REQOP1_SHIFT)

#define CAN_REQOP0_SHIFT					5
#define CAN_REQOP0_MASK						(1<<CAN_REQOP0_SHIFT)

//Abort All Pending Tranmissions
#define CAN_ABAT_SHIFT						4
#define CAN_ABAT_MASK						(1<<CAN_ABAT_SHIFT)

//One Shot Mode
#define CAN_OSM_SHIFT						3
#define CAN_OSM_MASK						(1<<CAN_OSM_SHIFT)

//CLKOUT Pin Enable bit
#define CAN_CLKEN_SHIFT						2
#define CAN_CLKEN_MASK						(1<<CAN_CLKEN_SHIFT)

//CLKOUT Pin Prescaler bits
#define CAN_CLKPRE_SHIFT					0
#define CAN_CLKPRE_MASK						0x3

#define CAN_CLKPRE1_SHIFT					1
#define CAN_CLKPRE1_MASK					(1<<CAN_CLKPRE1_SHIFT)

#define CAN_CLKPRE0_SHIFT					0
#define CAN_CLKPRE0_MASK					(1<<CAN_CLKPRE0_SHIFT)

/*
 * CANSTAT
 */
#define CAN_OPMOD_SHIFT						5
#define CAN_OPMOD_MASK						0xE0
#define CAN_ICOD_SHIFT						1
#define CAN_ICOD_MASK						0xE

#define OPMOD2								7
#define OPMOD1								6
#define OPMOD0								5
#define ICOD2								3
#define ICOD1								2
#define ICOD0								1

/*
 * CANINTE: CAN Interrupt Enable
 */
// Message Error Interrupt Enable
#define CAN_MERRE_SHIFT						7
#define CAN_MERRE_MASK						(1<<CAN_MERRE_SHIFT)
// Wakeup Interrupt Enable
#define CAN_WAKIE_SHIFT						6
#define CAN_WAKIE_MASK						(1<<CAN_WAKIE_SHIFT)
// Error Interrupt Enable (multiple sources in EFLG)
#define CAN_ERRIE_SHIFT						5
#define CAN_ERRIE_MASK						(1<<CAN_ERRIE_SHIFT)
// TXB2 Empty Interrupt Enable
#define CAN_TX2IE_SHIFT						4
#define CAN_TX2IE_MASK						(1<<CAN_TX2IE_SHIFT)
// TXB1 Empty Interrupt Enable
#define CAN_TX1IE_SHIFT						3
#define CAN_TX1IE_MASK						(1<<CAN_TX1IE_SHIFT)
// TXB0 Empty Interrupt Enable
#define CAN_TX0IE_SHIFT						2
#define CAN_TX0IE_MASK						(1<<CAN_TX0IE_SHIFT)
// RXB1 Full Interrupt Enable
#define CAN_RX1IE_SHIFT						1
#define CAN_RX1IE_MASK						(1<<CAN_RX1IE_SHIFT)
// RXB0 Full Interrupt Enable
#define CAN_RX0IE_SHIFT						0
#define CAN_RX0IE_MASK						(1<<CAN_RX0IE_SHIFT)


/*
 * CANINTF: CAN Interrupt Flags
 */
#define CAN_RX0IF_SHIFT 					0
#define CAN_RX0IF_MASK 						(1<<CAN_RX0IF_SHIFT)
#define CAN_RX1IF_SHIFT 					1
#define CAN_RX1IF_MASK 						(1<<CAN_RX1IF_SHIFT)
#define CAN_TX0IF_SHIFT 					2
#define CAN_TX0IF_MASK 						(1<<CAN_TX0IF_SHIFT)
#define CAN_TX1IF_SHIFT 					3
#define CAN_TX1IF_MASK 						(1<<CAN_TX1IF_SHIFT)
#define CAN_TX2IF_SHIFT 					4
#define CAN_TX2IF_MASK 						(1<<CAN_TX2IF_SHIFT)
#define CAN_ERRIF_SHIFT 					5
#define CAN_ERRIF_MASK 						(1<<CAN_ERRIF_SHIFT)
#define CAN_WAKIF_SHIFT 					6
#define CAN_WAKIF_MASK 						(1<<CAN_WAKIF_SHIFT)
#define CAN_MERRF_SHIFT 					7
#define CAN_MERRF_MASK 						(1<<CAN_MERRF_SHIFT)

/*
 * EFLG: Error Flag
 */
//RXB1 Overflow, set when a valid message is receive in RXB1 and CANINTF.RX1IF = 1, must be reset by MCU
#define CAN_RX1OVR_SHIFT					7
#define CAN_RX1OVR_MASK						(1<<CAN_RX1OVR_SHIFT)
//RXB0 Overflow, set when a valid message is receive in RXB0 and CANINTF.RX0IF = 1, must be reset by MCU
#define CAN_RX0OVR_SHIFT					6
#define CAN_RX0OVR_MASK						(1<<CAN_RX0OVR_SHIFT)
//Bus-off Error, set when TEC > 255, reset after a successful bus recovery sequence
#define CAN_TXB0_SHIFT						5
#define CAN_TXB0_MASK						(1<<CAN_TXB0_SHIFT)
//Transmit Error-Passive, set when TEC >= 128, reset when less
#define CAN_TXEP_SHIFT						4
#define CAN_TXEP_MASK						(1<<CAN_TXEP_SHIFT)
//Receive Error-Passive, set when REC >= 128, reset when less
#define CAN_RXEP_SHIFT						3
#define CAN_RXEP_MASK						(1<<CAN_RXEP_SHIFT)
//Transmit Error Warning, set when TEC >=96, reset when less
#define CAN_TXWAR_SHIFT						2
#define CAN_TXWAR_MASK						(1<<CAN_TXWAR_SHIFT)
//Receive Error Warning, set when REC >= 96, reset when less
#define CAN_RXWAR_SHIFT						1
#define CAN_RXWAR_MASK						(1<<CAN_RXWAR_SHIFT)
//Error Warning, set when TEC/REC >= 96 (TXWAR/RXWARE = 1), reset when both < 96.
#define CAN_EWARN_SHIFT						0
#define CAN_EWARN_MASK						(1 << CAN_EWARN_SHIFT)


/* TXBnCTRL(n):Transmit buffer 0,1,2 control */
#define CAN_TXP0_MASK						(1<<0)
#define CAN_TXP1_MASK	 					(1<<1)
//Transmit Buffer Priority, TXB_PRIORITY
#define CAN_TXP_MASK			 			0x03
#define CAN_TXP_SHIFT						0
//Buffer is currently pending transmission - MCU sets this bit to request msg to be sent.
#define CAN_TXREQ_SHIFT	 					3
#define CAN_TXREQ_MASK	 					(1<<CAN_TXREQ_SHIFT)
//Bus errors occurred while the msg was being transmitted
#define CAN_TXERR_MASK 						(1<<4)
//Message Lost Arbitration bit
#define CAN_MLOA_MASK 						(1<<5)
//Message Aborted Flag bit
#define CAN_ABTF_MASK 						(1<<6)

/* TXRTSCTRL:TXnRTS Pin control and status */
//#TX0RTS pin mode bit: 0 - Digital Input; 1 - RTS (falling edge)
#define CAN_B0RTSM_SHIFT					0
#define CAN_B0RTSM_MASK		 				(1<<CAN_B0RTSM_SHIFT)
//#TX1RTS pin mode bit: 0 - Digital Input; 1 - RTS (falling edge)
#define CAN_B1RTSM_SHIFT 					1
#define CAN_B1RTSM_MASK 					(1<<CAN_B1RTSM_SHIFT)

//#TX2RTS pin mode bit: 0 - Digital Input; 1 - RTS (falling edge)
#define CAN_B2RTSM_SHIFT 					2
#define CAN_B2RTSM_MASK 					(1<<CAN_B2RTSM_SHIFT)
//#TX0RTS pin state bit
#define CAN_B0RTS_SHIFT						3
#define CAN_B0RTS_MASK 						(1<<CAN_B0RTS_SHIFT)
//#TX1RTS pin state bit
#define CAN_B1RTS_SHIFT						4
#define CAN_B1RTS_MASK						(1<<CAN_B1RTS_SHIFT)
//#TX2RTS pin state bit
#define CAN_B2RTS_SHIFT						5
#define CAN_B2RTS_MASK						(1<<CAN_B2RTS_SHIFT)



/* RXB0CTRL: Receive buffer 0 control */
// Filter Hit bit: 1-RXF1, 0-RXF0, if rollover from RXB0 to RXB1, bit will reflect the filter that accept the message that rolled over
#define CAN_RXB0_FILHIT_SHIFT				0
#define CAN_RXB0_FILHIT_MASK 				(1<<CAN_RXB0_FILHIT_SHIFT)

//Read-only copy of BUKT bit (used internally by MCP2515)
#define CAN_RXB0_BUKT1_SHIFT				1
#define CAN_RXB0_BUKT1_MASK 				(1<<CAN_RXB0_BUKT1_SHIFT)
//Rollover Enable bit: 1-RXB0 message will roll over to RXB1 if RXB0 is full
#define CAN_RXB0_BUKT_SHIFT					2
#define CAN_RXB0_BUKT_MASK  				(1<<CAN_RXB0_BUKT_SHIFT)


/* RXB1CTRL: Receive buffer 1 control */
//Filter Hit bits: RXB_FILTER_HIT
#define CAN_RXB1_FILHIT_MASK				0x07
#define CAN_RXB1_FILHIT_SHIFT				0

/* Receiver buffer 0 & 1 control */
//Received Remote Transfer Request bit
#define CAN_RXRTR_SHIFT						3
#define CAN_RXRTR_MASK  					(1<<CAN_RXRTR_SHIFT)

//Receive Buffer Operating Mode bits: RXB_opMode
#define CAN_RXM_SHIFT						5
#define CAN_RXM_MASK						(0x60)

#define CAN_RXM0_SHIFT						5
#define CAN_RXM0_MASK 						(1<<CAN_RXM0_SHIFT)
#define CAN_RXM1_SHIFT						6
#define CAN_RXM1_MASK  						(1<<CAN_RXM1_SHIFT)


/* BFPCTRL: RXnBF pin control and status  */
// #RX0BF/#RX1BF pin opMode bit: 1 - RXBn Interrupts, 0 - Digital Output
#define CAN_B0BFM_SHIFT 					0
#define CAN_B1BFM_SHIFT 					1
#define CAN_B0BFM_MASK 						(1<<CAN_B0BFM_SHIFT)
#define CAN_B1BFM_MASK 						(1<<CAN_B1BFM_SHIFT)

// #RX0BF/#RX1BF pin Function Enable bit
#define CAN_B0BFE_SHIFT 					2
#define CAN_B1BFE_SHIFT						3
#define CAN_B0BFE_MASK 						(1<<CAN_B0BFE_SHIFT)
#define CAN_B1BFE_MASK 						(1<<CAN_B1BFE_SHIFT)

// #RX0BF/#RX1BF pin State bit
#define CAN_B0BFS_MASK 						(1<<4)
#define CAN_B1BFS_MASK						(1<<5)

/* Standard Identifier [10:3] */
#define CAN_SIDH8_MASK			 			0xFF
#define CAN_SIDH8_SHIFT						0
/* Standard Identifier [2:0] */
#define CAN_SIDL3_MASK						0xE0
#define CAN_SIDL3_SHIFT						5
//Standard Frame Remote Transmit Request bit (valid if IDE bit = 0)
#define CAN_SRR_SHIFT						4
#define CAN_SRR_MASK						(1<<CAN_SRR_SHIFT)
//IDE: Extended Identifier Flag bit: 1 - Exd Frame, 0 - Std Frame
#define CAN_IDE_SHIFT						3
#define CAN_IDE_MASK						(1<<CAN_IDE_SHIFT)

// Extended Identifier Enable bit: 1 = Message will transmit extended id
#define CAN_EXIDE_SHIFT						3
#define CAN_EXIDE_MASK						(1<<CAN_EXIDE_SHIFT)

// Extended Identifier
#define CAN_EID1716_MASK					0x03			//0b11: EID[17:16]
#define CAN_EID1716_SHIFT			   		0

#define CAN_EID8_MASK						0xFF
#define CAN_EID0_MASK						0xFF
#define CAN_EID8_SHIFT						0
#define CAN_EID0_SHIFT						0
// Extended Identifier
#define CAN_RTR_SHIFT						6
#define CAN_RTR_MASK 						(1<<CAN_RTR_SHIFT)
//Data Length Code[3:0]
#define CAN_DLC_MASK					 	0x0F
#define CAN_DLC_SHIFT						0


/* CN1: Configuration 1 */
// Synchronization Jump Width Length Bits [1:0]
#define CAN_SJW_SHIFT 						6
#define CAN_SJW_MASK					 	0xC8

//Baud Rate Prescaler bits [5:0]
#define CAN_BRP_SHIFT						0
#define CAN_BRP_MASK					 	0x1F

/* CN2: Configuration 2 */
//PS2 Bit Time Length bit
#define CAN_BTLMODE_SHIFT					7
#define CAN_BTLMODE_MASK 					(1<<CAN_BTLMODE_SHIFT)

//Sample Point Configuration bit
#define CAN_SAM_SHIFT						6
#define CAN_SAM_MASK 						(1<<CAN_SAM_SHIFT)

//PS1 Length bits [2:0]
#define CAN_PHSEG1_MASK						0x38
#define CAN_PHSEG1_SHIFT					3

//Propagation Segment Length bits [2:0]
#define CAN_PRSEG_MASK				 		0x07
#define CAN_PRSEG_SHIFT						0

/* CN3: Configuration 3 */
//Start-of-Frame signal bit
#define CAN_SOF_SHIFT						7
#define CAN_SOF_MASK						(1<<CAN_SOF_SHIFT)

//Wake-up Filter bit
#define CAN_WAKFIL_SHIFT					6
#define CAN_WAKFIL_MASK						(1<<CAN_WAKFIL_SHIFT)
//PS2 Length bits [2:0]
#define CAN_PHSEG2_MASK				 		0x07
#define CAN_PHSEG2_SHIFT					0

/*
 * Minimum TQs in 1 CAN bit time
 * NumTQ_min = SyncFixed(1) + Propsync(1) + PS1(2) + PS2(3), PS2 > SJW >= 1 && PS2min = IPT = 2, thus PS2 >= 3
 */
#define CAN_MIN_TQ							7
#define CAN_MAX_TQ							25
#define CAN_MAX_BRP         				0x1F    //Maximum baud rate prescaler clock
#define CAN_MAX_SJW         				0x04    //4 = Maximum Synchronization Jump Width.
#define CAN_MAX_SAMPLE_POINT				90  //maximum sampling point percentage

/*
 * Function Definitions
 */
void _mcp2515_reset(void);
uint8_t _mcp2515_read_reg(int reg, int rcount);
void _mcp2515_write_reg(int r, unsigned char v);
int _mcp2515_write_mult_regs(uint8_t reg, uint8_t * buf, int byte_len);
void _mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data);
int _mcp2515_read_status(void);
int _mcp2515_rx_status(void);
void _mcp2515_rts(int txb_id);
int mcp2515_check_free_buffer(void);
void _mcp2515_load_tx_buffer(int txb_id, CAN_msg * message);
int _mcp2515_read_rx_buffer(uint8_t rxb_id, CAN_msg * message, uint8_t rx_status);
void _mcp2515_set_opMode(REQOP_MODE mode);
int _mcp2515_get_opMode(void);
int mcp2515_check_Int(void);
int mcp2515_recovery(void);
void _mcp2515_local_init(void);


#endif
