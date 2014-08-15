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
 * exynos_uart.h
 *  Created on: Jun 2, 2013
 *      Author: Jiawei Xie
 */

#ifndef EXYNOS_UART_H_
#define EXYNOS_UART_H_

#define UART0		0
#define UART1		1
#define UART2		2
#define UART3		3
#define NUM_UART	4

#define TX_WRITE_DELAY_US	8	// a dummy delay for successive write to tx fifo

/*
 * SCLK_UART, subject to change
 */
#define SCLK_UART	24000000UL

/*
 * UART Register Map
 */
typedef struct{
	volatile uint32_t ULCON;      	// 0x0000 /* line control */
	volatile uint32_t UCON;			// 0x0004 /* control */
	volatile uint32_t UFCON ;    	// 0x0008 /* fifo control */
	volatile uint32_t UMCON ;     	// 0x000C /* modem control */
	volatile uint32_t UTRSTAT;    	// 0x0010 /* TX/RX status */
	volatile uint32_t UERSTAT ;   	// 0x0014 /* RX error status */
	volatile uint32_t UFSTAT ;    	// 0x0018 /* FIFO status */
	volatile uint32_t UMSTAT ;    	// 0x001C /* modem status */
	volatile uint32_t UTXH;		  	// 0x0020 /* TX buffer */
	volatile uint32_t URXH;			// 0x0024 /* RX buffer */
	volatile uint32_t UBRDIV; 		// 0x0028 /* baud rate divisor */
	volatile uint32_t UFRACVAL;		// 0x002C /* divisor fractional value */
	volatile uint32_t UINTP;		// 0x0030 /* interrupt pending */
	volatile uint32_t UINTSP;		// 0x0034 /* interrupt source pending */
	volatile uint32_t UINTM;		// 0x0038 /* interrupt mask */
}exynos_uart, *exynos_uart_p;

/* Circular buffer */
#define MAX_BUFFER_SIZE		1024				//1 KB circular buffer
typedef struct {
	char 		data[MAX_BUFFER_SIZE];		//rx circular buffer
	int			valids;						//size of the valid data in buffer
	int			head;						//head index, update upon read
	int			tail;						//tail index, update upon write
	volatile 	bool		lock;						//lock to the buffer
}circ_buf;

/*
 * A structure that holds the device information and client buffer
 */
typedef struct {
	exynos_uart_p 	portaddr;	//port address
	circ_buf 		rxbuf;		//rx circular buffer
	circ_buf 		txbuf;		//tx circular buffer
	uint32_t		fifo_depth;
	uint32_t 		rx_triglvl;
	uint32_t		tx_triglvl;
	char * 		client_buf;
}uart_dev, uart_dev_p;


typedef enum Uart_BaudRate {
    Uart_BaudRate_2_4K      = 2400u,     /**< Baudrate 2400    bps   */
    Uart_BaudRate_4_8K      = 4800u,     /**< Baudrate 4800    bps   */
    Uart_BaudRate_9_6K      = 9600u,     /**< Baudrate 9600    bps   */
    Uart_BaudRate_19_2K     = 19200u,    /**< Baudrate 19200   bps   */
    Uart_BaudRate_38_4K     = 38400u,    /**< Baudrate 38400   bps   */
    Uart_BaudRate_57_6K     = 57600u,    /**< Baudrate 57600   bps   */
    Uart_BaudRate_115_2K    = 115200u    /**< Baudrate 115200  bps   */
}Uart_BaudRate;

typedef enum Uart_opMode{
	UART_MODE = 0,
	INFRARED_MODE = 1
}Uart_opMode;

typedef enum Uart_Parity{
	NO_PARITY	= 0,
	ODD			= 4,
	EVEN 		= 5,
	FORCED_CK_1	= 6,
	FORCED_CK_0 = 7
}Uart_Parity;

typedef enum Uart_StopBit{
	ONE_STOP_BIT	= 0,
	TWO_STOP_BIT	= 1
}Uart_StopBit;

typedef enum Uart_Word{
	BIT5 = 0,
	BIT6 = 1,
	BIT7 = 2,
	BIT8 = 3
}Uart_Word;


/* macro */
#define UART_EXT_VAL(reg, REG_FIELD)    (((reg) & UART_##REG_FIELD##_MASK) >> UART_##REG_FIELD##_SHIFT)
#define UART_SET_VAL(REG_FIELD, val)	(((val) << UART_##REG_FIELD##_SHIFT) & UART_##REG_FIELD##_MASK)
#define UART_INST_VAL(reg, REG_FIELD, val)	((reg) = ((reg) & ~ UART_##REG_FIELD##_MASK) | UART_SET_VAL(REG_FIELD, val))




/*
 * UART FIFO Depth
 */
#define UART0_FIFO_DEPTH			256		//bytes
#define UART1_FIFO_DEPTH			64
#define UART2_FIFO_DEPTH			16
#define UART3_FIFO_DEPTH			16
#define ISP_UART_FIFO_DEPTH			64


/*
 * Error Conditions
 */
#define ERR_OVERRUN		-1		//new data has overwritten the old
#define ERR_PARITY		-2		//detected an unexpected parity condition
#define ERR_FRAME		-3		//received data doesn't have a valid stop bit
#define ERR_BREAK		-4		//RxDn input is held in the logic 0 state for more than one frame transmission time

/*
 * Bit Field Definitions
 */

/* ULCONn */
#define UART_ULCON_INFRARED_MODE_SHIFT			6
#define UART_ULCON_INFRARED_MODE_MASK			(1 << UART_ULCON_INFRARED_MODE_SHIFT)

#define UART_ULCON_PARITY_MODE_SHIFT			3
#define UART_ULCON_PARITY_MODE_MASK				(0x38)



#define UART_ULCON_STOP_BIT_SHIFT				2
#define UART_ULCON_STOP_BIT_MASK				(1 << UART_ULCON_STOP_BIT_SHIFT)



#define UART_ULCON_WORD_LENGTH_SHIFT			0
#define UART_ULCON_WORD_LENGTH_MASK				(0x3)


/* UCONn */

#define UART_UCON_TX_DMA_BURST_SIZE_SHIFT			20
#define UART_UCON_TX_DMA_BURST_SIZE_MASK			(0x700000)
#define UART_UCON_RX_DMA_BURST_SIZE_SHIFT			16
#define UART_UCON_RX_DMA_BURST_SIZE_MASK			(0x70000)

/* Rx interrupt occurs when no data is received during 8x(N+1) frame time. Default N = 3 */
#define UART_UCON_RX_TIMEOUT_INTERVAL_SHIFT			12
#define UART_UCON_RX_TIMEOUT_INTERVAL_MASK			(0xF000)

/* Valid only UCONn[7] = 1. Enables RX time-out feature when RX FIFO counter = 0 */
#define UART_UCON_RX_TIMEOUT_EMPTY_FIFO_SHIFT		11
#define UART_UCON_RX_TIMEOUT_EMPTY_FIFO_MASK		(1 << UART_UCON_RX_TIMEOUT_EMPTY_FIFO_SHIFT)

#define UART_UCON_RX_DMA_SUSPEND_SHIFT				10
#define UART_UCON_RX_DMA_SUSPEND_MASK				(1 << UART_UCON_RX_DMA_SUSPEND_SHIFT)

#define UART_UCON_TX_INT_TYPE_SHIFT					9
#define UART_UCON_TX_INT_TYPE_MASK					(1 << UART_UCON_TX_INT_TYPE_SHIFT)
#define UART_UCON_RX_INT_TYPE_SHIFT					8
#define UART_UCON_RX_INT_TYPE_MASK					(1 << UART_UCON_RX_INT_TYPE_SHIFT)

enum int_type_t{
	LEVEL	= 1	//level-trigger interrupt controller
};

#define UART_UCON_RX_TIMEOUT_SHIFT				7
#define UART_UCON_RX_TIMEOUT_MASK				(1 << UART_UCON_RX_TIMEOUT_SHIFT)

#define UART_UCON_RX_ERR_INT_SHIFT				6
#define UART_UCON_RX_ERR_INT_MASK				(1 << UART_UCON_RX_ERR_INT_SHIFT)

#define UART_UCON_LOOP_BACK_MODE_SHIFT			5
#define UART_UCON_LOOP_BACK_MODE_MASK			(1 << UART_UCON_LOOP_BACK_MODE_SHIFT)

#define UART_UCON_SEND_BRK_SIGNAL_SHIFT		4
#define UART_UCON_SEND_BRK_SIGNAL_MASK			(1 << UART_UCON_SEND_BRK_SIGNAL_SHIFT)

#define UART_UCON_TX_MODE_SHIFT					2
#define UART_UCON_TX_MODE_MASK					(0xC)
#define UART_UCON_RX_MODE_SHIFT					0
#define UART_UCON_RX_MODE_MASK					(0x3)

enum mode_t{
	DISABLES 		= 0,
	INT_POLL_MODE 	= 1,
	DMA_MODE		= 2
};

/* UFCONn */
/*
 * Tx FIFO Trigger Level
 * --------------------------------------------------------
 * 	LEVEL	[Channel 0]		[Channel 1]		[Channel 2,3]
 * 	--------------------------------------------------------
 * 		0		0				0				0
 *		1		32				8				2
 *		2		64				16				4
 *		3		96				24				6
 *		4		128				32				8
 *		5		160				40				10
 *		6		192				48				12
 *		7		224				56				14
 *
 *	Rx FIFO Trigger Level
 * --------------------------------------------------------
 * 	LEVEL	[Channel 0]		[Channel 1]		[Channel 2,3]
 * 	--------------------------------------------------------
 * 		0		32				8				2
 *		1		64				16				4
 *		2		96				24				6
 *		3		128				32				8
 *		4		160				40				10
 *		5		192				48				12
 *		6		224				56				14
 *		7		256				64				16
 */
enum trigger_level_t{
	LVL0	= 0,
	LVL1	= 1,
	LVL2	= 2,
	LVL3	= 3,
	LVL4	= 4,
	LVL5	= 5,
	LVL6	= 6,
	LVL7	= 7
};

#define UART_UFCON_TX_FIFO_LVL_SHIFT			8
#define UART_UFCON_TX_FIFO_LVL_MASK				(0x700)
#define UART_UFCON_RX_FIFO_LVL_SHIFT			4
#define UART_UFCON_RX_FIFO_LVL_MASK				(0x70)


#define UART_UFCON_TX_FIFO_RESET_SHIFT			2
#define UART_UFCON_TX_FIFO_RESET_MASK			(1 << UART_UFCON_TX_FIFO_RESET_SHIFT)
#define UART_UFCON_RX_FIFO_RESET_SHIFT			1
#define UART_UFCON_RX_FIFO_RESET_MASK			(1 << UART_UFCON_RX_FIFO_RESET_SHIFT)
#define UART_UFCON_FIFO_ENABLE_SHIFT			0
#define UART_UFCON_FIFO_ENABLE_MASK				(1 << UART_UFCON_FIFO_ENABLE_SHIFT)

#define UART_UFCON_RTS_LVL_SHIFT				5
#define UART_UFCON_RTS_LVL_MASK					(0xE0)
#define UART_UFCON_AFC_SHIFT					4
#define UART_UFCON_AFC_MASK						(1 << UART_UFCON_AFC_SHIFT)
#define UART_UFCON_MODEM_INT_SHIFT				3
#define UART_UFCON_MODEM_INT_MASK				(1 << UART_UFCON_MODEM_INT_SHIFT)
#define UART_UFCON_RTS_SHIFT					0
#define UART_UFCON_RTS_MASK						(1 << UART_UFCON_RTS_SHIFT)

/* UTRSTATn */
#define UART_UTRSTAT_RX_FIFO_CNT_TIMEOUT_SHIFT	16
#define UART_UTRSTAT_RX_FIFO_CNT_TIMEOUT_MASK	(0xFF0000)

#define UART_UTRSTAT_TX_DMA_STS_SHIFT			12
#define UART_UTRSTAT_TX_DMA_STS_MASK			(0xE000)

#define UART_UTRSTAT_RX_DMA_STS_SHIFT			8
#define UART_UTRSTAT_RX_DMA_STS_MASK			(0xF00)

#define UART_UTRSTAT_RX_TIMEOUT_STS_SHIFT		3
#define UART_UTRSTAT_RX_TIMEOUT_STS_MASK		(1 << UART_UTRSTAT_RX_TIME_OUT_STS_SHIFT)

#define UART_UTRSTAT_TX_EMPTY_SHIFT				2
#define UART_UTRSTAT_TX_EMPTY_MASK				(1 << UART_UTRSTAT_TX_EMPTY_SHIFT)
#define UART_UTRSTAT_TX_BUF_EMPTY_SHIFT			1
#define UART_UTRSTAT_TX_BUF_EMPTY_MASK			(1 << UART_UTRSTAT_TX_BUF_EMPTY_SHIFT)

#define UART_UTRSTAT_RX_DATA_READY_SHIFT		0
#define UART_UTRSTAT_RX_DATA_READY_MASK			(1 << UART_UTRSTAT_RX_DATA_READY_SHIFT)

/* UERSTATn */
#define UART_UERSTAT_BRK_DETECT_SHIFT			3
#define UART_UERSTAT_BRK_DETECT_MASK			(1 << UART_UERSTAT_BRK_DETECT_SHIFT)
#define UART_UERSTAT_FRAME_ERROR_SHIFT			2
#define UART_UERSTAT_FRAME_ERROR_MASK			(1 << UART_UERSTAT_FRAME_ERROR_SHIFT)
#define UART_UERSTAT_PARITY_ERROR_SHIFT			1
#define UART_UERSTAT_PARITY_ERROR_MASK			(1 << UART_UERSTAT_PARITY_ERROR_SHIFT)
#define UART_UERSTAT_OVERRUN_ERROR_SHIFT		0
#define UART_UERSTAT_OVERRUN_ERROR_MASK			(1 << UART_UERSTAT_OVERRUN_ERROR_SHIFT)

/* UFSTATn */
#define UART_UFSTAT_TX_FIFO_FULL_SHIFT			24
#define UART_UFSTAT_TX_FIFO_FULL_MASK			(1 << UART_UFSTAT_TX_FIFO_FULL_SHIFT)

#define UART_UFSTAT_TX_FIFO_CNT_SHIFT			16
#define UART_UFSTAT_TX_FIFO_CNT_MASK			(0xFF0000)

#define UART_UFSTAT_RX_FIFO_ERROR_SHIFT			9
#define UART_UFSTAT_RX_FIFO_ERROR_MASK			(1 << UART_UFSTAT_RX_FIFO_ERROR_SHIFT)

#define UART_UFSTAT_RX_FIFO_FULL_SHIFT			8
#define UART_UFSTAT_RX_FIFO_FULL_MASK			(1 << UART_UFSTAT_RX_FIFO_FULL_SHIFT)

#define UART_UFSTAT_RX_FIFO_CNT_SHIFT			0
#define UART_UFSTAT_RX_FIFO_CNT_MASK			(0xFF)

/* UMSTATn */
#define UART_UMSTAT_DELTA_CTS_SHIFT				4
#define UART_UMSTAT_DELTA_CTS_MASK				(1 << UART_UMSTAT_DELTA_CTS_SHIFT)
#define UART_UMSTAT_CTS_SHIFT					0
#define UART_UMSTAT_CTS_MASK					(1 << UART_UMSTAT_CTS_SHIFT)

/* UTXHn - Transmit Data for UARTn */
#define UART_UTXH_SHIFT							0
#define UART_UTXH_MASK							(0xFF)

/* URXHn - Receive Data for UARTn */
#define UART_URXH_SHIFT							0
#define UART_URXH_MASK							(0xFF)

/* UBRDIVn */
#define UART_UBRDIV_SHIFT						0
#define UART_UBRDIV_MASK						(0xFFFF)

/* UFRACVALn */
#define UART_UFRACVAL_SHIFT						0
#define UART_UFRACVAL_MASK						(0xF)

/* UINTPn */
#define UART_UINTP_MODEM_SHIFT					3
#define UART_UINTP_MODEM_MASK					(1 << UART_UINTP_MODEM_SHIFT)
#define UART_UINTP_TXD_SHIFT					2
#define UART_UINTP_TXD_MASK					(1 << UART_UINTP_TXD_SHIFT)
#define UART_UINTP_ERROR_SHIFT					1
#define UART_UINTP_ERROR_MASK					(1 << UART_UINTP_ERROR_SHIFT)
#define UART_UINTP_RXD_SHIFT					0
#define UART_UINTP_RXD_MASK					(1 << UART_UINTP_RXD_SHIFT)

/* UINTSn */
#define UART_UINTS_MODEM_SHIFT					3
#define UART_UINTS_MODEM_MASK					(1 << UART_UINTS_MODEM_SHIFT)
#define UART_UINTS_TXD_SHIFT					2
#define UART_UINTS_TXD_MASK						(1 << UART_UINTS_TXD_SHIFT)
#define UART_UINTS_ERROR_SHIFT					1
#define UART_UINTS_ERROR_MASK					(1 << UART_UINTS_ERROR_SHIFT)
#define UART_UINTS_RXD_SHIFT					0
#define UART_UINTS_RXD_MASK						(1 << UART_UINTS_RXD_SHIFT)

/* UINTMn */
#define UART_UINTM_MODEM_SHIFT					3
#define UART_UINTM_MODEM_MASK					(1 << UART_UINTM_MODEM_SHIFT)
#define UART_UINTM_TXD_SHIFT					2
#define UART_UINTM_TXD_MASK						(1 << UART_UINTM_TXD_SHIFT)
#define UART_UINTM_ERROR_SHIFT					1
#define UART_UINTM_ERROR_MASK					(1 << UART_UINTM_ERROR_SHIFT)
#define UART_UINTM_RXD_SHIFT					0
#define UART_UINTM_RXD_MASK						(1 << UART_UINTM_RXD_SHIFT)



#endif /* EXYNOS_UART_H_ */



