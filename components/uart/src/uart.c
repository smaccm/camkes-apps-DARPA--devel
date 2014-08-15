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
 * UART BASE ADDRESSES
#define UART0_PHY	0x12C00000
#define UART1_PHY	0x12C10000
#define UART2_PHY	0x12C20000	//reserve for 'printf' debugging, don't use
#define UART3_PHY	0x12C30000
#define ISP_UART_PHY	0x13190000
 */

/*
 * uart.c
 *
 *  Created on: Jun 2, 2013
 *      Author: Jiawei Xie
 */
#if 0
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "uart.h"

#include "exynos_uart.h"
#include "common.h"
#include "utils.h"

#define uart0 (exynos_uart_p)uart0base
#define uart1 (exynos_uart_p)uart1base
#define uart2 (exynos_uart_p)uart2base
#define uart3 (exynos_uart_p)uart3base

static uart_dev uart_dev_table[NUM_UART];

static void print_reg(int uart_num)
{
	exynos_uart_p uart = uart_dev_table[uart_num].portaddr;

	printf("ULCON%d: %x\n", uart_num, uart->ULCON);
	printf("UCON%d: %x\n", uart_num, uart->ULCON);
	printf("UFCON%d: %x\n", uart_num, uart->UFCON);
	printf("UMCON%d: %x\n", uart_num, uart->UMCON);
	printf("UTRSTAT%d: %x\n", uart_num, uart->UTRSTAT);
	printf("UERSTAT%d: %x\n", uart_num, uart->UERSTAT);
	printf("UFSTAT%d: %x\n", uart_num, uart->UFSTAT);
	printf("UMSTAT%d: %x\n", uart_num, uart->UMSTAT);
	printf("UTXH%d: %x\n", uart_num, uart->UTXH);
	printf("URXH%d: %x\n", uart_num, uart->URXH);
	printf("UBRDIV%d: %x\n", uart_num, uart->UBRDIV);
	printf("UFRACVAL%d: %x\n", uart_num, uart->UFRACVAL);
	printf("UINTP%d: %x\n", uart_num, uart->UINTP);
	printf("UINTS%d: %x\n", uart_num, uart->UINTSP);
	printf("UINTM%d: %x\n", uart_num, uart->UINTM);
}

static void uart_setbaudrate(exynos_uart_p uart, Uart_BaudRate baud)
{
	/*
	 * The value stored in the Baud Rate Divisor (UBRDIVn) and Divisor Fractional value (UFRACVALn) is
	 * used to determine the serial Tx/Rx clock rate (baud rate) as:
	 * DIV_VAL = UBRDIVn + UFRACVALn/16
	 * Or
	 * DIV_VAL = (SCLK_UART/(bps x 16)) – 1
	 * Where, the divisor should be from 1 to (216 – 1).
	 * Using UFRACVALn, you can generate the Baud Rate more accurately.
	 *
	 */
	unsigned long div_val,sclk = 0;

	sclk = SCLK_UART;

	div_val = (sclk/ ((unsigned long) baud));

	(uart->UBRDIV) = 0x21; //(((uint32_t)div_val/16) - 1) & 0xffff;
	(uart->UFRACVAL) = 0xb; //((uint32_t)div_val % 16 ) & 0xf;

	printf("baudrate = %d, UBRDIV = %d\n", baud, (uart->UBRDIV));
}

static void uart_reset_fifo(exynos_uart_p uart)
{
	//reset FIFOs, auto clear
	UART_INST_VAL(uart->UFCON, UFCON_TX_FIFO_RESET, ENABLE);
	UART_INST_VAL(uart->UFCON, UFCON_RX_FIFO_RESET, ENABLE);

	//wait until it reset
	while(UART_EXT_VAL(uart->UFCON, UFCON_TX_FIFO_RESET));
	while(UART_EXT_VAL(uart->UFCON, UFCON_RX_FIFO_RESET));
}

/*
 * Check the tx pending interrupt
 */
static int _isTx_pending(exynos_uart_p uart)
{
	int val = 0;
	printf("%d\n", __LINE__);
	val = UART_EXT_VAL(uart->UINTP, UINTP_TXD);
	printf("%d, %d\n", __LINE__, val);
	return val;
}

/*
 * Check the rx pending interrupt
 */
static int _isRx_pending(exynos_uart_p uart)
{
	int val = 0;
	printf("%d\n", __LINE__);
	val = UART_EXT_VAL(uart->UINTP, UINTP_RXD);
	printf("%d, %d\n", __LINE__, val);
	return val;
}

/*
 * clear the tx pending interrupt
 */
static int _clearTx_pending(exynos_uart_p uart)
{
	return UART_INST_VAL(uart->UINTP, UINTP_TXD, 1);
}

/*
 * clear the rx pending interrupt
 */
static int _clearRx_pending(exynos_uart_p uart)
{
	return UART_INST_VAL(uart->UINTP, UINTP_RXD, 1);
}

signed char uart_get_char(exynos_uart_p ptr)
{
	char c = 0;
	exynos_uart_p uart = (exynos_uart_p)ptr;

	//wait until a vaild byte comes in
	while(!UART_EXT_VAL(uart->UFSTAT,UFSTAT_RX_FIFO_FULL) &&
	      	!UART_EXT_VAL(uart->UFSTAT,UFSTAT_RX_FIFO_CNT));

	//read
	//if(_isRx_pending((exynos_uart_p)ptr)){
		c = (uart->URXH);
	//}
	return c;
}

int uart_put_char(exynos_uart_p ptr, char c)
{
	//FIFO full?
	exynos_uart_p uart = (exynos_uart_p)ptr;
	while(UART_EXT_VAL(uart->UFSTAT,UFSTAT_TX_FIFO_FULL));
	//performs write
	uart->UTXH = c;
	return 0;
}

/*
 * Read number of bytes into buffer
 * return number of bytes read
 */
int uart_read_poll(exynos_uart_p uart, char * buf, int size, int *error)
{
	int cnt = 0;
	int i = 0;

	for(i = 0; i<size; i++){
		//wait until rx data ready
		while(!UART_EXT_VAL(uart->UFSTAT,UFSTAT_RX_FIFO_FULL) &&
			!UART_EXT_VAL(uart->UFSTAT,UFSTAT_RX_FIFO_CNT));

		//performs read
		buf[i] = (uart->URXH);
		cnt ++;
		if(UART_EXT_VAL(uart->UFSTAT,UFSTAT_RX_FIFO_ERROR)){
			//error
			*error = uart->UERSTAT;
		}
	}

	return cnt;
}

/*
 * Write number of bytes from buffer
 * return number of bytes wrote
 */
int uart_write_poll(exynos_uart_p uart, char * buf, int size, int *error)
{
	int cnt = 0;
	int i = 0;

	for(i = 0; i<size; i++){
		//FIFO full?
		while(UART_EXT_VAL(uart->UFSTAT,UFSTAT_TX_FIFO_FULL));
		//performs write
		uart->UTXH = buf[i];
		cnt ++;
		*error = 0;
	}
	return cnt;
}

static void _RX_TIMEOUT_INT(exynos_uart_p uart, int flag)
{
	UART_INST_VAL(uart->UCON, UCON_RX_TIMEOUT, flag);
	UART_INST_VAL(uart->UCON, UCON_RX_TIMEOUT_EMPTY_FIFO, flag);
}

static void _RX_INT(exynos_uart_p uart, int enable)
{
	//on/off rx interrupt
	if(enable){
		UART_INST_VAL(uart->UINTM, UINTM_RXD, 0);
	}else{
		//mask out interrupt
		UART_INST_VAL(uart->UINTM, UINTM_RXD, 1);
	}
}

static void _TX_INT(exynos_uart_p uart, int enable)
{
	//on/off tx interrupt
	if(enable){
		UART_INST_VAL(uart->UINTM, UINTM_TXD, 0);
	}else{
		//mask out interrupt
		UART_INST_VAL(uart->UINTM, UINTM_TXD, 1);
	}
}

static void _MODEM_INT(exynos_uart_p uart, int enable)
{
	//mask out modem interrupt
	if(enable){
		UART_INST_VAL(uart->UINTM, UINTM_MODEM, 0);
	}else{
		//mask out interrupt
		UART_INST_VAL(uart->UINTM, UINTM_MODEM, 1);
	}
}

static void _ERROR_INT(exynos_uart_p uart, int enable)
{
	if(enable){
		UART_INST_VAL(uart->UINTM, UINTM_ERROR, 0);
	}else{
		//mask out error interrupt
		UART_INST_VAL(uart->UINTM, UINTM_ERROR, 1);
	}
}

static void uart_config(exynos_uart_p uart, Uart_BaudRate baud, 
	Uart_opMode mode, int loopback, Uart_Parity parity, 
	Uart_StopBit bits, Uart_Word len)
{
	//Normal mode
	UART_INST_VAL(uart->ULCON, ULCON_INFRARED_MODE, mode);
	//Parity mode
	UART_INST_VAL(uart->ULCON, ULCON_PARITY_MODE, parity);
	//Number of stop bit
	UART_INST_VAL(uart->ULCON, ULCON_STOP_BIT, bits);
	//word length
	UART_INST_VAL(uart->ULCON, ULCON_WORD_LENGTH, len);

	//interrupt type
	UART_INST_VAL(uart->UCON, UCON_TX_INT_TYPE, LEVEL);
	UART_INST_VAL(uart->UCON, UCON_RX_INT_TYPE, LEVEL);

	//Rx error interrupt - disable
	UART_INST_VAL(uart->UCON, UCON_RX_ERR_INT, DISABLE);

	//Loop-back mode
	UART_INST_VAL(uart->UCON, UCON_LOOP_BACK_MODE, loopback);
	//break signal - don't send
	UART_INST_VAL(uart->UCON, UCON_SEND_BRK_SIGNAL, DISABLE);
	//Transmit mode
	UART_INST_VAL(uart->UCON, UCON_TX_MODE, INT_POLL_MODE);
	//Receive mode
	UART_INST_VAL(uart->UCON, UCON_RX_MODE, INT_POLL_MODE);
	//baud rate
	uart_setbaudrate(uart, baud);
	//reset fifos
	uart_reset_fifo(uart);
	//set FIFO trigger levels
	UART_INST_VAL(uart->UFCON, UFCON_TX_FIFO_LVL, LVL0);
	UART_INST_VAL(uart->UFCON, UFCON_RX_FIFO_LVL, LVL0);
	//FIFO - enable
	UART_INST_VAL(uart->UFCON, UFCON_FIFO_ENABLE, ENABLE);

	//Interrupts
	_RX_INT(uart, ENABLE);
	_RX_TIMEOUT_INT(uart, DISABLE);
	_TX_INT(uart, DISABLE);
	_MODEM_INT(uart, DISABLE);
	_ERROR_INT(uart, DISABLE);

	_clearTx_pending(uart);
	_clearRx_pending(uart);
}

static void append_char_to_buf(circ_buf* buf, char in)
{
	_acquire_spin_lock(&(buf->lock));
	//if the tail is aligned with the head, advances the head
	if((buf->valids == MAX_BUFFER_SIZE) && (buf->head == buf->tail)){
		buf->head ++;
		if(buf->head >= MAX_BUFFER_SIZE){
			buf->head = buf->head % MAX_BUFFER_SIZE;
		}
	}
	buf->data[buf->tail] = in;
	buf->valids ++;
	buf->tail ++;
	if(buf->tail >= MAX_BUFFER_SIZE){
		buf->tail = buf->tail % MAX_BUFFER_SIZE;
	}
	if(buf->valids > MAX_BUFFER_SIZE){
		buf->valids = MAX_BUFFER_SIZE;
	}
	_release_spin_lock(&(buf->lock));
}

static int remove_char_from_buf(circ_buf* buf, char * out)
{
	int ret = 0;
	_acquire_spin_lock(&(buf->lock));
	if(buf->valids > 0){
		*out = buf->data[buf->head];
		buf->valids --;
		buf->head ++;
		if(buf->head >= MAX_BUFFER_SIZE){
			buf->head = buf->head % MAX_BUFFER_SIZE;
		}
		ret = 1;
	}else{
		//no valid data for read
		ret = 0;
	}
	_release_spin_lock(&(buf->lock));
	return ret;
}

/* Return the number of bytes successfully read for buffer */
int uart_read(int uart_num, int rsize)
{
	unsigned int i,size = 0;
	char c = 0;
	exynos_uart_p uart = uart_dev_table[uart_num].portaddr;
	circ_buf *rxbuf = &(uart_dev_table[uart_num].rxbuf);
	char *client_rx_buf = uart_dev_table[uart_num].client_buf;
	uint32_t rx_triglvl = uart_dev_table[uart_num].rx_triglvl;
	size = rsize;
	i = 0;

	while(size != 0){
		if(size < rx_triglvl){
			//turn on timeout interrupt in case size is less than rx fifo level
			_RX_TIMEOUT_INT(uart, ENABLE);
		}
		if(remove_char_from_buf(rxbuf, &c)){
			client_rx_buf[i] = c;
			size --;
			i ++;
		}else{
			//seL4_Yield();
		}
	}

//	uart_read_poll(uart, client_buf, rsize, &i);
	return i;
}

/* Return the number of bytes successfully write to buffer */
int uart_write(int uart_num, int wsize)
{
	unsigned int i,size = 0;
	exynos_uart_p uart = uart_dev_table[uart_num].portaddr;
/*	circ_buf *rxbuf = &(uart_dev_table[uart_num].rxbuf);
	circ_buf *txbuf = &(uart_dev_table[uart_num].txbuf);
	char *client_tx_buf = uart_dev_table[uart_num].client_buf;

	int empty = MAX_BUFFER_SIZE - (txbuf->valids);
	if(wsize >= empty){
		size = empty;
	}else{
		size = wsize;
	}
	for(i=0; i<size; i++){
		append_char_to_buf(txbuf, client_tx_buf[i]);
	}
	_TX_INT(uart, ENABLE);
*/
	uart_write_poll(uart, client_buf, wsize, &i);
	return wsize;
}

static void handle_interrputs(int uart_num)
{
	int i = 0;
	char c = 0;
	int size = 0;
	exynos_uart_p uart = uart_dev_table[uart_num].portaddr;
	circ_buf *rxbuf = &(uart_dev_table[uart_num].rxbuf);
	circ_buf *txbuf = &(uart_dev_table[uart_num].txbuf);

	if(_isRx_pending(uart)){
		//check if the buffer is full
		if(UART_EXT_VAL(uart->UFSTAT,UFSTAT_RX_FIFO_FULL)){
			size = uart_dev_table[uart_num].fifo_depth;
		}else{
			size = UART_EXT_VAL(uart->UFSTAT,UFSTAT_RX_FIFO_CNT);
		}
		//If there is incoming bytes, turn on rx timeout interrupt, otherwise,
		//we are getting timeout interrupt even with no incoming bytes, turn it off.
		if(size > 0){
			_RX_TIMEOUT_INT(uart, ENABLE);
			for(i=0;i<size;i++){
				append_char_to_buf(rxbuf, uart->URXH);
			}
		}else{
			_RX_TIMEOUT_INT(uart, DISABLE);
		}
		_clearRx_pending(uart);
	}

	if(_isTx_pending(uart)){
		while(!UART_EXT_VAL(uart->UFSTAT, UFSTAT_TX_FIFO_FULL)){
			if(remove_char_from_buf(txbuf, (char*)&(uart->UTXH))){
				udelay(TX_WRITE_DELAY_US); // a dummy delay for successive write to tx fifo
			}else{
				_TX_INT(uart, DISABLE);
				break;
			}
		}
		_clearTx_pending(uart);
	}
}

static void uart_interrupt_handler(void *data UNUSED)
{
	handle_interrputs(UART0);
	interrupt_reg_callback(&uart_interrupt_handler, NULL);
}

static void uart0_interrupt_handler(void *data UNUSED)
{
	//handle_interrputs(UART0);
	//interrupt_reg_callback(&uart0_interrupt_handler);
}

static void uart3_interrupt_handler(void *data UNUSED)
{
	//handle_interrputs(UART3);
	//interrupt_reg_callback(&uart3_interrupt_handler);
}

/*
 * Component Interfaces
 */
unsigned int uart_getuart0addr(void)
{
	return (unsigned int) uart0;
}
unsigned int uart_getuart1addr(void)
{
	return (unsigned int) uart1;
}

unsigned int uart_getuart3addr(void)
{
	return (unsigned int) uart3;
}

void uart__init(void)
{
	int i =0;
    /* Setup UART */
	for(i=0; i<NUM_UART; i++){
		uart_dev_table[i].portaddr = NULL;
		uart_dev_table[i].rxbuf.head = 0;
		uart_dev_table[i].rxbuf.tail = 0;
		uart_dev_table[i].rxbuf.valids = 0;
		uart_dev_table[i].txbuf.head = 0;
		uart_dev_table[i].txbuf.tail = 0;
		uart_dev_table[i].txbuf.valids = 0;
	}
	//install port address
	uart_dev_table[UART0].portaddr = uart0;
	uart_dev_table[UART1].portaddr = uart1;
	uart_dev_table[UART2].portaddr = uart2;
	uart_dev_table[UART3].portaddr = uart3;

	uart_dev_table[UART0].fifo_depth = UART0_FIFO_DEPTH;
	uart_dev_table[UART1].fifo_depth = UART1_FIFO_DEPTH;
	uart_dev_table[UART3].fifo_depth = UART3_FIFO_DEPTH;

	uart_dev_table[UART0].rx_triglvl = 2;
	uart_dev_table[UART1].rx_triglvl = 8;
	uart_dev_table[UART3].rx_triglvl = 2;

	uart_dev_table[UART0].tx_triglvl = 0;
	uart_dev_table[UART1].tx_triglvl = 0;
	uart_dev_table[UART3].tx_triglvl = 0;

	uart_dev_table[UART1].client_buf = (char *)client_buf;

	//baud rate
	//uart_setbaudrate(uart2, Uart_BaudRate_115_2K);

	uart_config(uart1, Uart_BaudRate_115_2K, UART_MODE, DISABLE, NO_PARITY, ONE_STOP_BIT, BIT8);
	uart_config(uart0, Uart_BaudRate_115_2K, UART_MODE, DISABLE, NO_PARITY, ONE_STOP_BIT, BIT8);
	//uart_config(uart2, Uart_BaudRate_115_2K, UART_MODE, DISABLE, NO_PARITY, ONE_STOP_BIT, BIT8);
	uart_config(uart3, Uart_BaudRate_115_2K, UART_MODE, ENABLE, NO_PARITY, ONE_STOP_BIT, BIT8);

	print_reg(2);
	print_reg(0);
	interrupt_reg_callback(&uart_interrupt_handler, NULL);
}

/*
 * Exynos UART driver
 * Test Suites
 */
#define TESTSIZE 255
char TEST_STR[] = "seL4 Uart Test\n";

static void zeros(char * buf, int size)
{
	int i = 0;
	for(i=0;i<size;i++)
		buf[i] = 0;
}

static int loopback_test(unsigned int ptr)
{
	char wbuf[TESTSIZE];
	char rbuf[TESTSIZE];
	int err = 0;
	int cnt = 0;
	exynos_uart_p uart = (exynos_uart_p) ptr;
	//save previous state
	int resmode = UART_EXT_VAL(uart->UCON,UCON_LOOP_BACK_MODE);
	//set to Loop-back mode
	UART_INST_VAL(uart->UCON, UCON_LOOP_BACK_MODE, ENABLE);
	uart_reset_fifo(uart);

	zeros(wbuf,TESTSIZE);
	zeros(rbuf,TESTSIZE);

	strncpy(wbuf, TEST_STR, 16);

	cnt = strlen(wbuf);

	uart_write_poll(uart, wbuf,cnt , &err);
	uart_read_poll(uart, rbuf, cnt, &err);
	//restore state
	UART_INST_VAL(uart->UCON, UCON_LOOP_BACK_MODE, resmode);

	return 	strncmp(rbuf,wbuf,cnt);
}

static void passthrough(exynos_uart_p ptr)
{
	char wbuf[TESTSIZE];
	char rbuf[TESTSIZE];
	exynos_uart_p uart = (exynos_uart_p) ptr;
	int err = 0;
	int cnt = 0;
	char c = 0;

	zeros(wbuf,TESTSIZE);
	strcpy(wbuf, TEST_STR);
	cnt = strlen(wbuf);
	uart_write_poll(uart, wbuf,cnt, &err);
	while(1){
		if(_isRx_pending((exynos_uart_p)uart)){
			c = uart_get_char(ptr);
			uart_put_char(ptr, c);
			_clearRx_pending((exynos_uart_p)uart);
		} }
}

int uart_test(void){

	printf("UART TEST, Version 1.0\n");

	printf("uart1 - ");
	if(loopback_test((unsigned int)uart1)){
		printf("FAILED\n");
	}
	printf("pass\n");
	printf("uart0 - ");
	if(loopback_test((unsigned int)uart0)){
		printf("FAILED\n");
	}
	printf("pass\n");
	printf("uart3 - ");
	if(loopback_test((unsigned int)uart3)){
		printf("FAILED\n");
	}
	printf("pass\n");

	//passthrough(uart1);

	return 0;
}
#endif
