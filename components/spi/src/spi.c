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
 * SPI driver
 */
#if 0
#include <autoconf.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

#include "spi.h"

#include "exynos_spi.h"

#include "common.h"
#include "utils.h"
#include "spi_inf.h"

enum mode{
	INTERRUPT,
	POLL
};

enum stage{
	IDLE,
	START,
	COMPLETED
};


exynos_spi_p spi;

/* Internal book-keeping */
static uint32_t spi_cfg;
static spi_dev_port_p dev;
static spi_dev_port_p spiDataBuf[NUM_APPS];
static int spiMode;
static int spiStage;

struct contBlk{
	uint32_t wcnt;
	uint32_t rcnt;
	uint32_t inbytes;
	uint32_t outbytes;
	char * wbuf;
	char * rbuf;
};

struct contBlk	spi1_contB;

static void assign_spi_bufs(void){
	spi = (exynos_spi_p) spi1_reg;
	spiDataBuf[CLIENT_APP_ID] 	= (spi_dev_port_p) spi1_client;
	spiDataBuf[CAN_APP_ID] 		= (spi_dev_port_p) spi1_can;
	//printf("assign can dev = %d\n", (unsigned int)spi1_can_data);
}

static spi_dev_port* find_buffer(unsigned int id){
	return spiDataBuf[id];
}


static void spi_enable(void)
{
    spi_cfg |= SPI_CH_TX_CH_ON;
    spi_cfg	|= SPI_CH_RX_CH_ON;
    spi->ch_cfg = spi_cfg;
}

static void spi_disable(void)
{
    spi_cfg &= ~(SPI_CH_TX_CH_ON | SPI_CH_RX_CH_ON);
    spi->ch_cfg = spi_cfg;
}

/*
 * Flush Tx/Rx FIFOs, SPI status
 */
static void spi_reset(void){

	//Channels off before reset
	spi_disable();
	//SW_RST Active
	spi_cfg = spi_cfg | SPI_CH_RST;
	spi->ch_cfg = spi_cfg;
	//Manually clear
	spi_cfg = spi_cfg & (~SPI_CH_RST);
	spi->ch_cfg = spi_cfg;
	//Re-enable Channels
	spi_enable();
}

int spi_register_slave(char *name, int peripheral)
{
}
int spi_unregister_slave(int id)
{
}

/*
 * Chip Select Functions for SPI1
 */
void spi_chip_unselect(int id){
    spi->cs_reg = 1;
}

void spi_chip_select(int id){
    spi->cs_reg = 0;
}

/*
 * Prepare for SPI transfer
 */
static void spi_request_bytes(unsigned int count){
    spi_reset();
    spi->packet_cnt_reg = SPI_PACKET_CNT_EN | count;
}

static void spi_enable_interrupt(void){
	spi->spi_int_en = (INT_EN_TX_FIFO_RDY | INT_EN_RX_FIFO_RDY);
}

static void spi_disable_interrupt(void){
	uint32_t mask = (INT_EN_TX_FIFO_RDY | INT_EN_RX_FIFO_RDY);
	spi->spi_int_en &= ~mask;
}


/*
 * In a single transaction write wcount bytes from wbuf and read rcount bytes
 * into rbuf
 */
int spi_transfer(unsigned int id, unsigned int wcount, unsigned int rcount)
{
    uint32_t status = 0;
    char * wbuf = NULL;
    char * rbuf = NULL;
    uint32_t temp;
    int outbytes,inbytes = 0;
    int todo = rcount + wcount;
    int rcnt = rcount;
    int wcnt = wcount;
    // assign port address
    dev = find_buffer(id);

    wbuf = (dev->txbuf);
    rbuf = (dev->rxbuf);
    assert(wbuf != NULL);
    assert(rbuf != NULL);

    assert(wcount < FIFODEPTH);
    assert(rcount < FIFODEPTH);

    spi_request_bytes(todo);

    outbytes = inbytes = todo;

    if(spiMode == POLL){
		while (inbytes) {
		//	status = spi->spi_status;

		//	if(TX_FIFO_LEVEL(status) < FIFODEPTH && outbytes){
		//		if(wcount != 0){
					spi->spi_tx_data = *wbuf++;
		//			wcount --;
		//		}else{
		//			spi->spi_tx_data = 0xff;
		//		}
		//		outbytes--;
		//	}
		//	temp = 0;
		//	if(RX_FIFO_LEVEL(status) > 0 && inbytes){
				temp = spi->spi_rx_data;
		//		if(rbuf != NULL){
					*rbuf++ = temp;
		//		}
				inbytes--;
		//	}
		}

    }else if(spiMode == INTERRUPT){
    	 spiStage = START;

    	 spi1_contB.inbytes = spi1_contB.outbytes = todo;
    	 spi1_contB.rbuf = rbuf;
    	 spi1_contB.wbuf = wbuf;
    	 spi1_contB.wcnt = wcount;
    	 spi1_contB.rcnt = rcount;
    	 spi_enable_interrupt();

    	 while(spiStage != COMPLETED){
    		 //wait()
    		 //handle_interrupts()
    		 //seL4_Yield(); //remove
    	 }
    	 inbytes = spi1_contB.inbytes;

    	 spiStage = IDLE;
    }
    return 0;
}

int spi_transfer_byte(int id, char byte)
{

    dev = find_buffer(id);
	memset(dev->txbuf, byte, 1);
	spi_transfer(id, 1, 0);
	return dev->rxbuf[0];
}


static void handle_interrupts(void){
    uint32_t status = 0;
    uint32_t temp;

    char * wbuf = spi1_contB.wbuf;
    char * rbuf = spi1_contB.rbuf;
    int inbytes = spi1_contB.inbytes;
    int outbytes= spi1_contB.outbytes;

	if(spiStage == START && inbytes) {
		status = spi->spi_status;
		while(TX_FIFO_LEVEL(status) < FIFODEPTH && outbytes){
			//printf("TX_FIFO_LEVEL %d\n",TX_FIFO_LEVEL(status));
			if(spi1_contB.wcnt != 0){
				temp = *wbuf++;
				spi->spi_tx_data = temp;
				printf("spi write %x\n",(uint32_t)temp);
				spi1_contB.wcnt --;
			}else{
				spi->spi_tx_data = 0xff;
			}
			outbytes--;
			status = spi->spi_status;
		}

		temp = 0;

		while(RX_FIFO_LEVEL(status) > 0 && inbytes){

			temp = spi->spi_rx_data;
			printf("spi read %x\n",(uint32_t)temp);
			if(rbuf != NULL){
				*rbuf++ = temp;
			}
			inbytes--;
			status = spi->spi_status;

		}
		spi1_contB.inbytes = inbytes;
		spi1_contB.outbytes = outbytes;
		spi1_contB.rbuf = rbuf;
		spi1_contB.wbuf = wbuf;


	}else if(spiStage == START){

		//disable interrupt
		spi_disable_interrupt();
		spiStage = COMPLETED;
	}

}

static void spi1_interrupt_handler(void *data UNUSED)
{
	handle_interrupts();
	spi1_int_reg_callback(&spi1_interrupt_handler, NULL);
}


void spi__init(void) {

	/* for CAmkES*/
	assign_spi_bufs();
    /* Reset SPI */
    spi->ch_cfg = SPI_CH_RST;
    /* Set as Master, mode 0,0 */
    spi_cfg = 0;
    spi->ch_cfg = spi_cfg;
    /*
     * No DMA or interrupts, 8-bit transfers.
     * Allow Tx FIFO to get half full
     */
    spi->mode_cfg = (32 << TX_RDY_LVL_BIT) | (1 << RX_RDY_LVL_BIT);
    /* Manual chip select */
    spi->cs_reg = 1;
    spi_enable();
    spiMode = POLL;
    spi1_int_reg_callback(&spi1_interrupt_handler, NULL); //remove
	printf("%d: %x, %x, %x, %x, %x, %x, %x, %x, %x\n", __LINE__,
		spi->ch_cfg, spi->mode_cfg, spi->cs_reg,
		spi->spi_int_en, spi->spi_status, spi->packet_cnt_reg,
		spi->pending_clr_reg, spi->swap_cfg, spi->fb_clk_sel);
}

void spi_dummy(void)
{
}


int run(void)
{
	uint32_t ret;

	printf("%d: %x, %x, %x, %x, %x, %x, %x, %x, %x\n", __LINE__,
		spi->ch_cfg, spi->mode_cfg, spi->cs_reg,
		spi->spi_int_en, spi->spi_status, spi->packet_cnt_reg,
		spi->pending_clr_reg, spi->swap_cfg, spi->fb_clk_sel);
	spi_chip_select(0);
	printf("%d: %x, %x, %x, %x, %x, %x, %x, %x, %x\n", __LINE__,
		spi->ch_cfg, spi->mode_cfg, spi->cs_reg,
		spi->spi_int_en, spi->spi_status, spi->packet_cnt_reg,
		spi->pending_clr_reg, spi->swap_cfg, spi->fb_clk_sel);
	spi_transfer_byte(0, 0xC0);
	printf("%d: %x, %x, %x, %x, %x, %x, %x, %x, %x\n", __LINE__,
		spi->ch_cfg, spi->mode_cfg, spi->cs_reg,
		spi->spi_int_en, spi->spi_status, spi->packet_cnt_reg,
		spi->pending_clr_reg, spi->swap_cfg, spi->fb_clk_sel);
	spi_chip_unselect(0);
	printf("%d: %x, %x, %x, %x, %x, %x, %x, %x, %x\n", __LINE__,
		spi->ch_cfg, spi->mode_cfg, spi->cs_reg,
		spi->spi_int_en, spi->spi_status, spi->packet_cnt_reg,
		spi->pending_clr_reg, spi->swap_cfg, spi->fb_clk_sel);

//	while(1) {
	spi_chip_select(0);
	spi_transfer_byte(0, 0x5);
	spi_transfer_byte(0, 0xE);
	ret = spi_transfer_byte(0, 0xFF);
	ret = spi_transfer_byte(0, 0xFF);

	spi_chip_unselect(0);
	printf("%x, %x, %x, %x\n", spi->ch_cfg, spi->mode_cfg, spi->cs_reg, spi->packet_cnt_reg);
//	}

	return 0;
}

#endif
/*
//not used - can be replaced with spi_transfer();
int spi_write(unsigned int id, unsigned int count){

    uint32_t status;
    uint32_t temp;
    uint8_t *buf = NULL;
    int outbytes,inbytes = 0;

    // assign port address
    dev = find_buffer(id);

    // assigned buffer address
    buf = (dev->txbuf);
    assert(buf != NULL);

    outbytes = inbytes = count;

    if (count > FIFODEPTH)
        return -1;

    spi_request_bytes(count);

    while(inbytes){

    	status = spi->spi_status;

    	if(TX_FIFO_LEVEL(status) < FIFODEPTH && outbytes){
    		 spi->spi_tx_data = *buf++;
    		 outbytes--;
    	}
        if(RX_FIFO_LEVEL(status) > 0 && inbytes){
        	temp = spi->spi_rx_data;
        	(void)temp;
        	inbytes--;
        }
	}

    return 0;
}

*/
