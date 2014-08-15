/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef EXYNOS_SPI_H
#define EXYNOS_SPI_H

#include <stdint.h>

typedef struct {
	volatile uint32_t ch_cfg;    /* 0x0 */
	volatile uint32_t pad0;
	volatile uint32_t mode_cfg;  /* 0x8 */
	volatile uint32_t cs_reg;    /* 0xc */
	volatile uint32_t spi_int_en;        /* 0x10 */
	volatile uint32_t spi_status;        /* 0x14 */
	volatile uint32_t spi_tx_data;       /* 0x18 */
	volatile uint32_t spi_rx_data;       /* 0x1C */
	volatile uint32_t packet_cnt_reg;    /* 0x20 */
	volatile uint32_t pending_clr_reg;   /* 0x24 */
	volatile uint32_t swap_cfg;          /* 0x28 */
	volatile uint32_t fb_clk_sel;        /* 0x2c */
}exynos_spi, *exynos_spi_p;


#define SPI0_PHYS (0x12d20000)
#define SPI1_PHYS (0x12d30000)
#define SPI2_PHYS (0x12d40000)
#define ISP_SPI0_PHYS (0x131a000)
#define ISP_SPI1_PHYS (0x131b0000)

#define FIFODEPTH 64

/* ch_cfg bits */
#define SPI_CH_TX_CH_ON        	(1<<0)
#define SPI_CH_RX_CH_ON        	(1<<1)
#define SPI_CH_CPHA_B          	(1<<2)
#define SPI_CH_CPOL_L          	(1<<3)
#define SPI_CH_SLAVE_MODE      	(1<<4)
#define SPI_CH_RST          	(1<<5)
#define SPI_CH_HS_EN   			(1<<6)

/* mode_cfg bits */
#define SPI_MODE_CH_WIDTH_BYTE        	(0<<29)
#define SPI_MODE_CH_WIDTH_HALFWORD    	(1<<29)
#define SPI_MODE_CH_WIDTH_WORD  		(2<<29)

#define SPI_MODE_BUS_WIDTH_BYTE 		(0<<17)
#define SPI_MODE_BUS_WIDTH_HALFWORD 	(1<<17)
#define SPI_MODE_BUS_WIDTH_WORD 		(2<<17)

#define TX_RDY_LVL_BIT  5
#define RX_RDY_LVL_BIT  11
#define DMA_TYPE_BURST  (1<<0)
#define TX_DMA_SW       (1<<1)
#define RX_DMA_SW       (1<<2)
#define TRAILING_CNT    19

/* SPI status  bits */
#define TX_FIFO_RDR             (1<<0)
#define RX_FIFO_RDY             (1<<1)
#define TX_UNDERRUN             (1<<2)
#define TX_OVERRUN              (1<<3)
#define RX_UNDERRUN             (1<<4)
#define RX_OVERRUN              (1<<5)
#define TX_FIFO_LVL_BIT         6               /* 9 bits */
#define RX_FIFO_LVL_BIT         15              /* 9 bits */
#define TRAILING_BYTE           (1<<24)
#define TX_DONE                 (1<<25)
#define SPI_FIFO_LVL_MASK		0x1ff
#define RX_FIFO_LEVEL(status) (((status) >> RX_FIFO_LVL_BIT) & SPI_FIFO_LVL_MASK)
#define TX_FIFO_LEVEL(status) (((status) >> TX_FIFO_LVL_BIT) & SPI_FIFO_LVL_MASK)

/* Packet Count */
#define SPI_PACKET_CNT_EN	(1<<16)

/* chip select bits cs_reg */
#define NSSOUT  (1<<0)
#define AUTO_N_MANUAL (1<<1)
#define NCS_TIME_COUNT_BIT 4    /* 6 bits */
#define SPI_SLAVE_SIG_INACT		(1<<0)
/* 
 * NSS_TIME_COUNT sets inactive time between packets 
 * on the bus as (NSS_TIME_COUNT + 3)/2.SPICLKout 
 */

/* SPI_INT_EN bits */
#define INT_EN_TX_FIFO_RDY      (1<<0)
#define INT_EN_RX_FIFO_RDY      (1<<1)
#define INT_EN_TX_UNDERRUN      (1<<2)
#define INT_EN_TX_OVERRUN       (1<<3)
#define INT_EN_RX_UNDERRUN      (1<<4)
#define INT_EN_RX_OVERRUN       (1<<5)
#define INT_EN_TRAILING         (1<<56)

/* PENDING_CLR_REG */
#define TRAILING_CLR            (1<<0)
#define RX_OVERRUN_CLR          (1<<1)
#define RX_UNDERRUN_CLR         (1<<2)
#define TX_OVERRUN_CLR          (1<<3)
#define TC_UNDERRUN_CLR         (1<<4)

/* SWAP_CFG (for big/little conversion */
#define TX_SWAP_EN              (1<<0)
#define TX_BIT_SWAP             (1<<1)
#define TX_BYTE_SWAP            (1<<2)
#define TX_HWORD_SWAP           (1<<3)
#define RX_SWAP_EN              (1<<4)
#define RX_BIT_SWAP             (1<<5)
#define RX_BYTE_SWAP            (1<<6)
#define RX_HWORD_SWAP           (1<<7)

#endif
