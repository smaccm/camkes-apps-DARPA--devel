/*
 * Copyright (c) 2014 National ICT Australia Limited (NICTA), ABN 62 102 206 173.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * exynos_mmc.h
 *
 *  Created on: Jul 30, 2013
 *      Author: jxie
 */

#ifndef EXYNOS_MMC_H_
#define EXYNOS_MMC_H_


/**
 * CAMKES DMA BUFFERS
 */
/*
 * cur_idmac descriptors
 * vaddr & paddr
 */
struct mshc_idmac * cur_idmac_vaddr;
struct mshc_idmac * cur_idmac_paddr;


/* Exynos_dw_mmc.c - uboot */
#define	EXYNOS_MSHC_MAX_CH_NUM			4
#define	EXYNOS_MSHC_MAX_FREQ			52000000
#define	EXYNOS_MSHC_MIN_FREQ			400000
#define	EXYNOS_MSHC_MMC0_CLKSEL_VAL		0x03030001
#define	EXYNOS_MSHC_MMC2_CLKSEL_VAL		0x03020001

#define MSHC_CTRL			0x000
#define MSHC_PWREN			0x004
#define MSHC_CLKDIV			0x008	/*Clock Divider*/
#define MSHC_CLKSRC			0x00C	/*Clock Source*/
#define MSHC_CLKENA			0x010	/*Clock Enable*/
#define MSHC_TMOUT			0x014	/*Time Out*/
#define MSHC_CTYPE			0x018	/*Card Type*/
#define MSHC_BLKSIZ			0x01C	/*Block Size*/
#define MSHC_BYTCNT			0x020	/*Byte Count*/
#define MSHC_INTMASK		0x024	/*Interrupt mask*/
#define MSHC_CMDARG			0x028	/*Command Argument*/
#define MSHC_CMD			0x02C	/*Command*/
#define MSHC_RESP0			0x030
#define MSHC_RESP1			0x034
#define MSHC_RESP2			0x038
#define MSHC_RESP3			0x03C
#define MSHC_MINTSTS		0x040
#define MSHC_RINTSTS		0x044	/*Interrupt status*/
#define MSHC_STATUS			0x048
#define MSHC_FIFOTH			0x04C
#define MSHC_CDETECT		0x050
#define MSHC_WRTPRT			0x054
#define MSHC_GPIO			0x058
#define MSHC_TCMCNT			0x05C
#define MSHC_TBBCNT			0x060
#define MSHC_DEBNCE			0x064
#define MSHC_USRID			0x068
#define MSHC_VERID			0x06C
#define MSHC_HCON			0x070
#define MSHC_UHS_REG		0x074
//							0x078
//							0x07C
#define MSHC_BMOD			0x080
#define MSHC_PLDMND			0x084
#define MSHC_DBADDR			0x088
#define MSHC_IDSTS			0x08C
#define MSHC_IDINTEN		0x090
#define MSHC_DSCADDR		0x094
#define MSHC_BUFADDR		0x098
#define MSHC_CLKSEL			0x09C
#define MSHC_DATA			0x200

/* MSHC_INTMASK (0x024): Interrupt Mask register */
#define MSHC_INTMSK_ALL	0xffffffff
#define MSHC_INTMSK_CD			(1 << 0)
#define MSHC_INTMSK_RE			(1 << 1)
#define MSHC_INTMSK_CDONE		(1 << 2)
#define MSHC_INTMSK_DTO		(1 << 3)
#define MSHC_INTMSK_TXDR		(1 << 4)
#define MSHC_INTMSK_RXDR		(1 << 5)
//							(1 << 6)
#define MSHC_INTMSK_DCRC		(1 << 7)
#define MSHC_INTMSK_RTO		(1 << 8)
#define MSHC_INTMSK_DRTO		(1 << 9)
#define MSHC_INTMSK_HTO		(1 << 10)
#define MSHC_INTMSK_FRUN		(1 << 11)
#define MSHC_INTMSK_HLE		(1 << 12)
#define MSHC_INTMSK_SBE		(1 << 13)
#define MSHC_INTMSK_ACD		(1 << 14)
#define MSHC_INTMSK_EBE		(1 << 15)
#define MSHC_INTMSK_SDIO(n)	(1 << (16 + n))


/* RINTSTS: Raw Interrupt Register */
#define MSHC_INT_ERROR		0xbfc2
#define MSHC_INT_DATA_ERROR		(MSHC_INTMSK_EBE | MSHC_INTMSK_SBE | MSHC_INTMSK_HLE |\
			MSHC_INTMSK_FRUN | MSHC_INTMSK_DCRC)
#define MSHC_INT_DATA_TOUT (MSHC_INTMSK_HTO | MSHC_INTMSK_DRTO)


/* MSHC_CTRL: CTRL Register */
#define MSHC_CTRL_RESET				(1 << 0)
#define MSHC_CTRL_FIFO_RESET		(1 << 1)
#define MSHC_CTRL_DMA_RESET			(1 << 2)
//									(1 << 3)
#define MSHC_CTRL_INT_ENABLE		(1 << 4)
#define MSHC_CTRL_DMA_EN			(1 << 5)
#define MSHC_CTRL_READ_WAIT			(1 << 6)
#define MSHC_CTRL_SEND_IRQ_RESP		(1 << 7)
#define MSHC_CTRL_ABRT_READ_DATA	(1 << 8)
#define MSHC_CTRL_SEND_CCSD			(1 << 9)
#define MSHC_CTRL_SEND_AS_CCSD		(1 << 10)
#define MSHC_CTRL_CEATA_INT_EN		(1 << 11)
//									(1 << 12)
//									(1 << 24)
#define MSHC_CTRL_IDMA_EN			(1 << 25)

#define MSHC_RESET_ALL		(MSHC_CTRL_RESET | MSHC_CTRL_FIFO_RESET |\
				MSHC_CTRL_DMA_RESET)

/* MSHC_CMD (0x02C): CMD register */
#define MSHC_CMD_INDX(n)		((n) & 0x1F)
#define MSHC_CMD_RESP_EXP		(1 << 6)
#define MSHC_CMD_RESP_LENGTH	(1 << 7)
#define MSHC_CMD_CHECK_CRC		(1 << 8)
#define MSHC_CMD_DATA_EXP		(1 << 9)
#define MSHC_CMD_RW				(1 << 10)
#define MSHC_CMD_STRM_MODE		(1 << 11)
#define MSHC_CMD_SEND_STOP		(1 << 12)
#define MSHC_CMD_PRV_DAT_WAIT	(1 << 13)
#define MSHC_CMD_ABORT_STOP		(1 << 14)
#define MSHC_CMD_INIT			(1 << 15)
#define MSHC_CMD_UPD_CLK		(1 << 21)
#define MSHC_CMD_CEATA_RD		(1 << 22)
#define MSHC_CMD_CCS_EXP		(1 << 23)
#define MSHC_CMD_USE_HOLD_REG	(1 << 29)
#define MSHC_CMD_START			(1U << 31)

/* CLKENA register */
#define MSHC_CLKEN_ENABLE	(1 << 0)
#define MSHC_CLKEN_LOW_PWR	(1 << 16)

/* MSHC_TMOUT time-out register defines */
#define MSHC_TMOUT_DATA(n)		((n) << 8)
#define MSHC_TMOUT_DATA_MSK		0xFFFFFF00
#define MSHC_TMOUT_RESP(n)		((n) & 0xFF)
#define MSHC_TMOUT_RESP_MSK		0xFF

/* Card-type register */
#define MSHC_CTYPE_1BIT			0
#define MSHC_CTYPE_4BIT			(1 << 0)
#define MSHC_CTYPE_8BIT			(1 << 16)

/* Status Register */
#define MSHC_BUSY		(1 << 9)
/* Status register defines */
#define MSHC_GET_FCNT(x)		(((x) >> 17) & 0x1FFF)

/* Internal DMAC interrupt defines */
#define MSHC_IDMAC_INT_AI		(1 << 9)
#define MSHC_IDMAC_INT_NI		(1 << 8)
#define MSHC_IDMAC_INT_CES		(1 << 5)
#define MSHC_IDMAC_INT_DU		(1 << 4)
#define MSHC_IDMAC_INT_FBE		(1 << 2)
#define MSHC_IDMAC_INT_RI		(1 << 1)
#define MSHC_IDMAC_INT_TI		(1 << 0)

/*  Bus Mode Register */
#define MSHC_BMOD_IDMAC_RESET	(1 << 0)
#define MSHC_BMOD_IDMAC_FB		(1 << 1)
#define MSHC_BMOD_IDMAC_EN		(1 << 7)

/* FIFOTH Register */
#define MSIZE(x)				((x) << 28)
#define RX_WMARK(x)				((x) << 16)
#define TX_WMARK(x)				 (x)
#define RX_WMARK_SHIFT				16
#define RX_WMARK_MASK			(0xfff << RX_WMARK_SHIFT)

/* IDMA descriptor fields */
#define MSHC_IDMAC_OWN			(1U << 31)
#define MSHC_IDMAC_CH			(1 << 4)
#define MSHC_IDMAC_FS			(1 << 3)
#define MSHC_IDMAC_LD			(1 << 2)

/* Version ID register define */
#define MSHC_GET_VERID(x)		((x) & 0xFFFF)

/* MSHC_CLKSEL: Clock Select */
#define MSHC_CLKSEL_CCLK_SAMPLE(x)		(((x) & 7) << 0)
#define MSHC_CLKSEL_CCLK_DRIVE(x)		(((x) & 7) << 16)
#define MSHC_CLKSEL_CCLK_DIVIDER(x)		(((x) & 7) << 24)
#define MSHC_CLKSEL_GET_DRV_WD3(x)		(((x) >> 16) & 0x7)

#define MSHC_CLKSEL_TIMING(x, y, z)	(	MSHC_CLKSEL_CCLK_SAMPLE(x) |	\
										MSHC_CLKSEL_CCLK_DRIVE(y) |	\
										MSHC_CLKSEL_CCLK_DIVIDER(z))

/*
 * Data structure to set up individual MSHC
 */
struct exynos_mshc{
	uint32_t status;
	uint32_t id;
	uint32_t reg;
	uint32_t bus_width;		//bits
	uint32_t timing[3];
};


/* chained descriptor structure */
struct mshc_idmac {
	uint32_t flags;
	uint32_t cnt;
	uint32_t addr;
	uint32_t next_addr;
};


/* Exynos implementation specific driver private data */
struct mshc_exynos_priv_data {
	uint8_t						ciu_div;	//<3>
	uint32_t					sdr_timing;	//<2, 3>
	uint32_t					ddr_timing;	//<1, 2>
};

/* Exynos5250 controller specific capabilities */
/*
static unsigned long exynos5250_mshc_caps[4] = {
	MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
		MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
	MMC_CAP_CMD23,
};
*/
/* Interface with mmc layer */
struct mshc_host {
	char *		 name;
	void *		 ioaddr;
	void *		 priv;			/* private data */
	unsigned int quirks;	/* Workaround flags */
	unsigned int caps;
	unsigned int version;
	unsigned int clock;
	unsigned int bus_hz;
	int			 dev_index;
	int 		 buswidth;
	uint32_t 	 clksel_val;
	uint32_t 	 fifoth_val;
	struct mmc * mmc;

	void (*clksel)(struct mshc_host *host);
	unsigned int (*mmc_clk)(int dev_index);
};


static inline void mshc_write(struct mshc_host *host, int reg, uint32_t val)
{
	(*(volatile unsigned int *)(host->ioaddr + reg)) = (val);
}

static inline uint32_t mshc_read(struct mshc_host *host, int reg)
{
	return (*(volatile unsigned int *)(host->ioaddr + reg));
}


/*
 * Function used as callback function to initialise the
 * CLKSEL register for every mmc channel.
 */
void exynos_mshc_clksel(struct mshc_host *host);

unsigned int exynos_mshc_get_clk(int dev_index);
int mshc_wait_reset(struct mshc_host * host, uint32_t value);
int exynos_mshc_init(void);
void mshc__init (void);


#endif /* EXYNOS_MMC_H_ */
