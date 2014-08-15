/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/* SPI driver */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "utils.h"
#include <spi_inf.h>

#include <spi.h>

/* Hardware Registers */
#define CH_CFG          0x0    //SPI configuration 
#define MODE_CFG        0x8    //FIFO control
#define CS_REG          0xC    //Slave selection control
#define INT_EN          0x10   //Interrupt enable
#define STATUS          0x14   //SPI status
#define TX_DATA         0x18   //Tx data
#define RX_DATA         0x1C   //Rx data
#define PACKET_CNT_REG  0x20   //Packet count
#define PENDING_CLR_REG 0x24   //Interrupt pending clear
#define SWAP_CFG        0x28   //Swap configuration
#define FB_CLK_SEL      0x2C   //Feedback clock selection

/* SPI configuration */
#define CH_CFG_HIGH_SPEED_EN   (1 << 6)
#define CH_CFG_SW_RST          (1 << 5)
#define CH_CFG_SLAVE           (1 << 4)
#define CH_CFG_CPOL            (1 << 3)
#define CH_CFG_CPHA            (1 << 2)
#define CH_CFG_RX_CH_ON        (1 << 1)
#define CH_CFG_TX_CH_ON        (1 << 0)

/* FIFO control */
#define MODE_CFG_CH_WIDTH_SHF      (29)
#define MODE_CFG_TRAILING_CNT_SHF  (19)
#define MODE_CFG_BUS_WIDTH_SHF     (17)
#define MODE_CFG_RX_RDY_LVL_SHF    (11)
#define MODE_CFG_TX_RDY_LVL_SHF    (5)
#define MODE_CFG_RX_DMA_SW         (1 << 2)
#define MODE_CFG_TX_DMA_SW         (1 << 1)
#define MODE_CFG_DMA_TYPE          (1 << 0)

/* Slave selection control */
#define CS_REG_NCS_TIME_COUNT_SHF  (4)
#define CS_REG_AUTO_N_MANUAL       (1 << 1)
#define CS_REG_NSSOUT              (1 << 0)

/* Interrupt enable */
#define INT_EN_TRAILING     (1 << 6)
#define INT_EN_RX_OVERRUN   (1 << 5)
#define INT_EN_RX_UNDERRUN  (1 << 4)
#define INT_EN_TX_OVERRUN   (1 << 3)
#define INT_EN_TX_UNDERRUN  (1 << 2)
#define INT_EN_RX_FIFO_RDY  (1 << 1)
#define INT_EN_TX_FIFO_RDY  (1 << 0)

/* SPI status */
#define STATUS_TX_DONE          (1 << 25)
#define STATUS_TRAILING_BYTE    (1 << 24)
#define STATUS_RX_FIFO_LVL_SHF  (15)
#define STATUS_TX_FIFO_LVL_SHF  (6)
#define STATUS_RX_OVERRUN       (1 << 5)
#define STATUS_RX_UNDERRUN      (1 << 4)
#define STATUS_TX_OVERRUN       (1 << 3)
#define STATUS_TX_UNDERRUN      (1 << 2)
#define STATUS_RX_FIFO_RDY      (1 << 1)
#define STATUS_TX_FIFO_RDY      (1 << 0)

/* Packet count */
#define PACKET_CNT_EN         (1 << 16)
#define PACKET_CNT_VALUE_SHF  (1 << 0)

/* Interrupt pending clear */
#define PENDING_CLR_TX_UNDERRUN  (1 << 4)
#define PENDING_CLR_TX_OVERRUN   (1 << 3)
#define PENDING_CLR_RX_UNDERRUN  (1 << 2)
#define PENDING_CLR_RX_OVERRUN   (1 << 1)
#define PENDING_CLR_TRAILING     (1 << 0)

/* Swap configuration */
#define SWAP_CFG_RX_HWORD  (1 << 7)
#define SWAP_CFG_RX_BYTE   (1 << 6)
#define SWAP_CFG_RX_BIT    (1 << 5)
#define SWAP_CFG_RX_EN     (1 << 4)
#define SWAP_CFG_TX_HWORD  (1 << 3)
#define SWAP_CFG_TX_BYTE   (1 << 2)
#define SWAP_CFG_TX_BIT    (1 << 1)
#define SWAP_CFG_TX_EN     (1 << 0)

/* Feedback clock selection */
#define FB_CLK_SEL_SHF     (0)

#define D(args...) \
	do { \
		printf("%s(%d):", __func__, __LINE__); \
		printf(args); \
		printf("\n"); \
	} while(0);

enum spi_mode {
	MASTER_MODE = 0,
	SLAVE_MODE
};

struct spi_slave {
	int  id;                //Peripheral ID
	char name[16];          //Device name
	struct spi_slave *next; //Next device
};	

struct spi_device {
	unsigned char *base;
	int lock;
	int mode:1;               //0 -- Master, 1 -- Slave
	int high_speed:1;         //High speed operation in slave mode.
	int cs_auto:1;            //Auto chip selection.
	struct spi_slave *slaves; //Slaves' link list
};

static struct spi_device *spi_dev = NULL;

static void print_reg(void)
{
	printf("%x, %x, %x, %x, %x, %x, %x, %x, %x\n",
		readl(spi_dev->base + CH_CFG),
		readl(spi_dev->base + MODE_CFG),
		readl(spi_dev->base + CS_REG),
		readl(spi_dev->base + INT_EN),
		readl(spi_dev->base + STATUS),
		readl(spi_dev->base + PACKET_CNT_REG),
		readl(spi_dev->base + PENDING_CLR_REG),
		readl(spi_dev->base + SWAP_CFG),
		readl(spi_dev->base + FB_CLK_SEL));
}

/*
 * FIXME: Hack, should be done in the clock driver.
 */
static void spi_set_clk(void)
{
#define CLK_SRC_PERIC1       0x254
#define CLK_SRC_MASK_PERIC1  0x354
#define CLK_DIV_PERIC1       0x55C
#define CLK_DIV_STAT_PERIC1  0x65C
#define CLK_GATE_IP_PERIC    0x950

	unsigned char *base = (unsigned char*)clk_reg;

	/*
	 * FIXME: Set the clock to an appropriate freq.
	 * Freq: Min. 0xFFFF0000
	 *       Max. 0x1AF00000 (Max. of MCP2515 CAN controller)
	 */
	writel(0xFFFF0000, base + CLK_DIV_PERIC1);
//	printf("CLK: %s: %x, %x, %x, %x, %x\n", __func__,
//		readl(base + CLK_SRC_PERIC1),
//		readl(base + CLK_SRC_MASK_PERIC1),
//		readl(base + CLK_DIV_PERIC1),
//		readl(base + CLK_DIV_STAT_PERIC1),
//		readl(base + CLK_GATE_IP_PERIC));
}

static void spi_reset(struct spi_device *dev)
{
	uint32_t val;

	/* Turn off the channel */
	val = readl(dev->base + CH_CFG);
	val &= ~(CH_CFG_RX_CH_ON | CH_CFG_TX_CH_ON);
	writel(val, dev->base + CH_CFG);

	/* Write to reset bit */
	val = readl(dev->base + CH_CFG);
	writel(val | CH_CFG_SW_RST, dev->base);

	/* Clear reset bit */
	val = readl(dev->base + CH_CFG);
	writel(val & ~CH_CFG_SW_RST, dev->base);

	/* Turn on the channel */
	val = readl(dev->base + CH_CFG);
	val |= (CH_CFG_RX_CH_ON | CH_CFG_TX_CH_ON);
	writel(val, dev->base + CH_CFG);
}

static void spi_enable_interrupt(struct spi_device *dev)
{
	uint32_t val;

	val = (INT_EN_RX_FIFO_RDY | INT_EN_TX_FIFO_RDY);
	writel(val, dev->base + INT_EN);
}

static void spi_disable_interrupt(struct spi_device *dev)
{
	uint32_t val;

	val = readl(dev->base + INT_EN);
	val &= ~(INT_EN_RX_FIFO_RDY | INT_EN_TX_FIFO_RDY);
	writel(val, dev->base + INT_EN);
}

static void spi_config(struct spi_device *dev)
{
	uint32_t val;

	/* Step1: SPI configuration */
	val = readl(dev->base + CH_CFG);

	/* Set transfer type */
	val &= ~(CH_CFG_CPOL | CH_CFG_CPHA);

	/* Master/Slave mode */
	if (dev->mode == SLAVE_MODE) {
		val |= CH_CFG_SLAVE;

		/* High speed mode is slave mode only */
		if (dev->high_speed) {
			val |= CH_CFG_HIGH_SPEED_EN;
			val &= ~CH_CFG_CPHA;
		}
	}
	writel(val, dev->base + CH_CFG);

	/*
	 * Step2: Feedback clock
	 * The default clock is 33MHz, so we use a 180 degree phase feedback.
	 * The feedback clock only works in the master mode.
	 */
	if (dev->mode == MASTER_MODE) {
		writel((0x0 << FB_CLK_SEL_SHF), dev->base + FB_CLK_SEL);
	}

	/*
	 * Step3: FIFO control
	 * Channel width to Byte, No DMA, only need to set trigger level.
	 */
	val = (0x20 << MODE_CFG_RX_RDY_LVL_SHF) | (0x20 << MODE_CFG_TX_RDY_LVL_SHF);
	writel(val, dev->base + MODE_CFG);
	
	/* Step4: Interrupts */
	//spi_enable_interrupt(dev);

	/* Step5: Packet control */
	writel(0x0, dev->base + PACKET_CNT_REG);

	/* Step6: Turn on the channel */
	val = readl(dev->base + CH_CFG);
	val |= (CH_CFG_RX_CH_ON | CH_CFG_TX_CH_ON);
	writel(val, dev->base + CH_CFG);

	/* Step7: Chip selection */
	val = (0x0 << CS_REG_NCS_TIME_COUNT_SHF);
	if (dev->cs_auto) {
		val |= CS_REG_AUTO_N_MANUAL;
	} else {
		val |= CS_REG_NSSOUT;
	}
	writel(val, dev->base + CS_REG);
}

static void spi_drain(struct spi_device *dev)
{
	uint32_t val;

	/* Drain TxFIFO */
	do {
		val = readl(dev->base + STATUS);
	} while ((val >> STATUS_TX_FIFO_LVL_SHF) & 0x1FF);

	/* Wait until Tx finishes in master mode. */
	if (dev->mode == MASTER_MODE) {
		while (!(readl(dev->base + STATUS) & STATUS_TX_DONE)); 
	}
}

static void spi_flush(struct spi_device *dev)
{
	uint32_t val;

	writel(0, dev->base + PACKET_CNT_REG);

	/* Disable channel */
	val = readl(dev->base + CH_CFG);
	val &= ~(CH_CFG_RX_CH_ON | CH_CFG_TX_CH_ON);
	writel(val, dev->base + CH_CFG);

	/* Reset */
	val = readl(dev->base + CH_CFG);
	val |= CH_CFG_SW_RST;
	writel(val, dev->base + CH_CFG);

	/* Flush TxFIFO */
	do {
		val = readl(dev->base + STATUS);
	} while ((val >> STATUS_TX_FIFO_LVL_SHF) & 0x1FF);
	
	/* Flush RxFIFO */
	do {
		val = readl(dev->base + STATUS);
		if ((val >> STATUS_RX_FIFO_LVL_SHF) & 0x1FF) {
			readl(dev->base + RX_DATA);
		} else {
			break;
		}
	} while (1);

	/* Clear reset bit */
	val = readl(dev->base + CH_CFG);
	val &= ~CH_CFG_SW_RST;
	writel(val, dev->base + CH_CFG);

	/* Enable channel */
	val = readl(dev->base + CH_CFG);
	val |= (CH_CFG_RX_CH_ON | CH_CFG_TX_CH_ON);
	writel(val, dev->base + CH_CFG);
}

void spi_interrupt(void *device)
{
	uint32_t val = 0;
	struct spi_device *dev = (struct spi_device*)device;

	val = readl(dev->base + STATUS);

	spi_disable_interrupt(dev);
	spi1_int_reg_callback(&spi_interrupt, device);
}

void spi__init(void)
{
	spi_dev = (struct spi_device*)malloc(sizeof(struct spi_device));
	if (!spi_dev) {
		printf("Not enough memory.\n");
		return;
	}
	
	spi_dev->base = (unsigned char*)spi1_reg;
	spi_dev->lock = 0;
	spi_dev->mode = 0;
	spi_dev->high_speed = 0;
	spi_dev->cs_auto = 0;
	spi_dev->slaves = NULL;

	spi_set_clk();

	spi_reset(spi_dev);
	spi_config(spi_dev);

	spi1_int_reg_callback(&spi_interrupt, spi_dev);
}

void spi_chip_select(int id)
{
	uint32_t val;

	/* TODO: Add multiple slave support */
	gpio_pinmux_config(0, 1);

	if (!spi_dev->cs_auto) {
		val = readl(spi_dev->base + CS_REG);
		val &= ~CS_REG_NSSOUT;
		writel(val, spi_dev->base + CS_REG);
	}
}

void spi_chip_unselect(int id)
{
	uint32_t val;

	/* TODO: Add multiple slave support */
	gpio_pinmux_config(0, 0);

	if (!spi_dev->cs_auto) {
		val = readl(spi_dev->base + CS_REG);
		val |= CS_REG_NSSOUT;
		writel(val, spi_dev->base + CS_REG);
	}
}

int spi_transfer_byte(int id, char byte)
{
	uint32_t ret, val, lvl;

	/* Send out data */
	writel(byte, spi_dev->base + TX_DATA);
	spi_drain(spi_dev);

	/* Receive data */
	do {
		val = readl(spi_dev->base + STATUS);
		lvl = (val >> STATUS_RX_FIFO_LVL_SHF) & 0x1FF;
	} while (lvl < 1);
	ret = readl(spi_dev->base + RX_DATA);

	return ret;
}

/*
 * Backward compatibility for CAN driver.
 */
int spi_transfer(unsigned int id, unsigned int wcount, unsigned int rcount)
{
	uint32_t ret, val, lvl;
	spi_dev_port_p buf = (spi_dev_port_p)spi1_can;

	/* Send out data */
	for (int i = 0; i < wcount + rcount; i++) {
		if (i < wcount) {
			writel(buf->txbuf[i], spi_dev->base + TX_DATA);
		} else {
			writel(0x0, spi_dev->base + TX_DATA);
		}
		spi_drain(spi_dev);
	}

	/* Receive data */
	do {
		val = readl(spi_dev->base + STATUS);
		lvl = (val >> STATUS_RX_FIFO_LVL_SHF) & 0x1FF;
	} while (lvl < wcount + rcount);

	for (int i = 0; i < wcount + rcount; i++) {
		ret = readl(spi_dev->base + RX_DATA);
		buf->rxbuf[i] = ret;
	}

	return 0;
}

int spi_register_slave(char *name, int peripheral)
{
	struct spi_slave *slave = malloc(sizeof(struct spi_slave));
	if (!slave) {
		printf("Not enough memory!\n");
		return -1;
	}

	/* TODO: Check if the slave has been registered. */

	slave->id = peripheral;
	strcpy(slave->name, name);
	
	spinlock_lock(&spi_dev->lock);
	slave->next = spi_dev->slaves;
	spi_dev->slaves = slave;
	spinlock_unlock(&spi_dev->lock);

	printf("%s is registered as a SPI slave.\n", slave->name);

	return peripheral;
}

int spi_unregister_slave(int id)
{
	/* TODO: Implement this. */
	printf("%s(%d): Not implemented.\n", __func__, id);
	return -1;
}

/* Self test, enable control thread to run. */
int run(void)
{
	uint32_t ret;

	spi_chip_select(0);
	udelay(1);
	spi_transfer_byte(0, 0xC0);
	spi_chip_unselect(0);
	udelay(1);

	while(1) {
		spi_chip_select(0);
		udelay(1);
		spi_transfer_byte(0, 0x3);
		spi_transfer_byte(0, 0xE);
		ret = spi_transfer_byte(0, 0x0);
		spi_chip_unselect(0);
		udelay(1);
		printf("ret: %u\n", ret);
	}

	return 0;
} 
