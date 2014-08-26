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

extern unsigned char* clk_reg;

/**
 * Write to 32-bit register.
 */
inline void writel(uint32_t value, const void *address)
{
	//printf("writing %x @ %x\n", value, address);
	*((volatile uint32_t *)(address)) = value;
}

/*
 * FIXME: Hack, should be done in the clock driver.
 */
void spi_set_clk(void)
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


