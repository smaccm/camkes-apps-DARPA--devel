
/*
 * exynos_clk.c
 *
 *  Created on: Aug 5, 2013
 *      Modify by jxie
 */

/*
 * Clock setup for SMDK5250 board based on EXYNOS5
 *
 * Copyright (C) 2012 Samsung Electronics
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * clk.c
 *
 *  Created on: Jun 5, 2013
 *      Author: Jiawei Xie
 */


/*
 * CAmkES headers
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "clk.h"

#include "exynos_config.h"
#include "exynos_clk.h"
#include "periph.h"
#include "common.h"
#include "utils.h"



struct exynos5_clk_cmu_cpu	* 	cmu_cpu_clk_reg; //0x10010000...
struct exynos5_clk_cmu_top	*	cmu_top_clk_reg; //0x10020000...
struct exynos5_clk_cmu_core *	cmu_core_clk_reg;//0x10030000...



/* Epll Clock division values to achive different frequency output */
static struct set_epll_con_val exynos5_epll_div[] = {
	{ 192000000, 0, 48, 3, 1, 0 },
	{ 180000000, 0, 45, 3, 1, 0 },
	{  73728000, 1, 73, 3, 3, 47710 },
	{  67737600, 1, 90, 4, 3, 20762 },
	{  49152000, 0, 49, 3, 3, 9961 },
	{  45158400, 0, 45, 3, 3, 10381 },
	{ 180633600, 0, 45, 3, 1, 10381 }
};


#define PLL_DIV_1024	1024
#define PLL_DIV_65535	65535
#define PLL_DIV_65536	65536

/* *
 * This structure is to store the src bit, div bit and prediv bit
 * positions of the peripheral clocks of the src and div registers
 */
struct clk_bit_info {
	int8_t src_bit;
	int8_t div_bit;
	int8_t prediv_bit;
};

/* src_bit div_bit prediv_bit */
static struct clk_bit_info clk_bit_info[PERIPH_ID_COUNT] = {
	{0,		0,	-1},
	{4,		4,	-1},
	{8,		8,	-1},
	{12,	12,	-1},
	{0,		0,	8},
	{4,		16,	24},
	{8,		0,	8},
	{12,	16,	24},
	{-1,	-1,	-1},
	{16,	0,	8},
	{20,	16,	24},
	{24,	0,	8},
	{0,		0,	4},
	{4,		12,	16},
	{-1,	-1,	-1},
	{-1,	-1,	-1},
	{-1,	24,	0},
	{-1,	24,	0},
	{-1,	24,	0},
	{-1,	24,	0},
	{-1,	24,	0},
	{-1,	24,	0},
	{-1,	24,	0},
	{-1,	24,	0},
	{24,	0,	-1},
	{24,	0,	-1},
	{24,	0,	-1},
	{24,	0,	-1},
	{24,	0,	-1},
};


struct arm_clk_ratios arm_clk_ratios[] = {
	{
		.arm_freq_mhz = 600,

		.apll_mdiv = 0xc8,
		.apll_pdiv = 0x4,
		.apll_sdiv = 0x1,

		.arm2_ratio = 0x0,
		.apll_ratio = 0x1,
		.pclk_dbg_ratio = 0x1,
		.atb_ratio = 0x2,
		.periph_ratio = 0x7,
		.acp_ratio = 0x7,
		.cpud_ratio = 0x1,
		.arm_ratio = 0x0,
	}, {
		.arm_freq_mhz = 800,

		.apll_mdiv = 0x64,
		.apll_pdiv = 0x3,
		.apll_sdiv = 0x0,

		.arm2_ratio = 0x0,
		.apll_ratio = 0x1,
		.pclk_dbg_ratio = 0x1,
		.atb_ratio = 0x3,
		.periph_ratio = 0x7,
		.acp_ratio = 0x7,
		.cpud_ratio = 0x2,
		.arm_ratio = 0x0,
	}, {
		.arm_freq_mhz = 1000,

		.apll_mdiv = 0x7d,
		.apll_pdiv = 0x3,
		.apll_sdiv = 0x0,

		.arm2_ratio = 0x0,
		.apll_ratio = 0x1,
		.pclk_dbg_ratio = 0x1,
		.atb_ratio = 0x4,
		.periph_ratio = 0x7,
		.acp_ratio = 0x7,
		.cpud_ratio = 0x2,
		.arm_ratio = 0x0,
	}, {
		.arm_freq_mhz = 1200,

		.apll_mdiv = 0x96,
		.apll_pdiv = 0x3,
		.apll_sdiv = 0x0,

		.arm2_ratio = 0x0,
		.apll_ratio = 0x3,
		.pclk_dbg_ratio = 0x1,
		.atb_ratio = 0x5,
		.periph_ratio = 0x7,
		.acp_ratio = 0x7,
		.cpud_ratio = 0x3,
		.arm_ratio = 0x0,
	}, {
		.arm_freq_mhz = 1400,

		.apll_mdiv = 0xaf,
		.apll_pdiv = 0x3,
		.apll_sdiv = 0x0,

		.arm2_ratio = 0x0,
		.apll_ratio = 0x3,
		.pclk_dbg_ratio = 0x1,
		.atb_ratio = 0x6,
		.periph_ratio = 0x7,
		.acp_ratio = 0x7,
		.cpud_ratio = 0x3,
		.arm_ratio = 0x0,
	}, {
		.arm_freq_mhz = 1700,

		.apll_mdiv = 0x1a9,
		.apll_pdiv = 0x6,
		.apll_sdiv = 0x0,

		.arm2_ratio = 0x0,
		.apll_ratio = 0x3,
		.pclk_dbg_ratio = 0x1,
		.atb_ratio = 0x6,
		.periph_ratio = 0x7,
		.acp_ratio = 0x7,
		.cpud_ratio = 0x3,
		.arm_ratio = 0x0,
	}
};
struct mem_timings mem_timings = {
		.mem_manuf = MEM_MANUF_SAMSUNG,
		.mem_type = DDR_MODE_DDR3,
		.frequency_mhz = 800,
		.mpll_mdiv = 0xc8,
		.mpll_pdiv = 0x3,
		.mpll_sdiv = 0x0,
		.cpll_mdiv = 0xde,
		.cpll_pdiv = 0x4,
		.cpll_sdiv = 0x2,
		.gpll_mdiv = 0x215,
		.gpll_pdiv = 0xc,
		.gpll_sdiv = 0x1,
		.epll_mdiv = 0x60,
		.epll_pdiv = 0x3,
		.epll_sdiv = 0x3,
		.vpll_mdiv = 0x96,
		.vpll_pdiv = 0x3,
		.vpll_sdiv = 0x2,

		.bpll_mdiv = 0x64,
		.bpll_pdiv = 0x3,
		.bpll_sdiv = 0x0,
		.pclk_cdrex_ratio = 0x5,
		.direct_cmd_msr = {
			0x00020018, 0x00030000, 0x00010000, 0x00000d70
		},
		.timing_ref = 0x000000bb,
		.timing_row = 0x8c36650e,
		.timing_data = 0x3630580b,
		.timing_power = 0x41000a44,
		.phy0_dqs = 0x08080808,
		.phy1_dqs = 0x08080808,
		.phy0_dq = 0x08080808,
		.phy1_dq = 0x08080808,
		.phy0_tFS = 0x8,
		.phy1_tFS = 0x8,
		.phy0_pulld_dqs = 0xf,
		.phy1_pulld_dqs = 0xf,

		.lpddr3_ctrl_phy_reset = 0x1,
		.ctrl_start_point = 0x10,
		.ctrl_inc = 0x10,
		.ctrl_start = 0x1,
		.ctrl_dll_on = 0x1,
		.ctrl_ref = 0x8,

		.ctrl_force = 0x1a,
		.ctrl_rdlat = 0x0b,
		.ctrl_bstlen = 0x08,

		.fp_resync = 0x8,
		.iv_size = 0x7,
		.dfi_init_start = 1,
		.aref_en = 1,

		.rd_fetch = 0x3,

		.zq_mode_dds = 0x5,
		.zq_mode_term = 0x1,
		.zq_mode_noterm = 1,

		/*
		* Dynamic Clock: Always Running
		* Memory Burst length: 8
		* Number of chips: 1
		* Memory Bus width: 32 bit
		* Memory Type: DDR3
		* Additional Latancy for PLL: 0 Cycle
		*/
		.memcontrol = DMC_MEMCONTROL_CLK_STOP_DISABLE |
			DMC_MEMCONTROL_DPWRDN_DISABLE |
			DMC_MEMCONTROL_DPWRDN_ACTIVE_PRECHARGE |
			DMC_MEMCONTROL_TP_DISABLE |
			DMC_MEMCONTROL_DSREF_ENABLE |
			DMC_MEMCONTROL_ADD_LAT_PALL_CYCLE(0) |
			DMC_MEMCONTROL_MEM_TYPE_DDR3 |
			DMC_MEMCONTROL_MEM_WIDTH_32BIT |
			DMC_MEMCONTROL_NUM_CHIP_1 |
			DMC_MEMCONTROL_BL_8 |
			DMC_MEMCONTROL_PZQ_DISABLE |
			DMC_MEMCONTROL_MRR_BYTE_7_0,
		.memconfig = DMC_MEMCONFIGX_CHIP_MAP_INTERLEAVED |
			DMC_MEMCONFIGX_CHIP_COL_10 |
			DMC_MEMCONFIGX_CHIP_ROW_15 |
			DMC_MEMCONFIGX_CHIP_BANK_8,
		.membaseconfig0 = DMC_MEMBASECONFIG_VAL(0x40),
		.membaseconfig1 = DMC_MEMBASECONFIG_VAL(0x80),
		.prechconfig_tp_cnt = 0xff,
		.dpwrdn_cyc = 0xff,
		.dsref_cyc = 0xffff,
		.concontrol = DMC_CONCONTROL_DFI_INIT_START_DISABLE |
			DMC_CONCONTROL_TIMEOUT_LEVEL0 |
			DMC_CONCONTROL_RD_FETCH_DISABLE |
			DMC_CONCONTROL_EMPTY_DISABLE |
			DMC_CONCONTROL_AREF_EN_DISABLE |
			DMC_CONCONTROL_IO_PD_CON_DISABLE,
		.dmc_channels = 2,
		.chips_per_channel = 2,
		.chips_to_configure = 1,
		.send_zq_init = 1,
		.impedance = IMP_OUTPUT_DRV_40_OHM,
		.gate_leveling_enable = 1,
};

static int CLK_HAS_INIT = 0;

static void wait_for_init(void){
	while(!CLK_HAS_INIT){
		;
	}
}

void clk__init(void){

	cmu_cpu_clk_reg  = (struct exynos5_clk_cmu_cpu	*) cmu_cpu_clk;
	cmu_top_clk_reg  = (struct exynos5_clk_cmu_top	*) cmu_top_clk;
	cmu_core_clk_reg = (struct exynos5_clk_cmu_core	*) cmu_core_clk;

	writel(CLK_DIV_PERIC3_VAL, &cmu_top_clk_reg->div_peric3);

	CLK_HAS_INIT = 1;

//#ifdef PRINTF_ENABLE
	printf("&cpu_clk = %x \n", cmu_cpu_clk_reg);
	printf("&top_clk = %x \n", cmu_top_clk_reg);
	printf("&core_clk = %x \n", cmu_core_clk_reg);
	//setclk();
	//system_clock_init();
	printf("ARM clk: %ld MHz\n", exynos5_get_arm_clk()/1000000);
	printf("APLL clk: %ld MHz\n", exynos5_get_pll_clk(APLL)/1000000);
//	printf("MPLL clk: %ld MHz\n", exynos5_get_pll_clk(MPLL)/1000000);
//	printf("EPLL clk: %ld MHz\n", exynos5_get_pll_clk(EPLL)/1000000);
//	printf("VPLL clk: %ld MHz\n", exynos5_get_pll_clk(VPLL)/1000000);
//	printf("BPLL clk: %ld MHz\n", exynos5_get_pll_clk(BPLL)/1000000);
	printf("UART0 rate: %ld MHz\n",exynos5_get_periph_rate(PERIPH_ID_UART0)/1000000);
	exynos5_set_spi_clk(PERIPH_ID_SPI1, 10000000);
	printf("SPI1 rate: %ld MHz\n",exynos5_get_periph_rate(PERIPH_ID_SPI1)/1000000);
	//
	printf("PWM0 rate: %ld MHz\n",exynos5_get_periph_rate(PERIPH_ID_PWM0)/1000000);
//#endif

}


unsigned long clock_get_periph_rate(int peripheral)
{
	return exynos5_get_periph_rate(peripheral);
}


unsigned long get_pll_clk(int pllreg)
{
	return exynos5_get_pll_clk(pllreg);
}

unsigned long get_arm_clk(void)
{
	return exynos5_get_arm_clk();
}

unsigned long clk_get_i2c_freq(void)
{
	return exynos5_get_i2c_clk();
}

void set_lcd_clk(void)
{
	exynos5_set_lcd_clk();
}

int set_spi_clk(int periph_id, unsigned int rate)
{
	return exynos5_set_spi_clk(periph_id, rate);
}

int set_i2s_clk_prescaler(unsigned int src_frq, unsigned int dst_frq)
{
	return exynos5_set_i2s_clk_prescaler(src_frq, dst_frq);
}

void set_i2s_clk_source(void)
{
	exynos5_set_i2s_clk_source();
}

int set_epll_clk(unsigned long rate)
{
	return exynos5_set_epll_clk(rate);
}


void clock_init_dp_clock(void)
{
	/* DP clock enable */
	setbits_le32(&cmu_top_clk_reg->gate_ip_disp1, CLK_GATE_DP1_ALLOW);
	/* We run DP at 267 Mhz */
	setbits_le32(&cmu_top_clk_reg->div_disp1_0, CLK_DIV_DISP1_0_FIMD1);
}


/* exynos: return pll clock frequency */
int exynos_get_pll_clk(int pllreg, unsigned int r, unsigned int k)
{
	unsigned long m, p, s = 0, mask, fout;
	unsigned int div;
	unsigned int freq;
	/*
	 * APLL_CON: MIDV [25:16]
	 * MPLL_CON: MIDV [25:16]
	 * EPLL_CON: MIDV [24:16]
	 * VPLL_CON: MIDV [24:16]
	 * BPLL_CON: MIDV [25:16]: Exynos5
	 */
	if (pllreg == APLL || pllreg == MPLL || pllreg == BPLL)
		mask = 0x3ff;
	else
		mask = 0x1ff;

	m = (r >> 16) & mask;

	/* PDIV [13:8] */
	p = (r >> 8) & 0x3f;
	/* SDIV [2:0] */
	s = r & 0x7;

	freq = CONFIG_SYS_CLK_FREQ;

	if (pllreg == EPLL) {
		k = k & 0xffff;
		/* FOUT = (MDIV + K / 65536) * FIN / (PDIV * 2^SDIV) */
		fout = (m + k / PLL_DIV_65536) * (freq / (p * (1 << s)));
	} else if (pllreg == VPLL) {
		k = k & 0xfff;

		/*
		 * Exynos5250
		 * FOUT = (MDIV + K / 65536) * FIN / (PDIV * 2^SDIV)
		 */
		div = PLL_DIV_65536; //0xffff
		fout = (m + k / div) * (freq / (p * (1 << s)));

	} else {
		/*
		 * Exynos5250
		 * FOUT = MDIV * FIN / (PDIV * 2^(SDIV-1))
		 */
		fout = m * (freq / (p * (1 << (s))));
	}
	//printf("m %ld, p %ld, s %ld, k %ld, fout: %ld\n",m,p,s,k, fout);
	return fout;
}


/* exynos5: return pll clock frequency */
unsigned long exynos5_get_pll_clk(int pllreg)
{
	unsigned long r, k = 0, fout;
	unsigned int pll_div2_sel, fout_sel;

	switch (pllreg) {
	case APLL:
		r = readl(&cmu_cpu_clk_reg->apll_con0);
		break;
	case MPLL:
		r = readl(&cmu_cpu_clk_reg->mpll_con0);
		break;
	case EPLL:
		r = readl(&cmu_top_clk_reg->epll_con0);
		k = readl(&cmu_top_clk_reg->epll_con1);
		break;
	case VPLL:
		r = readl(&cmu_top_clk_reg->vpll_con0);
		k = readl(&cmu_top_clk_reg->vpll_con1);
		break;
	case BPLL:
		r = readl(&cmu_core_clk_reg->bpll_con0);
		break;
	default:
		printf("Unsupported PLL (%d)\n", pllreg);
		return 0;
	}

	fout = exynos_get_pll_clk(pllreg, r, k);

	/* According to the user manual, in EVT1 MPLL and BPLL always gives
	 * 1.6GHz clock, so divide by 2 to get 800MHz MPLL clock.*/
	if (pllreg == MPLL || pllreg == BPLL) {
		pll_div2_sel = readl(&cmu_core_clk_reg->pll_div2_sel);

		switch (pllreg) {
		case MPLL:
			fout_sel = (pll_div2_sel >> MPLL_FOUT_SEL_SHIFT)
					& MPLL_FOUT_SEL_MASK;
			break;
		case BPLL:
			fout_sel = (pll_div2_sel >> BPLL_FOUT_SEL_SHIFT)
					& BPLL_FOUT_SEL_MASK;
			break;
		default:
			fout_sel = -1;
			break;
		}

		if (fout_sel == 0)
			fout /= 2;
	}

	return fout;
}

unsigned long exynos5_get_periph_rate(int peripheral)
{
	struct clk_bit_info *bit_info = &clk_bit_info[peripheral];
	unsigned long sclk, sub_clk;
	unsigned int src, div, sub_div;

	switch (peripheral) {
	case PERIPH_ID_UART0:
	case PERIPH_ID_UART1:
	case PERIPH_ID_UART2:
	case PERIPH_ID_UART3:
		src = readl(&cmu_top_clk_reg->src_peric0);
		div = readl(&cmu_top_clk_reg->div_peric0);
		break;
	case PERIPH_ID_PWM0:
	case PERIPH_ID_PWM1:
	case PERIPH_ID_PWM2:
	case PERIPH_ID_PWM3:
	case PERIPH_ID_PWM4:
		src = readl(&cmu_top_clk_reg->src_peric0);
		div = readl(&cmu_top_clk_reg->div_peric3);
		break;
	case PERIPH_ID_SPI0:
	case PERIPH_ID_SPI1:
		src = readl(&cmu_top_clk_reg->src_peric1);
		div = readl(&cmu_top_clk_reg->div_peric1);
		break;
	case PERIPH_ID_SPI2:
		src = readl(&cmu_top_clk_reg->src_peric1);
		div = readl(&cmu_top_clk_reg->div_peric2);
		break;
	case PERIPH_ID_SPI3:
	case PERIPH_ID_SPI4:
		src = readl(&cmu_top_clk_reg->sclk_src_isp);
		div = readl(&cmu_top_clk_reg->sclk_div_isp);
		break;
	case PERIPH_ID_SDMMC0:
	case PERIPH_ID_SDMMC1:
	case PERIPH_ID_SDMMC2:
	case PERIPH_ID_SDMMC3:
		src = readl(&cmu_top_clk_reg->src_fsys);
		div = readl(&cmu_top_clk_reg->div_fsys1);
		break;
	case PERIPH_ID_I2C0:
	case PERIPH_ID_I2C1:
	case PERIPH_ID_I2C2:
	case PERIPH_ID_I2C3:
	case PERIPH_ID_I2C4:
	case PERIPH_ID_I2C5:
	case PERIPH_ID_I2C6:
	case PERIPH_ID_I2C7:
		sclk = exynos5_get_pll_clk(MPLL);
		sub_div = ((readl(&cmu_top_clk_reg->div_top1) >> bit_info->div_bit) & 0x7) + 1;
		div = ((readl(&cmu_top_clk_reg->div_top0) >> bit_info->prediv_bit) & 0x7) + 1;
		return (sclk / sub_div) / div;
	default:
		printf("invalid peripheral: %d\n", peripheral);
		return -1;
	};

	src = (src >> bit_info->src_bit) & 0xf;

	switch (src) {
	case EXYNOS_SRC_MPLL:
		sclk = exynos5_get_pll_clk(MPLL);
		break;
	case EXYNOS_SRC_EPLL:
		sclk = exynos5_get_pll_clk(EPLL);
		break;
	case EXYNOS_SRC_VPLL:
		sclk = exynos5_get_pll_clk(VPLL);
		break;
	default:
		return 0;
	}

	/* Ratio clock division for this peripheral */
	sub_div = (div >> bit_info->div_bit) & 0xf;
	sub_clk = sclk / (sub_div + 1);

	/* Pre-ratio clock division for SDMMC0 and 2 */
	if (peripheral == PERIPH_ID_SDMMC0 || peripheral == PERIPH_ID_SDMMC2) {
		div = (div >> bit_info->prediv_bit) & 0xff;
		return sub_clk / (div + 1);
	}

	return sub_clk;
}


/* exynos5: return ARM clock frequency */
unsigned long exynos5_get_arm_clk(void)
{
	unsigned long div;
	unsigned long armclk;
	unsigned int arm_ratio;
	unsigned int arm2_ratio;

	div = readl(&cmu_cpu_clk_reg->div_cpu0);

	/* ARM_RATIO: [2:0], ARM2_RATIO: [30:28] */
	arm_ratio = (div >> 0) & 0x7;
	arm2_ratio = (div >> 28) & 0x7;

	armclk = get_pll_clk(APLL) / (arm_ratio + 1);
	armclk /= (arm2_ratio + 1);

	return armclk;
}

/* exynos5: return uart clock frequency */
unsigned long exynos5_get_uart_clk(int dev_index)
{
	unsigned long uclk, sclk;
	unsigned int sel;
	unsigned int ratio;

	/*
	 * CLK_SRC_PERIC0
	 * UART0_SEL [3:0]
	 * UART1_SEL [7:4]
	 * UART2_SEL [8:11]
	 * UART3_SEL [12:15]
	 * UART4_SEL [16:19]
	 * UART5_SEL [23:20]
	 */
	sel = readl(&cmu_top_clk_reg->src_peric0);
	sel = (sel >> (dev_index << 2)) & 0xf;

	if (sel == 0x6)
		sclk = get_pll_clk(MPLL);
	else if (sel == 0x7)
		sclk = get_pll_clk(EPLL);
	else if (sel == 0x8)
		sclk = get_pll_clk(VPLL);
	else
		return 0;

	/*
	 * CLK_DIV_PERIC0
	 * UART0_RATIO [3:0]
	 * UART1_RATIO [7:4]
	 * UART2_RATIO [8:11]
	 * UART3_RATIO [12:15]
	 * UART4_RATIO [16:19]
	 * UART5_RATIO [23:20]
	 */
	ratio = readl(&cmu_top_clk_reg->div_peric0);
	ratio = (ratio >> (dev_index << 2)) & 0xf;

	uclk = sclk / (ratio + 1);

	return uclk;
}

unsigned long exynos5_get_mmc_clk(int dev_index)
{
	unsigned long uclk, sclk;
	unsigned int sel, ratio, pre_ratio;
	int shift = 0;

	sel = readl(&cmu_top_clk_reg->src_fsys);
	sel = (sel >> (dev_index << 2)) & 0xf;

	if (sel == 0x6)
		sclk = get_pll_clk(MPLL);
	else if (sel == 0x7)
		sclk = get_pll_clk(EPLL);
	else if (sel == 0x8)
		sclk = get_pll_clk(VPLL);
	else
		return 0;

	switch (dev_index) {
	case 0:
	case 1:
		ratio = readl(&cmu_top_clk_reg->div_fsys1);
		pre_ratio = readl(&cmu_top_clk_reg->div_fsys1);
		break;
	case 2:
	case 3:
		ratio = readl(&cmu_top_clk_reg->div_fsys2);
		pre_ratio = readl(&cmu_top_clk_reg->div_fsys2);
		break;
	default:
		return 0;
	}

	if (dev_index == 1 || dev_index == 3)
		shift = 16;

	ratio = (ratio >> shift) & 0xf;
	pre_ratio = (pre_ratio >> (shift + 8)) & 0xff;
	uclk = (sclk / (ratio + 1)) / (pre_ratio + 1);

	return uclk;
}

/* exynos5: set the mmc clock */
void exynos5_set_mmc_clk(int dev_index, unsigned int div)
{
	unsigned int addr;
	unsigned int val;

	/*
	 * CLK_DIV_FSYS1
	 * MMC0_PRE_RATIO [15:8], MMC1_PRE_RATIO [31:24]
	 * CLK_DIV_FSYS2
	 * MMC2_PRE_RATIO [15:8], MMC3_PRE_RATIO [31:24]
	 */
	if (dev_index < 2) {
		addr = (unsigned int)&cmu_top_clk_reg->div_fsys1;
	} else {
		addr = (unsigned int)&cmu_top_clk_reg->div_fsys2;
		dev_index -= 2;
	}

	val = readl((void*)addr);
	val &= ~(0xff << ((dev_index << 4) + 8));
	val |= (div & 0xff) << ((dev_index << 4) + 8);
	writel(val, (const uint32_t *)addr);
}


void exynos5_set_lcd_clk(void)
{
	unsigned int cfg = 0;

	/*
	 * CLK_GATE_BLOCK
	 * CLK_CAM	[0]
	 * CLK_TV	[1]
	 * CLK_MFC	[2]
	 * CLK_G3D	[3]
	 * CLK_LCD0	[4]
	 * CLK_LCD1	[5]
	 * CLK_GPS	[7]
	 */
	cfg = readl(&cmu_top_clk_reg->gate_block);
	cfg |= 1 << 4;
	writel(cfg, &cmu_top_clk_reg->gate_block);

	/*
	 * CLK_SRC_LCD0
	 * FIMD0_SEL		[3:0]
	 * MDNIE0_SEL		[7:4]
	 * MDNIE_PWM0_SEL	[8:11]
	 * MIPI0_SEL		[12:15]
	 * set lcd0 src clock 0x6: SCLK_MPLL
	 */
	cfg = readl(&cmu_top_clk_reg->src_disp1_0);
	cfg &= ~(0xf);
	cfg |= 0x6;
	writel(cfg, &cmu_top_clk_reg->src_disp1_0);

	/*
	 * CLK_GATE_IP_LCD0
	 * CLK_FIMD0		[0]
	 * CLK_MIE0		[1]
	 * CLK_MDNIE0		[2]
	 * CLK_DSIM0		[3]
	 * CLK_SMMUFIMD0	[4]
	 * CLK_PPMULCD0		[5]
	 * Gating all clocks for FIMD0
	 */
	cfg = readl(&cmu_top_clk_reg->gate_ip_disp1);
	cfg |= 1 << 0;
	writel(cfg, &cmu_top_clk_reg->gate_ip_disp1);

	/*
	 * CLK_DIV_LCD0
	 * FIMD0_RATIO		[3:0]
	 * MDNIE0_RATIO		[7:4]
	 * MDNIE_PWM0_RATIO	[11:8]
	 * MDNIE_PWM_PRE_RATIO	[15:12]
	 * MIPI0_RATIO		[19:16]
	 * MIPI0_PRE_RATIO	[23:20]
	 * set fimd ratio
	 */
	cfg &= ~(0xf);
	cfg |= 0x0;
	writel(cfg, &cmu_top_clk_reg->div_disp1_0);
}


/*
 * I2C
 *
 * exynos5: obtaining the I2C clock
 */
unsigned long exynos5_get_i2c_clk(void)
{
	unsigned long aclk_66, aclk_66_pre, sclk;
	unsigned int ratio;

	sclk = get_pll_clk(MPLL);

	ratio = (readl(&cmu_top_clk_reg->div_top1)) >> 24;
	ratio &= 0x7;
	aclk_66_pre = sclk / (ratio + 1);
	ratio = readl(&cmu_top_clk_reg->div_top0);
	ratio &= 0x7;
	aclk_66 = aclk_66_pre / (ratio + 1);
	return aclk_66;
}

unsigned long get_timer(unsigned long x){
	return 0;
}


int exynos5_set_epll_clk(unsigned long rate)
{
	unsigned int epll_con, epll_con_k;
	unsigned int i;
	unsigned int lockcnt;
	unsigned int start;

	epll_con = readl(&cmu_top_clk_reg->epll_con0);
	epll_con &= ~((EPLL_CON0_LOCK_DET_EN_MASK <<
			EPLL_CON0_LOCK_DET_EN_SHIFT) |
		EPLL_CON0_MDIV_MASK << EPLL_CON0_MDIV_SHIFT |
		EPLL_CON0_PDIV_MASK << EPLL_CON0_PDIV_SHIFT |
		EPLL_CON0_SDIV_MASK << EPLL_CON0_SDIV_SHIFT);

	for (i = 0; i < ARRAY_SIZE(exynos5_epll_div); i++) {
		if (exynos5_epll_div[i].freq_out == rate)
			break;
	}

	if (i == ARRAY_SIZE(exynos5_epll_div))
		return -1;

	epll_con_k = exynos5_epll_div[i].k_dsm << 0;
	epll_con |= exynos5_epll_div[i].en_lock_det <<
				EPLL_CON0_LOCK_DET_EN_SHIFT;
	epll_con |= exynos5_epll_div[i].m_div << EPLL_CON0_MDIV_SHIFT;
	epll_con |= exynos5_epll_div[i].p_div << EPLL_CON0_PDIV_SHIFT;
	epll_con |= exynos5_epll_div[i].s_div << EPLL_CON0_SDIV_SHIFT;

	/*
	 * Required period ( in cycles) to genarate a stable clock output.
	 * The maximum clock time can be up to 3000 * PDIV cycles of PLLs
	 * frequency input (as per spec)
	 */
	lockcnt = 3000 * exynos5_epll_div[i].p_div;

	writel(lockcnt, &cmu_top_clk_reg->epll_lock);
	writel(epll_con, &cmu_top_clk_reg->epll_con0);
	writel(epll_con_k, &cmu_top_clk_reg->epll_con1);

	start = get_timer(0);

	 while (!(readl(&cmu_top_clk_reg->epll_con0) &
			(0x1 << EXYNOS5_EPLLCON0_LOCKED_SHIFT))) {
		if (get_timer(start) > TIMEOUT_EPLL_LOCK) {
			printf("Timeout waiting for EPLL lock\n");
			return -1;
		}
	}
	return 0;
}


int exynos5_set_spi_clk(enum periph_id periph_id, unsigned int rate)
{
	int main;
	unsigned int fine;
	unsigned shift, pre_shift;
	unsigned mask = 0xff;
	unsigned int *reg;

	main = clock_calc_best_scalar(4, 8, 400000000, rate, &fine);
	if (main < 0) {
		printf("Cannot set clock rate for peripheral %d", periph_id);
		return -1;
	}
	main = main - 1;
	fine = fine - 1;

	switch (periph_id) {
	case PERIPH_ID_SPI0:
		reg = &cmu_top_clk_reg->div_peric1;
		shift = 0;
		pre_shift = 8;
		break;
	case PERIPH_ID_SPI1:
		reg = &cmu_top_clk_reg->div_peric1;
		shift = 16;
		pre_shift = 24;
		break;
	case PERIPH_ID_SPI2:
		reg = &cmu_top_clk_reg->div_peric2;
		shift = 0;
		pre_shift = 8;
		break;
	case PERIPH_ID_SPI3:
		reg = &cmu_top_clk_reg->sclk_div_isp;
		shift = 0;
		pre_shift = 4;
		break;
	case PERIPH_ID_SPI4:
		reg = &cmu_top_clk_reg->sclk_div_isp;
		shift = 12;
		pre_shift = 16;
		break;
	default:
		printf("Unsupported peripheral ID %d\n", periph_id);
		return -1;
	}
	clrsetbits_le32(reg, mask << shift, (main & mask) << shift);
	clrsetbits_le32(reg, mask << pre_shift, (fine & mask) << pre_shift);

	return 0;
}


void exynos5_set_i2s_clk_source(void)
{
	clrsetbits_le32(&cmu_top_clk_reg->src_peric1, AUDIO1_SEL_MASK,
			(CLK_SRC_SCLK_EPLL));
}

int exynos5_set_i2s_clk_prescaler(unsigned int src_frq,
					unsigned int dst_frq)
{
	unsigned int div;

	if ((dst_frq == 0) || (src_frq == 0)) {
		printf("Invalid input for prescaler, \
				src = %d des = %d ", src_frq, dst_frq);
		return -1;
	}

	div = (src_frq / dst_frq);
	if (div > AUDIO_1_RATIO_MASK) {
		printf("Frequency ratio is out of range	\
				src = %d des = %d ", src_frq, dst_frq);
		return -1;
	}
	clrsetbits_le32(&cmu_top_clk_reg->div_peric4, AUDIO_1_RATIO_MASK,
				(div & AUDIO_1_RATIO_MASK));
	return 0;
}


/**
 * Linearly searches for the most accurate main and fine stage clock scalars
 * (divisors) for a specified target frequency and scalar bit sizes by checking
 * all multiples of main_scalar_bits values. Will always return scalars up to or
 * slower than target.
 *
 * @param main_scalar_bits	Number of main scalar bits, must be > 0 and < 32
 * @param fine_scalar_bits	Number of fine scalar bits, must be > 0 and < 32
 * @param input_freq		Clock frequency to be scaled in Hz
 * @param target_freq		Desired clock frequency in Hz
 * @param best_fine_scalar	Pointer to store the fine stage divisor
 *
 * @return best_main_scalar	Main scalar for desired frequency or -1 if none
 * found
 */
int clock_calc_best_scalar(unsigned int main_scaler_bits,
	unsigned int fine_scalar_bits, unsigned int input_rate,
	unsigned int target_rate, unsigned int *best_fine_scalar)
{
	int i;
	int best_main_scalar = -1;
	unsigned int best_error = target_rate;
	const unsigned int cap = (1 << fine_scalar_bits) - 1;
	const unsigned int loops = 1 << main_scaler_bits;

	printf("Input Rate is %u, Target is %u, Cap is %u\n", input_rate,
			target_rate, cap);

	assert(best_fine_scalar != NULL);
	assert(main_scaler_bits <= fine_scalar_bits);

	*best_fine_scalar = 1;

	if (input_rate == 0 || target_rate == 0)
		return -1;

	if (target_rate >= input_rate)
		return 1;

	for (i = 1; i <= loops; i++) {
		const unsigned int effective_div = max(min(input_rate / i /
							target_rate, cap), 1);
		const unsigned int effective_rate = input_rate / i /
							effective_div;
		const int error = target_rate - effective_rate;

		printf("%d|effdiv:%u, effrate:%u, error:%d\n", i, effective_div,
				effective_rate, error);

		if (error >= 0 && error <= best_error) {
			best_error = error;
			best_main_scalar = i;
			*best_fine_scalar = effective_div;
		}
	}

	return best_main_scalar;
}



void clk_dummy(void){

}

struct mem_timings * clock_get_mem_timings(void){
	return &mem_timings;
}

struct arm_clk_ratios * get_arm_ratios(void){
	return &arm_clk_ratios[5];
}

void system_clock_init(void)
{
	struct mem_timings *mem;
	struct arm_clk_ratios *arm_clk_ratio;
	unsigned int val, tmp;

	mem = clock_get_mem_timings();
	arm_clk_ratio = get_arm_ratios();

	/* Deselect all PLL clock source *
	 * MUX_APLL, MUX_MPLL, MUX_BPLL
	 * MUX_GPLL, MUX_VPLL, MUX_CPLL,
	 * MUX_EPLL
	 * */
	/* MUX_APLL */
	printf("1\n");
/*	clrbits_le32(&cmu_cpu_clk_reg->src_cpu, MUX_APLL_SEL_MASK);
	do {
		val = readl(&cmu_cpu_clk_reg->mux_stat_cpu);
	} while ((val | MUX_APLL_SEL_MASK) != val);

	/*	printf("2, accessing %x\n",&cmu_cpu_clk_reg->src_core1);

	/* MUX_MPLL */
	//clrbits_le32(&cmu_cpu_clk_reg->src_core1, MUX_MPLL_SEL_MASK);
/*	printf("running\n");
	do {
		val = readl(&cmu_cpu_clk_reg->mux_stat_core1);
		printf("val %x\n",val);
	} while ((val | MUX_MPLL_SEL_MASK) != val);
*/
	printf("3\n");
	/* MUX_CPLL */
	/*	clrbits_le32(&cmu_top_clk_reg->src_top2, MUX_CPLL_SEL_MASK);
	/* MUX_EPLL */
	/*	clrbits_le32(&cmu_top_clk_reg->src_top2, MUX_EPLL_SEL_MASK);
	/* MUX_VPLL */
	/*	clrbits_le32(&cmu_top_clk_reg->src_top2, MUX_VPLL_SEL_MASK);
	/* MUX_GPLL */
	/*	clrbits_le32(&cmu_top_clk_reg->src_top2, MUX_GPLL_SEL_MASK);
	tmp = MUX_CPLL_SEL_MASK | MUX_EPLL_SEL_MASK | MUX_VPLL_SEL_MASK
		| MUX_GPLL_SEL_MASK;

	printf("4\n");

	do {
		val = readl(&cmu_top_clk_reg->mux_stat_top2);
	} while ((val | tmp) != val);

	printf("5\n");
	/* MUX_bPLL */
	/*	clrbits_le32(&cmu_core_clk_reg->src_cdrex, MUX_BPLL_SEL_MASK);
	do {
		val = readl(&cmu_core_clk_reg->mux_stat_cdrex);
	} while ((val | MUX_BPLL_SEL_MASK) != val);

	printf("6\n");
	/* Now ready to start to configure PLL clock */
	/* PLL locktime */
//	writel(APLL_LOCK_VAL, &cmu_cpu_clk_reg->apll_lock);

//	writel(MPLL_LOCK_VAL, &cmu_cpu_clk_reg->mpll_lock);

//	writel(BPLL_LOCK_VAL, &cmu_core_clk_reg->bpll_lock);

//	writel(CPLL_LOCK_VAL, &cmu_top_clk_reg->cpll_lock);

//	writel(GPLL_LOCK_VAL, &cmu_top_clk_reg->gpll_lock);

//	writel(EPLL_LOCK_VAL, &cmu_top_clk_reg->epll_lock);

//	writel(VPLL_LOCK_VAL, &cmu_top_clk_reg->vpll_lock);

//	writel(CLK_REG_DISABLE, &cmu_core_clk_reg->pll_div2_sel);
/*
	writel(MUX_HPM_SEL_MASK, &cmu_cpu_clk_reg->src_cpu);
	do {
		val = readl(&cmu_cpu_clk_reg->mux_stat_cpu);
	} while ((val | HPM_SEL_SCLK_MPLL) != val);

	/* update CLK_DIV_CPU0/1 */
/*	val = arm_clk_ratio->arm2_ratio << 28
		| arm_clk_ratio->apll_ratio << 24
		| arm_clk_ratio->pclk_dbg_ratio << 20
		| arm_clk_ratio->atb_ratio << 16
		| arm_clk_ratio->periph_ratio << 12
		| arm_clk_ratio->acp_ratio << 8
		| arm_clk_ratio->cpud_ratio << 4
		| arm_clk_ratio->arm_ratio;
	writel(val, &cmu_cpu_clk_reg->div_cpu0);
	do {
		val = readl(&cmu_cpu_clk_reg->div_stat_cpu0);
	} while (0 != val);

	writel(CLK_DIV_CPU1_VAL, &cmu_cpu_clk_reg->div_cpu1);
	do {
		val = readl(&cmu_cpu_clk_reg->div_stat_cpu1);
	} while (0 != val);

	/* Set APLL */
/*	writel(APLL_CON1_VAL, &cmu_cpu_clk_reg->apll_con1);
	val = set_pll(arm_clk_ratio->apll_mdiv, arm_clk_ratio->apll_pdiv,
			arm_clk_ratio->apll_sdiv);
	writel(val, &cmu_cpu_clk_reg->apll_con0);
	while ((readl(&cmu_cpu_clk_reg->apll_con0) & APLL_CON0_LOCKED) == 0)
		;

	/* Set MPLL */
	/*
	writel(MPLL_CON1_VAL, &cmu_cpu_clk_reg->mpll_con1);
	val = set_pll(mem->mpll_mdiv, mem->mpll_pdiv, mem->mpll_sdiv);
	writel(val, &cmu_cpu_clk_reg->mpll_con0);
	while ((readl(&cmu_cpu_clk_reg->mpll_con0) & MPLL_CON0_LOCKED) == 0)
		;

	/* Set BPLL */
/*	writel(BPLL_CON1_VAL, &cmu_core_clk_reg->bpll_con1);
	val = set_pll(mem->bpll_mdiv, mem->bpll_pdiv, mem->bpll_sdiv);
	writel(val, &cmu_core_clk_reg->bpll_con0);
	while ((readl(&cmu_core_clk_reg->bpll_con0) & BPLL_CON0_LOCKED) == 0)
		;
*/
	/* Set CPLL */
/*	writel(CPLL_CON1_VAL, &cmu_top_clk_reg->cpll_con1);
	val = set_pll(mem->cpll_mdiv, mem->cpll_pdiv, mem->cpll_sdiv);
	writel(val, &cmu_top_clk_reg->cpll_con0);
	while ((readl(&cmu_top_clk_reg->cpll_con0) & CPLL_CON0_LOCKED) == 0)
		;

	/* Set GPLL */
	/*	writel(GPLL_CON1_VAL, &cmu_top_clk_reg->gpll_con1);
	val = set_pll(mem->gpll_mdiv, mem->gpll_pdiv, mem->gpll_sdiv);
	writel(val, &cmu_top_clk_reg->gpll_con0);
	while ((readl(&cmu_top_clk_reg->gpll_con0) & GPLL_CON0_LOCKED) == 0)
		;

	/* Set EPLL */
	/*	writel(EPLL_CON2_VAL, &cmu_top_clk_reg->epll_con2);
	writel(EPLL_CON1_VAL, &cmu_top_clk_reg->epll_con1);
	val = set_pll(mem->epll_mdiv, mem->epll_pdiv, mem->epll_sdiv);
	writel(val, &cmu_top_clk_reg->epll_con0);
	while ((readl(&cmu_top_clk_reg->epll_con0) & EPLL_CON0_LOCKED) == 0)
		;

	/* Set VPLL */
	/*	writel(VPLL_CON2_VAL, &cmu_top_clk_reg->vpll_con2);
	writel(VPLL_CON1_VAL, &cmu_top_clk_reg->vpll_con1);
	val = set_pll(mem->vpll_mdiv, mem->vpll_pdiv, mem->vpll_sdiv);
	writel(val, &cmu_top_clk_reg->vpll_con0);
	while ((readl(&cmu_top_clk_reg->vpll_con0) & VPLL_CON0_LOCKED) == 0)
		;

	/* Set CLK_DIV_CORE0 */
/*	writel(CLK_SRC_CORE0_VAL, &cmu_cpu_clk_reg->src_core0);
	writel(CLK_DIV_CORE0_VAL, &cmu_cpu_clk_reg->div_core0);
	while (readl(&cmu_cpu_clk_reg->div_stat_core0) != 0)
		;
	/* Set CLK_DIV_CORE1 */
/*	writel(CLK_DIV_CORE1_VAL, &cmu_cpu_clk_reg->div_core1);
	while (readl(&cmu_cpu_clk_reg->div_stat_core1) != 0)
		;
	/* Set CLK_DIV_SYSRGT */
/*	writel(CLK_DIV_SYSRGT_VAL, &cmu_cpu_clk_reg->div_sysrgt);
	while (readl(&cmu_cpu_clk_reg->div_stat_sysrgt) != 0)
		;
	/* Set CLK_DIV_ACP */
/*	writel(CLK_DIV_ACP_VAL, &cmu_cpu_clk_reg->div_acp);
	while (readl(&cmu_cpu_clk_reg->div_stat_acp) != 0)
		;
	/* Set CLK_DIV_SYSLFT */
/*	writel(CLK_DIV_SYSLFT_VAL, &cmu_cpu_clk_reg->div_syslft);
	while (readl(&cmu_cpu_clk_reg->div_stat_syslft) != 0)
		;

	/* ==================
	 * TOP Mutex
	 * ==================*/
	/* update mutexes in CMU_TOP region */
	writel(CLK_SRC_TOP0_VAL, 	&cmu_top_clk_reg->src_top0);
	writel(CLK_SRC_TOP1_VAL, 	&cmu_top_clk_reg->src_top1);
	writel(TOP2_VAL, 			&cmu_top_clk_reg->src_top2);
	writel(CLK_SRC_TOP3_VAL, 	&cmu_top_clk_reg->src_top3);

	/* ==================
	 * TOP Dividers
	 * ==================*/
	/* set CLK_DIV_TOP0 */
	writel(CLK_DIV_TOP0_VAL, &cmu_top_clk_reg->div_top0);
	while (readl(&cmu_top_clk_reg->div_stat_top0))
		;

	/* set CLK_DIV_TOP1 */
	writel(CLK_DIV_TOP1_VAL, &cmu_top_clk_reg->div_top1);
	while (readl(&cmu_top_clk_reg->div_stat_top1))
		;

	/* ==================
	 * CMU_LEX
	 * ==================*/
	/* Note: this is fixed to ACLK_200 */
	writel(CLK_SRC_LEX_VAL, &cmu_top_clk_reg->src_lex);
	while (1) {
		val = readl(&cmu_top_clk_reg->mux_stat_lex);
		if (val == (val | 1))
			break;
	}
	writel(CLK_DIV_LEX_VAL, &cmu_top_clk_reg->div_lex);
	while (readl(&cmu_top_clk_reg->div_stat_lex))
		;

	/* ==================
	 * CMU_R0X
	 * ==================*/
	writel(CLK_DIV_R0X_VAL, &cmu_top_clk_reg->div_r0x);
	while (readl(&cmu_top_clk_reg->div_stat_r0x))
		;

	/* ==================
	 * CMU_R1X
	 * ==================*/
	writel(CLK_DIV_R1X_VAL, &cmu_top_clk_reg->div_r1x);
	while (readl(&cmu_top_clk_reg->div_stat_r1x))
		;

	/* ==================
	 * CMU_CDREX
	 * ==================*/
	writel(CLK_REG_DISABLE, &cmu_core_clk_reg->src_cdrex);
	writel(CLK_DIV_CDREX_VAL, &cmu_core_clk_reg->div_cdrex);
	while (readl(&cmu_core_clk_reg->div_stat_cdrex))
		;

	/* ==================
	 * CMU_CPU
	 * ==================*/
	/* Select CPU clock sources  */
	val = readl(&cmu_cpu_clk_reg->src_cpu);
	val |= CLK_SRC_CPU_VAL;
	writel(val, &cmu_cpu_clk_reg->src_cpu);

	val = readl(&cmu_cpu_clk_reg->src_core1);
	val |= CLK_SRC_CORE1_VAL;
	writel(val, &cmu_cpu_clk_reg->src_core1);

	/* ==================
	 * CMU_TOP
	 * ==================*/
	val = readl(&cmu_top_clk_reg->src_top2);
	val |= CLK_SRC_TOP2_VAL;
	writel(val, &cmu_top_clk_reg->src_top2);

	/* ==================
	 * CMU_TOP: FSYS1/2
	 * ==================*/
	/* MMC[0-3] clocks */
	writel(CLK_SRC_FSYS0_VAL, &cmu_top_clk_reg->src_fsys);
	writel(CLK_DIV_FSYS0_VAL, &cmu_top_clk_reg->div_fsys0);
	while (readl(&cmu_top_clk_reg->div_stat_fsys0))
		;
	writel(CLK_DIV_FSYS1_VAL, &cmu_top_clk_reg->div_fsys1);
	while (readl(&cmu_top_clk_reg->div_stat_fsys1))
		;
	writel(CLK_DIV_FSYS2_VAL, &cmu_top_clk_reg->div_fsys2);
	while (readl(&cmu_top_clk_reg->div_stat_fsys2))
		;

	/* ==================
	 * CMU: clear all mux
	 * ==================*/
	writel(CLK_REG_DISABLE, &cmu_cpu_clk_reg->clkout_cmu_cpu);
	writel(CLK_REG_DISABLE, &cmu_cpu_clk_reg->clkout_cmu_core);
	writel(CLK_REG_DISABLE, &cmu_cpu_clk_reg->clkout_cmu_acp);
	writel(CLK_REG_DISABLE, &cmu_top_clk_reg->clkout_cmu_top);
	writel(CLK_REG_DISABLE, &cmu_top_clk_reg->clkout_cmu_lex);
	writel(CLK_REG_DISABLE, &cmu_top_clk_reg->clkout_cmu_r0x);
	writel(CLK_REG_DISABLE, &cmu_top_clk_reg->clkout_cmu_r1x);
	writel(CLK_REG_DISABLE, &cmu_core_clk_reg->clkout_cmu_cdrex);

	/* ==================
	 * CMU_TOP: Peripheral dividers
	 * ==================*/
	writel(CLK_SRC_PERIC0_VAL, &cmu_top_clk_reg->src_peric0);
	writel(CLK_DIV_PERIC0_VAL, &cmu_top_clk_reg->div_peric0);
	writel(CLK_SRC_PERIC1_VAL, &cmu_top_clk_reg->src_peric1);
	writel(CLK_DIV_PERIC1_VAL, &cmu_top_clk_reg->div_peric1);
	writel(CLK_DIV_PERIC2_VAL, &cmu_top_clk_reg->div_peric2);
	writel(CLK_DIV_PERIC3_VAL, &cmu_top_clk_reg->div_peric3);
	/* ISPs */
	writel(CLK_DIV_ISP0_VAL, 	&cmu_cpu_clk_reg->div_isp0);
	writel(CLK_DIV_ISP1_VAL, 	&cmu_cpu_clk_reg->div_isp1);
	writel(CLK_DIV_ISP2_VAL, 	&cmu_cpu_clk_reg->div_isp2);
	writel(SCLK_SRC_ISP_VAL, 	&cmu_top_clk_reg->sclk_src_isp);
	writel(SCLK_DIV_ISP_VAL, 	&cmu_top_clk_reg->sclk_div_isp);
	writel(CLK_DIV_DISP1_O_VAL, &cmu_top_clk_reg->div_disp1_0);

	/* FIMD1 SRC CLK SELECTION */
	writel(CLK_SRC_DISP1_0_VAL, &cmu_top_clk_reg->src_disp1_0);
}


/*
 * dum.c
 *
 *  Created on: Aug 19, 2013
 *      Author: jxie
 */


#define PLL_BASE_ADDR 	(uint32_t *) cmu_top_clk

#define EPLL_LOCK 					(PLL_BASE_ADDR + 0x30/4)
#define EPLL_CON0 					(PLL_BASE_ADDR + 0x130/4)
#define EPLL_CON1 					(PLL_BASE_ADDR + 0x134/4)
#define EPLL_CON2 					(PLL_BASE_ADDR + 0x138/4)
#define CLK_GATE_IP_PERIC 			(PLL_BASE_ADDR + 0x950/4)
#define CLK_SRC_PERIC0 				(PLL_BASE_ADDR + 0x250/4)
#define CLK_SRC_PERIC1 				(PLL_BASE_ADDR + 0x254/4)
#define CLK_DIV_PERIC0 				(PLL_BASE_ADDR + 0x558/4)
#define CLK_DIV_PERIC1 				(PLL_BASE_ADDR + 0x55C/4)
#define CLK_SRC_MASK_PERIC0 		(PLL_BASE_ADDR + 0x350/4)
#define CLK_SRC_MASK_PERIC1 		(PLL_BASE_ADDR + 0x354/4) /* Table 5.9.1.108 */

#define CLK_SRC_TOP2 				(PLL_BASE_ADDR + 0x218/4)

#define MUX_EPLL_SEL (1<<12)

#define USE_EPLL	0
    /*
     * Assume FIN=24MHz.  From table 5-3 in the Exynos 5250 user manual.
     * Set PLL to 192MHz.
     */

#define P 2
#define M 64
#define S 2
#define K 0
#define DCC_ENB 1
#define ICP_BOOST 0
#define SSCG_EN  0
#define AFC_ENB 0
#define EXTAFC 0


/*
 * Set the EPLL to 192MHz, the divider to 4x48, to give a 1MHz SPI clock (SI has internal divide by two)
 * It can go faster -- up to 10MHz -- but 1MHz is easier to see on the scope.
 */
void setclk(void)
{
		uint32_t v;

		if(USE_EPLL){



		/*
		 * Turn off EPLL
		 * Set divider ratios
		 * Set PLL PMS values
		 * Set other control values
		 * Turn on PLL (PLL_CON0[31] = 1
		 * Wait lock time
		 * Set MUX to select the input.
		 */
		*(volatile uint32_t *)EPLL_CON0 = (0x30<<16)|(3<<8)|1; /* turn off EPLL */

		/* Set SPI1_PRE_RATIO to 4, SPI1_RATIO to 48 */
		/* Low order 16 bits are SPI0, high order are SPI1 */
		v =  *(volatile uint32_t *)CLK_DIV_PERIC1;
		v &= ~(0xffff << 16);
		v |= (3 << 24) | (47 << 16);
		*(volatile uint32_t *)CLK_DIV_PERIC1 = v;


		/*
		 * Set UART divider to 8, to obtain 24 MHz SCLK_UARTn
		 * UART0_RATIO = 7
		 * UART1_RATIO = 7
		 * UART3_RATIO = 7
		 * UART2_RATIO = NOT TOUCH
		 */
		v =  *(volatile uint32_t *)CLK_DIV_PERIC0;
		v &= ~((0xf << 12) | (0xf << 4) | (0xf << 0));
		v |= (7 << 12) | (7 << 4) | (7 << 0);
		*(volatile uint32_t *)CLK_DIV_PERIC0 = v;



		/* Set PMS in PLL  -- see 5.9.81 */
		*((volatile uint32_t *)EPLL_CON0) = (M<<16) | (P<<8) | (S);
		/* Set other control values */
		*(volatile uint32_t *)EPLL_CON1 = K;
		*(volatile uint32_t *)EPLL_CON2 = (EXTAFC << 8) | (DCC_ENB << 7) | (AFC_ENB << 6) | (SSCG_EN<<5);
		/* Turn on PLL */
		*((volatile uint32_t *)EPLL_CON0) = (M<<16) |(P<<8) | (S) | (1U << 31);
		/* Wait for lock */
		while ((*(volatile uint32_t *)EPLL_CON0) & (1<<29))
			;

		/*
		 * Turn off EPLL bypass
		 */
		v = *CLK_SRC_TOP2;
		v |= MUX_EPLL_SEL;
		*CLK_SRC_TOP2 = v;



		/* Set SPI1 clock MUX to use EPLL clock */
		v = *CLK_SRC_PERIC1;
		v &= ~(0xf<<20);
		v |= (7 << 20);
		*CLK_SRC_PERIC1 = v;

		/* Set UART clock MUX to use EPLL clock */
		v = *CLK_SRC_PERIC0;
		v &= ~((0xf << 12) | (0xf << 4) | (0xf << 0));
		v |= (7 << 12) | (7 << 4) | (7 << 0);
		*CLK_SRC_PERIC0 = v;

	}else{
		/*
		 * 24MHz clock from XXTI
		 */


		/* Set SPI1_PRE_RATIO to 1, SPI1_RATIO to 12 */
		/* Low order 16 bits are SPI0, high order are SPI1 */
		v =  *(volatile uint32_t *)CLK_DIV_PERIC1;
		v &= ~(0xffff << 16);
		v |= (0 << 24) | (12 << 16);
		*(volatile uint32_t *)CLK_DIV_PERIC1 = v;

		/*
		 * Set UART divider to 1, to obtain 24 MHz SCLK_UARTn
		 * UART0_RATIO = 0
		 * UART1_RATIO = 0
		 * UART3_RATIO = 0
		 * UART2_RATIO = NOT TOUCH
		 */
		v =  *(volatile uint32_t *)CLK_DIV_PERIC0;
		v &= ~((0xf << 12) |(0xf << 8) | (0xf << 4) | (0xf << 0));
		v |= (0 << 12) | (0 << 8) | (0 << 4) | (0 << 0);
		*(volatile uint32_t *)CLK_DIV_PERIC0 = v;

		/* Set SPI1 clock MUX to use XXTI clock */
		v = *CLK_SRC_PERIC1;
		v &= ~(0xf << 20);
		v |= (1<<20);
		*CLK_SRC_PERIC1 = v;

		/* Set UART SCLK MUX to use XXTI clock */
		v = *CLK_SRC_PERIC0;
		v &= ~((0xf << 12) | (0xf << 8) | (0xf << 4) | (0xf << 0));
		v |= (1 << 12) |(1 << 8) | (1 << 4) | (1 << 0);
		*CLK_SRC_PERIC0 = v;

	}

	/* SPI1 should now have a 1MHz clock, but it may be gated or masked */
	/* See 5.9.108  -- -reset value is unmasked */
	v = *CLK_SRC_MASK_PERIC1;
	v |= (1<<20);
	*CLK_SRC_MASK_PERIC1 = v;

	/*
	 * ungate the clock (reset value is ungated)
	 */
	v = *CLK_GATE_IP_PERIC;
	v |= (1<<17);
	*CLK_GATE_IP_PERIC = v;


	/* UART should now have a 24 MHz clock, but it may be gated or masked */
	/* See 5.9.108  -- -reset value is unmasked */
	v = *CLK_SRC_MASK_PERIC0;
	v |= (1 << 12) | (1 << 8)| (1 << 4)| (1 << 0);
	*CLK_SRC_MASK_PERIC0 = v;

	/*
	 * ungate the clock (reset value is ungated)
	 */
	v = *CLK_GATE_IP_PERIC;
	v |= (1 << 3) | (1 << 2) | (1 << 1)|(1 << 0);
	*CLK_GATE_IP_PERIC = v;

}


void clktimer__init(void){
	clk__init();
}


int clktimer_get_pwm_freq(int id)
{
	int err = 0;
	wait_for_init();
	//printf("clktimer_get_pwm_freq, id %d\n",id);
	err = (int)clock_get_periph_rate(id);
	//printf("clktimer_get_pwm_freq exit\n");
	return err;
}

int clktimer_get_mmc(int dev_index)
{
	int r = 0;
	return r;
}
void clktimer_set_mmc(int dev_index, unsigned int div)
{

}


int clkmmc_get_pwm_freq(int id)
{
	int err = 0;
	return err;
}
int clkmmc_get_mmc(int dev_index)
{
	int r = 0;
	wait_for_init();
	//printf("clkmmc_get_mmc, dev_index %d\n",dev_index);
	r = (int) exynos5_get_mmc_clk(dev_index);
	//printf("clkmmc_get_mmc exit\n");
	return r;
}
void clkmmc_set_mmc(int dev_index, unsigned int div)
{
	wait_for_init();
	//printf("clkmmc_set_mmc, dev_index %d\n",dev_index);
	exynos5_set_mmc_clk(dev_index, div);
}


unsigned long get_uart_clk(int dev_index)
{
	return exynos5_get_uart_clk(dev_index);
}



