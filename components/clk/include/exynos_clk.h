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
 * exynos_clk.h
 *
 *  Created on: Aug 5, 2013
 *      Author: jxie
 */

#ifndef EXYNOS_CLK_H_
#define EXYNOS_CLK_H_

#include "dmc.h"
#include "periph.h"


#define APLL	0
#define MPLL	1
#define EPLL	2
#define HPLL	3
#define VPLL	4
#define BPLL	5

enum pll_src_bit {
	EXYNOS_SRC_MPLL = 6,
	EXYNOS_SRC_EPLL,
	EXYNOS_SRC_VPLL,
};


/* These are the ratio's for configuring ARM clock */
struct arm_clk_ratios {
	unsigned arm_freq_mhz;		/* Frequency of ARM core in MHz */

	unsigned apll_mdiv;
	unsigned apll_pdiv;
	unsigned apll_sdiv;

	unsigned arm2_ratio;
	unsigned apll_ratio;
	unsigned pclk_dbg_ratio;
	unsigned atb_ratio;
	unsigned periph_ratio;
	unsigned acp_ratio;
	unsigned cpud_ratio;
	unsigned arm_ratio;
};

/* structure for epll configuration used in audio clock configuration */
struct set_epll_con_val {
	unsigned int freq_out;		/* frequency out */
	unsigned int en_lock_det;	/* enable lock detect */
	unsigned int m_div;		/* m divider value */
	unsigned int p_div;		/* p divider value */
	unsigned int s_div;		/* s divider value */
	unsigned int k_dsm;		/* k value of delta signal modulator */
};


#define MEM_TIMINGS_MSR_COUNT		4

/* These are the memory timings for a particular memory type and speed */
struct mem_timings {
	enum mem_manuf mem_manuf;	/* Memory manufacturer */
	enum ddr_mode mem_type;		/* Memory type */
	unsigned frequency_mhz;		/* Frequency of memory in MHz */

	/* Here follow the timing parameters for the selected memory */
	unsigned apll_mdiv;
	unsigned apll_pdiv;
	unsigned apll_sdiv;
	unsigned mpll_mdiv;
	unsigned mpll_pdiv;
	unsigned mpll_sdiv;
	unsigned cpll_mdiv;
	unsigned cpll_pdiv;
	unsigned cpll_sdiv;
	unsigned gpll_mdiv;
	unsigned gpll_pdiv;
	unsigned gpll_sdiv;
	unsigned epll_mdiv;
	unsigned epll_pdiv;
	unsigned epll_sdiv;
	unsigned vpll_mdiv;
	unsigned vpll_pdiv;
	unsigned vpll_sdiv;
	unsigned bpll_mdiv;
	unsigned bpll_pdiv;
	unsigned bpll_sdiv;
	unsigned pclk_cdrex_ratio;
	unsigned direct_cmd_msr[MEM_TIMINGS_MSR_COUNT];

	unsigned timing_ref;
	unsigned timing_row;
	unsigned timing_data;
	unsigned timing_power;

	/* DQS, DQ, DEBUG offsets */
	unsigned phy0_dqs;
	unsigned phy1_dqs;
	unsigned phy0_dq;
	unsigned phy1_dq;
	unsigned phy0_tFS;
	unsigned phy1_tFS;
	unsigned phy0_pulld_dqs;
	unsigned phy1_pulld_dqs;

	unsigned lpddr3_ctrl_phy_reset;
	unsigned ctrl_start_point;
	unsigned ctrl_inc;
	unsigned ctrl_start;
	unsigned ctrl_dll_on;
	unsigned ctrl_ref;

	unsigned ctrl_force;
	unsigned ctrl_rdlat;
	unsigned ctrl_bstlen;

	unsigned fp_resync;
	unsigned iv_size;
	unsigned dfi_init_start;
	unsigned aref_en;

	unsigned rd_fetch;

	unsigned zq_mode_dds;
	unsigned zq_mode_term;
	unsigned zq_mode_noterm;	/* 1 to allow termination disable */

	unsigned memcontrol;
	unsigned memconfig;

	unsigned membaseconfig0;
	unsigned membaseconfig1;
	unsigned prechconfig_tp_cnt;
	unsigned dpwrdn_cyc;
	unsigned dsref_cyc;
	unsigned concontrol;
	/* Channel and Chip Selection */
	unsigned char 	dmc_channels;		/* number of memory channels */
	unsigned char 	chips_per_channel;	/* number of chips per channel */
	unsigned char 	chips_to_configure;	/* number of chips to configure */
	unsigned char 	send_zq_init;		/* 1 to send this command */
	unsigned 		impedance;		/* drive strength impedeance */
	unsigned char 	gate_leveling_enable;	/* check gate leveling is enabled */
};


#define MPLL_FOUT_SEL_SHIFT				4
#define EXYNOS5_EPLLCON0_LOCKED_SHIFT	29  /* EPLL Locked bit position*/
#define TIMEOUT_EPLL_LOCK				1000

#define AUDIO_0_RATIO_MASK				0x0f
#define AUDIO_1_RATIO_MASK				0x0f

#define AUDIO1_SEL_MASK					0xf
#define CLK_SRC_SCLK_EPLL				0x7

/* CON0 bit-fields */
#define EPLL_CON0_MDIV_MASK				0x1ff
#define EPLL_CON0_PDIV_MASK				0x3f
#define EPLL_CON0_SDIV_MASK				0x7
#define EPLL_CON0_MDIV_SHIFT			16
#define EPLL_CON0_PDIV_SHIFT			8
#define EPLL_CON0_SDIV_SHIFT			0
#define EPLL_CON0_LOCK_DET_EN_SHIFT		28
#define EPLL_CON0_LOCK_DET_EN_MASK		1

#define MPLL_FOUT_SEL_MASK				0x1
#define BPLL_FOUT_SEL_SHIFT				0
#define BPLL_FOUT_SEL_MASK				0x1


struct exynos5_clk_cmu_cpu {
	unsigned int	apll_lock;
	unsigned char	res1[0xfc];
	unsigned int	apll_con0;
	unsigned int	apll_con1;
	unsigned char	res2[0xf8];
	unsigned int	src_cpu;
	unsigned char	res3[0x1fc];
	unsigned int	mux_stat_cpu;
	unsigned char	res4[0xfc];
	unsigned int	div_cpu0;
	unsigned int	div_cpu1;
	unsigned char	res5[0xf8];
	unsigned int	div_stat_cpu0;
	unsigned int	div_stat_cpu1;
	unsigned char	res6[0x1f8];
	unsigned int	gate_sclk_cpu;
	unsigned char	res7[0x1fc];
	unsigned int	clkout_cmu_cpu;
	unsigned int	clkout_cmu_cpu_div_stat;
	unsigned char	res8[0x5f8];
	unsigned int	armclk_stopctrl;
	unsigned char	res9[0x0c];
	unsigned int	parityfail_status;
	unsigned int	parityfail_clear;
	unsigned char	res10[0x8];
	unsigned int	pwr_ctrl;
	unsigned int	pwr_ctr2;
	unsigned char	res11[0xd8];
	unsigned int	apll_con0_l8;
	unsigned int	apll_con0_l7;
	unsigned int	apll_con0_l6;
	unsigned int	apll_con0_l5;
	unsigned int	apll_con0_l4;
	unsigned int	apll_con0_l3;
	unsigned int	apll_con0_l2;
	unsigned int	apll_con0_l1;
	unsigned int	iem_control;
	unsigned char	res12[0xdc];
	unsigned int	apll_con1_l8;
	unsigned int	apll_con1_l7;
	unsigned int	apll_con1_l6;
	unsigned int	apll_con1_l5;
	unsigned int	apll_con1_l4;
	unsigned int	apll_con1_l3;
	unsigned int	apll_con1_l2;
	unsigned int	apll_con1_l1;
	unsigned char	res13[0xe0];
	unsigned int	div_iem_l8;
	unsigned int	div_iem_l7;
	unsigned int	div_iem_l6;
	unsigned int	div_iem_l5;
	unsigned int	div_iem_l4;
	unsigned int	div_iem_l3;
	unsigned int	div_iem_l2;
	unsigned int	div_iem_l1;
	unsigned char	res14[0x2ce0];
	unsigned int	mpll_lock;
	unsigned char	res15[0xfc];
	unsigned int	mpll_con0;
	unsigned int	mpll_con1;
	unsigned char	res16[0xf8];
	unsigned int	src_core0;
	unsigned int	src_core1;
	unsigned char	res17[0xf8];
	unsigned int	src_mask_core;
	unsigned char	res18[0x100];
	unsigned int	mux_stat_core1;
	unsigned char	res19[0xf8];
	unsigned int	div_core0;
	unsigned int	div_core1;
	unsigned int	div_sysrgt;
	unsigned char	res20[0xf4];
	unsigned int	div_stat_core0;
	unsigned int	div_stat_core1;
	unsigned int	div_stat_sysrgt;
	unsigned char	res21[0x2f4];
	unsigned int	gate_ip_core;
	unsigned int	gate_ip_sysrgt;
	unsigned char	res22[0x8];
	unsigned int	c2c_monitor;
	unsigned char	res23[0xec];
	unsigned int	clkout_cmu_core;
	unsigned int	clkout_cmu_core_div_stat;
	unsigned char	res24[0x5f8];
	unsigned int	dcgidx_map0;
	unsigned int	dcgidx_map1;
	unsigned int	dcgidx_map2;
	unsigned char	res25[0x14];
	unsigned int	dcgperf_map0;
	unsigned int	dcgperf_map1;
	unsigned char	res26[0x18];
	unsigned int	dvcidx_map;
	unsigned char	res27[0x1c];
	unsigned int	freq_cpu;
	unsigned int	freq_dpm;
	unsigned char	res28[0x18];
	unsigned int	dvsemclk_en;
	unsigned int	maxperf;
	unsigned char	res29[0xf78];
	unsigned int	c2c_config;
	unsigned char	res30[0x24fc];
	unsigned int	div_acp;
	unsigned char	res31[0xfc];
	unsigned int	div_stat_acp;
	unsigned char	res32[0x1fc];
	unsigned int	gate_ip_acp;
	unsigned char	res33[0xfc];
	unsigned int	div_syslft;
	unsigned char	res34[0xc];
	unsigned int	div_stat_syslft;
	unsigned char	res35[0x1c];
	unsigned int	gate_ip_syslft;
	unsigned char	res36[0xcc];
	unsigned int	clkout_cmu_acp;
	unsigned int	clkout_cmu_acp_div_stat;
	unsigned char	res37[0x8];
	unsigned int	ufmc_config;
	unsigned char	res38[0x38ec];
	unsigned int	div_isp0;
	unsigned int	div_isp1;
	unsigned int	div_isp2;
	unsigned char	res39[0xf4];
	unsigned int	div_stat_isp0;
	unsigned int	div_stat_isp1;
	unsigned int	div_stat_isp2;
	unsigned char	res40[0x3f4];
	unsigned int	gate_ip_isp0;
	unsigned int	gate_ip_isp1;
	unsigned char	res41[0xf8];
	unsigned int	gate_sclk_isp;
	unsigned char	res42[0xc];
	unsigned int	mcuisp_pwr_ctrl;
	unsigned char	res43[0xec];
	unsigned int	clkout_cmu_isp;
	unsigned int	clkout_cmu_isp_div_stat;
	//unsigned char	res44[0x3618];
};


struct exynos5_clk_cmu_top {
	unsigned char	res44[0x20];
	unsigned int	cpll_lock;
	unsigned char	res45[0xc];
	unsigned int	epll_lock;
	unsigned char	res46[0xc];
	unsigned int	vpll_lock;
	unsigned char	res47[0xc];
	unsigned int	gpll_lock;
	unsigned char	res48[0xcc];
	unsigned int	cpll_con0;
	unsigned int	cpll_con1;
	unsigned char	res49[0x8];
	unsigned int	epll_con0;
	unsigned int	epll_con1;
	unsigned int	epll_con2;
	unsigned char	res50[0x4];
	unsigned int	vpll_con0;
	unsigned int	vpll_con1;
	unsigned int	vpll_con2;
	unsigned char	res51[0x4];
	unsigned int	gpll_con0;
	unsigned int	gpll_con1;
	unsigned char	res52[0xb8];
	unsigned int	src_top0;
	unsigned int	src_top1;
	unsigned int	src_top2;
	unsigned int	src_top3;
	unsigned int	src_gscl;
	unsigned char	res53[0x8];
	unsigned int	src_disp1_0;
	unsigned char	res54[0x10];
	unsigned int	src_mau;
	unsigned int	src_fsys;
	unsigned int	src_gen;
	unsigned char	res55[0x4];
	unsigned int	src_peric0;
	unsigned int	src_peric1;
	unsigned char	res56[0x18];
	unsigned int	sclk_src_isp;
	unsigned char	res57[0x9c];
	unsigned int	src_mask_top;
	unsigned char	res58[0xc];
	unsigned int	src_mask_gscl;
	unsigned char	res59[0x8];
	unsigned int	src_mask_disp1_0;
	unsigned char	res60[0x4];
	unsigned int	src_mask_mau;
	unsigned char	res61[0x8];
	unsigned int	src_mask_fsys;
	unsigned int	src_mask_gen;
	unsigned char	res62[0x8];
	unsigned int	src_mask_peric0;
	unsigned int	src_mask_peric1;
	unsigned char	res63[0x18];
	unsigned int	src_mask_isp;
	unsigned char	res67[0x9c];
	unsigned int	mux_stat_top0;
	unsigned int	mux_stat_top1;
	unsigned int	mux_stat_top2;
	unsigned int	mux_stat_top3;
	unsigned char	res68[0xf0];
	unsigned int	div_top0;
	unsigned int	div_top1;
	unsigned char	res69[0x8];
	unsigned int	div_gscl;
	unsigned char	res70[0x8];
	unsigned int	div_disp1_0;
	unsigned char	res71[0xc];
	unsigned int	div_gen;
	unsigned char	res72[0x4];
	unsigned int	div_mau;
	unsigned int	div_fsys0;
	unsigned int	div_fsys1;
	unsigned int	div_fsys2;
	unsigned char	res73[0x4];
	unsigned int	div_peric0;
	unsigned int	div_peric1;
	unsigned int	div_peric2;
	unsigned int	div_peric3;
	unsigned int	div_peric4;
	unsigned int	div_peric5;
	unsigned char	res74[0x10];
	unsigned int	sclk_div_isp;
	unsigned char	res75[0xc];
	unsigned int	div2_ratio0;
	unsigned int	div2_ratio1;
	unsigned char	res76[0x8];
	unsigned int	div4_ratio;
	unsigned char	res77[0x6c];
	unsigned int	div_stat_top0;
	unsigned int	div_stat_top1;
	unsigned char	res78[0x8];
	unsigned int	div_stat_gscl;
	unsigned char	res79[0x8];
	unsigned int	div_stat_disp1_0;
	unsigned char	res80[0xc];
	unsigned int	div_stat_gen;
	unsigned char	res81[0x4];
	unsigned int	div_stat_mau;
	unsigned int	div_stat_fsys0;
	unsigned int	div_stat_fsys1;
	unsigned int	div_stat_fsys2;
	unsigned char	res82[0x4];
	unsigned int	div_stat_peric0;
	unsigned int	div_stat_peric1;
	unsigned int	div_stat_peric2;
	unsigned int	div_stat_peric3;
	unsigned int	div_stat_peric4;
	unsigned int	div_stat_peric5;
	unsigned char	res83[0x10];
	unsigned int	sclk_div_stat_isp;
	unsigned char	res84[0xc];
	unsigned int	div2_stat0;
	unsigned int	div2_stat1;
	unsigned char	res85[0x8];
	unsigned int	div4_stat;
	unsigned char	res86[0x184];
	unsigned int	gate_top_sclk_disp1;
	unsigned int	gate_top_sclk_gen;
	unsigned char	res87[0xc];
	unsigned int	gate_top_sclk_mau;
	unsigned int	gate_top_sclk_fsys;
	unsigned char	res88[0xc];
	unsigned int	gate_top_sclk_peric;
	unsigned char	res89[0x1c];
	unsigned int	gate_top_sclk_isp;
	unsigned char	res90[0xac];
	unsigned int	gate_ip_gscl;
	unsigned char	res91[0x4];
	unsigned int	gate_ip_disp1;
	unsigned int	gate_ip_mfc;
	unsigned int	gate_ip_g3d;
	unsigned int	gate_ip_gen;
	unsigned char	res92[0xc];
	unsigned int	gate_ip_fsys;
	unsigned char	res93[0x8];
	unsigned int	gate_ip_peric;
	unsigned char	res94[0xc];
	unsigned int	gate_ip_peris;
	unsigned char	res95[0x1c];
	unsigned int	gate_block;
	unsigned char	res96[0x1c];
	unsigned int	mcuiop_pwr_ctrl;
	unsigned char	res97[0x5c];
	unsigned int	clkout_cmu_top;
	unsigned int	clkout_cmu_top_div_stat;
	unsigned char	res98[0x37f8];
	unsigned int	src_lex;
	unsigned char	res99[0x1fc];
	unsigned int	mux_stat_lex;
	unsigned char	res100[0xfc];
	unsigned int	div_lex;
	unsigned char	res101[0xfc];
	unsigned int	div_stat_lex;
	unsigned char	res102[0x1fc];
	unsigned int	gate_ip_lex;
	unsigned char	res103[0x1fc];
	unsigned int	clkout_cmu_lex;
	unsigned int	clkout_cmu_lex_div_stat;
	unsigned char	res104[0x3af8];
	unsigned int	div_r0x;
	unsigned char	res105[0xfc];
	unsigned int	div_stat_r0x;
	unsigned char	res106[0x1fc];
	unsigned int	gate_ip_r0x;
	unsigned char	res107[0x1fc];
	unsigned int	clkout_cmu_r0x;
	unsigned int	clkout_cmu_r0x_div_stat;
	unsigned char	res108[0x3af8];
	unsigned int	div_r1x;
	unsigned char	res109[0xfc];
	unsigned int	div_stat_r1x;
	unsigned char	res110[0x1fc];
	unsigned int	gate_ip_r1x;
	unsigned char	res111[0x1fc];
	unsigned int	clkout_cmu_r1x;
	unsigned int	clkout_cmu_r1x_div_stat;
	//unsigned char	res112[0x3608];
};

struct exynos5_clk_cmu_core {
	unsigned char	res112[0x10];
	unsigned int	bpll_lock;
	unsigned char	res113[0xfc];
	unsigned int	bpll_con0;
	unsigned int	bpll_con1;
	unsigned char	res114[0xe8];
	unsigned int	src_cdrex;
	unsigned char	res115[0x1fc];
	unsigned int	mux_stat_cdrex;
	unsigned char	res116[0xfc];
	unsigned int	div_cdrex;
	unsigned char	res117[0xfc];
	unsigned int	div_stat_cdrex;
	unsigned char	res118[0x2fc];
	unsigned int	gate_ip_cdrex;
	unsigned char	res119[0x10];
	unsigned int	dmc_freq_ctrl;
	unsigned char	res120[0x4];
	unsigned int	drex2_pause;
	unsigned char	res121[0xe0];
	unsigned int	clkout_cmu_cdrex;
	unsigned int	clkout_cmu_cdrex_div_stat;
	unsigned char	res122[0x8];
	unsigned int	lpddr3phy_ctrl;
	unsigned int	lpddr3phy_con0;
	unsigned int	lpddr3phy_con1;
	unsigned int	lpddr3phy_con2;
	unsigned int	lpddr3phy_con3;
	unsigned int	pll_div2_sel;
	//unsigned char	res123[0xf5d8];
};

/*
unsigned long get_arm_clk(void);
unsigned long get_i2c_clk(void);
unsigned long get_pwm_clk(void);
unsigned long get_uart_clk(int dev_index);
unsigned long get_mmc_clk(int dev_index);
void set_mmc_clk(int dev_index, unsigned int div);
unsigned long get_lcd_clk(void);
void set_lcd_clk(void);
int set_spi_clk(int periph_id, unsigned int rate);
int set_i2s_clk_prescaler(unsigned int src_frq, unsigned int dst_frq);
void set_i2s_clk_source(void);
int set_epll_clk(unsigned long rate);
*/
int exynos5_set_spi_clk(enum periph_id periph_id, unsigned int rate);
int clock_calc_best_scalar(unsigned int main_scaler_bits,
	unsigned int fine_scalar_bits, unsigned int input_rate,
	unsigned int target_rate, unsigned int *best_fine_scalar);
int exynos5_set_i2s_clk_prescaler(unsigned int src_frq,
					unsigned int dst_frq);
void exynos5_set_i2s_clk_source(void);
int exynos5_set_epll_clk(unsigned long rate);
unsigned long exynos5_get_i2c_clk(void);
void exynos5_set_lcd_clk(void);
unsigned long exynos5_get_lcd_clk(void);
void exynos5_set_mmc_clk(int dev_index, unsigned int div);
unsigned long exynos5_get_mmc_clk(int dev_index);
unsigned long exynos5_get_uart_clk(int dev_index);
unsigned long exynos5_get_arm_clk(void);
unsigned long clock_get_periph_rate(int peripheral);
unsigned long exynos5_get_periph_rate(int peripheral);
unsigned long exynos5_get_pll_clk(int pllreg);
int exynos_get_pll_clk(int pllreg, unsigned int r, unsigned int k);
void clock_init_dp_clock(void);
void system_clock_init(void);
unsigned long get_pll_clk(int pllreg);
void * samsung_get_base_clock(void);

void setclk(void);


#endif /* EXYNOS_CLK_H_ */


