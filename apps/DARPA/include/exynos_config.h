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
 * exynos_config.h
 *
 * This configuration file is used as initial setup for Exynos 5250
 *  Created on: Aug 1, 2013
 *      Author: jxie
 */

#ifndef EXYNOS_CONFIG_H_
#define EXYNOS_CONFIG_H_


/* input clock of PLL:  24MHz input clock */
#define CONFIG_SYS_CLK_FREQ		24000000
#define CONFIG_SYS_HZ 			88000000

/*
 * LOCK_CON_IN = 0x3;
 * LOCK_CON_DLY = 0x8;
 * Enable DCC
 * Enable AFC
 */
/* APLL_CON1 */
#define APLL_CON1_VAL	(0x00203800)
/* MPLL_CON1 */
#define MPLL_CON1_VAL   (0x00203800)
/* CPLL_CON1	*/
#define CPLL_CON1_VAL	(0x00203800)
/* GPLL_CON1	*/
#define GPLL_CON1_VAL	(0x00203800)

/* DSM K = 0
 *  DCC: ENABLE
 *  AFC: ENABLE
 *  SSCG: DISABLE
 *  FVCO: DISABLE
 *  FSEL: 0 (FVCO_OUT = FREF)
 */
/* EPLL_CON1, CON2	*/
#define EPLL_CON1_VAL	0x00000000
#define EPLL_CON2_VAL	0x00000080

/* VPLL_CON1, CON2	*/
/* SEL_PF: Down spread
 * MRR = 0
 * MFR = 0
 * K = 0
 * -------------------
 * Modulation Frequency MF = FIN/PDIV/MFR/2^5 [Hz]
 * Modulation Rate MR = MFR x MRR/MDIV/2^6 x 100 [%]
 */
#define VPLL_CON1_VAL	0x00000000
#define VPLL_CON2_VAL	0x00000080

/* BPLL_CON1	*/
/* DCC DISABLE
 * AFC ENABLE
 * FSEL = 0 (FVCO_OUT = FREF)
 * FEED DISABLE
 * LOCK_CON_OUT = 0
 * LOCK_CON_IN = 0x3
 * LOCK_CON_DLY = 0x8
 * FVCO_OUT DISABLE
 */
#define BPLL_CON1_VAL	0x00203800

/* Set PLL */
#define set_pll(mdiv, pdiv, sdiv)	(1U<<31 | mdiv<<16 | pdiv<<8 | sdiv)

/* CLK_SRC_CPU	*/
/* 0 = MOUTAPLL,  1 = SCLKMPLL	*/
#define MUX_HPM_SEL             0
#define MUX_CPU_SEL             0
#define MUX_APLL_SEL            1

#define CLK_SRC_CPU_VAL		((MUX_HPM_SEL << 20)    \
							| (MUX_CPU_SEL << 16)  \
							| (MUX_APLL_SEL))

/* MEMCONTROL register bit fields */
#define DMC_MEMCONTROL_CLK_STOP_DISABLE	(0 << 0)
#define DMC_MEMCONTROL_DPWRDN_DISABLE	(0 << 1)
#define DMC_MEMCONTROL_DPWRDN_ACTIVE_PRECHARGE	(0 << 2)
#define DMC_MEMCONTROL_TP_DISABLE	(0 << 4)
#define DMC_MEMCONTROL_DSREF_DISABLE	(0 << 5)
#define DMC_MEMCONTROL_DSREF_ENABLE	(1 << 5)
#define DMC_MEMCONTROL_ADD_LAT_PALL_CYCLE(x)    (x << 6)

#define DMC_MEMCONTROL_MEM_TYPE_LPDDR3  (7 << 8)
#define DMC_MEMCONTROL_MEM_TYPE_DDR3    (6 << 8)
#define DMC_MEMCONTROL_MEM_TYPE_LPDDR2  (5 << 8)

#define DMC_MEMCONTROL_MEM_WIDTH_32BIT  (2 << 12)

#define DMC_MEMCONTROL_NUM_CHIP_1       (0 << 16)
#define DMC_MEMCONTROL_NUM_CHIP_2       (1 << 16)

#define DMC_MEMCONTROL_BL_8             (3 << 20)
#define DMC_MEMCONTROL_BL_4             (2 << 20)

#define DMC_MEMCONTROL_PZQ_DISABLE      (0 << 24)

#define DMC_MEMCONTROL_MRR_BYTE_7_0     (0 << 25)
#define DMC_MEMCONTROL_MRR_BYTE_15_8    (1 << 25)
#define DMC_MEMCONTROL_MRR_BYTE_23_16   (2 << 25)
#define DMC_MEMCONTROL_MRR_BYTE_31_24   (3 << 25)

/* MEMCONFIG0 register bit fields */
#define DMC_MEMCONFIGX_CHIP_MAP_INTERLEAVED     (1 << 12)
#define DMC_MEMCONFIGX_CHIP_COL_10              (3 << 8)
#define DMC_MEMCONFIGX_CHIP_ROW_14              (2 << 4)
#define DMC_MEMCONFIGX_CHIP_ROW_15              (3 << 4)
#define DMC_MEMCONFIGX_CHIP_BANK_8              (3 << 0)

#define DMC_MEMBASECONFIGX_CHIP_BASE(x)         (x << 16)
#define DMC_MEMBASECONFIGX_CHIP_MASK(x)         (x << 0)
#define DMC_MEMBASECONFIG_VAL(x)        (       \
	DMC_MEMBASECONFIGX_CHIP_BASE(x) |       \
	DMC_MEMBASECONFIGX_CHIP_MASK(0x780)     \
)

#define DMC_MEMBASECONFIG0_VAL  DMC_MEMBASECONFIG_VAL(0x40)
#define DMC_MEMBASECONFIG1_VAL  DMC_MEMBASECONFIG_VAL(0x80)

#define DMC_PRECHCONFIG_VAL             0xFF000000
#define DMC_PWRDNCONFIG_VAL             0xFFFF00FF

#define DMC_CONCONTROL_RESET_VAL	0x0FFF0000
#define DFI_INIT_START		(1 << 28)
#define EMPTY			(1 << 8)
#define AREF_EN			(1 << 5)

#define DFI_INIT_COMPLETE_CHO	(1 << 2)
#define DFI_INIT_COMPLETE_CH1	(1 << 3)

#define RDLVL_COMPLETE_CHO	(1 << 14)
#define RDLVL_COMPLETE_CH1	(1 << 15)

#define CLK_STOP_EN	(1 << 0)
#define DPWRDN_EN	(1 << 1)
#define DSREF_EN	(1 << 5)

/* COJCONTROL register bit fields */
#define DMC_CONCONTROL_IO_PD_CON_DISABLE	(0 << 3)
#define DMC_CONCONTROL_AREF_EN_DISABLE		(0 << 5)
#define DMC_CONCONTROL_EMPTY_DISABLE		(0 << 8)
#define DMC_CONCONTROL_EMPTY_ENABLE		(1 << 8)
#define DMC_CONCONTROL_RD_FETCH_DISABLE		(0x0 << 12)
#define DMC_CONCONTROL_TIMEOUT_LEVEL0		(0xFFF << 16)
#define DMC_CONCONTROL_DFI_INIT_START_DISABLE	(0 << 28)

/* CLK_DIV_CPU0_VAL -- not used */
#define CLK_DIV_CPU0_VAL	((ARM2_RATIO << 28)             \
				| (APLL_RATIO << 24)            \
				| (PCLK_DBG_RATIO << 20)        \
				| (ATB_RATIO << 16)             \
				| (PERIPH_RATIO << 12)          \
				| (ACP_RATIO << 8)              \
				| (CPUD_RATIO << 4)             \
				| (ARM_RATIO))



/* CLK_FSYS */
/*
 * All use SCLK_MPLL:
 * USBDRD30_SEL
 * SATA_SEL
 * MMC3, MMC2, MMC1
 */
#define CLK_SRC_FSYS0_VAL              0x66666


/* CLK_DIV_CPU1	*/
#define HPM_RATIO               0x2
#define COPY_RATIO              0x0

/* CLK_DIV_CPU1 = 0x00000003 */
#define CLK_DIV_CPU1_VAL        ((HPM_RATIO << 4)		\
								| (COPY_RATIO))

/* CLK_SRC_CORE0 */
#define MUX_RSVD3_CORE_SEL_VAL	0x0
#define CLK_SRC_CORE0_VAL       (MUX_RSVD3_CORE_SEL_VAL << 16)

/* CLK_SRC_CORE1 */
#define MUX_MPLL_SEL_VAL		0x1
#define CLK_SRC_CORE1_VAL       (MUX_MPLL_SEL_VAL << 8)

/* CLK_DIV_CORE0 */
#define COREP_RATIO_VAL			0x1
#define CORED_RATIO_VAL			0x2
#define CLK_DIV_CORE0_VAL       (COREP_RATIO_VAL << 20) | (COREP_RATIO_VAL << 16)

/* CLK_DIV_CORE1 */
#define RSVD1_CORE_RATIO_VAL	0x7
#define RSVD2_CORE_RATIO_VAL	0x7
#define RSVD3_CORE_RATIO_VAL	0x7
#define CLK_DIV_CORE1_VAL       ((RSVD1_CORE_RATIO_VAL << 24) 	\
								|(RSVD2_CORE_RATIO_VAL << 16)	\
								|(RSVD3_CORE_RATIO_VAL << 8))		//0x07070700

/* CLK_DIV_SYSRGT */
#define ACLK_C2C_200_RATIO_VAL	0x1
#define C2C_CLK_400_RATIO_VAL	0x1
#define ACLK_R1BX_RATIO_VAL		0x1
#define CLK_DIV_SYSRGT_VAL      ((ACLK_C2C_200_RATIO_VAL << 8)	\
								|(C2C_CLK_400_RATIO_VAL << 4)	\
								|(ACLK_R1BX_RATIO_VAL))			//0x00000111

/* CLK_DIV_ACP */
#define PCLK_ACP_RATIO_VAL		0x1
#define ACLK_ACP_RATIO_VAL		0x2
#define CLK_DIV_ACP_VAL         ((PCLK_ACP_RATIO_VAL << 4)	\
								|(ACLK_ACP_RATIO_VAL))			//0x12

/* CLK_DIV_SYSLFT */
#define EFCLK_SYSLFT_RATIO_VAL	0x1
#define PCLK_SYSLFT_RATIO_VAL	0x1
#define ACLK_SYSLFT_RATIO_VAL	0x1
#define CLK_DIV_SYSLFT_VAL      ((EFCLK_SYSLFT_RATIO_VAL << 8)	\
								|(PCLK_SYSLFT_RATIO_VAL << 4)	\
								|(ACLK_SYSLFT_RATIO_VAL))		//0x00000311

/* CLK_SRC_CDREX */
#define MUX_MCLK_DPHY_SEL_VAL	0x0		//SCLK_MPLL
#define MUX_MCLK_CDREX_SEL_VAL	0x0		//SCLK_MPLL
#define MUX_BPLL_SEL_VAL		0x1		//MOUT_BPLL_FOUT
#define CLK_SRC_CDREX_VAL       ((MUX_MCLK_DPHY_SEL_VAL << 8)	\
								|(MUX_MCLK_CDREX_SEL_VAL << 4)	\
								|(MUX_BPLL_SEL_VAL))

/* CLK_DIV_CDREX */
#define MCLK_CDREX2_RATIO       0x0
#define ACLK_SFRTZASCP_RATIO	0x1
#define MCLK_DPHY_RATIO	        0x1
#define MCLK_CDREX_RATIO		0x2
#define PCLK_CDREX_RATIO		0x1
#define ACLK_CDREX_RATIO		0x1

#define CLK_DIV_CDREX_VAL	((MCLK_CDREX2_RATIO << 28)		\
							| (ACLK_SFRTZASCP_RATIO << 24)	\
							| (MCLK_DPHY_RATIO << 20)		\
							| (MCLK_CDREX_RATIO << 16)		\
							| (PCLK_CDREX_RATIO << 4)		\
							| (ACLK_CDREX_RATIO))
/*
#define CLK_DIV_CDREX_VAL	((MCLK_DPHY_RATIO << 24)        \
				| (C2C_CLK_400_RATIO << 6)	\
				| (PCLK_CDREX_RATIO << 4)	\
				| (ACLK_CDREX_RATIO))
*/
/* CLK_SRC_TOP0	*/
#define MUX_ACLK_300_GSCL_SEL           0x0
#define MUX_ACLK_300_GSCL_MID_SEL       0x0
#define MUX_ACLK_400_G3D_MID_SEL        0x0
#define MUX_ACLK_333_SEL	        	0x0
#define MUX_ACLK_300_DISP1_SEL	        0x0
#define MUX_ACLK_300_DISP1_MID_SEL      0x0
#define MUX_ACLK_200_SEL	        	0x0
#define MUX_ACLK_166_SEL	        	0x0
#define CLK_SRC_TOP0_VAL	((MUX_ACLK_300_GSCL_SEL  << 25)		\
				| (MUX_ACLK_300_GSCL_MID_SEL << 24)	\
				| (MUX_ACLK_400_G3D_MID_SEL << 20)	\
				| (MUX_ACLK_333_SEL << 16)		\
				| (MUX_ACLK_300_DISP1_SEL << 15)	\
				| (MUX_ACLK_300_DISP1_MID_SEL << 14)	\
				| (MUX_ACLK_200_SEL << 12)		\
				| (MUX_ACLK_166_SEL << 8))

/* CLK_SRC_TOP1	*/
#define MUX_ACLK_400_G3D_SEL            0x1
#define MUX_ACLK_400_ISP_SEL            0x0
#define MUX_ACLK_400_IOP_SEL            0x0
#define MUX_ACLK_MIPI_HSI_TXBASE_SEL    0x0
#define MUX_ACLK_300_GSCL_MID1_SEL      0x0
#define MUX_ACLK_300_DISP1_MID1_SEL     0x0
#define CLK_SRC_TOP1_VAL	((MUX_ACLK_400_G3D_SEL << 28)           \
				|(MUX_ACLK_400_ISP_SEL << 24)           \
				|(MUX_ACLK_400_IOP_SEL << 20)           \
				|(MUX_ACLK_MIPI_HSI_TXBASE_SEL << 16)   \
				|(MUX_ACLK_300_GSCL_MID1_SEL << 12)     \
				|(MUX_ACLK_300_DISP1_MID1_SEL << 8))

/* CLK_SRC_TOP2 */
#define MUX_GPLL_SEL                    0x1
#define MUX_BPLL_USER_SEL               0x0
#define MUX_MPLL_USER_SEL               0x0
#define MUX_VPLL_SEL                    0x1
#define MUX_EPLL_SEL                    0x1
#define MUX_CPLL_SEL                    0x1
#define VPLLSRC_SEL                     0x0
#define CLK_SRC_TOP2_VAL	((MUX_GPLL_SEL << 28)		\
				| (MUX_BPLL_USER_SEL << 24)	\
				| (MUX_MPLL_USER_SEL << 20)	\
				| (MUX_VPLL_SEL << 16)	        \
				| (MUX_EPLL_SEL << 12)	        \
				| (MUX_CPLL_SEL << 8)           \
				| (VPLLSRC_SEL))
/* THIS ONLY SET BPLL_USER_SEL & MUX_MPLL_USER_SEL */
#define TOP2_VAL		0x00110000


/* CLK_SRC_TOP3 */
#define MUX_ACLK_333_SUB_SEL            0x1
#define MUX_ACLK_400_ISP_SUB_SEL        0x1
#define MUX_ACLK_266_ISP_SUB_SEL        0x1
#define MUX_ACLK_266_GPS_SUB_SEL        0x0 				//reserved
#define MUX_ACLK_300_GSCL_SUB_SEL       0x1
#define MUX_ACLK_266_GSCL_SUB_SEL       0x1
#define MUX_ACLK_300_DISP1_SUB_SEL      0x1
#define MUX_ACLK_200_DISP1_SUB_SEL      0x1
#define CLK_SRC_TOP3_VAL	((MUX_ACLK_333_SUB_SEL << 24)	        \
				| (MUX_ACLK_400_ISP_SUB_SEL << 20)	        \
				| (MUX_ACLK_266_ISP_SUB_SEL << 16)			\
				| (MUX_ACLK_266_GPS_SUB_SEL << 12)     		 \
				| (MUX_ACLK_300_GSCL_SUB_SEL << 10)    		 \
				| (MUX_ACLK_266_GSCL_SUB_SEL << 8)      		\
				| (MUX_ACLK_300_DISP1_SUB_SEL << 6)     		\
				| (MUX_ACLK_200_DISP1_SUB_SEL << 4))

/* CLK_DIV_TOP0	*/
#define ACLK_300_DISP1_RATIO	0x2
#define ACLK_400_G3D_RATIO		0x0
#define ACLK_333_RATIO			0x0
#define ACLK_266_RATIO			0x2
#define ACLK_200_RATIO			0x3
#define ACLK_166_RATIO			0x1
#define ACLK_133_RATIO			0x1		//undefined
#define ACLK_66_RATIO			0x5

#define CLK_DIV_TOP0_VAL	((ACLK_300_DISP1_RATIO << 28)	\
				| (ACLK_400_G3D_RATIO << 24)	\
				| (ACLK_333_RATIO  << 20)	\
				| (ACLK_266_RATIO << 16)	\
				| (ACLK_200_RATIO << 12)	\
				| (ACLK_166_RATIO << 8)		\
				| (ACLK_133_RATIO << 4)		\
				| (ACLK_66_RATIO))

/* CLK_DIV_TOP1	*/
#define ACLK_MIPI_HSI_TX_BASE_RATIO     0x3
#define ACLK_66_PRE_RATIO               0x1
#define ACLK_400_ISP_RATIO              0x1
#define ACLK_400_IOP_RATIO              0x1
#define ACLK_300_GSCL_RATIO             0x2

#define CLK_DIV_TOP1_VAL	((ACLK_MIPI_HSI_TX_BASE_RATIO << 28)	\
				| (ACLK_66_PRE_RATIO << 24)		\
				| (ACLK_400_ISP_RATIO  << 20)		\
				| (ACLK_400_IOP_RATIO << 16)		\
				| (ACLK_300_GSCL_RATIO << 12))

/* APLL_LOCK	*/
#define APLL_LOCK_VAL	(0x546)
/* MPLL_LOCK	*/
#define MPLL_LOCK_VAL	(0x546)
/* CPLL_LOCK	*/
#define CPLL_LOCK_VAL	(0x546)
/* GPLL_LOCK	*/
#define GPLL_LOCK_VAL	(0x546)
/* EPLL_LOCK	*/
#define EPLL_LOCK_VAL	(0x3A98)
/* VPLL_LOCK	*/
#define VPLL_LOCK_VAL	(0x3A98)
/* BPLL_LOCK	*/
#define BPLL_LOCK_VAL	(0x546)

/* CLK_SRC_CPU */
#define MUX_APLL_SEL_MASK	(1 << 0)
#define MUX_HPM_SEL_MASK	(1 << 20)
/* CLK_SRC_CORE1 */
#define MUX_MPLL_SEL_MASK	(1 << 8)
/* CLK_MUX_STAT_CORE1 */
#define MPLL_SEL_MOUT_MPLLFOUT	(2 << 8)
/* CLK_SRC_TOP2 */
#define MUX_CPLL_SEL_MASK	(1 << 8)
#define MUX_EPLL_SEL_MASK	(1 << 12)
#define MUX_VPLL_SEL_MASK	(1 << 16)
#define MUX_GPLL_SEL_MASK	(1 << 28)
/* CLK_SRC_CDREX */
#define MUX_BPLL_SEL_MASK	(1 << 0)
/* CLK_MUX_STAT_CPU */
#define HPM_SEL_SCLK_MPLL	(1 << 21)
/* APLL_CON0 */
#define APLL_CON0_LOCKED	(1 << 29)
/* MPLL_CON0 */
#define MPLL_CON0_LOCKED	(1 << 29)
/* BPLL_CON0 */
#define BPLL_CON0_LOCKED	(1 << 29)
/* CPLL_CON0 */
#define CPLL_CON0_LOCKED	(1 << 29)
/* EPLL_CON0 */
#define EPLL_CON0_LOCKED	(1 << 29)
/* GPLL_CON0 */
#define GPLL_CON0_LOCKED	(1 << 29)
/* VPLL_CON0 */
#define VPLL_CON0_LOCKED	(1 << 29)

#define CLK_REG_DISABLE		0x0



/* CLK_SRC_PERIC0 */
#define PWM_SEL		6
#define UART3_SEL	6
#define UART2_SEL	6
#define UART1_SEL	6
#define UART0_SEL	6
/* SRC_CLOCK = SCLK_MPLL */
#define CLK_SRC_PERIC0_VAL	((PWM_SEL << 24)        \
				| (UART3_SEL << 12)     \
				| (UART2_SEL << 8)       \
				| (UART1_SEL << 4)      \
				| (UART0_SEL))

/* CLK_SRC_PERIC1 */
/* SRC_CLOCK = SCLK_MPLL */
#define SPI0_SEL		6
#define SPI1_SEL		6
#define SPI2_SEL		6
#define CLK_SRC_PERIC1_VAL	((SPI2_SEL << 24) \
				| (SPI1_SEL << 20) \
				| (SPI0_SEL << 16))

/* SCLK_SRC_ISP - set SPI0/1 to 6 = SCLK_MPLL_USER */
#define SPI0_ISP_SEL		6
#define SPI1_ISP_SEL		6
#define SCLK_SRC_ISP_VAL	(SPI1_ISP_SEL << 4) \
				| (SPI0_ISP_SEL << 0)

/* SCLK_DIV_ISP - set SPI0/1 to 0xf = divide by 16 */
#define SPI0_ISP_RATIO		0xf
#define SPI1_ISP_RATIO		0xf
#define SCLK_DIV_ISP_VAL	((SPI1_ISP_RATIO << 12) | (SPI0_ISP_RATIO << 0))

/* CLK_DIV_PERIL0	*/
#define UART5_RATIO	7
#define UART4_RATIO	7
#define UART3_RATIO	7
#define UART2_RATIO	7
#define UART1_RATIO	7
#define UART0_RATIO	7
/* divide by 8 */
#define CLK_DIV_PERIC0_VAL	((UART3_RATIO << 12)    \
							| (UART2_RATIO << 8)    \
							| (UART1_RATIO << 4)    \
							| (UART0_RATIO))

/* CLK_DIV_PERIC1 */
#define SPI1_RATIO			0x7
#define SPI1_PRE_RATIO		0x0
#define SPI0_RATIO			0xf
#define SPI0_PRE_RATIO		0x0
/*
 * SPI1: divide by 8
 * SPI2: divide by 16
 */
#define CLK_DIV_PERIC1_VAL	((SPI1_PRE_RATIO << 24) \
							| ((SPI1_RATIO << 16) \
							| (SPI0_PRE_RATIO << 8) \
							| (SPI0_RATIO << 0)))


/* CLK_DIV_PERIC2 */
/* SPI2 divide by 16*/
#define SPI2_RATIO				0xf
#define SPI2_PRE_RATIO			0x0
#define CLK_DIV_PERIC2_VAL		((SPI2_PRE_RATIO << 8) \
								| (SPI2_RATIO << 0))


/* CLK_DIV_PERIC3 */
/* This register is missing in data sheet
 * so I guess PWM is divided by 9 */
#define PWM_RATIO				9
#define CLK_DIV_PERIC3_VAL		(PWM_RATIO << 0)

/* CLK_DIV_FSYS0 */
#define USBDRD30_RATIO_VAL		0xB		//divide by 12
#define STAT_RATIO_VAL			0xB		//divide by 12
#define CLK_DIV_FSYS0_VAL		((USBDRD30_RATIO_VAL << 24) \
								| (STAT_RATIO_VAL << 20))

/* CLK_DIV_FSYS1 */
#define MMC1_PRE_RATIO_VAL		0x0
#define MMC1_RATIO_VAL			0x0
#define MMC0_PRE_RATIO_VAL		0x0
#define MMC0_RATIO_VAL			0x0
#define CLK_DIV_FSYS1_VAL		((MMC1_PRE_RATIO_VAL << 24)	\
								|(MMC1_RATIO_VAL << 16)		\
								|(MMC0_PRE_RATIO_VAL << 8)	\
								|(MMC0_RATIO_VAL))

/* CLK_DIV_FSYS2 */
/* MMC2: divide by 4*/
#define MMC2_RATIO_MASK			0xf
#define MMC2_RATIO_VAL			0x3
#define MMC2_RATIO_OFFSET		0
/* MMC2: pre-divide by 10 */
#define MMC2_PRE_RATIO_MASK		0xff
#define MMC2_PRE_RATIO_VAL		0x9
#define MMC2_PRE_RATIO_OFFSET	8
/* MMC3: divide by 2*/
#define MMC3_RATIO_MASK			0xf
#define MMC3_RATIO_VAL			0x1
#define MMC3_RATIO_OFFSET		16
/* MMC3: pre-divide by 1 */
#define MMC3_PRE_RATIO_MASK		0xff
#define MMC3_PRE_RATIO_VAL		0x0
#define MMC3_PRE_RATIO_OFFSET	24
#define CLK_DIV_FSYS2_VAL		((MMC3_PRE_RATIO_VAL << 24)	\
								|(MMC3_RATIO_VAL << 16)		\
								|(MMC2_PRE_RATIO_VAL << 8)	\
								|(MMC2_RATIO_VAL))



/* CLK_SRC_LEX */
/* MUX_ATCLK_LEX
 * 0 choose ACLK_200
 */
#define MUX_ATCLK_LEX_VAL		0x0
#define CLK_SRC_LEX_VAL         (MUX_ATCLK_LEX_VAL << 0)

/* CLK_DIV_LEX */
#define ATCLK_LEX_RATIO_VAL		0x0
#define PCLK_LEX_RATIO_VAL		0x1
#define CLK_DIV_LEX_VAL         ((ATCLK_LEX_RATIO_VAL << 8) \
								|(PCLK_LEX_RATIO_VAL << 4))

/* CLK_DIV_R0X */
#define PCLK_R0X_RATIO_VAL		0x1
#define CLK_DIV_R0X_VAL         (PCLK_R0X_RATIO_VAL << 4)

/* CLK_DIV_R1X */
#define PCLK_R1X_RATIO_VAL		0x1
#define CLK_DIV_R1X_VAL         (PCLK_R1X_RATIO_VAL << 4)

/* CLK_DIV_ISP0 */
#define ISPDIV1_RATIO_VAL		0x3
#define ISPDIV0_RATIO_VAL		0x1
#define CLK_DIV_ISP0_VAL        ((ISPDIV1_RATIO_VAL << 4)| (ISPDIV0_RATIO_VAL))		//0x31

/* CLK_DIV_ISP1 */
#define MCUISPDIV1_RATIO_VAL	0x3
#define MCUISPDIV0_RATIO_VAL	0x1
#define CLK_DIV_ISP1_VAL		((MCUISPDIV1_RATIO_VAL << 4) \
								| (MCUISPDIV0_RATIO_VAL))

/* CLK_DIV_ISP2 */
#define MPWMDIV_RATIO_VAL		0x0
#define CLK_DIV_ISP2_VAL        (MPWMDIV_RATIO_VAL)

/* CLK_SRC_DISP1_0 */
#define MUX_HDMI_SEL_VAL			0x0
#define MUX_DP1_EXT_MST_VID_VAL		0x0
#define MUX_MIPI1_SEL_VAL			0x0
#define MUX_FIMD1_SEL_VAL			0x6
#define CLK_SRC_DISP1_0_VAL		((MUX_HDMI_SEL_VAL << 20) \
								|(MUX_DP1_EXT_MST_VID_VAL << 16)	\
								|(MUX_MIPI1_SEL_VAL << 12)			\
								|(MUX_FIMD1_SEL_VAL))				//0x6

/*
 * DIV_DISP1_0
 * For DP, divisor should be 2
 */
#define CLK_DIV_DISP1_0_FIMD1			(2 << 0)

#define HDMI_PIXEL_RATIO_VAL			0x0
#define DP1_EXT_MST_VID_RATIO_VAL		0x0
#define MIPI1_PRE_RATIO_VAL				0x0
#define MIPI1_RATIO_VAL					0x0
#define FIMD1_RATIO_VAL					0x2
#define CLK_DIV_DISP1_O_VAL			((HDMI_PIXEL_RATIO_VAL << 28) 		\
									|(DP1_EXT_MST_VID_RATIO_VAL << 24)	\
									|(MIPI1_PRE_RATIO_VAL << 20)		\
									|(MIPI1_RATIO_VAL << 16)			\
									|(MIPI1_RATIO_VAL))


/* CLK_GATE_IP_DISP1 */
#define CLK_GATE_DP1_ALLOW				(1 << 4)

#define DDR3PHY_CTRL_PHY_RESET			(1 << 0)
#define DDR3PHY_CTRL_PHY_RESET_OFF		(0 << 0)

#define PHY_CON0_RESET_VAL				0x17020a40
#define P0_CMD_EN						(1 << 14)
#define BYTE_RDLVL_EN					(1 << 13)
#define CTRL_SHGATE						(1 << 8)

#define PHY_CON1_RESET_VAL				0x09210100
#define CTRL_GATEDURADJ_MASK			(0xf << 20)

#define PHY_CON2_RESET_VAL				0x00010004
#define INIT_DESKEW_EN					(1 << 6)
#define RDLVL_GATE_EN					(1 << 24)

/*ZQ Configurations */
#define PHY_CON16_RESET_VAL				0x08000304

#define ZQ_CLK_DIV_EN					(1 << 18)
#define ZQ_MANUAL_STR					(1 << 1)
#define ZQ_DONE							(1 << 0)

#define CTRL_RDLVL_GATE_ENABLE			1
#define CTRL_RDLVL_GATE_DISABLE			1

/* Direct Command */
#define DIRECT_CMD_NOP					0x07000000
#define DIRECT_CMD_PALL					0x01000000
#define DIRECT_CMD_ZQINIT				0x0a000000
#define DIRECT_CMD_CHANNEL_SHIFT		28
#define DIRECT_CMD_CHIP_SHIFT			20

/* DMC PHY Control0 register */
#define PHY_CONTROL0_RESET_VAL			0x0
#define MEM_TERM_EN						(1U << 31)	/* Termination enable for memory */
#define PHY_TERM_EN						(1 << 30)	/* Termination enable for PHY */
#define DMC_CTRL_SHGATE					(1 << 29)	/* Duration of DQS gating signal */
#define FP_RSYNC						(1 << 3)	/* Force DLL resyncronization */

/* Driver strength for CK, CKE, CS & CA */
#define IMP_OUTPUT_DRV_40_OHM			0x5
#define IMP_OUTPUT_DRV_30_OHM			0x7
#define CA_CK_DRVR_DS_OFFSET			9
#define CA_CKE_DRVR_DS_OFFSET			6
#define CA_CS_DRVR_DS_OFFSET			3
#define CA_ADR_DRVR_DS_OFFSET			0

#define PHY_CON42_CTRL_BSTLEN_SHIFT	8
#define PHY_CON42_CTRL_RDLAT_SHIFT	0

struct mem_timings;



#endif /* EXYNOS_CONFIG_H_ */
