/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#ifndef EXYNOS_GPIO_H_
#define EXYNOS_GPIO_H_


/* one two-bit field per GPIO */
#define GP_PUD_BITS(x) 		((x)*2)
/*
 * For setting pad mux
 */
//volatile void 	*		gpabase_data;
#define GPABASE_ADDR		(uint32_t*) gpio1base_data

//SPI1
#define gpa2con 			(GPABASE_ADDR + 0x0040/4)
#define gpa2pud 			(GPABASE_ADDR + 0x0048/4)
#define gpa2drv 			(GPABASE_ADDR + 0x004C/4)
//UART0
#define gpa0con 			(GPABASE_ADDR + 0x0000/4)
#define gpa0pud 			(GPABASE_ADDR + 0x0008/4)
#define gpa0drv 			(GPABASE_ADDR + 0x000C/4)
//UART 2/3
#define gpa1con 			(GPABASE_ADDR + 0x0020/4)
#define gpa1pud 			(GPABASE_ADDR + 0x0028/4)
#define gpa1drv 			(GPABASE_ADDR + 0x002C/4)
//uart1
#define gpd0con 			(GPABASE_ADDR + 0x0160/4)
#define gpd0pud 			(GPABASE_ADDR + 0x0168/4)
#define gpd0drv 			(GPABASE_ADDR + 0x016C/4)

#define PULL_UP_DISABLED 		0
#define PULL_DOWN        		1
#define PULL_UP          		3



#define CON_MASK(x)			(0xf << ((x) << 2))
#define CON_SFR(x, v)		((v) << ((x) << 2))

#define DAT_MASK(x)			(0x1 << (x))
#define DAT_SET(x)			(0x1 << (x))

#define PULL_MASK(x)		(0x3 << ((x) << 1))
#define PULL_MODE(x, v)		((v) << ((x) << 1))

#define DRV_MASK(x)			(0x3 << ((x) << 1))
#define DRV_SET(x, m)		((m) << ((x) << 1))
#define RATE_MASK(x)		(0x1 << (x + 16))
#define RATE_SET(x)			(0x1 << (x + 16))

#define EINT_CON(b)			(b + 0x700)
#define EINT_FLT(b)			(b + 0x800)
#define EINT_MASK(b)		(b + 0x900)
#define EINT_PEND(b)		(b + 0xA00)


/* GPIO pins per bank  */
#define GPIO_PER_BANK 	8

/* Pin configurations */
#define GPIO_INPUT		0x0
#define GPIO_OUTPUT		0x1
#define GPIO_IRQ		0xf
#define GPIO_FUNC(x)	(x)

/* Pull mode */
#define GPIO_PULL_NONE	0x0
#define GPIO_PULL_DOWN	0x1
#define GPIO_PULL_UP	0x3

/* Drive Strength level */
#define GPIO_DRV_1X		0x0
#define GPIO_DRV_3X		0x1
#define GPIO_DRV_2X		0x2
#define GPIO_DRV_4X		0x3
#define GPIO_DRV_FAST	0x0
#define GPIO_DRV_SLOW	0x1

/* IRQ types common for all s5p platforms */
#define S5P_IRQ_TYPE_LEVEL_LOW		(0x00)
#define S5P_IRQ_TYPE_LEVEL_HIGH		(0x01)
#define S5P_IRQ_TYPE_EDGE_FALLING	(0x02)
#define S5P_IRQ_TYPE_EDGE_RISING	(0x03)
#define S5P_IRQ_TYPE_EDGE_BOTH		(0x04)

enum {
	IRQ_TYPE_NONE		= 0x00000000,
	IRQ_TYPE_EDGE_RISING	= 0x00000001,
	IRQ_TYPE_EDGE_FALLING	= 0x00000002,
	IRQ_TYPE_EDGE_BOTH	= (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING),
	IRQ_TYPE_LEVEL_HIGH	= 0x00000004,
	IRQ_TYPE_LEVEL_LOW	= 0x00000008,
	IRQ_TYPE_LEVEL_MASK	= (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH),
	IRQ_TYPE_SENSE_MASK	= 0x0000000f,
	IRQ_TYPE_DEFAULT	= IRQ_TYPE_SENSE_MASK,

	IRQ_TYPE_PROBE		= 0x00000010,

	IRQ_LEVEL		= (1 <<  8),
	IRQ_PER_CPU		= (1 <<  9),
	IRQ_NOPROBE		= (1 << 10),
	IRQ_NOREQUEST		= (1 << 11),
	IRQ_NOAUTOEN		= (1 << 12),
	IRQ_NO_BALANCING	= (1 << 13),
	IRQ_MOVE_PCNTXT		= (1 << 14),
	IRQ_NESTED_THREAD	= (1 << 15),
	IRQ_NOTHREAD		= (1 << 16),
	IRQ_PER_CPU_DEVID	= (1 << 17),
};



struct gpio_bank {
	unsigned int	con;
	unsigned int	dat;
	unsigned int	pull;
	unsigned int	drv;
	unsigned int	pdn_con;
	unsigned int	pdn_pull;
	unsigned char	res1[8];
};

struct exynos5_gpio_part1 {
	struct gpio_bank a0;
	struct gpio_bank a1;
	struct gpio_bank a2;
	struct gpio_bank b0;
	struct gpio_bank b1;
	struct gpio_bank b2;
	struct gpio_bank b3;
	struct gpio_bank c0;
	struct gpio_bank c1;
	struct gpio_bank c2;
	struct gpio_bank c3;
	struct gpio_bank d0;
	struct gpio_bank d1;
	struct gpio_bank y0;
	struct gpio_bank y1;
	struct gpio_bank y2;
	struct gpio_bank y3;
	struct gpio_bank y4;
	struct gpio_bank y5;
	struct gpio_bank y6;
	struct gpio_bank res1[0x3];
	struct gpio_bank c4;
	struct gpio_bank res2[0x48];
	struct gpio_bank x0;
	struct gpio_bank x1;
	struct gpio_bank x2;
	struct gpio_bank x3;
};

struct exynos5_gpio_part2 {
	struct gpio_bank e0;
	struct gpio_bank e1;
	struct gpio_bank f0;
	struct gpio_bank f1;
	struct gpio_bank g0;
	struct gpio_bank g1;
	struct gpio_bank g2;
	struct gpio_bank h0;
	struct gpio_bank h1;
};

struct exynos5_gpio_part3 {
	struct gpio_bank v0;
	struct gpio_bank v1;
	struct gpio_bank res1[0x1];
	struct gpio_bank v2;
	struct gpio_bank v3;
	struct gpio_bank res2[0x1];
	struct gpio_bank v4;
};

struct exynos5_gpio_part4 {
	struct gpio_bank z;
};


/* base = 0x1140_7000 */
struct exynos_ext_int_con_part1{
	uint32_t ext_int1_con;
	uint32_t ext_int2_con;
	uint32_t ext_int3_con;
	uint32_t ext_int4_con;
	uint32_t ext_int5_con;
	uint32_t ext_int6_con;
	uint32_t ext_int7_con;
	uint32_t ext_int8_con;
	uint32_t ext_int9_con;
	uint32_t ext_int10_con;
	uint32_t ext_int11_con;
	uint32_t ext_int12_con;
	uint32_t ext_int13_con;
	uint32_t ext_int30_con;
};
/* base = 0x1140_8000 */
struct exynos_ext_int_fltcon_part1{
	uint32_t ext_int1_fltcon[2];
	uint32_t ext_int2_fltcon[2];
	uint32_t ext_int3_fltcon[2];
	uint32_t ext_int4_fltcon[2];
	uint32_t ext_int5_fltcon[2];
	uint32_t ext_int6_fltcon[2];
	uint32_t ext_int7_fltcon[2];
	uint32_t ext_int8_fltcon[2];
	uint32_t ext_int9_fltcon[2];
	uint32_t ext_int10_fltcon[2];
	uint32_t ext_int11_fltcon[2];
	uint32_t ext_int12_fltcon[2];
	uint32_t ext_int13_fltcon[2];
	uint32_t ext_int30_fltcon[2];
};
/* base = 0x1140_9000 */
struct exynos_ext_int_mask_part1{
	uint32_t ext_int1_mask;
	uint32_t ext_int2_mask;
	uint32_t ext_int3_mask;
	uint32_t ext_int4_mask;
	uint32_t ext_int5_mask;
	uint32_t ext_int6_mask;
	uint32_t ext_int7_mask;
	uint32_t ext_int8_mask;
	uint32_t ext_int9_mask;
	uint32_t ext_int10_mask;
	uint32_t ext_int11_mask;
	uint32_t ext_int12_mask;
	uint32_t ext_int13_mask;
	uint32_t ext_int30_mask;
};
/* base = 0x1140_A000 */
struct exynos_ext_int_pend_part1{
	uint32_t ext_int1_pend;
	uint32_t ext_int2_pend;
	uint32_t ext_int3_pend;
	uint32_t ext_int4_pend;
	uint32_t ext_int5_pend;
	uint32_t ext_int6_pend;
	uint32_t ext_int7_pend;
	uint32_t ext_int8_pend;
	uint32_t ext_int9_pend;
	uint32_t ext_int10_pend;
	uint32_t ext_int11_pend;
	uint32_t ext_int12_pend;
	uint32_t ext_int13_pend;
	uint32_t ext_int30_pend;
};



struct exynos_ext_int_con_x{
	uint32_t ext_int40_con;
	uint32_t ext_int41_con;
	uint32_t ext_int42_con;
	uint32_t ext_int43_con;
};

struct exynos_ext_int_mask_x{
	uint32_t ext_int40_mask;
	uint32_t ext_int41_mask;
	uint32_t ext_int42_mask;
	uint32_t ext_int43_mask;
};

struct exynos_ext_int_pend_x{
	uint32_t ext_int40_pend;
	uint32_t ext_int41_pend;
	uint32_t ext_int42_pend;
	uint32_t ext_int43_pend;
};



/* base = 0x1340_7000 */
struct exynos_ext_int_con_part2{
	uint32_t ext_int14_con;
	uint32_t ext_int15_con;
	uint32_t ext_int16_con;
	uint32_t ext_int17_con;
	uint32_t ext_int18_con;
	uint32_t ext_int19_con;
	uint32_t ext_int20_con;
	uint32_t ext_int21_con;
	uint32_t ext_int22_con;
};
/* base = 0x1340_8000 */
struct exynos_ext_int_fltcon_part2{
	uint32_t ext_int14_fltcon;
	uint32_t ext_int15_fltcon;
	uint32_t ext_int16_fltcon;
	uint32_t ext_int17_fltcon;
	uint32_t ext_int18_fltcon;
	uint32_t ext_int19_fltcon;
	uint32_t ext_int20_fltcon;
	uint32_t ext_int21_fltcon;
	uint32_t ext_int22_fltcon;
};

/* base = 0x1340_9000 */
struct exynos_ext_int_mask_part2{
	uint32_t ext_int14_mask;
	uint32_t ext_int15_mask;
	uint32_t ext_int16_mask;
	uint32_t ext_int17_mask;
	uint32_t ext_int18_mask;
	uint32_t ext_int19_mask;
	uint32_t ext_int20_mask;
	uint32_t ext_int21_mask;
	uint32_t ext_int22_mask;
};

/* base = 0x1340_A000 */
struct exynos_ext_int_pend_part2{
	uint32_t ext_int14_pend;
	uint32_t ext_int15_pend;
	uint32_t ext_int16_pend;
	uint32_t ext_int17_pend;
	uint32_t ext_int18_pend;
	uint32_t ext_int19_pend;
	uint32_t ext_int20_pend;
	uint32_t ext_int21_pend;
	uint32_t ext_int22_pend;
};

struct exynos_ext_int_x{
	uint32_t GRPPRI;
	uint32_t priority;
	uint32_t service;
	uint32_t pend;
};

/*
 * Combiner Section
 */
#define COMBINER_PA_BASE		0x10440000
#define COMBINER_ENABLE_SET		0x0
#define COMBINER_ENABLE_CLEAR	0x4
#define COMBINER_INT_STATUS		0xC

struct combiner_data{
	uint32_t	IESR;
	uint32_t	IECR;
	uint32_t	ISTR;
	uint32_t	IMSR;
};

//Group number 0 ~ 31 (by %32)
//ING number can be found by GN % 4
#define MAX_GROUP_NUMBER	32
#define MAX_INT_PER_GROUP	8
#define ID_INTG(x)			(x / 4)	//find out the idx into combiner, x is group number 0 ~ 31
#define GROUP_BIT(x)		((x % 4) * MAX_INT_PER_GROUP)	//finds out the group bit position
#define INTG_BIT(x,y)		((GROUP_BIT(x) + y))

#define MAX_NUM_COMBINER	8




#endif
