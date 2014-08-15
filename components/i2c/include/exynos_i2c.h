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
 * exynos_i2c.h
 *
 *  Created on: Sep 25, 2013
 *      Author: jxie
 */

#ifndef EXYNOS_I2C_H_
#define EXYNOS_I2C_H_


#define NUM_I2C		8

typedef struct {
	volatile uint32_t CON;
	volatile uint32_t STAT;
	volatile uint32_t ADD;
	volatile uint32_t DS;
	volatile uint32_t LC;
}exynos_i2c, *exynos_i2c_p;

/* I2CCONx */
#define I2C_CON_ACK_GEN_SHIFT			7
#define I2C_CON_TX_CLK_SRC_SHIFT		6
#define I2C_CON_TXRX_INT_SHIFT			5
#define I2C_CON_TXRX_INT_PENDING_SHIFT	4
#define I2C_CON_TX_CLK_PRESCALE_SHIFT	0

#define I2C_CON_ACK_GEN_MASK			(1 << I2C_CON_ACK_GEN_SHIFT)
#define I2C_CON_TX_CLK_SRC_MASK			(1 << I2C_CON_TX_CLK_SRC_SHIFT)
#define I2C_CON_TXRX_INT_MASK			(1 << I2C_CON_TXRX_INT_SHIFT)
#define I2C_CON_TXRX_INT_PENDING_MASK	(1 << I2C_CON_TXRX_INT_PENDING_SHIFT)
#define I2C_CON_TX_CLK_PRESCALE_MASK	(0xf << I2C_CON_TX_CLK_PRESCALE_SHIFT)

/* I2CSTATx */
#define I2C_STAT_MODE_SEL_SHIFT			6
#define I2C_STAT_BUSY_STS_SHIFT			5
#define I2C_STAT_DATA_OUT_SHIFT			4
#define I2C_STAT_ARBITRATION_STS_SHIFT	3
#define I2C_STAT_ADDR_SLAVE_STS_SHIFT	2
#define I2C_STAT_ADDR_ZERO_STS_SHIFT	1
#define I2C_LAST_REC_BIT_STS_SHIFT		0

#define I2C_STAT_MODE_SEL_MASK			(0x3 << I2C_STAT_MODE_SEL_SHIFT)
#define I2C_STAT_BUSY_STS_MASK			(1 << I2C_STAT_BUSY_STS_SHIFT)
#define I2C_STAT_DATA_OUT_MASK			(1 << I2C_STAT_DATA_OUT_SHIFT)
#define I2C_STAT_ARBITRATION_STS_MASK	(1 << I2C_STAT_ARBITRATION_STS_SHIFT)
#define I2C_STAT_ADDR_SLAVE_STS_MASK	(1 << I2C_STAT_ADDR_SLAVE_STS_SHIFT)
#define I2C_STAT_ADDR_ZERO_STS_MASK		(1 << I2C_STAT_ADDR_ZERO_STS_SHIFT)
#define I2C_LAST_REC_BIT_STS_MASK		(1 << I2C_LAST_REC_BIT_STS_SHIFT)

/* I2CADDx */
#define I2C_ADD_SLAVE_SHIFT				0
#define I2C_ADD_SLAVE_MASK				(0xff << I2C_ADD_SLAVE_SHIFT)

/* I2CDSx: data shift register */
#define I2C_DS_SHIFT					0
#define I2C_DS_MASK						(0xff << I2C_DS_SHIFT)

/* I2CLCx */
#define I2C_LC_FILTER_SHIFT				2
#define I2C_LC_SDA_DELAY_SHIFT			0
#define I2C_LC_FILTER_MASK				(1 << I2C_LC_FILTER_SHIFT)
#define I2C_LC_SDA_DELAY_MASK			(0X3 << I2C_LC_SDA_DELAY_SHIFT)

/* macro */
#define I2C_EXT_VAL(reg, REG_FIELD)    (((reg) & I2C_##REG_FIELD##_MASK) >> I2C_##REG_FIELD##_SHIFT)
#define I2C_SET_VAL(REG_FIELD, val)	(((val) << I2C_##REG_FIELD##_SHIFT) & I2C_##REG_FIELD##_MASK)
#define I2C_INST_VAL(reg, REG_FIELD, val)	((reg) = ((reg) & ~ I2C_##REG_FIELD##_MASK) | I2C_SET_VAL(REG_FIELD, val))



#endif /* EXYNOS_I2C_H_ */
