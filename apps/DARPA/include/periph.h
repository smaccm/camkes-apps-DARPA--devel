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
 * periph.h
 *
 *  Created on: Aug 5, 2013
 *      Author: jxie
 */

#ifndef PERIPH_H_
#define PERIPH_H_


enum {
	PINMUX_FLAG_NONE	= 0x00000000,

	/* Flags for eMMC */
	PINMUX_FLAG_8BIT_MODE	= 1 << 0,       /* SDMMC 8-bit mode */

	/* Flags for SROM controller */
	PINMUX_FLAG_BANK	= 3 << 0,       /* bank number (0-3) */
	PINMUX_FLAG_16BIT	= 1 << 2,       /* 16-bit width */
};


/*
 * Peripherals required for pinmux configuration. List will
 * grow with support for more devices getting added.
 * Numbering based on interrupt table.
 *
 */
enum periph_id {

	PERIPH_ID_UART0 = 51,
	PERIPH_ID_UART1,
	PERIPH_ID_UART2,
	PERIPH_ID_UART3,
	PERIPH_ID_I2C0 = 56,
	PERIPH_ID_I2C1,
	PERIPH_ID_I2C2,
	PERIPH_ID_I2C3,
	PERIPH_ID_I2C4,
	PERIPH_ID_I2C5,
	PERIPH_ID_I2C6,
	PERIPH_ID_I2C7,
	PERIPH_ID_SPI0 = 68,
	PERIPH_ID_SPI1,
	PERIPH_ID_SPI2,
	PERIPH_ID_SDMMC0 = 75,
	PERIPH_ID_SDMMC1,
	PERIPH_ID_SDMMC2,
	PERIPH_ID_SDMMC3,
	PERIPH_ID_I2S1 = 99,

	/* Since following peripherals do
	 * not have shared peripheral interrupts (SPIs)
	 * they are numbered arbitiraly after the maximum
	 * SPIs Exynos has (128)
	 */
	PERIPH_ID_SROMC = 128,
	PERIPH_ID_SPI3,
	PERIPH_ID_SPI4,
	PERIPH_ID_SDMMC4,
	PERIPH_ID_PWM0,
	PERIPH_ID_PWM1,
	PERIPH_ID_PWM2,
	PERIPH_ID_PWM3,
	PERIPH_ID_PWM4,

	/* Slave devices that connected to SPI bus. */
	PERIPH_ID_CAN,
	PERIPH_ID_MPU,
	PERIPH_ID_ACCMAG,
	PERIPH_ID_GYRO,
	PERIPH_ID_BARO,
	PERIPH_ID_SPIEXT,

	PERIPH_ID_COUNT,
	PERIPH_ID_NONE = -1,
};


#endif /* PERIPH_H_ */
