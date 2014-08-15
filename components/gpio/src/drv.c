/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <platsupport/mux.h>
#include <platsupport/gpio.h>

#include "utils.h"

#include "gpio.h"

#define XEINT0  GPIOID(GPX0, 0)
#define XEINT1  GPIOID(GPX0, 1)
#define XEINT2  GPIOID(GPX0, 2)
#define XEINT3  GPIOID(GPX0, 3)
#define XEINT4  GPIOID(GPX0, 4)
#define XEINT5  GPIOID(GPX0, 5)
#define XEINT6  GPIOID(GPX0, 6)
#define XEINT7  GPIOID(GPX0, 7)
#define XEINT8  GPIOID(GPX1, 0)
#define XEINT9  GPIOID(GPX1, 1)
#define XEINT10 GPIOID(GPX1, 2)
#define XEINT11 GPIOID(GPX1, 3)
#define XEINT12 GPIOID(GPX1, 4)
#define XEINT13 GPIOID(GPX1, 5)
#define XEINT14 GPIOID(GPX1, 6)
#define XEINT15 GPIOID(GPX1, 7)
#define XEINT16 GPIOID(GPX2, 0)
#define XEINT17 GPIOID(GPX2, 1)
#define XEINT18 GPIOID(GPX2, 2)
#define XEINT19 GPIOID(GPX2, 3)
#define XEINT20 GPIOID(GPX2, 4)
#define XEINT21 GPIOID(GPX2, 5)
#define XEINT22 GPIOID(GPX2, 6)
#define XEINT23 GPIOID(GPX2, 7)
#define XEINT24 GPIOID(GPX3, 0)
#define XEINT25 GPIOID(GPX3, 1)
#define XEINT26 GPIOID(GPX3, 2)
#define XEINT27 GPIOID(GPX3, 3)
#define XEINT28 GPIOID(GPX3, 4)
#define XEINT29 GPIOID(GPX3, 5)
#define XEINT30 GPIOID(GPX3, 6)
#define XEINT31 GPIOID(GPX3, 7)
#define UART0_CTSN  GPIOID(GPA0, 2)
#define UART0_RTSN  GPIOID(GPA0, 3)
#define CAN_CSn          XEINT16
#define CAN_INTn         XEINT15
#define CAN_RESETn       XEINT25
#define MPU_CS           XEINT14
#define MPU_INT          XEINT8
#define ACC_MAG_CS       XEINT21
#define ACC_INT          XEINT18
#define MAG_INT          XEINT23
#define GYRO_CS          XEINT11
#define GYRO_INT         XEINT20
#define BARO_CS          XEINT10
#define SPI_EXT_CS       XEINT13
#define SPI_EXT_INT      XEINT19

mux_sys_t exynos_mux;
gpio_sys_t gpio_sys;
gpio_t gpio;

void gpio__init(void) {

	exynos_mux_init(gpio1base, gpio2base, NULL, NULL, &exynos_mux);

	/* Enable UART0, UART3 and SPI0. */
	mux_feature_enable(&exynos_mux, MUX_UART0);
	mux_feature_enable(&exynos_mux, MUX_UART1);
	mux_feature_enable(&exynos_mux, MUX_UART3);
	mux_feature_enable(&exynos_mux, MUX_SPI1);

	exynos_gpio_sys_init(&exynos_mux, &gpio_sys);

	gpio_new(&gpio_sys, MPU_CS, GPIO_DIR_OUT, &gpio);
	gpio_set(&gpio);

	gpio_new(&gpio_sys, ACC_MAG_CS, GPIO_DIR_OUT, &gpio);
	gpio_set(&gpio);

	gpio_new(&gpio_sys, GYRO_CS, GPIO_DIR_OUT, &gpio);
	gpio_set(&gpio);

	gpio_new(&gpio_sys, BARO_CS, GPIO_DIR_OUT, &gpio);
	gpio_set(&gpio);

	gpio_new(&gpio_sys, SPI_EXT_CS, GPIO_DIR_OUT, &gpio);
	gpio_set(&gpio);

	gpio_new(&gpio_sys, CAN_CSn, GPIO_DIR_OUT, &gpio);
	gpio_set(&gpio);
}

int gpio_mmc_config(int peripheral, int flags)
{
	/* TODO: To be moved to libplatsupport. */
}

int gpio_pinmux_config(int peripheral, int flags)
{
	char *buf = (char*)gpio1base;

	if (flags) {
		gpio_clr(&gpio);
	} else {
		gpio_set(&gpio);
	}
}

int run(void)
{
	gpio_t gpio_int;
	gpio_new(&gpio_sys, CAN_INTn, GPIO_DIR_IN, &gpio_int);

	while (1) {
		if (!gpio_get(&gpio_int)) {
			printf("Interrupt Pass ...\n");
			break;
		}
	}
}
