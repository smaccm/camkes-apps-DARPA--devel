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

#define UART0_CTSN  GPIOID(GPA0, 2)
#define UART0_RTSN  GPIOID(GPA0, 3)

#define CAN_CSn     XEINT16
#define CAN_INTn    XEINT15
#define CAN_RESETn  XEINT25
#define MPU_CS      XEINT14
#define MPU_INT     XEINT8
#define ACC_MAG_CS  XEINT21
#define ACC_INT     XEINT18
#define MAG_INT     XEINT23
#define GYRO_CS     XEINT11
#define GYRO_INT    XEINT20
#define BARO_CS     XEINT10
#define SPI_EXT_CS  XEINT13
#define SPI_EXT_INT XEINT19
#define LIDAR_INT   UART0_CTSN

#define CS_FUNC(gpio)             \
    void                          \
    gpio_##gpio(int state)        \
    {                             \
        if(state)                 \
            gpio_set(&o_##gpio);  \
        else                      \
            gpio_clr(&o_##gpio);  \
    }


mux_sys_t exynos_mux;
gpio_sys_t gpio_sys;

/* SPI chip select */
gpio_t o_spi_can_nss;
gpio_t o_spi_mpu_nss;
gpio_t o_spi_acc_mag_nss;
gpio_t o_spi_gyro_nss;
gpio_t o_spi_baro_nss;
gpio_t o_spi_ext_nss;
CS_FUNC(spi_can_nss);
CS_FUNC(spi_mpu_nss);
CS_FUNC(spi_acc_mag_nss);
CS_FUNC(spi_gyro_nss);
CS_FUNC(spi_baro_nss);
CS_FUNC(spi_ext_nss);

/* SPI slave private IRQ */
gpio_t i_spi_can_int;
gpio_t i_spi_mpu_int;
gpio_t i_spi_acc_int;
gpio_t i_spi_mag_int;
gpio_t i_spi_gyro_int;
gpio_t i_spi_ext_int;
/* Lidar interrupt */
gpio_t i_lidar_int;

/* CAN reset */
gpio_t o_can_resetn;

void gpio__init(void) {
    exynos_mux_init(gpio1base, gpio2base, NULL, NULL, &exynos_mux);
    exynos_gpio_sys_init(&exynos_mux, &gpio_sys);

    /* Enable UART0, UART3 and SPI0. */
    mux_feature_enable(&exynos_mux, MUX_UART0);
    mux_feature_enable(&exynos_mux, MUX_UART1);
    mux_feature_enable(&exynos_mux, MUX_UART3);
    mux_feature_enable(&exynos_mux, MUX_SPI1);

    /* SPI chip selects */
    gpio_new(&gpio_sys, CAN_CSn,    GPIO_DIR_OUT, &o_spi_can_nss);
    gpio_new(&gpio_sys, MPU_CS,     GPIO_DIR_OUT, &o_spi_mpu_nss);
    gpio_new(&gpio_sys, ACC_MAG_CS, GPIO_DIR_OUT, &o_spi_acc_mag_nss);
    gpio_new(&gpio_sys, GYRO_CS,    GPIO_DIR_OUT, &o_spi_gyro_nss);
    gpio_new(&gpio_sys, BARO_CS,    GPIO_DIR_OUT, &o_spi_baro_nss);
    gpio_new(&gpio_sys, SPI_EXT_CS, GPIO_DIR_OUT, &o_spi_ext_nss);
    gpio_set(&o_spi_can_nss);
    gpio_set(&o_spi_mpu_nss);
    gpio_set(&o_spi_acc_mag_nss);
    gpio_set(&o_spi_gyro_nss);
    gpio_set(&o_spi_baro_nss);
    gpio_set(&o_spi_ext_nss);
    /* SPI private IRQ */
    gpio_new(&gpio_sys, CAN_INTn,    GPIO_DIR_IRQ_FALL, &i_spi_can_int);
    gpio_new(&gpio_sys, MPU_INT,     GPIO_DIR_IRQ_FALL, &i_spi_mpu_int);
    gpio_new(&gpio_sys, ACC_INT,     GPIO_DIR_IRQ_FALL, &i_spi_acc_int);
    gpio_new(&gpio_sys, MAG_INT,     GPIO_DIR_IRQ_FALL, &i_spi_mag_int);
    gpio_new(&gpio_sys, GYRO_INT,    GPIO_DIR_IRQ_FALL, &i_spi_gyro_int);
    gpio_new(&gpio_sys, SPI_EXT_INT, GPIO_DIR_IRQ_FALL, &i_spi_ext_int);

    /* LIDAR sync IRQ  */
    gpio_new(&gpio_sys, LIDAR_INT,   GPIO_DIR_IRQ_FALL, &i_lidar_int);

    /* CAN reset */
    gpio_new(&gpio_sys, CAN_RESETn,  GPIO_DIR_OUT,      &o_can_resetn);
    gpio_set(&o_can_resetn);
}

int gpio_mmc_config(int peripheral, int flags)
{
	/* TODO: To be moved to libplatsupport. */
    return -1;
}





int run(void)
{
	while (0) {
		if (!gpio_get(&i_spi_can_int)){
			printf("Interrupt Pass ...\n");
			break;
		}
	}
    return 0;
}
