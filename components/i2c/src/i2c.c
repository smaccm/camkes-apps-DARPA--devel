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
 * i2c.c
 *
 *  Created on: Sep 25, 2013
 *      Author: jxie
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "exynos_i2c.h"
#include "periph.h"

typedef struct{
	uint32_t		id;
	uint32_t	 	bus_freq;
	bool			ack_bit;
	bool
	exynos_i2c_p 	ctrl_reg;
}i2c_channel, *i2c_handler;

i2c_channel	i2c_device_list[NUM_I2C];

/* devid = devid - PERIPH_ID_I2C0 */

void i2c__init(void){
	int i = 0;
	exynos_i2c_p * i2c_ctrl_reg;

	for(i = 0; i < NUM_I2C; i++){
		i2c_device_list[i].bus_freq  = clk_get_i2c_freq();
		i2c_ctrl_reg = i2c_device_list[i].ctrl_reg;


		//DEBUG
		printf("I2C%d busfreq: %d\n", i, i2c_device_list[i].bus_freq);
	}
}
