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
 * gpio.c
 *
 *  Created on: Jun 5, 2013
 *      Author: Jiawei Xie
 */
#if 0
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <platsupport/mux.h>
#include "gpio.h"

#include "exynos_gpio.h"
#include "periph.h"

#define CIPSR0	(*(uint32_t *)((void *)irqcbase_data + 0x100))

struct exynos5_gpio_part1 * gpio1reg;
struct exynos5_gpio_part2 * gpio2reg;

struct exynos_ext_int_con_part1 * con1;
struct exynos_ext_int_con_part2 * con2;

struct exynos_ext_int_fltcon_part1 * flt1;
struct exynos_ext_int_fltcon_part2 * flt2;

struct exynos_ext_int_mask_part1 * msk1;
struct exynos_ext_int_mask_part2 * msk2;

struct exynos_ext_int_pend_part1 * pend1;
struct exynos_ext_int_pend_part2 * pend2;


struct exynos_ext_int_con_x * xcon;
struct exynos_ext_int_mask_x * xmsk;
struct exynos_ext_int_pend_x * xpend;

static int GPIO_HAS_INIT = 0;

void assign_base_addr(void){

	gpio1reg = (struct exynos5_gpio_part1 *) gpio1base;
	gpio2reg = (struct exynos5_gpio_part2 *) gpio2base;

	con1 = (struct exynos_ext_int_con_part1 *) EINT_CON((void*)gpio1base);
	flt1 = (struct exynos_ext_int_fltcon_part1 *) EINT_FLT((void*)gpio1base);
	msk1 = (struct exynos_ext_int_mask_part1 *) EINT_MASK((void*)gpio1base);
	pend1 = (struct exynos_ext_int_pend_part1 *) EINT_PEND((void*)gpio1base);

	con2 = (struct exynos_ext_int_con_part2 *) EINT_CON((void*)gpio2base);
	flt2 = (struct exynos_ext_int_fltcon_part2 *) EINT_FLT((void*)gpio2base);
	msk2 = (struct exynos_ext_int_mask_part2 *) EINT_MASK((void*)gpio2base);
	pend2 = (struct exynos_ext_int_pend_part2 *) EINT_PEND((void*)gpio2base);

	xcon = (struct exynos_ext_int_con_x *) ((void*)gpio1base + 0xE00);
	xmsk = (struct exynos_ext_int_mask_x *) ((void*)gpio1base + 0xF00);
	xpend = (struct exynos_ext_int_pend_x *) ((void*)gpio1base + 0xF40);
/*
	printf("gpio1reg = %x\n", (uint32_t) gpio1reg);
	printf("gpio2reg= %x\n", (uint32_t) gpio2reg);

	printf("con1 = %x\n", (uint32_t) con1);
	printf("flt1 = %x\n", (uint32_t) flt1);
	printf("msk1 = %x\n", (uint32_t) msk1);
	printf("pend1 = %x\n", (uint32_t) pend1);

	printf("con2 = %x\n", (uint32_t) con2);
	printf("flt2 = %x\n", (uint32_t) flt2);
	printf("msk2 = %x\n", (uint32_t) msk2);
	printf("pend2 = %x\n", (uint32_t) pend2);

	printf("xcon = %x\n", (uint32_t) xcon);
	printf("xmsk = %x\n", (uint32_t) xmsk);
	printf("xpend = %x\n", (uint32_t) xpend);
	*/
}




void gpio_cfg_pin(struct gpio_bank *bank, int gpio, int cfg)
{
	unsigned int value;

	value = bank->con;
	value &= ~CON_MASK(gpio);
	value |= CON_SFR(gpio, cfg);
	bank->con = value;
}

void gpio_direction_output(struct gpio_bank *bank, int gpio, int en)
{
	unsigned int value;

	gpio_cfg_pin(bank, gpio, GPIO_OUTPUT);

	value = bank->dat;
	value &= ~DAT_MASK(gpio);
	if (en)
		value |= DAT_SET(gpio);
	bank->dat = value;
}

void gpio_direction_input(struct gpio_bank *bank, int gpio)
{
	gpio_cfg_pin(bank, gpio, GPIO_INPUT);
}

void gpio_set_value(struct gpio_bank *bank, int gpio, int en)
{
	unsigned int value;

	value = bank->dat;
	value &= ~DAT_MASK(gpio);
	if (en)
		value |= DAT_SET(gpio);
	bank->dat = value;
}

unsigned int gpio_get_value(struct gpio_bank *bank, int gpio)
{
	unsigned int value;

	value = bank->dat;
	return !!(value & DAT_MASK(gpio));
}

void gpio_set_pull(struct gpio_bank *bank, int gpio, int mode)
{
	unsigned int value;

	value = bank->pull;
	value &= ~PULL_MASK(gpio);

	switch (mode) {
	case GPIO_PULL_DOWN:
	case GPIO_PULL_UP:
		value |= PULL_MODE(gpio, mode);
		break;
	default:
		break;
	}

	bank->pull = value;
}

void gpio_set_drv(struct gpio_bank *bank, int gpio, int mode)
{
	unsigned int value;

	value = bank->drv;
	value &= ~DRV_MASK(gpio);

	switch (mode) {
	case GPIO_DRV_1X:
	case GPIO_DRV_2X:
	case GPIO_DRV_3X:
	case GPIO_DRV_4X:
		value |= DRV_SET(gpio, mode);
		break;
	default:
		return;
	}

	bank->drv = value;
}

void gpio_set_rate(struct gpio_bank *bank, int gpio, int mode)
{
	unsigned int value;

	value = bank->drv;

	value &= ~RATE_MASK(gpio);

	switch (mode) {
	case GPIO_DRV_FAST:
	case GPIO_DRV_SLOW:
		value |= RATE_SET(gpio);
		break;
	default:
		return;
	}

	bank->drv = value;
}


static void spi_setmux(void){

    /* Set GPA2CON[4--7] to SPI_1 -- see manual */
	gpio_cfg_pin(&(gpio1reg->a2), 4, GPIO_FUNC(2)); //GPA2CON[4], SPI_1_CLK
	gpio_cfg_pin(&(gpio1reg->a2), 5, GPIO_FUNC(2)); //GPA2CON[5], SPI_1_nSS
	gpio_cfg_pin(&(gpio1reg->a2), 6, GPIO_FUNC(2)); //GPA2CON[6], SPI_1_MISO
	gpio_cfg_pin(&(gpio1reg->a2), 7, GPIO_FUNC(2)); //GPA2CON[7], SPI_1_MOSI

	gpio_set_pull(&(gpio1reg->a2), 4, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a2), 5, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a2), 6, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a2), 7, GPIO_PULL_UP);

	gpio_set_drv(&(gpio1reg->a2), 4, GPIO_DRV_3X);
	gpio_set_drv(&(gpio1reg->a2), 5, GPIO_DRV_3X);
	gpio_set_drv(&(gpio1reg->a2), 6, GPIO_DRV_3X);
	gpio_set_drv(&(gpio1reg->a2), 7, GPIO_DRV_3X);

}

static void uart_setmux(void){

	// uint32_t v = *gpa0con;

    /* Set GPA0CON[0--3] to UART0-- see manual */
	gpio_cfg_pin(&(gpio1reg->a0), 0, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->a0), 1, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->a0), 2, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->a0), 3, GPIO_FUNC(2));

	gpio_set_pull(&(gpio1reg->a0), 0, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a0), 1, GPIO_PULL_NONE);
	gpio_set_pull(&(gpio1reg->a0), 2, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a0), 3, GPIO_PULL_UP);

	gpio_set_drv(&(gpio1reg->a0), 0, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->a0), 1, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->a0), 2, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->a0), 3, GPIO_DRV_1X);

    /* Set GPD0CON[0--3] to UART1-- see manual */
	gpio_cfg_pin(&(gpio1reg->d0), 0, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->d0), 1, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->d0), 2, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->d0), 3, GPIO_FUNC(2));

	gpio_set_pull(&(gpio1reg->d0), 0, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->d0), 1, GPIO_PULL_NONE);
	gpio_set_pull(&(gpio1reg->d0), 2, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->d0), 3, GPIO_PULL_UP);

	gpio_set_drv(&(gpio1reg->d0), 0, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->d0), 1, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->d0), 2, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->d0), 3, GPIO_DRV_1X);


    /* Set GPA1CON[0--5] to UART2/3-- see manual */
	gpio_cfg_pin(&(gpio1reg->a1), 0, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->a1), 1, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->a1), 2, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->a1), 3, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->a1), 4, GPIO_FUNC(2));
	gpio_cfg_pin(&(gpio1reg->a1), 5, GPIO_FUNC(2));

	gpio_set_pull(&(gpio1reg->a1), 0, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a1), 1, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a1), 2, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a1), 3, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a1), 4, GPIO_PULL_UP);
	gpio_set_pull(&(gpio1reg->a1), 5, GPIO_PULL_UP);


	gpio_set_drv(&(gpio1reg->a1), 0, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->a1), 1, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->a1), 2, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->a1), 3, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->a1), 4, GPIO_DRV_1X);
	gpio_set_drv(&(gpio1reg->a1), 5, GPIO_DRV_1X);

/*
    v = *gpa1con;
    v &= ~((0xf << 20) | (0xf << 16) | (0xf << 12) | (0xf << 8) | (0xf << 4) | (0xf << 0));
    v |= (2 << 20) | (2 << 16) |(2 << 12) | (2 << 8) | (2 << 4) | (2 << 0);
    *gpa1con = v;

    v = *gpa1pud;
    v &= ~((3 << GP_PUD_BITS(5)) |
    	   (3 << GP_PUD_BITS(4)) |
    	   (3 << GP_PUD_BITS(3)) |
           (3 << GP_PUD_BITS(2)) |
           (3 << GP_PUD_BITS(1)) |
           (3 << GP_PUD_BITS(0)));

    v |= ((PULL_UP << GP_PUD_BITS(5)) |
    	  (PULL_UP << GP_PUD_BITS(4)) |
    	  (PULL_UP << GP_PUD_BITS(3)) |
          (PULL_UP << GP_PUD_BITS(2)) |
          (PULL_UP << GP_PUD_BITS(1)) |
          (PULL_UP << GP_PUD_BITS(0)));
    *gpa1pud = v;

    v = *gpa1drv;
    v = ( (1 << GP_PUD_BITS(5)) |
    	  (1 << GP_PUD_BITS(4)) |
    	  (1 << GP_PUD_BITS(3)) |
          (1 << GP_PUD_BITS(2)) |
          (1 << GP_PUD_BITS(1)) |
          (1 << GP_PUD_BITS(0)));
    *gpa1drv = 0;
    *gpa1drv */

}

static void spi_slave_setmux(void){

	/* CAN, MPU and Gyro */
	gpio_direction_output(&(gpio1reg->x2), 0, 0);
	gpio_direction_output(&(gpio1reg->x1), 6, 0);
	gpio_direction_output(&(gpio1reg->x1), 3, 0);
	gpio_direction_output(&(gpio1reg->x2), 5, 0);
	gpio_direction_output(&(gpio1reg->x1), 2, 0);
	gpio_direction_output(&(gpio1reg->x2), 3, 0);

	gpio_set_pull(&(gpio1reg->x2), 0, GPIO_PULL_NONE);
	gpio_set_pull(&(gpio1reg->x1), 6, GPIO_PULL_NONE);
	gpio_set_pull(&(gpio1reg->x1), 3, GPIO_PULL_DOWN);
	gpio_set_pull(&(gpio1reg->x1), 2, GPIO_PULL_NONE);
	gpio_set_pull(&(gpio1reg->x2), 3, GPIO_PULL_NONE);
	gpio_set_pull(&(gpio1reg->x2), 5, GPIO_PULL_NONE);

	gpio_set_drv(&(gpio1reg->x2), 0, GPIO_DRV_3X);
	gpio_set_drv(&(gpio1reg->x1), 6, GPIO_DRV_3X);
	gpio_set_drv(&(gpio1reg->x1), 3, GPIO_DRV_3X);
	gpio_set_drv(&(gpio1reg->x1), 2, GPIO_DRV_3X);
	gpio_set_drv(&(gpio1reg->x2), 3, GPIO_DRV_3X);
	gpio_set_drv(&(gpio1reg->x2), 5, GPIO_DRV_3X);

}

static inline void exynos_irq_eint_mask(uint32_t * msk, uint32_t bit)
{
	uint32_t mask;

	mask = *msk;
	mask |= (1 << bit);
	*msk = mask;
}

static void exynos_irq_eint_enable(uint32_t * msk, uint32_t bit)
{
	uint32_t mask;

	mask = *msk;
	mask &= ~(1 << bit);
	*msk = mask;
}

static inline void exynos_irq_eint_ack(uint32_t * pend, uint32_t bit)
{
	*pend = (1 << bit);
}


static int exynos_irq_eint_set_type(uint32_t * con, uint32_t bit, uint32_t type)
{

	int shift;
	uint32_t ctrl, mask;
	uint32_t newvalue = 0;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		newvalue = IRQ_TYPE_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		newvalue = IRQ_TYPE_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		newvalue = IRQ_TYPE_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		newvalue = IRQ_TYPE_LEVEL_LOW;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		newvalue = IRQ_TYPE_LEVEL_HIGH;
		break;

	default:
		//printk(KERN_ERR "No such irq type %d", type);
		return -1;
	}

	shift = (bit & 0x7) * 4;
	mask = 0x7 << shift;

	ctrl = *con;
	ctrl &= ~mask;
	ctrl |= (newvalue << shift);
	*con = ctrl;

	return 0;
}

struct combiner_data * combiner;

static void combiner_enable_irq(uint32_t reg_id, uint32_t bit)
{
	uint32_t mask = 1 << (bit % 32);
	reg_id = reg_id % MAX_NUM_COMBINER;
	combiner[reg_id].IESR = mask;
}

static void combiner_disable_irq(uint32_t reg_id, uint32_t bit)
{
	uint32_t mask = 1 << (bit % 32);
	reg_id = reg_id % MAX_NUM_COMBINER;
	combiner[reg_id].IECR = mask;
}


static void handle_EINT14_interrupt(void){
	/* PEND */
	exynos_irq_eint_ack(&(xpend->ext_int41_pend), 6);
}

static void EINT14_interrupt_handler(void *data UNUSED){
	handle_EINT14_interrupt();
	EINT14_reg_callback(&EINT14_interrupt_handler, NULL);
}


int gpio_check_for_gpe0_6_int(void){
	if((pend2->ext_int14_pend & (1 << 6))){
		//call CAN message interrupt handler
		//can_mcp2515_handle_interrupts();
		//CANInt_emit();
		//wait for acknowledgment from CAN interrupt
		//CANIntAck_wait();
		exynos_irq_eint_ack(&(pend2->ext_int14_pend), 6);
	}
	return 0;
}

/*
 * GPIO running thread
 */
int run(void)
{
	while(1){
		gpio_check_for_gpe0_6_int();
//		seL4_Yield();
	}
}

void wait_for_init(void){
	while(GPIO_HAS_INIT == 0){
		;
	}
}

mux_sys_t mux;
void gpio__init(void) {

	assign_base_addr();
    /* Setup multiplexor */
//	exynos_mux_init(gpio2base, gpio1base, NULL, NULL, &mux);
//	mux_feature_enable(&mux, MUX_SPI1);

	spi_setmux();
	uart_setmux();
	spi_slave_setmux();

	/* GPE0[6] registers*/
	gpio_cfg_pin( &(gpio2reg->e0), 6, GPIO_IRQ);
	gpio_set_pull( &(gpio2reg->e0), 6, GPIO_PULL_UP);
	/* EXT_INTx registers*/
	/* CON */
	exynos_irq_eint_set_type( &(con2->ext_int14_con), 6, IRQ_TYPE_LEVEL_LOW);
	/* MASK */
	exynos_irq_eint_enable( &(msk2->ext_int14_mask), 6);

	/* GPX1[6] registers*/
	gpio_cfg_pin( &(gpio1reg->x1), 6, GPIO_IRQ);
	gpio_set_pull( &(gpio1reg->x1), 6, GPIO_PULL_UP);
	/* EXT_INTx registers*/
	/* CON */
	exynos_irq_eint_set_type( &(xcon->ext_int41_con), 6, IRQ_TYPE_LEVEL_LOW);
	/* MASK */
	exynos_irq_eint_enable( &(xmsk->ext_int41_mask), 6);

	/* Combiner - EINT14 */
	combiner = (struct combiner_data *) irqcbase;
	combiner_enable_irq(ID_INTG(31), INTG_BIT(31,0));
	EINT14_reg_callback(&EINT14_interrupt_handler, NULL);
	GPIO_HAS_INIT = 1;
}


void gpio_dummy(void)
{
	//if(combiner[7].IESR & (1 << 24)){
	//	printf("IESR7 is enabled\n");
	//}
	//printf("[7].IESR %x\n", (uint32_t)&(combiner[7].IESR));
	//printf("SERVICE_XB %x\n",  (*(uint32_t *)((void*)gpio2base + 0xB08)));
	//printf("SVC_PEND %x\n",  (*(uint32_t *)((void*)gpio2base + 0xB0C)));

//	int timer = 0;
	int* EXT_INT_GRPFIXPRI_XB = (int *)((void *)gpio2base + 0xB10); // = 1 group 1 EINT14
	int* EXT_INT14_FIXPRI = (int *)((void *)gpio2base + 0xB14);	//= 6


	*EXT_INT_GRPFIXPRI_XB 	= 1;
	*EXT_INT14_FIXPRI 		= 6;

/*
	while(1){
		if(timer % 1000 == 0){
			printf("-----\n");
			if(pend2->ext_int14_pend & (1 << 6)){
				printf("EINT14 fired!\n\n");
			}
			printf("S 0x%x : 0x%x\n", (uint32_t)&combiner[7].ISTR, (uint32_t)combiner[7].ISTR);
			printf("M 0x%x : 0x%x\n", (uint32_t)&combiner[7].IMSR, (uint32_t)combiner[7].IMSR);
			printf("E 0x%x : 0x%x\n", (uint32_t)&combiner[7].IESR, (uint32_t)combiner[7].IESR);
			printf("\n");
			printf("P 0x%x : 0x%x\n", (uint32_t)&pend2->ext_int14_pend, (uint32_t)pend2->ext_int14_pend);
			printf("M 0x%x : 0x%x\n", (uint32_t)&msk2->ext_int14_mask , (uint32_t)msk2->ext_int14_mask);
			printf("SERVICE_XB %x\n",  (*(uint32_t *)((void*)gpio2base + 0xB08)));
			printf("SVC_PEND %x\n",  (*(uint32_t *)((void*)gpio2base + 0xB0C)));
			printf("\n");
			printf("xP 0x%x : 0x%x\n", (uint32_t)&xpend->ext_int41_pend, (uint32_t)xpend->ext_int41_pend);
			printf("xM 0x%x : 0x%x\n", (uint32_t)&xmsk->ext_int41_mask , (uint32_t)xmsk->ext_int41_mask);
			printf("\n\n");

			handle_interrupt();
		}
		timer ++;
	}

*/
}


/* ----------------------------------------------------------
 * 		PINMUX UTILITIES
 * ----------------------------------------------------------*/

static void gpio_uart_config(int peripheral)
{
	struct exynos5_gpio_part1 *gpio1 = gpio1reg;
	struct gpio_bank *bank;
	int i, start, count;

	switch (peripheral) {
	case PERIPH_ID_UART0:
		bank = &gpio1->a0;
		start = 0;
		count = 4;
		break;
	case PERIPH_ID_UART1:
		bank = &gpio1->d0;
		start = 0;
		count = 4;
		break;
	case PERIPH_ID_UART2:
		bank = &gpio1->a1;
		start = 0;
		count = 4;
		break;
	case PERIPH_ID_UART3:
		bank = &gpio1->a1;
		start = 4;
		count = 2;
		break;
	}
	for (i = start; i < start + count; i++) {
		gpio_set_pull(bank, i, GPIO_PULL_NONE);
		gpio_cfg_pin(bank, i, GPIO_FUNC(0x2));
	}
}

int gpio_mmc_config(int peripheral, int flags)
{
	wait_for_init();

	struct exynos5_gpio_part1 *gpio1 = gpio1reg;
	struct gpio_bank *bank, *bank_ext = NULL;
	int i, start = 0, gpio_func = 0;

	switch (peripheral) {
	case 0:
		bank = &gpio1->c0;
		bank_ext = &gpio1->c1;
		start = 0;
		gpio_func = GPIO_FUNC(0x2);
		break;
	case 1:
		bank = &gpio1->c2;
		bank_ext = NULL;
		break;
	case 2:
		bank = &gpio1->c3;
		bank_ext = &gpio1->c4;
		start = 3;
		gpio_func = GPIO_FUNC(0x3);
		break;
	case 3:
		bank = &gpio1->c4;
		bank_ext = NULL;
		break;
	default:
		printf("Device ID not supported !\n");
		return -1;
	}
	if ((flags & PINMUX_FLAG_8BIT_MODE) && !bank_ext) {
		printf("SDMMC device %d does not support 8bit mode",
				peripheral);
		return -1;
	}
	if (flags & PINMUX_FLAG_8BIT_MODE) {
		for (i = start; i <= (start + 3); i++) {
			gpio_cfg_pin(bank_ext, i, gpio_func);
			gpio_set_pull(bank_ext, i, GPIO_PULL_UP);
			gpio_set_drv(bank_ext, i, GPIO_DRV_4X);
		}
	}
	for (i = 0; i < 2; i++) {
		gpio_cfg_pin(bank, i, GPIO_FUNC(0x2));
		gpio_set_pull(bank, i, GPIO_PULL_NONE);
		gpio_set_drv(bank, i, GPIO_DRV_4X);
	}
	for (i = 3; i <= 6; i++) {
		gpio_cfg_pin(bank, i, GPIO_FUNC(0x2));
		gpio_set_pull(bank, i, GPIO_PULL_UP);
		gpio_set_drv(bank, i, GPIO_DRV_4X);
	}
	return 0;
}

static void gpio_sromc_config(int flags)
{
	struct exynos5_gpio_part1 *gpio1 = gpio1reg;
	int i;

	/*
	 * SROM:CS1 and EBI
	 *
	 * GPY0[0]	SROM_CSn[0]
	 * GPY0[1]	SROM_CSn[1](2)
	 * GPY0[2]	SROM_CSn[2]
	 * GPY0[3]	SROM_CSn[3]
	 * GPY0[4]	EBI_OEn(2)
	 * GPY0[5]	EBI_EEn(2)
	 *
	 * GPY1[0]	EBI_BEn[0](2)
	 * GPY1[1]	EBI_BEn[1](2)
	 * GPY1[2]	SROM_WAIT(2)
	 * GPY1[3]	EBI_DATA_RDn(2)
	 */
	gpio_cfg_pin(&gpio1->y0, (flags & PINMUX_FLAG_BANK),
				GPIO_FUNC(2));
	gpio_cfg_pin(&gpio1->y0, 4, GPIO_FUNC(2));
	gpio_cfg_pin(&gpio1->y0, 5, GPIO_FUNC(2));

	for (i = 0; i < 4; i++)
		gpio_cfg_pin(&gpio1->y1, i, GPIO_FUNC(2));

	/*
	 * EBI: 8 Addrss Lines
	 *
	 * GPY3[0]	EBI_ADDR[0](2)
	 * GPY3[1]	EBI_ADDR[1](2)
	 * GPY3[2]	EBI_ADDR[2](2)
	 * GPY3[3]	EBI_ADDR[3](2)
	 * GPY3[4]	EBI_ADDR[4](2)
	 * GPY3[5]	EBI_ADDR[5](2)
	 * GPY3[6]	EBI_ADDR[6](2)
	 * GPY3[7]	EBI_ADDR[7](2)
	 *
	 * EBI: 16 Data Lines
	 *
	 * GPY5[0]	EBI_DATA[0](2)
	 * GPY5[1]	EBI_DATA[1](2)
	 * GPY5[2]	EBI_DATA[2](2)
	 * GPY5[3]	EBI_DATA[3](2)
	 * GPY5[4]	EBI_DATA[4](2)
	 * GPY5[5]	EBI_DATA[5](2)
	 * GPY5[6]	EBI_DATA[6](2)
	 * GPY5[7]	EBI_DATA[7](2)
	 *
	 * GPY6[0]	EBI_DATA[8](2)
	 * GPY6[1]	EBI_DATA[9](2)
	 * GPY6[2]	EBI_DATA[10](2)
	 * GPY6[3]	EBI_DATA[11](2)
	 * GPY6[4]	EBI_DATA[12](2)
	 * GPY6[5]	EBI_DATA[13](2)
	 * GPY6[6]	EBI_DATA[14](2)
	 * GPY6[7]	EBI_DATA[15](2)
	 */
	for (i = 0; i < 8; i++) {
		gpio_cfg_pin(&gpio1->y3, i, GPIO_FUNC(2));
		gpio_set_pull(&gpio1->y3, i, GPIO_PULL_UP);

		gpio_cfg_pin(&gpio1->y5, i, GPIO_FUNC(2));
		gpio_set_pull(&gpio1->y5, i, GPIO_PULL_UP);

		gpio_cfg_pin(&gpio1->y6, i, GPIO_FUNC(2));
		gpio_set_pull(&gpio1->y6, i, GPIO_PULL_UP);
	}
}

static void gpio_i2c_config(int peripheral, int flags)
{

	struct exynos5_gpio_part1 *gpio1 = gpio1reg;

	switch (peripheral) {
	case PERIPH_ID_I2C0:
		gpio_cfg_pin(&gpio1->b3, 0, GPIO_FUNC(0x2));
		gpio_cfg_pin(&gpio1->b3, 1, GPIO_FUNC(0x2));
		break;
	case PERIPH_ID_I2C1:
		gpio_cfg_pin(&gpio1->b3, 2, GPIO_FUNC(0x2));
		gpio_cfg_pin(&gpio1->b3, 3, GPIO_FUNC(0x2));
		break;
	case PERIPH_ID_I2C2:
		gpio_cfg_pin(&gpio1->a0, 6, GPIO_FUNC(0x3));
		gpio_cfg_pin(&gpio1->a0, 7, GPIO_FUNC(0x3));
		break;
	case PERIPH_ID_I2C3:
		gpio_cfg_pin(&gpio1->a1, 2, GPIO_FUNC(0x3));
		gpio_cfg_pin(&gpio1->a1, 3, GPIO_FUNC(0x3));
		break;
	case PERIPH_ID_I2C4:
		gpio_cfg_pin(&gpio1->a2, 0, GPIO_FUNC(0x3));
		gpio_cfg_pin(&gpio1->a2, 1, GPIO_FUNC(0x3));
		break;
	case PERIPH_ID_I2C5:
		gpio_cfg_pin(&gpio1->a2, 2, GPIO_FUNC(0x3));
		gpio_cfg_pin(&gpio1->a2, 3, GPIO_FUNC(0x3));
		break;
	case PERIPH_ID_I2C6:
		gpio_cfg_pin(&gpio1->b1, 3, GPIO_FUNC(0x4));
		gpio_cfg_pin(&gpio1->b1, 4, GPIO_FUNC(0x4));
		break;
	case PERIPH_ID_I2C7:
		gpio_cfg_pin(&gpio1->b2, 2, GPIO_FUNC(0x3));
		gpio_cfg_pin(&gpio1->b2, 3, GPIO_FUNC(0x3));
		break;
	}
}

static void gpio_i2s_config(int peripheral)
{
	int i;
	struct exynos5_gpio_part1 *gpio1 = gpio1reg;

	for (i = 0; i < 5; i++)
		gpio_cfg_pin(&gpio1->b0, i, GPIO_FUNC(0x02));
}

static void gpio_spi_config(int peripheral)
{
	int cfg = 0, pin = 0, i;
	struct gpio_bank *bank = NULL;
	struct exynos5_gpio_part1 *gpio1 = gpio1reg;
	struct exynos5_gpio_part2 *gpio2 = gpio2reg;

	switch (peripheral) {
	case PERIPH_ID_SPI0:
		bank = &gpio1->a2;
		cfg = GPIO_FUNC(0x2);
		pin = 0;
		break;
	case PERIPH_ID_SPI1:
		bank = &gpio1->a2;
		cfg = GPIO_FUNC(0x2);
		pin = 4;
		break;
	case PERIPH_ID_SPI2:
		bank = &gpio1->b1;
		cfg = GPIO_FUNC(0x5);
		pin = 1;
		break;
	case PERIPH_ID_SPI3:
		bank = &gpio2->f1;
		cfg = GPIO_FUNC(0x2);
		pin = 0;
		break;
	case PERIPH_ID_SPI4:
		for (i = 0; i < 2; i++) {
			gpio_cfg_pin(&gpio2->f0, i + 2, GPIO_FUNC(0x4));
			gpio_cfg_pin(&gpio2->e0, i + 4, GPIO_FUNC(0x4));
		}
		break;
	}
	if (peripheral != PERIPH_ID_SPI4) {
		for (i = pin; i < pin + 4; i++)
			gpio_cfg_pin(bank, i, cfg);
	}
}

static void gpio_spi_slave_config(int peripheral, int flags)
{
	int pin = 0, i;
	struct gpio_bank *bank = NULL;
	struct exynos5_gpio_part1 *gpio1 = gpio1reg;

	switch (peripheral) {
	case PERIPH_ID_CAN:
		bank = &gpio1->x2;
		pin = 0;
		break;
	case PERIPH_ID_MPU:
		bank = &gpio1->x1;
		pin = 6;
		break;
	case PERIPH_ID_ACCMAG:
		printf("ACC MAG not implemented!\n");
		break;
	case PERIPH_ID_GYRO:
		bank = &gpio1->x1;
		pin = 3;
		break;
	case PERIPH_ID_BARO:
		printf("Baro not implemented!\n");
		break;
	case PERIPH_ID_SPIEXT:
		printf("SPI ext not implemented!\n");
		break;
	default:
		printf("Not a SPI slave!\n");
	}

	gpio_set_value(bank, pin, flags);
}

int gpio_pinmux_config(int peripheral, int flags)
{
	switch (peripheral) {
	case PERIPH_ID_UART0:
	case PERIPH_ID_UART1:
	case PERIPH_ID_UART2:
	case PERIPH_ID_UART3:
		gpio_uart_config(peripheral);
		break;
	case PERIPH_ID_SDMMC0:
	case PERIPH_ID_SDMMC1:
	case PERIPH_ID_SDMMC2:
	case PERIPH_ID_SDMMC3:
		return gpio_mmc_config(peripheral, flags);
	case PERIPH_ID_SROMC:
		gpio_sromc_config(flags);
		break;
	case PERIPH_ID_I2C0:
	case PERIPH_ID_I2C1:
	case PERIPH_ID_I2C2:
	case PERIPH_ID_I2C3:
	case PERIPH_ID_I2C4:
	case PERIPH_ID_I2C5:
	case PERIPH_ID_I2C6:
	case PERIPH_ID_I2C7:
		gpio_i2c_config(peripheral, flags);
		break;
	case PERIPH_ID_I2S1:
		gpio_i2s_config(peripheral);
		break;
	case PERIPH_ID_SPI0:
	case PERIPH_ID_SPI1:
	case PERIPH_ID_SPI2:
	case PERIPH_ID_SPI3:
	case PERIPH_ID_SPI4:
		gpio_spi_config(peripheral);
		break;
	case PERIPH_ID_CAN:
	case PERIPH_ID_ACCMAG:
	case PERIPH_ID_MPU:
	case PERIPH_ID_GYRO:
	case PERIPH_ID_BARO:
	case PERIPH_ID_SPIEXT:
		gpio_spi_slave_config(peripheral, flags);
		break;
	default:
		printf("GPIO_PINMUX: Invalid peripheral %d\n", peripheral);
		return -1;
	}

	return 0;
}
#endif
