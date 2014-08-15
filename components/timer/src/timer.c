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
 * regs.c
 *
 *  Created on: Aug 1, 2013
 *      Author: jxie
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "timer.h"

#include "exynos_config.h"
#include "exynos_timer.h"
#include "common.h"
#include "periph.h"

/* =====================
 * TIMER INIT SETUP
 * ===================== */
#define PRESCALER1	10	//prescaler for regs 0 & 1
#define PRESCALER2	1	//prescaler for regs 2, 3 & 4
#define DEAD_ZONE_LENGTH_VAL	0x0

/* TCFG0
 * Timer Input Clock Frequency = PCLK/({prescaler value + 1})/{divider value}
 * {prescaler value} = 1 to 255
 * {divider value} = 1, 2, 4, 8, 16
 * Dead Zone Length = 0 to 254
 */
#define TIMER_PRESCALER0_VAL	(PRESCALER1 - 1)
#define TIMER_PRESCALER1_VAL	(PRESCALER2 - 1)

/* we always count down the max. */
#define TIMER_LOAD_VAL 0x3E8	// 1 milli-second

static unsigned long timestamp;
static unsigned long lastdec;

struct pwm_timer * regs;	//0x12DD 0000

struct timer_device {
	uint8_t id;				//time ID
	uint8_t	pwm_high_time;	//comparator value
	uint8_t pwm_cycle_time;	//regs reload value
	int pclk;		//input clock source frequency
	uint32_t	prescaler;
	uint32_t	divider;
	float	resolution;	//in uS
	//flag to service client ID
	int 	in_service_id;
};

#define NUM_TIMERS	5
struct timer_device timer_table[] = {
		{
			.id = 0,
			.pwm_high_time = 0,
			.pwm_cycle_time = 0x3E8,
			.pclk = 88000000,
			.prescaler = TIMER_PRESCALER0_VAL,
			.divider = DIV_8,
			.in_service_id = -1,
		},
		{
			.id = 1,
			.pwm_high_time = 0,
			.pwm_cycle_time = 0x3E8,
			.pclk = 88000000,
			.prescaler = TIMER_PRESCALER0_VAL,
			.divider = DIV_8,
			.in_service_id = -1,
		},
		{
			.id = 2,
			.pwm_high_time = 0,
			.pwm_cycle_time = 1000000,
			.pclk = 88000000,
			.prescaler = TIMER_PRESCALER1_VAL,
			.divider = DIV_8,
			.in_service_id = -1,
		},
		{
			.id = 3,
			.pwm_high_time = 0,
			.pwm_cycle_time = 1000000,
			.pclk = 88000000,
			.prescaler = TIMER_PRESCALER1_VAL,
			.divider = DIV_8,
			.in_service_id = -1,
		},
		{
			.id = 4,
			.pwm_high_time = 0,
			.pwm_cycle_time = 1000000,
			.pclk = 88000000,
			.prescaler = TIMER_PRESCALER1_VAL,
			.divider = DIV_8,
			.in_service_id = -1,
		},
};

struct timer_queue{
	int cid;				//client ID
	int wait;			//request waiting time
	unsigned long	reset;	//time counter to release the client
	struct timer_device * dev;	//the drvice it is using
};

#define MAX_NUM_TASKS		5

struct timer_queue sq[MAX_NUM_TASKS];

int timer1_insert_client(int cid, int wait){
	int i = 0;
	struct timer_queue * slot = NULL;


	for(i=0; i<MAX_NUM_TASKS; i++){

		if(sq[i].cid == -1){
			//find empty
			slot = & sq[i];
			slot->cid = cid;
			slot->wait = wait;
			return 0;
		}
	}

	return -1;
}

void remove_client(int cid){
	int i = 0;
	struct timer_queue * slot = NULL;
	for(i=0; i<MAX_NUM_TASKS; i++){
		if(sq[i].cid == cid){
			//find client
			slot = & sq[i];
			break;
		}
	}
	slot->cid = -1;
	slot->wait = 0;
	slot->dev = NULL;
}

void emit(void){

}

struct fxn_ptr{
	void (*wakeup) (void);
};

struct fxn_ptr wakeup_client[] = {
	{
		.wakeup = &client_sleep_emit, //CLIENT_APP_ID
	},
	{
		.wakeup = &client_sleep_emit,
	},
};

//client should call wait on timer event after the successful return of this function
int timer_sleep_ms(int cid, int delayms){
	unsigned int reload_val = 0;
	return timer1_insert_client(cid, delayms);
}

static int system_time = 0; //in ms

void timer1_interrupt_handler(void){
	int i = 0;

	for(i = 0; i < MAX_NUM_TASKS; i++){
		if(sq[i].cid != -1){
			sq[i].wait -= 1;	//1 milli-second
			if(sq[i].wait <= 0){
				//wake up client
				wakeup_client[sq[i].cid].wakeup();
				printf("waking up %d\n",sq[i].cid);
				sq[i].cid = -1;	//empty the space
			}
		}
	}
	system_time ++;
}

void init_queue(void){
	int i = 0;
	for(i=0; i<MAX_NUM_TASKS; i++){
		sq[i].cid = -1;
	}
}

int timer_init (void){

	int i = 0;
	int d = 0;

	init_queue();

	for (i=0; i< NUM_TIMERS; i++){

		timer_table[i].pclk = clk_get_pwm_freq(PERIPH_ID_PWM0);

		switch(timer_table[i].divider){
		case DIV_16:
			d = 16;
			break;
		case DIV_8:
			d = 8;
			break;
		case DIV_4:
			d = 4;
			break;
		case DIV_2:
			d = 2;
			break;
		case DIV_1:
			d = 1;
			break;
		default:
			printf("not valid divider value, timer %d\n", timer_table[i].id);
		}
		//calculate the resolution
		timer_table[i].resolution = 1.00/(float)((float)timer_table[i].pclk/1000000/(timer_table[i].prescaler + 1)/d); //in second
		//printf("timer %d resolution: %f, pclk: %d\n",timer_table[i].id, timer_table[i].resolution, timer_table[i].pclk);
	}

	TIMER_INST_VAL(regs->TCFG0, TCFG0_PRESCALER_0, TIMER_PRESCALER0_VAL);
	TIMER_INST_VAL(regs->TCFG0, TCFG0_PRESCALER_1, TIMER_PRESCALER1_VAL);
	TIMER_INST_VAL(regs->TCFG1, TCFG1_DIV_MUX4, timer_table[4].divider);
	TIMER_INST_VAL(regs->TCFG1, TCFG1_DIV_MUX3, timer_table[3].divider);
	TIMER_INST_VAL(regs->TCFG1, TCFG1_DIV_MUX2, timer_table[2].divider);
	TIMER_INST_VAL(regs->TCFG1, TCFG1_DIV_MUX1, timer_table[1].divider);
	TIMER_INST_VAL(regs->TCFG1, TCFG1_DIV_MUX0, timer_table[0].divider);
	//regs->TCFG0 = 0x0000000A;	//prescaler1 = 0; prescaler0 = 11;
	//regs->TCFG1 = 0x00000034;	//MUX0 = 1/16, MUX1 = 1/8
	TIMER_INST_VAL(regs->TCON, TCON_TIMER4_AUTO_RELOAD, ENABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER3_AUTO_RELOAD, ENABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER2_AUTO_RELOAD, ENABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER1_AUTO_RELOAD, ENABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER0_AUTO_RELOAD, ENABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER4_MANUAL_UPDATE, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER3_MANUAL_UPDATE, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER2_MANUAL_UPDATE, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER1_MANUAL_UPDATE, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER0_MANUAL_UPDATE, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER3_OUTPUT_INVERT, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER2_OUTPUT_INVERT, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER1_OUTPUT_INVERT, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER0_OUTPUT_INVERT, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER4_START_STOP, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER3_START_STOP, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER2_START_STOP, DISABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER1_START_STOP, ENABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER0_START_STOP, ENABLE);
	//regs->TCON = 0x00000900;
	regs->TCNTB0 = timer_table[0].pwm_cycle_time;
	regs->TCMPB0 = timer_table[0].pwm_high_time;

	regs->TCNTB1 = timer_table[1].pwm_cycle_time;
	regs->TCMPB1 = timer_table[1].pwm_high_time;

	regs->TCNTB2 = timer_table[2].pwm_cycle_time;
	regs->TCMPB2 = timer_table[2].pwm_high_time;

	regs->TCNTB3 = timer_table[3].pwm_cycle_time;
	regs->TCMPB3 = timer_table[3].pwm_high_time;
	/* Timer 4 doesn't have output capability */
	regs->TCNTB4 = timer_table[4].pwm_cycle_time;

	TIMER_INST_VAL(regs->TCON, TCON_TIMER1_MANUAL_UPDATE, ENABLE);
	TIMER_INST_VAL(regs->TCON, TCON_TIMER1_MANUAL_UPDATE, DISABLE);
	//regs->TCON = 0x00000B00;
	//regs->TCON = 0x00000900;

	//interrupts
	TIMER_INST_VAL(regs->TINT_CSTAT, TIMER4_INIT_EN, DISABLE);
	TIMER_INST_VAL(regs->TINT_CSTAT, TIMER3_INIT_EN, DISABLE);
	TIMER_INST_VAL(regs->TINT_CSTAT, TIMER2_INIT_EN, DISABLE);
	TIMER_INST_VAL(regs->TINT_CSTAT, TIMER1_INIT_EN, ENABLE);
	TIMER_INST_VAL(regs->TINT_CSTAT, TIMER0_INIT_EN, DISABLE);

	lastdec = regs->TCNTB1 = TIMER_LOAD_VAL;
	timestamp = 0;

	return 0;
}



void timer1_init_handler(void *data UNUSED){
	static cnt = 0;

	timer1_interrupt_handler();
	if(cnt % 1000 == 0)
		//printf("timer1 %d\n", cnt);
 	cnt ++;
	//clear interrupt
	TIMER_INST_VAL(regs->TINT_CSTAT, TIMER1_INIT_STS, 1);
	timer1_init_reg_callback(&timer1_init_handler, NULL);
}

void timer__init(void){
	regs = (struct pwm_timer *)timerbase; //base vaddr
	timer_init();
	timer1_init_reg_callback(&timer1_init_handler, NULL);
	//update timer 1 counter
	timestamp += timer_table[1].resolution;
}


int x_run(void){
	timer__init();
	printf("timer_clk: %d\n", clk_get_pwm_freq(0));
}


//return current timer ticks
unsigned long timer_get_current (void)
{
	unsigned long now = (regs->TCNTO1);
	timestamp += now;
	return timestamp;
}

/*
 * timer without interrupts
 */
unsigned int timer_get (unsigned int base)
{
	return (system_time - base);
	//return ((unsigned int)timer_get_current ()) - base;
}

void timer_busy_udelay (unsigned long usec)
{
	unsigned long tmo;
	unsigned long endtime;
	signed long diff;

	endtime = timer_get(0) + tmo;

	do {
		unsigned long now = timer_get_current();
		diff = endtime - now;
	} while (diff >= 0);
}
