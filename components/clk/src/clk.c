/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <stdint.h>
#include <platsupport/clock.h>

#include <clk.h>

clock_sys_t clock_sys;

static int set_spi_clock(int periph_id, unsigned long rate)
{
    volatile uint32_t *base = (uint32_t*)cmu_top_clk;
    uint32_t r, rpre, v;
    int fin = 24000000; /* XXTI */
    int div = fin / rate;

    /* Tune PRE_RATIO with RATIO at maximum */
    rpre = (div / 0xf);
    /* Now tune RATIO */
    r = div / (rpre + 1);

    /* Apply the change */
    v = base[0x55C/4];
    v &= 0xffff;
    v |= rpre << 24 | r << 16;
    base[0x55C/4] = v; 

    /* Read it back */
    rpre = (v >> 24) & 0xf;
    r = (v >> 16) & 0xf;
    return fin / (rpre + 1) / (r + 1);
}

unsigned int clktree_get_spi1_freq(void){
    return 0;
}


unsigned int clktree_set_spi1_freq(unsigned int rate){
    return set_spi_clock(0, rate); 
}


void clktree__init(void){
    int err;
    err = exynos5_clock_sys_init(cmu_cpu_clk,
                                 cmu_core_clk,
                                 NULL,
                                 NULL,
                                 cmu_top_clk,
                                 NULL,
                                 NULL,
                                 NULL,
                                 NULL,
                                 &clock_sys);
    assert(!err);
    if(err){
        printf("Failed to initialise clock tree\n");
    }
}


