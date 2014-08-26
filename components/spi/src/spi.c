/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <spi_inf.h>
#include <platsupport/spi.h>
#include <utils.h>
#include <string.h>
#include <spi.h>
#include <errno.h>

#define IRQ_ENABLE

#define SPI_PORT SPI1

extern void spi_set_clk(void);
extern int gpio_pinmux_config(int, int);

/// A handle to the SPI bus that this component drives
spi_bus_t* spi_bus;

/**
 * SPI driver calls this when the transfer is complete.
 * All we need to do is store the status and post on the semaphore
 * that the main thread is waiting on.
 */
static void
spi_complete_callback(spi_bus_t* bus, int status, void* token){
    *(int*)token = status;
    bus_sem_post();
}

/**
 * Called on every SPI IRQ. Redirect control to the driver
 */
static void
spi_irq_event(void *arg)
{
    (void)arg;
    spi_handle_irq(spi_bus);
    spi1_int_reg_callback(&spi_irq_event, NULL);
}

/* Camkes entry point */
void
spi__init(void)
{
    int err;
    spi_set_clk();
    /* Initialise the SPI bus */
    err = exynos_spi_init(SPI_PORT, spi1_reg, NULL, NULL, &spi_bus); 
    assert(!err);
    if(err){
        LOG_ERROR("Failed to initialise SPI port\n");
        return;
    }
    /* Prime the semaphore such that the first call to 'wait' will block */
    bus_sem_wait();
    /* Register an IRQ callback for the driver */
    spi1_int_reg_callback(&spi_irq_event, spi_bus);
}

/**
 * Performs an SPI transfer
 */
static int
do_spi_transfer(int id, void* txbuf, unsigned int wcount, void* rxbuf, unsigned int rcount)
{
    int ret;
    int status;
    /* Select NSS */
    gpio_pinmux_config(id, 1);
    udelay(10);
    /* Begin the transfer */
    ret = spi_xfer(spi_bus, txbuf, wcount, rxbuf, rcount, spi_complete_callback, &status);
    if(ret < 0){
        gpio_pinmux_config(id, 0);
        udelay(10);
        return ret;
    }
    bus_sem_wait();
    /* Deselect NSS */
    gpio_pinmux_config(id, 0);
    udelay(10);
    ret = status;
    return ret;
}

/**
 * Initiate the transfer of shared data
 */
int
spi_transfer(int id, unsigned int wcount, unsigned int rcount)
{
    spi_dev_port_p buf = (spi_dev_port_p)spi1_can;
    return do_spi_transfer(id, buf->txbuf, wcount, buf->rxbuf, rcount);
}

/**
 * Initiate the transfer of a single byte, passed as an argument. Return the byte received.
 */
int
spi_transfer_byte(int id, char byte)
{
    char data[2];
    int ret;
    ret = do_spi_transfer(id, &byte, 1, data, 1);
    if(ret < 0){
        return ret;
    }else{
        return data[1];
    }
}

