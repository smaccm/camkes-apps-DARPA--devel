/*
 * Copyright (c) 2014 National ICT Australia Limited (NICTA), ABN 62 102 206 173.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * emmc_draft.c
 *
 *  Created on: Jul 18, 2013
 *      Author: Jiawei Xie
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <errno.h>

#include <mmc.h>
#include <camkes/dma.h>
#include <utils/util.h>

#include "common.h"
#include "periph.h"
#include "exynos_mshc.h"
#include "generic_mmc.h"


struct exynos_mshc exynos_mshc_list[] = {
		{
			.status = ENABLE,
			.id = 0,
			.reg = 12200000, //!!!replace with virtual address
			.bus_width = 8,
			.timing = {1, 3, 3},
		},
		{
			.status = DISABLE,
			.id = 1,
			.reg = 12210000, //!!!replace with virtual address
			.bus_width = 4,
			.timing = {0, 0, 0},
		},
		{
			.status = ENABLE,
			.id = 2,
			.reg = 12220000, //!!!replace with virtual address
			.bus_width = 4,
			.timing = {1, 2, 3},
		},
		{
			.status = DISABLE,
			.id = 3,
			.reg = 12230000, //!!!replace with virtual address
			.bus_width = 4,
			.timing = {0, 0, 0},
		},

};

/*
 * Function used as callback function to initialise the
 * CLKSEL register for every mmc channel.
 */
void exynos_mshc_clksel(struct mshc_host *host)
{
	mshc_write(host, MSHC_CLKSEL, host->clksel_val);
}

unsigned int exynos_mshc_get_clk(int dev_index){

	return	clk_get_mmc(dev_index);
}

int mshc_wait_reset(struct mshc_host * host, uint32_t value){
	unsigned long timeout = 1000;
	uint32_t ctrl;

	mshc_write(host, MSHC_CTRL, value);

	while (timeout--) {
		ctrl = mshc_read(host, MSHC_CTRL);
		if (!(ctrl & MSHC_RESET_ALL))
			return 1;
	}

	return 0;
}


int mshc_set_transfer_mode(struct mshc_host *host,
		struct mmc_data *data)
{
	unsigned long mode;

	mode = MSHC_CMD_DATA_EXP;
	if (data->flags & MMC_DATA_WRITE)
		mode |= MSHC_CMD_RW;

	return mode;
}

/*
 * chained
 */
static void mshc_set_idmac_desc(struct mshc_idmac *idmac,
		uint32_t idmac_paddr, uint32_t desc0, uint32_t desc1, uint32_t desc2)
{
	struct mshc_idmac *desc = idmac;
	desc->flags = desc0;
	desc->cnt = desc1;
	desc->addr = desc2;
	//work out the next descriptor address
	desc->next_addr = (unsigned int)idmac_paddr + sizeof(struct mshc_idmac);
}


static void mshc_prepare_data(struct mshc_host * host, struct mmc_data * data){

	unsigned long ctrl;
	unsigned int i = 0, flags, cnt, blk_cnt;
	unsigned long start_addr = 0;
	struct mshc_idmac * cur_idmac_paddr_tmp;
	struct mshc_idmac * cur_idmac_vaddr_tmp;

	//need paddr for cur_idmac, estimated max size with around 4096/512 * sizeof(sturct mshc_idmac)
	//but keep vaddr for I/Os

	//printf("mshc_prepare_data ... ");
	cur_idmac_paddr_tmp = cur_idmac_paddr;
	cur_idmac_vaddr_tmp = cur_idmac_vaddr;

	blk_cnt = data->blocks;

	mshc_wait_reset(host, MSHC_CTRL_FIFO_RESET);

	mshc_write(host, MSHC_DBADDR, (unsigned int)cur_idmac_paddr);

	if(data->flags == MMC_DATA_READ){
		start_addr = (unsigned int)data->dest;	//!!! need to be physical address
	}else{
		start_addr = (unsigned int)data->src;	//!!! need to be physical address
	}

	do{
		/* Descriptor is owned by the internal DMA controller
		 *  The second address in the descriptor is the next descriptor address
		 *  rather than the second buffer address. BS2(DES1[25:13]) must all be zeros.
		 */
		flags= MSHC_IDMAC_OWN | MSHC_IDMAC_CH;
		/* First descriptor indicated contain first buffer of the data */
		flags |= (i == 0) ? MSHC_IDMAC_FS : 0;
		//why blk_cnt == 8? blksze == 512 bytes
		if(blk_cnt <= 8){
			flags |= MSHC_IDMAC_LD;	//last descriptor
			cnt = data->blocksize * blk_cnt;
		}else{
			cnt = data->blocksize * 8;
		}

		mshc_set_idmac_desc(cur_idmac_vaddr_tmp, (uint32_t)cur_idmac_paddr_tmp,
				flags, cnt, start_addr + (i * PAGE_SIZE_4K));

		if(blk_cnt < 8)
			break;
		blk_cnt -= 8;
		cur_idmac_vaddr_tmp ++;
		cur_idmac_paddr_tmp ++;
		i++;

	}while(1);

	//enable IDMA & DMA
	ctrl = mshc_read(host, MSHC_CTRL);
	ctrl |= MSHC_CTRL_IDMA_EN | MSHC_CTRL_DMA_EN;
	mshc_write(host, MSHC_CTRL, ctrl);

	ctrl = mshc_read(host, MSHC_BMOD);
	ctrl |= MSHC_BMOD_IDMAC_FB | MSHC_BMOD_IDMAC_EN;
	mshc_write(host, MSHC_BMOD, ctrl);

	mshc_write(host, MSHC_BLKSIZ, data->blocksize);
	mshc_write(host, MSHC_BYTCNT, data->blocksize * data->blocks);

	//printf("DONE \n");
}


int mshc_send_cmd(struct mmc * mmc, struct mmc_cmd *cmd, struct mmc_data * data)
{
	struct mshc_host * host = (struct mshc_host *)mmc->priv;
	int flags = 0;
	int i = 0;
	uint32_t timeout = 1000;
	uint32_t retry = 10000;
	uint32_t mask, ctrl;
	//!! need timer driver !!!!
	unsigned long start = timer_get(0);

	while(mshc_read(host, MSHC_STATUS) & MSHC_BUSY){
		if(timer_get(start) > timeout){
			printf("Timeout on data busy\n");
			return TIMEOUT;
		}
	}

	mshc_write(host, MSHC_RINTSTS, MSHC_INTMSK_ALL);

	if(data){
		mshc_prepare_data(host, data);
	}

	mshc_write(host, MSHC_CMDARG, cmd->cmdarg);

	if(data){
		//indicate a data command
		flags = mshc_set_transfer_mode(host, data);
	}

	if ((cmd->resp_type & MMC_RSP_136) && (cmd->resp_type & MMC_RSP_BUSY))
		return -1;

	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		flags |= MSHC_CMD_ABORT_STOP;
	else{
		/* Before sending a command on the command line, the host must wait for completion
		 * of any data command already in process. Set to 1 unless the current command is to query status
		 * or stop data transfer when transfer is in progress */
		flags |= MSHC_CMD_PRV_DAT_WAIT;
	}

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		/* expect responses excpt for CMD4 or CMD15 */
		flags |= MSHC_CMD_RESP_EXP;
		if (cmd->resp_type & MMC_RSP_136){
			/* R2 (long) response */
			flags |= MSHC_CMD_RESP_LENGTH;
		}
	}

	if (cmd->resp_type & MMC_RSP_CRC){
		flags |= MSHC_CMD_CHECK_CRC;
	}

	flags |= (cmd->cmdidx | MSHC_CMD_START | MSHC_CMD_USE_HOLD_REG);

	printf("Sending CMD%d\n",cmd->cmdidx);

	mshc_write(host, MSHC_CMD, flags);

	for (i = 0; i < retry; i++) {
		mask = mshc_read(host, MSHC_RINTSTS);
		if (mask & MSHC_INTMSK_CDONE) {
			if (!data)
				mshc_write(host, MSHC_RINTSTS, mask);
			break;
		}
	}

	if (i == retry)
		return TIMEOUT;

	if (mask & MSHC_INTMSK_RTO) {
		printf("Response Timeout..\n");
		return TIMEOUT;
	} else if (mask & MSHC_INTMSK_RE) {
		printf("Response Error..\n");
		return -1;
	}

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_136) {
			cmd->response[0] = mshc_read(host, MSHC_RESP3);
			cmd->response[1] = mshc_read(host, MSHC_RESP2);
			cmd->response[2] = mshc_read(host, MSHC_RESP1);
			cmd->response[3] = mshc_read(host, MSHC_RESP0);
		} else {
			cmd->response[0] = mshc_read(host, MSHC_RESP0);
		}
	}

	if (data) {
		do {
			mask = mshc_read(host, MSHC_RINTSTS);
			if (mask & (MSHC_INT_DATA_ERROR | MSHC_INT_DATA_TOUT)) {
				printf("DATA ERROR!\n");
				return -1;
			}
		} while (!(mask & MSHC_INTMSK_DTO));

		mshc_write(host, MSHC_RINTSTS, mask);

		ctrl = mshc_read(host, MSHC_CTRL);
		ctrl &= ~(MSHC_CTRL_DMA_EN);
		mshc_write(host, MSHC_CTRL, ctrl);
	}
	return 0;
}


/* CLKSRC
 * CLKDIV
 * CLKENA
 * The controller loads these registers when it receives an update clocks command:
 * 1. Before disabling the clocks, ensure that the card is not busy with any previous
 * data command, by verify that the data_busy bit of the status register (status) is 0.
 * 2. Reset the cclk_enable bit of the clkena register to 0, to disable the card clock generation.
 * 3. Reset the clksrc to 0.
 * 4. Set the following bits in the cmd register to 1:
 * 		- update_clk_regs_only: specifies the update clocks command
 * 		- wait_prvdata_complete: Ensure that clock parameters do not change until any ongoing data
 * 		  transfer is completed.
 * 		- start_cmd: Initialise the command
 *
 * 5. Wait unitl the start_cmd and update_clk_regs_only bits change to 0. There is not interrupt when the
 *    clock modification completes. The controller does not set the command_done bit in the rintsts register
 *    upon command completion. The controller might signal a hardware lock error if it already has another command in the
 *    queue. In this case, return to step 4.
 * 6. Reset the clock enable bit in PLL (skipped)
 * 7. pahse shift value - skipped.
 * 8. Re-enable the pll - skipped.
 * 9. Set the clkdiv to the correct value for the required clock frequence.
 * 10. Set the clk_enable bit fo the clkena register to 1, to enable the card clock
 * 	 generation.
 * 	 Low-power mode: automatically stops the sdmmc_cclk_out clock when the card is idle for more than eight clock cycles.
 */
static int mshc_setup_bus(struct mshc_host *host, uint32_t freq)
{
	uint32_t div, status;
	int timeout = 10000;
	unsigned long sclk;

	if ((freq == host->clock) || (freq == 0))
		return 0;
	/*
	 * If host->mmc_clk didn't define,
	 * then assume that host->bus_hz is source clock value.
	 * host->bus_hz should be set from user.
	 */
	if (host->mmc_clk)
		sclk = host->mmc_clk(host->dev_index);
	else if (host->bus_hz)
		sclk = host->bus_hz;
	else {
		printf("Didn't get source clock value..\n");
		return -EINVAL;
	}

	div = DIV_ROUND_UP(sclk, 2 * freq);

	mshc_write(host, MSHC_CLKENA, 0);
	mshc_write(host, MSHC_CLKSRC, 0);

	mshc_write(host, MSHC_CLKDIV, div);
	mshc_write(host, MSHC_CMD, MSHC_CMD_PRV_DAT_WAIT |
			MSHC_CMD_UPD_CLK | MSHC_CMD_START);

	do {
		status = mshc_read(host, MSHC_CMD);
		if (timeout-- < 0) {
			printf("TIMEOUT error!!\n");
			return -ETIMEDOUT;
		}
	} while (status & MSHC_CMD_START);

	mshc_write(host, MSHC_CLKENA, MSHC_CLKEN_ENABLE |
			MSHC_CLKEN_LOW_PWR);

	mshc_write(host, MSHC_CMD, MSHC_CMD_PRV_DAT_WAIT |
			MSHC_CMD_UPD_CLK | MSHC_CMD_START);

	timeout = 10000;
	do {
		status = mshc_read(host, MSHC_CMD);
		if (timeout-- < 0) {
			printf("TIMEOUT error!!\n");
			return -ETIMEDOUT;
		}
	} while (status & MSHC_CMD_START);

	host->clock = freq;

	return 0;
}

static void mshc_set_ios(struct mmc *mmc)
{
	struct mshc_host *host = (struct mshc_host *)mmc->priv;
	uint32_t ctype;

	printf("Bus Width = %d, Clock: %d\n",mmc->bus_width, mmc->clock);

	mshc_setup_bus(host, mmc->clock);
	switch (mmc->bus_width) {
	case 8:
		ctype = MSHC_CTYPE_8BIT;
		break;
	case 4:
		ctype = MSHC_CTYPE_4BIT;
		break;
	default:
		ctype = MSHC_CTYPE_1BIT;
		break;
	}

	mshc_write(host, MSHC_CTYPE, ctype);

	if (host->clksel)
		host->clksel(host);
}

static int mshc_init(struct mmc *mmc)
{
	struct mshc_host *host = (struct mshc_host *)mmc->priv;
	uint32_t fifo_size;

	mshc_write(host, MSHC_PWREN, 1);

	if (!mshc_wait_reset(host, MSHC_RESET_ALL)) {
		printf("MSHC init Fail-reset!!\n");
		return -1;
	}

	/* Enumerate at 400KHz */
	mshc_setup_bus(host, mmc->f_min);

	mshc_write(host, MSHC_RINTSTS, 0xFFFFFFFF);
	mshc_write(host, MSHC_INTMASK, 0);

	mshc_write(host, MSHC_TMOUT, 0xFFFFFFFF);

	mshc_write(host, MSHC_IDINTEN, 0);
	mshc_write(host, MSHC_BMOD, 1);

	if (!host->fifoth_val) {
		fifo_size = mshc_read(host, MSHC_FIFOTH);
		fifo_size = ((fifo_size & RX_WMARK_MASK) >> RX_WMARK_SHIFT) + 1;
		host->fifoth_val = MSIZE(0x2) | RX_WMARK(fifo_size / 2 - 1) |
			TX_WMARK(fifo_size / 2);
	}
	mshc_write(host, MSHC_FIFOTH, host->fifoth_val);

	mshc_write(host, MSHC_CLKENA, 0);
	mshc_write(host, MSHC_CLKSRC, 0);

	return 0;
}

int add_mshc(struct mshc_host * host, uint32_t max_clk, uint32_t min_clk){

	struct mmc * mmc;
	int err = 0;

	//printf("mmc malloc ... ");
	mmc = malloc(sizeof(struct mmc));
	if(!mmc){
		printf("mmc malloc failed!\n");
		return -1;
	}
	//printf("DONE\n");

	mmc->priv = host;
	host->mmc = mmc;

	sprintf(mmc->name, "%s", host->name);

	mmc->send_cmd = mshc_send_cmd;
	mmc->set_ios = mshc_set_ios;
	mmc->init = mshc_init;
	mmc->f_min = min_clk;
	mmc->f_max = max_clk;

	mmc->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;

	mmc->host_caps = host->caps;

	if (host->buswidth == 8) {
		mmc->host_caps |= MMC_MODE_8BIT;
		mmc->host_caps &= ~MMC_MODE_4BIT;
	} else {
		mmc->host_caps |= MMC_MODE_4BIT;
		mmc->host_caps &= ~MMC_MODE_8BIT;
	}
	mmc->host_caps |= MMC_MODE_HS | MMC_MODE_HS_52MHz | MMC_MODE_HC;

	//printf("mmc register ... ");
	err = mmc_register(mmc);
	//printf("DONE \n");

	return err;

}

/*
 * This function adds the mmc channel to be registered with mmc core.
 * index -	mmc channel number.
 * regbase -	register base address of mmc channel specified in 'index'.
 * bus_width -	operating bus width of mmc channel specified in 'index'.
 * clksel -	value to be written into CLKSEL register in case of FDT.
 *		NULL in case od non-FDT.
 */
int exynos_mshc_add_port(int index, uint32_t regbase, int bus_width, uint32_t clksel){

	struct mshc_host * host = NULL;
	uint32_t	div;
	unsigned long freq, sclk;

	//printf("mshc_host malloc\n");
	host = malloc(sizeof(struct mshc_host));
	if(host == NULL){
		printf("mshc_host malloc failed!\n");
		return -ENOMEM;
	}
	//printf("mshc_host malloc: DONE\n");

	/* request mmc clock value of 52MHz.  */
	freq = 52000000;
	sclk = clk_get_mmc(index);
	div = DIV_ROUND_UP(sclk, freq);
	/* set the clock divisor for mmc */
	clk_set_mmc(index, div);

	host->name = "EXYNOS MSHC";
	host->ioaddr = (void *)regbase;
	host->buswidth = bus_width;

	if (clksel) {
		host->clksel_val = clksel;
	} else {
		//use default when clksel == 0
		if (0 == index)
			host->clksel_val = EXYNOS_MSHC_MMC0_CLKSEL_VAL;
		if (2 == index)
			host->clksel_val = EXYNOS_MSHC_MMC2_CLKSEL_VAL;
	}

	/* function pointers */
	host->clksel = exynos_mshc_clksel;
	host->dev_index = index;
	host->mmc_clk = exynos_mshc_get_clk;

	//printf("Add MSHC... \n");
	/* Add the mmc channel to be registered with mmc core */
	if (add_mshc(host, EXYNOS_MSHC_MAX_FREQ, EXYNOS_MSHC_MIN_FREQ)) {
		printf("mshc %d registration failed\n", index);
		return -1;
	}
	//printf("Add MSHC... DONE \n");

	return 0;
}

int exynos_mshc_init(void){

	int err = 0, flag, i;
	uint32_t clksel_val, timing[3];
	struct exynos_mshc * node;

	for(i = 0; i < EXYNOS_MSHC_MAX_CH_NUM; i++){

		node = &(exynos_mshc_list[i]);

		if(node->status == ENABLE){

			//printf("Initializing MSHC channels: %d\n", i);

			/* config pinmux for each mmc channel */
			if(node->bus_width == 8){
				flag = PINMUX_FLAG_8BIT_MODE;
			}else{
				flag = PINMUX_FLAG_NONE;
			}
			//connect with GPIO
			err = gpio_mmc_config(node->id, flag);
			if (err) {
				printf("MSHC not configured\n");
				return err;
			}

			clksel_val = (MSHC_CLKSEL_CCLK_SAMPLE(timing[0]) |	\
					MSHC_CLKSEL_CCLK_DRIVE(timing[1]) |			\
					MSHC_CLKSEL_CCLK_DIVIDER(timing[2]));

			/* Initialise each mshc channel */
			//printf("\n MSHC Add port ... \n");
			err = exynos_mshc_add_port(node->id, node->reg, node->bus_width, clksel_val);
			if (err){
				printf("Initialisation of MSHC Channel %d failed\n", i);
			}
		}

	}
	//printf("\n MSHC channel initialization: Done\n");

	return 0;
}


static int exynos_mshc_priv_init(struct mshc_host * host){

	struct mshc_exynos_priv_data * priv;
	priv = malloc(sizeof(struct mshc_exynos_priv_data));
	if(priv == NULL){
		return -ENOMEM;
	}
	host->priv = priv;
	/* we don't have a device tree so just replace this function with simple assignments */
	priv->ciu_div = 3;
	priv->sdr_timing = MSHC_CLKSEL_TIMING(2, 3, 3);
	priv->ddr_timing = MSHC_CLKSEL_TIMING(1, 3, 3);
	return 0;
}

void mshc__init (void){

	/* CAMKES DEPENDENT */
compile_time_assert(allocation_fits_in_a_page, 8 * sizeof(struct mshc_idmac) <= PAGE_SIZE_4K);
	cur_idmac_vaddr = (struct mshc_idmac *)camkes_dma_alloc_page();
	if (cur_idmac_vaddr == NULL) {
		printf("Error: Ran out of memory ! \n");
		return;
	}

	//update physical address
	cur_idmac_paddr = (struct mshc_idmac *)camkes_dma_get_paddr(cur_idmac_vaddr);
	exynos_mshc_list[0].reg = (uint32_t)mshc0_phy;
	exynos_mshc_list[1].reg = (uint32_t)mshc1_phy;
	exynos_mshc_list[2].reg = (uint32_t)mshc2_phy;
	exynos_mshc_list[3].reg = (uint32_t)mshc3_phy;

	exynos_mshc_init();
}




