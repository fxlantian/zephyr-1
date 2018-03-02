/*
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <board.h>
#include <emmc.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <gpio.h>
#include <emmc_callback.h>
#include <misc/printk.h>
#include "emmc_ppu.h"

/* convenience defines */
#define DEV_EMMC_CFG(dev)                           \
        ((struct emmc_ppu_config * const)(dev)->config->config_info)
#define DEV_EMMC(dev)                               \
        ((volatile struct emmc_ppu_t *)(DEV_EMMC_CFG(dev))->emmc_base_addr)
#define SPI_START_TRANSACTION(trans_type, csnum)   \
        (((1 << (csnum + 8)) & 0xF00) | ((1 << trans_type) & 0xFF))

//u32_t cmd_status;
current_task_status current_task;
//Card_info Card_info_inst[2];
//Card_info *the_card_info = Card_info_inst;
//IP_status_info the_ip_status;

int interrupt_already_done ;
int data_transfer_already_done;


static u32_t PPU_emmc_set_register(struct device *dev, u32_t reg, u32_t val)
{
  u32_t *reg_addr;
  reg_addr = (u32_t *)(reg);
  *reg_addr = val;
  return 0;
}

static u32_t PPU_emmc_read_register(struct device *dev, u32_t reg)
{
  u32_t *reg_addr;
  u32_t retval;
  reg_addr = (u32_t *)(reg);
  retval = *reg_addr;
  return retval;
}

/**
  * Set the particular bits of the specified register.
  * @param[in] reg  The particular register to which the bits are to be set
  * @param[in] val  The bitmask for the bits which are to be set.
  * \return   The new value of the register
  */
static u32_t PPU_emmc_set_bits(struct device *dev, u32_t reg, u32_t val)
{
  u32_t *reg_addr;
  reg_addr = (u32_t *)(reg);
  *reg_addr |= val;
  return *reg_addr;
}

/**
  * Clear the particular bits of the specified register.
  * @param[in] reg  The particular register to which the bits are to be cleared.
  * @param[in] val  The bitmask for the bits which are to be cleared.
  * \return   The new value of the register
  */
static u32_t PPU_emmc_clear_bits(struct device *dev, u32_t reg, u32_t val)
{
  u32_t *reg_addr;
  reg_addr = (u32_t *)(reg);
  *reg_addr &= (~val);
  return *reg_addr;
}

static void PPU_emmc_reset_fifo(struct device *dev)
{
  PPU_emmc_set_bits(dev, EMMC_REG_CTRL, FIFO_RESET);
  while (L_EMMC_REG_CTRL & FIFO_RESET);
  return;
}

void emmc_ppu_isr (void){ 
static int data_flag_kludge = 0;
u32_t cmd_status;
struct device *emmc_dev = device_get_binding("emmc_0");
printk("ISR.RINTSTS:0x%08x CMD_STATUS:%u ERROR_STATUS:%u\n",L_EMMC_REG_RINTSTS, current_task.cmd_status, current_task.error_status); 
    if (TSK_STAT_ABSENT != current_task.cmd_status) {
          current_task.postproc_callback(&current_task, EMMC_REG_RINTSTS);
          printk("After_callback:CMD_STATUS:%u ERROR_STATUS:%u\n", current_task.cmd_status, current_task.error_status);
            if (current_task.error_status) {
                interrupt_already_done = 1;
                 if(current_task.cmd_status == TSK_COMMAND_DONE){ //Command done has already came and error is set. So clean up the interrupt status
                    PPU_emmc_reset_fifo(emmc_dev); //FIFO is flushed to avoid RXDR and TXDR Interrupts in Slave mode of operation
                    PPU_emmc_set_register(emmc_dev, EMMC_REG_RINTSTS, 0xFFFFFFFF);
                }
          }
          /* the interrupt could be for the read/write data */
        cmd_status = current_task.cmd_status;
          if ((!data_flag_kludge) && ((TSK_STATE_READDAT == cmd_status)||(TSK_STATE_WRITEDAT == cmd_status))) {
            data_flag_kludge = 1;
        }
          /*
                If command status is TSK_COMMAND_DONE then wake up the main thread which is waiting...
                Make the command status TSK_STAT_ABSENT as nothing  needs  to be done 
              */
          if (TSK_COMMAND_DONE == current_task.cmd_status) {
                /* Schedule back the task waiting on  this interrupt */
            interrupt_already_done = 1;
            if (data_flag_kludge) {
              data_transfer_already_done = 1;
              data_flag_kludge = 0;
            }
          current_task.cmd_status = TSK_STAT_ABSENT;
          }
    }         
    else{
        PPU_emmc_reset_fifo(emmc_dev); //FIFO is flushed to avoid RXDR and TXDR Interrupts in Slave mode of operation
        PPU_emmc_set_register(emmc_dev, EMMC_REG_RINTSTS, 0xFFFFFFFF);      
      }
         PPU_emmc_set_register(emmc_dev, EMMC_REG_RINTSTS,0xFFFFFFFF);  //clear int
         PPU_ICP |= (1 << 15);
}

static const struct emmc_driver_api emmc_ppu_drv_api_funcs = {
    .ppu_emmc_set_register = PPU_emmc_set_register,
    .ppu_emmc_read_register = PPU_emmc_read_register,
    .ppu_emmc_set_bits = PPU_emmc_set_bits,
    .ppu_emmc_clear_bits = PPU_emmc_clear_bits,
    .ppu_emmc_reset_fifo = PPU_emmc_reset_fifo,
};

static int emmc_ppu_init(struct device *dev)
{
  return 0;
}

#ifdef CONFIG_EMMC

static void emmc_ppu_irq_config_func_0(void);

static const struct emmc_ppu_config emmc_ppu_cfg_0 = {
    .emmc_base_addr = PPU_EMMC_BASE,
    .irq_config = emmc_ppu_irq_config_func_0,
};

DEVICE_AND_API_INIT(emmc_ppu_0, "emmc_0", &emmc_ppu_init,
                    NULL, &emmc_ppu_cfg_0,
                    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                    &emmc_ppu_drv_api_funcs);

static void emmc_ppu_irq_config_func_0(void)
{
    IRQ_CONNECT(PPU_EMMC_IRQ,
                CONFIG_EMMC_IRQ_PRI,
                emmc_ppu_isr,
                DEVICE_GET(emmc_ppu_0),
                0);
    irq_enable(PPU_EMMC_IRQ);
}
#endif