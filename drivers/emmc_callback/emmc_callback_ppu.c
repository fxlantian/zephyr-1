/*
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <board.h>
#include <emmc_callback.h>
#include <emmc.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <gpio.h>
#include <misc/printk.h>
#include "emmc_callback_ppu.h"

extern Card_info *the_card_info;

current_task_status current_task;

IP_status_info the_ip_status;

static void no_response_preproc(u32_t card_num, u32_t cmd_index,      //ly
        u32_t * cmd_reg, u32_t * arg_reg)
{

  UNSET_BITS(*cmd_reg, CMD_ABRT_CMD_BIT);
  UNSET_BITS(*cmd_reg, CMD_RESP_EXP_BIT);
  UNSET_BITS(*cmd_reg, CMD_RESP_LENGTH_BIT);
  SET_CARD_NUM(*cmd_reg, card_num);
  if (cmd_index > 200) {
    cmd_index -= 200;
  }
  SET_CMD_INDEX(*cmd_reg, cmd_index);
}

static void no_response_preproc_abrt(u32_t card_num, u32_t cmd_index,     //ly
             u32_t * cmd_reg, u32_t * arg_reg)
{

  no_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
  SET_BITS(*cmd_reg, CMD_ABRT_CMD_BIT);

}
static void long_response_preproc(u32_t card_num, u32_t cmd_index,   //ly
          u32_t * cmd_reg, u32_t * arg_reg)
{
  UNSET_BITS(*cmd_reg, CMD_ABRT_CMD_BIT);
  SET_BITS(*cmd_reg, CMD_RESP_EXP_BIT);
  SET_BITS(*cmd_reg, CMD_RESP_LENGTH_BIT);
  SET_CARD_NUM(*cmd_reg, card_num);
  if (cmd_index > 200) {
    cmd_index -= 200;
  }
  SET_CMD_INDEX(*cmd_reg, cmd_index);
}


static void short_response_preproc(u32_t card_num, u32_t cmd_index,    //ly
           u32_t * cmd_reg, u32_t * arg_reg)
{
  SET_BITS(*cmd_reg, CMD_RESP_EXP_BIT);
  UNSET_BITS(*cmd_reg, CMD_RESP_LENGTH_BIT);
  if (cmd_index > 200) {
    cmd_index -= 200;
  }  
  SET_CMD_INDEX(*cmd_reg, cmd_index);
  SET_CARD_NUM(*cmd_reg, card_num);
}

static void short_response_preproc_abrt(u32_t card_num, u32_t cmd_index,    //ly
          u32_t * cmd_reg, u32_t * arg_reg)
{
  short_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
  SET_BITS(*cmd_reg, CMD_ABRT_CMD_BIT);
}


static void short_response_preproc_with_init(u32_t card_num, u32_t cmd_index,   //ly
               u32_t * cmd_reg, u32_t * arg_reg)
{
  short_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
  SET_BITS(*cmd_reg, CMD_SEND_INIT_BIT);
}


#if 0
static void no_response_preproc_unadd(u32_t card_num, u32_t cmd_index,
              u32_t * cmd_reg, u32_t * arg_reg)
{
  SET_CMD_INDEX(*cmd_reg, (cmd_index - UNADD_OFFSET));
  SET_CARD_NUM(*cmd_reg, card_num);
}
#endif



static void short_response_block_data_preproc(u32_t card_num,     //ly
                u32_t cmd_index,
                u32_t * cmd_reg, u32_t * arg_reg)
{
        short_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
        SET_BITS(*cmd_reg, CMD_DATA_EXP_BIT);
        /*
        Some of MMC/SD cards misbehave (block skip problem) when auto_stop_bit is set for
        a multi block read. So Driver should send the STOP CMD (CMD12) after multi block read
        is complete.
        */
        UNSET_BITS(*cmd_reg, CMD_SENT_AUTO_STOP_BIT);
        if(cmd_index == CMD17)
                UNSET_BITS(*cmd_reg, CMD_SENT_AUTO_STOP_BIT);
 
        SET_BITS(*cmd_reg, CMD_WAIT_PRV_DAT_BIT);
}


static void short_response_block_data_preproc_noac(u32_t card_num,   //  ly 
               u32_t cmd_index,
               u32_t * cmd_reg,
               u32_t * arg_reg)
{
  short_response_block_data_preproc(card_num, cmd_index, cmd_reg,arg_reg);
  UNSET_BITS(*cmd_reg, CMD_SENT_AUTO_STOP_BIT);
}

#if 1     //ly
static void short_response_stream_data_preproc(u32_t card_num,
                 u32_t cmd_index,
                 u32_t * cmd_reg,
                 u32_t * arg_reg)
{
  short_response_block_data_preproc(card_num, cmd_index, cmd_reg,arg_reg);
  SET_BITS(*cmd_reg, CMD_TRANSMODE_BIT);
}
#endif       


static void short_response_stream_data_preproc_noac(u32_t card_num,    
                u32_t cmd_index,
                u32_t * cmd_reg,
                u32_t * arg_reg)
{
  short_response_stream_data_preproc(card_num, cmd_index, cmd_reg,
             arg_reg);
  UNSET_BITS(*cmd_reg, CMD_SENT_AUTO_STOP_BIT);
}

#if 0
static void short_response_block_data_preproc_noac(u32_t card_num,
               u32_t cmd_index,
               u32_t * cmd_reg,
               u32_t * arg_reg)
{
  short_response_block_data_preproc(card_num, cmd_index, cmd_reg,
            arg_reg);
  UNSET_BITS(*cmd_reg, CMD_SENT_AUTO_STOP_BIT);
}
#endif



static void short_response_block_write_preproc(u32_t card_num,   //ly
                 u32_t cmd_index,
                 u32_t * cmd_reg,
                 u32_t * arg_reg)
{
        short_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
        SET_BITS(*cmd_reg, CMD_DATA_EXP_BIT);
        SET_BITS(*cmd_reg, CMD_RW_BIT);
        /*
        Some of MMC/SD cards misbehave (block skip problem) when auto_stop_bit is set for
        a multi block write. So Driver should send the STOP CMD (CMD12) after multi block write
        is complete.
        */
//        UNSET_BITS(*cmd_reg, CMD_SENT_AUTO_STOP_BIT);//Just to check with CMD23 instead of autotstopcommand
        if((cmd_index == CMD24) || (cmd_index == CMD42))
                UNSET_BITS(*cmd_reg, CMD_SENT_AUTO_STOP_BIT);
    
        SET_BITS(*cmd_reg, CMD_WAIT_PRV_DAT_BIT);

}

#if 1
static void short_response_stream_write_preproc(u32_t card_num,
            u32_t cmd_index,
            u32_t * cmd_reg,
            u32_t * arg_reg)
{
  short_response_block_write_preproc(card_num, cmd_index, cmd_reg,arg_reg);
  SET_BITS(*cmd_reg, CMD_TRANSMODE_BIT);
}



static void short_response_block_write_preproc_noac(u32_t card_num,
                u32_t cmd_index,
                u32_t * cmd_reg,
                u32_t * arg_reg)
{
  short_response_block_write_preproc(card_num, cmd_index, cmd_reg,arg_reg);
  UNSET_BITS(*cmd_reg, CMD_SENT_AUTO_STOP_BIT);
}
#endif 


#if 0
static void abort_preproc(u32_t card_num, u32_t cmd_index, u32_t * cmd_reg,
        u32_t * arg_reg)
{
  SET_BITS(*cmd_reg, CMD_ABRT_CMD_BIT);
  SET_BITS(*cmd_reg, CMD_RESP_EXP_BIT);
  UNSET_BITS(*cmd_reg, CMD_RESP_LENGTH_BIT);
  SET_CMD_INDEX(*cmd_reg, cmd_index);
}
#endif


static void short_response_rca_preproc(u32_t card_num, u32_t cmd_index,    //ly
               u32_t * cmd_reg, u32_t * arg_reg)
{
  short_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
  SET_RCA(*arg_reg, (the_card_info[card_num].the_rca));
}

#if 0
static void no_response_rca_preproc(u32_t card_num, u32_t cmd_index,
            u32_t * cmd_reg, u32_t * arg_reg)
{
  no_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
  SET_RCA(*arg_reg, (the_card_info[card_num].the_rca));
}
#endif

#if 1   //ly
static void long_response_rca_preproc(u32_t card_num, u32_t cmd_index,
              u32_t * cmd_reg, u32_t * arg_reg)
{
  //printk("this is 33333333333333 = %0x\n", card_num);
  long_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
  //printk("this is1111 %08x\n",the_card_info[card_num].the_rca );
  SET_RCA(*arg_reg, (the_card_info[card_num].the_rca));
}
#endif

void short_response_postproc(void *the_data, u32_t * interrupt_status)   //ly
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status =  (current_task_status *) the_data;

  /* Handle standard interrupt handler */
//#ifdef IDMAC_SUPPORT  
if(current_task.idma_mode_on ==1)
        emmc_handle_standard_idsts(the_data , *interrupt_status);
//#else
else
  emmc_handle_standard_rinsts(the_data, *interrupt_status);
//#endif

  /* Read the short response and set the command status */
  if (the_task_status->resp_buffer) {
  //  printk("Entering short_response_buf ,%x\n",the_task_status->resp_buffer );
    the_task_status->resp_buffer[0] =  emmc_read_register(emmc_dev, EMMC_REG_RESP0);
    printk(" the_task_status->resp_buffer,%x\n",the_task_status->resp_buffer[0]);
  }
}

#if 1   //ly
static void short_response_preproc_volt_switch_stage_1(u32_t card_num, u32_t cmd_index,u32_t * cmd_reg, u32_t * arg_reg)
{
  SET_BITS(*cmd_reg, CMD_RESP_EXP_BIT);
  SET_BITS(*cmd_reg, CMD_VOLT_SW_BIT);

  UNSET_BITS(*cmd_reg, CMD_RESP_LENGTH_BIT);
  if (cmd_index > 200) {
    cmd_index -= 200;
  }
  SET_CMD_INDEX(*cmd_reg, cmd_index);
  SET_CARD_NUM(*cmd_reg, card_num);
}
#endif

void short_response_postproc_volt_switch_stage_1(void * the_data, u32_t * interrupt_status)   //ly
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
    current_task_status *the_task_status =  (current_task_status *) the_data;

  /* Handle standard interrupt handler */
  emmc_handle_standard_rinsts(the_data, *interrupt_status);
    
    if(the_task_status->error_status == ERRUNDERWRITE){ 
        // The ERRUNDERWRITE is SET because of Voltage Switch Interrupt. Since this is not the error condition revert back the error status.
        the_task_status->error_status = ERRNOERROR;
    the_task_status->cmd_status = TSK_COMMAND_DONE; //This is required for us to return from ISR
    }
  /* Read the short response and set the command status */
  if (the_task_status->resp_buffer) {
    the_task_status->resp_buffer[0] =  emmc_read_register(emmc_dev, EMMC_REG_RESP0);
  }
      
    return;
}


void long_response_postproc(void *the_data, u32_t * interrupt_status)   //ly
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status =
      (current_task_status *) the_data;

  /* Handle standard interrupt handler */
  emmc_handle_standard_rinsts(the_data, *interrupt_status);

  if (the_task_status->resp_buffer) {
    the_task_status->resp_buffer[0] =  emmc_read_register(emmc_dev, EMMC_REG_RESP0);
    the_task_status->resp_buffer[1] =  emmc_read_register(emmc_dev, EMMC_REG_RESP1);
    the_task_status->resp_buffer[2] =  emmc_read_register(emmc_dev, EMMC_REG_RESP2);
    the_task_status->resp_buffer[3] =  emmc_read_register(emmc_dev, EMMC_REG_RESP3);
  }
}


void no_response_postproc(void *the_data, u32_t * interrupt_status)   //ly
{
  current_task_status *the_task_status = (current_task_status *) the_data;

  /* Handle standard interrupt handler */
  emmc_handle_standard_rinsts(the_data, *interrupt_status);

  /* Check if there is any error */
  if (the_task_status->error_status) {
    the_task_status->cmd_status = TSK_STAT_ABSENT;
    return;
  }
  return;
}


static void r1_r6_response_postproc(void *the_data, u32_t * interrupt_status)    //ly
{
  u32_t r1_check_val;
  current_task_status *the_task_status = (current_task_status *) the_data;

  short_response_postproc(the_data, interrupt_status);

  if (the_task_status->error_status) {
    the_task_status->bus_corruption_occured = 1;
    the_task_status->error_status = 0;
  }

  if (the_task_status->resp_buffer) {
    if ((SD_TYPE == the_card_info[the_task_status->slot_num].card_type) || 
            (SDCOMBO_TYPE == the_card_info[the_task_status->slot_num].card_type) || 
            (SD_MEM_2_0_TYPE == the_card_info[the_task_status->slot_num].card_type) ||
            (SD_MEM_3_0_TYPE == the_card_info[the_task_status->slot_num].card_type)) {
      r1_check_val =  the_task_status->resp_buffer[0] & 0x0000ffff;
    } 
        else if (SDIO_TYPE == the_card_info[the_task_status->slot_num].card_type) {
      r1_check_val =  the_task_status->resp_buffer[0] & 0xffff0000;
      if (r1_check_val & 0xe010) {
                the_task_status->error_status =   ERRHARDWARE;
      }
      return;
    } 
        else {
      r1_check_val = the_task_status->resp_buffer[0];
    }

    the_task_status->error_status = emmc_check_r1_resp(r1_check_val);
  }
  return;
}

#if 1 //ly
static void r5_response_postproc(void *the_data, u32_t * interrupt_status)
{
  current_task_status *the_task_status =(current_task_status *) the_data;
  short_response_postproc(the_data, interrupt_status);
  if (the_task_status->error_status) {
    return;
  }
  if (the_task_status->resp_buffer) {
    the_task_status->error_status = emmc_check_r5_resp(the_task_status->resp_buffer[0]);
  }
  return;
}
#endif

static void r1_response_postproc(void *the_data, u32_t * interrupt_status)    //ly
{
  current_task_status *the_task_status = (current_task_status *) the_data;
  short_response_postproc(the_data, interrupt_status);
  if (the_task_status->error_status) {
    return;
  }

  if (the_task_status->resp_buffer) {
    the_task_status->error_status = emmc_check_r1_resp(the_task_status->resp_buffer[0]);
  }
  return;
}

static void r1b_response_postproc(void *the_data, u32_t * interrupt_status)    //ly
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  r1_response_postproc(the_data,interrupt_status);
  while ((emmc_read_register(emmc_dev, EMMC_REG_STATUS)) & STATUS_DAT_BUSY_BIT);
  return;
}

#if 1
static void r4_response_postproc(void *the_data, u32_t * interrupt_status)
{
  current_task_status *the_task_status = (current_task_status *) the_data;
  short_response_postproc(the_data, interrupt_status);
  if (the_task_status->error_status) {
    return;
  }
  if (the_task_status->resp_buffer) {
    if (!(*(the_task_status->resp_buffer) | R4_RESP_ERROR_BIT)) {
      the_task_status->error_status = ERRHARDWARE;
    }
  }
  return;
}

static void r1_response_write_bstst_postproc(void *the_data,
               u32_t * interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;

  //The following variables are required for IDMAC mode interrupt handling
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;

  r1_response_postproc(the_data, interrupt_status);

  if (ERRENDBITERR == the_task_status->error_status) {
    the_task_status->error_status = 0;
  }

  if (the_task_status->error_status) {
    if (TSK_STATE_WRITEDAT == the_task_status->cmd_status) {
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }
    return;
  }

  cmd_status = the_task_status->cmd_status;
  // The interrupts are interpreted as IDMAC interrupts for data transfer commands in IDMAC mode
  if(the_task_status->idma_mode_on == 1){
    // The data command is supposed to be handled by IDMAC 
    if(*interrupt_status & IDMAC_NI){ // Interrupt for this command
      printk("Relax: Normal Interrupt received for IDMAC\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_NI);
    }
    if(*interrupt_status & IDMAC_TI){ // Transmit Interrupt
      printk("Transmit Interrupt received in IDMAC mode\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_TI);
    }
            //Whether DMA operation is Successful or nor we need to set cmd_status to COMMAND_DONE
            //and get the qptr from the Descriptor list
    if(TSK_STATE_POLLD==the_task_status->cmd_status){
      //Dont give up the qptr
      the_task_status->error_status = 0;//Make error status zero as long as we are issuing poll demand
      the_task_status->cmd_status = TSK_STAT_STARTED;
    }
    else{
                  emmc_get_qptr(&status, &buffer1, &buffer1_virt, &buffer2, &buffer2_virt);
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }

    }
  
  else{
    if (cmd_status != TSK_STATE_WRITEDAT) {
      the_task_status->cmd_status = TSK_STATE_WRITEDAT;
//      if (*interrupt_status & INTMSK_DAT_OVER) {
//        *interrupt_status &= (*interrupt_status & ~INTMSK_DAT_OVER);
//      }
    }
  
    if ((*interrupt_status & INTMSK_TXDR)) {
      emmc_write_out_data(the_task_status, INTMSK_TXDR);
    }

    if (*interrupt_status & INTMSK_DAT_OVER) {
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }

  }
  return;
}


static void r5_response_write_data_postproc(void *the_data,
              u32_t * interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;

  //The following variables are required for IDMAC mode interrupt handling
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;

  r5_response_postproc(the_data, interrupt_status);

  printk("%s: The error status = %d\n", __FUNCTION__,the_task_status->error_status);

  if (ERRENDBITERR == the_task_status->error_status) {
    the_task_status->bus_corruption_occured = 1;
    the_task_status->error_status = 0;
  }
    
  if (the_task_status->error_status) {
    if (TSK_STATE_WRITEDAT == the_task_status->cmd_status) {
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }
    return;
  }

  cmd_status = the_task_status->cmd_status;

  if(the_task_status->idma_mode_on == 1){
    // The data command is supposed to be handled by IDMAC 
    if(*interrupt_status & IDMAC_NI){ // Interrupt for this command
      printk("Relax: Normal Interrupt received for IDMAC\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_NI);
    }
    if(*interrupt_status & IDMAC_TI){ // Transmit Interrupt
      printk("Transmit Interrupt received in IDMAC mode\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_TI);
    }
            //Whether DMA operation is Successful or nor we need to set cmd_status to COMMAND_DONE
            //and get the qptr from the Descriptor list
    if(TSK_STATE_POLLD==the_task_status->cmd_status){
      //Dont give up the qptr
      the_task_status->error_status = 0;//Make error status zero as long as we are issuing poll demand
      the_task_status->cmd_status = TSK_STAT_STARTED;
    }
    else{
                  emmc_get_qptr(&status, &buffer1, &buffer1_virt, &buffer2, &buffer2_virt);
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }


  }
  else{
    if (cmd_status != TSK_STATE_WRITEDAT) {
      the_task_status->cmd_status = TSK_STATE_WRITEDAT;
    } 
      else if ((*interrupt_status & INTMSK_TXDR)) {
      emmc_write_out_data(the_task_status, INTMSK_TXDR);
    }

    if (*interrupt_status & INTMSK_DAT_OVER) {
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }
  }

  return;
}


static void r1b_response_write_data_postproc(void *the_data,
              u32_t * interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;

  //The following variables are required for IDMAC mode interrupt handling
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;

  r1b_response_postproc(the_data, interrupt_status);

  printk("%s: The error status = %d\n", __FUNCTION__,the_task_status->error_status);

  if (ERRENDBITERR == the_task_status->error_status) {
    the_task_status->bus_corruption_occured = 1;
    the_task_status->error_status = 0;
  }

  if (the_task_status->error_status) {
    if (TSK_STATE_WRITEDAT == the_task_status->cmd_status) {
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }
    return;
  }

  cmd_status = the_task_status->cmd_status;

  if(the_task_status->idma_mode_on == 1){
    // The data command is supposed to be handled by IDMAC 
    if(*interrupt_status & IDMAC_NI){ // Interrupt for this command
      printk("Relax: Normal Interrupt received for IDMAC\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_NI);
    }
    if(*interrupt_status & IDMAC_TI){ // Transmit Interrupt
      printk("Transmit Interrupt received in IDMAC mode\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_TI);
    }
            //Whether DMA operation is Successful or nor we need to set cmd_status to COMMAND_DONE
            //and get the qptr from the Descriptor list
    if(TSK_STATE_POLLD==the_task_status->cmd_status){
      //Dont give up the qptr
      the_task_status->error_status = 0;//Make error status zero as long as we are issuing poll demand
      the_task_status->cmd_status = TSK_STAT_STARTED;
    }
    else{
                  emmc_get_qptr(&status, &buffer1, &buffer1_virt, &buffer2, &buffer2_virt);
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }


  }
  else{
    if (cmd_status != TSK_STATE_WRITEDAT) {
      the_task_status->cmd_status = TSK_STATE_WRITEDAT;
    } 
      else if ((*interrupt_status & INTMSK_TXDR)) {
      emmc_write_out_data(the_task_status, INTMSK_TXDR);
    }

    if (*interrupt_status & INTMSK_DAT_OVER) {
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }
  }

  return;
}
#endif

static void r1_response_write_data_postproc(void *the_data,   //ly
              u32_t * interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;

  //The following variables are required for IDMAC mode interrupt handling
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;

  r1_response_postproc(the_data, interrupt_status);
  printk("%s: The error status = %d\n", __FUNCTION__, the_task_status->error_status);

  if (ERRENDBITERR == the_task_status->error_status) {
    the_task_status->bus_corruption_occured = 1;
    the_task_status->error_status = 0;
  }
    /*enable data_command_task*/
  if (the_task_status->error_status) {
    //if (TSK_STATE_WRITEDAT == the_task_status->cmd_status) {
    //  emmc_enable_data_command_tasks();
    //  printk("enable data command task.");
    //}
    return;
  }

  cmd_status = the_task_status->cmd_status;
  // The interrupts are interpreted as IDMAC interrupts for data transfer commands in IDMAC mode
  if(the_task_status->idma_mode_on == 1){
    // The data command is supposed to be handled by IDMAC 
    if(*interrupt_status & IDMAC_NI){ // Interrupt for this command
      printk("Relax: Normal Interrupt received for IDMAC\n");
      emmc_set_register(emmc_dev, EMMC_REG_IDSTS, IDMAC_NI);
    }
    if(*interrupt_status & IDMAC_TI){ // Transmit Interrupt
      printk("Transmit Interrupt received in IDMAC mode\n");
      emmc_set_register(emmc_dev, EMMC_REG_IDSTS, IDMAC_TI);
    }
            //Whether DMA operation is Successful or nor we need to set cmd_status to COMMAND_DONE
            //and get the qptr from the Descriptor list
    if(TSK_STATE_POLLD==the_task_status->cmd_status){
      //Dont give up the qptr
      the_task_status->error_status = 0;//Make error status zero as long as we are issuing poll demand
      the_task_status->cmd_status = TSK_STAT_STARTED;
    }
    else{
      emmc_get_qptr(&status, &buffer1, &buffer1_virt, &buffer2, &buffer2_virt);
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }

    }
  else{
    if (cmd_status != TSK_STATE_WRITEDAT) {
      printk("~~~~~~~~~~~~~~~the value is %08x\n", *interrupt_status);
      the_task_status->cmd_status = TSK_STATE_WRITEDAT;
    }  
        else if ((*interrupt_status & INTMSK_TXDR)) {
      printk("the value is = %08x\n", *interrupt_status);
      emmc_write_out_data(the_task_status, INTMSK_TXDR);
    }

    if (*interrupt_status & INTMSK_DAT_OVER) {
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }
  }
  return;
}



#if 1 //ly
static void r1_response_read_bstst_postproc(void *the_data,
              u32_t * interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;
  //The following variables are required for IDMAC mode interrupt handling
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;

  r1_response_postproc(the_data, interrupt_status);

  if ((ERRDCRC == the_task_status->error_status) || (ERRENDBITERR == the_task_status->error_status)) {
        the_task_status->error_status = 0;
  }

  if (the_task_status->error_status) {
    return;
  }

  if (!(the_task_status->error_status)) {
    the_task_status->cmd_status = TSK_STATE_READDAT;
  }
  
  cmd_status = the_task_status->cmd_status;

  if(the_task_status->idma_mode_on == 1){
    // The data command is supposed to be handled by IDMAC 
    if(*interrupt_status & IDMAC_NI){ // Interrupt for this command
      printk("Relax: Normal Interrupt received for IDMAC\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_NI);
    }
    if(*interrupt_status & IDMAC_RI){ // Receive Interrupt;
      printk("Receive Interrupt received in IDMAC mode\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_RI);
    }
                //Whether DMA operation is Successful or nor we need to set cmd_status to COMMAND_DONE
                //and get the qptr from the Descriptor list. Only exception is when, Descriptor unavailable interrupt occured,
                //Dont get the qptr as ISR may retry using Poll demand
    if(TSK_STATE_POLLD==the_task_status->cmd_status){
      //Dont give up the qptr
      the_task_status->error_status = 0;//Make error status zero as long as we are issuing poll demand
      the_task_status->cmd_status = TSK_STAT_STARTED;
    }
    else{
                  emmc_get_qptr(&status, &buffer1, &buffer1_virt, &buffer2, &buffer2_virt);
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");
    }

  }
    else{
    if (TSK_STATE_READDAT == cmd_status) {
      if ((*interrupt_status & INTMSK_RXDR)) {
        emmc_read_in_data(the_task_status, INTMSK_RXDR);
      }

      if ((*interrupt_status & INTMSK_DAT_OVER)) {
        emmc_read_in_data(the_task_status, INTMSK_DAT_OVER);
      }
      if ((*interrupt_status & INTMSK_DAT_OVER) && !(the_task_status->error_status)) {
        the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");     }
    }
  }
  return;
}

static void r5_response_read_data_postproc(void *the_data,
             u32_t * interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;
  //The following variables are required for IDMAC mode interrupt handling
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;

  r5_response_postproc(the_data, interrupt_status);

//  printk("%s: The error status = %d\n", __FUNCTION__,the_task_status->error_status);

  if (ERRILLEGALCOMMAND == the_task_status->error_status) {
    the_task_status->error_status = 0;
    the_task_status->bus_corruption_occured = 1;
  }

  the_task_status->cmd_status = TSK_STATE_READDAT;

  cmd_status = the_task_status->cmd_status;
  if(the_task_status->idma_mode_on == 1){
    // The data command is supposed to be handled by IDMAC 
    if(*interrupt_status & IDMAC_NI){ // Interrupt for this command
      printk("Relax: Normal Interrupt received for IDMAC\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_NI);
    }
    if(*interrupt_status & IDMAC_RI){ // Receive Interrupt;
      printk("Receive Interrupt received in IDMAC mode\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_RI);
    }
                //Whether DMA operation is Successful or nor we need to set cmd_status to COMMAND_DONE
                //and get the qptr from the Descriptor list. Only exception is when, Descriptor unavailable interrupt occured,
                //Dont get the qptr as ISR may retry using Poll demand
    if(TSK_STATE_POLLD==the_task_status->cmd_status){
      //Dont give up the qptr
      the_task_status->error_status = 0;             //Make error status zero as long as we are issuing poll demand
      the_task_status->cmd_status = TSK_STAT_STARTED;
    }
    else{
                  emmc_get_qptr(&status, &buffer1, &buffer1_virt, &buffer2, &buffer2_virt);
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");   }

  }
    else{
    if ((*interrupt_status & INTMSK_RXDR)) {
      emmc_read_in_data(the_task_status, INTMSK_RXDR);
    }

    if ((*interrupt_status & INTMSK_DAT_OVER)) {
      emmc_read_in_data(the_task_status, INTMSK_DAT_OVER);
    }

    if ((*interrupt_status & INTMSK_DAT_OVER) && (!(the_task_status->error_status))) {
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");   }
  }
  return;

}
#endif

static void r1_response_read_data_postproc(void *the_data,    //ly
             u32_t * interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status = (current_task_status *) the_data;
        u32_t cmd_status;
  //The following variables are required for IDMAC mode interrupt handling
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;

  r1_response_postproc(the_data, interrupt_status);

  if (ERRDCRC == the_task_status->error_status) {
    printk("ACMD51 bus set and err_statys = %x\n",the_task_status->error_status);
    the_task_status->bus_corruption_occured = 1;
    the_task_status->error_status = 0;
  }

    /* cmd_status = the_task_status->cmd_status;*/ /*should come only after cmd_status is updated */

  if ((!(the_task_status->error_status)) && (the_task_status->cmd_status != TSK_STATE_POLLD)) {
    the_task_status->cmd_status = TSK_STATE_READDAT;
  }

        cmd_status = the_task_status->cmd_status; /*should come here */

  if(the_task_status->idma_mode_on == 1){
    // The data command is supposed to be handled by IDMAC 
    if(*interrupt_status & IDMAC_NI){ // Interrupt for this command
      printk("Relax: Normal Interrupt received for IDMAC\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_NI);
    }
    if(*interrupt_status & IDMAC_RI){ // Receive Interrupt;
      printk("Receive Interrupt received in IDMAC mode\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_RI);
    }
                //Whether DMA operation is Successful or nor we need to set cmd_status to COMMAND_DONE
                //and get the qptr from the Descriptor list. Only exception is when, Descriptor unavailable interrupt occured,
                //Dont get the qptr as ISR may retry using Poll demand
    if(TSK_STATE_POLLD==the_task_status->cmd_status){
      //Dont give up the qptr
      the_task_status->error_status = 0;//Make error status zero as long as we are issuing poll demand
      the_task_status->cmd_status = TSK_STAT_STARTED;
    }
    else{
                  emmc_get_qptr(&status, &buffer1, &buffer1_virt, &buffer2, &buffer2_virt);
      the_task_status->cmd_status = TSK_COMMAND_DONE;
      printk("Need to enable data command ( Kernel )\n");
    }
  }
  else{ //Slave mode interrupt handling
    if (TSK_STATE_READDAT == cmd_status) {
      if ((*interrupt_status & INTMSK_RXDR)) {
        emmc_read_in_data(the_task_status, INTMSK_RXDR);
      }

      if ((*interrupt_status & INTMSK_DAT_OVER)) {
        emmc_read_in_data(the_task_status, INTMSK_DAT_OVER);
      }
      if ((*interrupt_status & INTMSK_DAT_OVER) && !(the_task_status->error_status)) {
        the_task_status->cmd_status = TSK_COMMAND_DONE;
    //    emmc_enable_data_command_tasks();   //ly
        printk("Need to enable data command ( Kernel )\n");
      }
    } 
      else {
      if (!(the_task_status->error_status)) {
        the_task_status->cmd_status = TSK_STATE_READDAT;
      }
    }
  }
  return;
}


static void r1_response_stream_read_data_postproc(void *the_data,
              u32_t * interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;
  //The following variables are required for IDMAC mode interrupt handling
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;

  r1_response_postproc(the_data, interrupt_status);

  if (ERRDCRC == the_task_status->error_status) {
    the_task_status->bus_corruption_occured = 1;
    the_task_status->error_status = 0;
  }

  if (ERRUNDERRUN == the_task_status->error_status) {
    the_task_status->bus_corruption_occured = 1;
  }

  the_task_status->cmd_status = TSK_STATE_READDAT;
  
  cmd_status = the_task_status->cmd_status;
  if(the_task_status->idma_mode_on == 1){
    // The data command is supposed to be handled by IDMAC 
    if(*interrupt_status & IDMAC_NI){ // Interrupt for this command
      printk("Relax: Normal Interrupt received for IDMAC\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_NI);
    }
    if(*interrupt_status & IDMAC_RI){ // Receive Interrupt;
      printk("Receive Interrupt received in IDMAC mode\n");
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_RI);
    }
                //Whether DMA operation is Successful or nor we need to set cmd_status to COMMAND_DONE
                //and get the qptr from the Descriptor list. Only exception is when, Descriptor unavailable interrupt occured,
                //Dont get the qptr as ISR may retry using Poll demand
    if(TSK_STATE_POLLD==the_task_status->cmd_status){
      //Dont give up the qptr
      the_task_status->error_status = 0;//Make error status zero as long as we are issuing poll demand
      the_task_status->cmd_status = TSK_STAT_STARTED;
    }
    else{
                  emmc_get_qptr(&status, &buffer1, &buffer1_virt, &buffer2, &buffer2_virt);
      the_task_status->cmd_status = TSK_COMMAND_DONE;
    //    emmc_enable_data_command_tasks();   //ly
        printk("Need to enable data command ( Kernel )\n");
    }

  }
  else{ // Slave mode is handled here
    if (TSK_STATE_READDAT == cmd_status) {
      if ((*interrupt_status & INTMSK_RXDR)) {
        emmc_read_in_data(the_task_status, INTMSK_RXDR);
      }

      if ((*interrupt_status & INTMSK_DAT_OVER)) {
        emmc_read_in_data(the_task_status, INTMSK_DAT_OVER);
      }
      if ((*interrupt_status & INTMSK_DAT_OVER) && !(the_task_status->error_status)) {
        the_task_status->cmd_status = TSK_COMMAND_DONE;
      //    emmc_enable_data_command_tasks();   //ly
        printk("Need to enable data command ( Kernel )\n");
      }
    } 
      else {
      if (!(the_task_status->error_status)) {
        the_task_status->cmd_status = TSK_STATE_READDAT;
      }
    }
  }
  return;
}

#if  1  //ly
static void short_response_sd_app_specific_data(u32_t card_num,
            u32_t cmd_index,
            u32_t * cmd_reg,
            u32_t * arg_reg)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  emmc_send_serial_command(emmc_dev, card_num, CMD55, 0, NULL, NULL, 0, NULL, NULL);
  short_response_preproc(card_num, cmd_index, cmd_reg, arg_reg);
  SET_BITS(*cmd_reg, CMD_WAIT_PRV_DAT_BIT);

  return;
}

static void short_response_sd_data_app_specific_data(u32_t card_num,
                 u32_t cmd_index,
                 u32_t * cmd_reg,
                 u32_t * arg_reg)
{

  short_response_sd_app_specific_data(card_num, cmd_index, cmd_reg,arg_reg);
  SET_BITS(*cmd_reg, CMD_DATA_EXP_BIT);

  return;
}


#ifdef EMULATE_BOOT

void emmc_alt_boot_preproc(u32_t card_num, u32_t cmd_index, u32_t * cmd_reg, u32_t * arg_reg)
{
  SET_BITS(*cmd_reg, CMD_BOOT_MODE);              // Setting BOOT_MODE indicates Alternate boot mode operation
  SET_CMD_INDEX(*cmd_reg, cmd_index);           // Set the CMD index as CMD0
  SET_CARD_NUM(*cmd_reg, card_num);           // Set the Card number to appropriate value
  UNSET_BITS(*cmd_reg, CMD_DISABLE_BOOT);         // UnSet disable boot for Alternate booting

    SET_BITS(*cmd_reg, CMD_DATA_EXP_BIT);       // Boot Mode expects the data so DATA expected bit is set
}

void emmc_alt_boot_with_ack_preproc(u32_t card_num, u32_t cmd_index, u32_t * cmd_reg, u32_t * arg_reg)
{
  SET_BITS(*cmd_reg, CMD_BOOT_MODE);              // Setting BOOT_MODE indicates Alternate boot mode operation
  SET_CMD_INDEX(*cmd_reg, cmd_index);           // Set the CMD index as CMD0
  SET_CARD_NUM(*cmd_reg, card_num);           // Set the Card number to appropriate value
  UNSET_BITS(*cmd_reg, CMD_DISABLE_BOOT);         // UnSet disable boot for Alternate booting
  SET_BITS(*cmd_reg, CMD_EXP_BOOT_ACK);           // Set Boot ACK expected in the command register

    SET_BITS(*cmd_reg, CMD_DATA_EXP_BIT);       // Boot Mode expects the data so DATA expected bit is set
}


void emmc_boot_preproc(u32_t card_num, u32_t cmd_index, u32_t * cmd_reg, u32_t * arg_reg)
{
  SET_CMD_INDEX(*cmd_reg, cmd_index);            // Set the CMD index as CMD0
  SET_CARD_NUM(*cmd_reg, card_num);              // Set the Card number to appropriate value
  SET_BITS(*cmd_reg, CMD_ENABLE_BOOT);             // Set enable_boot for mandatory booting
  UNSET_BITS(*cmd_reg, CMD_DISABLE_BOOT);          // UnSet disable boot for mandatory booting

    SET_BITS(*cmd_reg, CMD_DATA_EXP_BIT);        // Boot Mode expects the data so DATA expected bit is set
}

void emmc_boot_with_ack_preproc(u32_t card_num, u32_t cmd_index, u32_t * cmd_reg, u32_t * arg_reg)
{
  SET_CMD_INDEX(*cmd_reg, cmd_index);            // Set the CMD index as CMD0
  SET_CARD_NUM(*cmd_reg, card_num);              // Set the Card number to appropriate value
  SET_BITS(*cmd_reg, CMD_ENABLE_BOOT);             // Set enable_boot for mandatory booting
  UNSET_BITS(*cmd_reg, CMD_DISABLE_BOOT);          // UnSet disable boot for mandatory booting
  SET_BITS(*cmd_reg, CMD_EXP_BOOT_ACK);            // Set Boot ACK expected in the command register

    SET_BITS(*cmd_reg, CMD_DATA_EXP_BIT);        // Boot Mode expects the data so DATA expected bit is set
}

void emmc_alt_boot_postproc(void * the_data, u32_t * interrupt_status)
{
    //take the private data and cast it to current_task_status
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;

  /* The following reads the ISTSTS and populates the the_task_status->error_status*/
  r1_response_postproc(the_data, interrupt_status);
  
  if (ERRDCRC == the_task_status->error_status) {
    the_task_status->bus_corruption_occured = 1;
    the_task_status->error_status = 0;
  }

  /* Please note that ERRRESTIMEOUT will be set in r1_response_postproc if BAR interrupt has been received. 
   * Since the ISR is not modified, for booting ERRRESPTIMEOUT error_status indicates that the core received BAR Received
   */
  
  if(ERRRESPTIMEOUT == the_task_status->error_status){
  printk("BOOT ACK Received....\n");
  the_task_status->error_status = 0; // This is not the error condition for boot operation. The interrupt is due to Boot ack reception
  }

   /* cmd_status = the_task_status->cmd_status;*/ /*should come only after cmd_status is updated */

  if (!(the_task_status->error_status)) {
    the_task_status->cmd_status = TSK_STATE_READDAT; // Even in case of Booting the state is maintained 
                                                         // as TSK_STATE_READDAT. Same routine is used to read the data
    }

  cmd_status = the_task_status->cmd_status; /*should come here */
  printk("COMMAND_STATUS is %d\n",cmd_status);

  if (TSK_STATE_READDAT == cmd_status) {
    if ((*interrupt_status & INTMSK_RXDR)) {
      emmc_read_in_data(the_task_status, INTMSK_RXDR);
    }

    if ((*interrupt_status & INTMSK_DAT_OVER)) {
      emmc_read_in_data(the_task_status,INTMSK_DAT_OVER);
    }
    if ((*interrupt_status & INTMSK_DAT_OVER) && !(the_task_status->error_status)) {
      the_task_status->cmd_status = TSK_COMMAND_DONE;
//      emmc_enable_data_command_tasks();   ly
      printk("Need to enable data command ( Kernel )\n");   }
  }
    else {
    if (!(the_task_status->error_status)) {
      the_task_status->cmd_status = TSK_STATE_READDAT;
    }
  }
  return;
}


void emmc_boot_postproc(void *the_data, u32_t * interrupt_status)
{
    //take the private data and cast it to current_task_status
  current_task_status *the_task_status = (current_task_status *) the_data;
  u32_t cmd_status;

  /* The following reads the ISTSTS and populates the the_task_status->error_status*/
  r1_response_postproc(the_data, interrupt_status);

  if (ERRDCRC == the_task_status->error_status) {
    the_task_status->bus_corruption_occured = 1;
    the_task_status->error_status = 0;
  }

    /* Please note that ERRRESTIMEOUT will be set in r1_response_postproc if BAR interrupt has 
     * been received. Since the ISR is not modified, for booting ERRRESPTIMEOUT error_status 
     * indicates that the core received BAR Received
     */
  
  if(ERRRESPTIMEOUT == the_task_status->error_status){
  printk("BOOT ACK Received....\n");
  the_task_status->error_status = 0; // This is not the error condition for boot operation. 
                                       // The interrupt is due to Boot ack reception
  }

/*  cmd_status = the_task_status->cmd_status;*/ /*should come only after cmd_status is updated */

  if (!(the_task_status->error_status)) {
    the_task_status->cmd_status = TSK_STATE_READDAT; // Even in case of Booting the state is maintained as
                                                         // TSK_STATE_READDAT. Same routine is used to read the data
    }

  cmd_status = the_task_status->cmd_status; /*should come here */
  printk("COMMAND_STATUS is %d\n",cmd_status);

  if (TSK_STATE_READDAT == cmd_status) {
    if ((*interrupt_status & INTMSK_RXDR)) {
      emmc_read_in_data(the_task_status, INTMSK_RXDR);
    }

    if ((*interrupt_status & INTMSK_DAT_OVER)) {
      emmc_read_in_data(the_task_status,INTMSK_DAT_OVER);
    }
    if ((*interrupt_status & INTMSK_DAT_OVER) && !(the_task_status->error_status)) {
      the_task_status->cmd_status = TSK_COMMAND_DONE;
      printk("Need to enable data command ( Kernel )\n");   }
  } 
    else {
    if (!(the_task_status->error_status)) {
      the_task_status->cmd_status = TSK_STATE_READDAT;
    }
  }
  return;
}
#endif
#endif

/**
  * Command Index callback table.
  * This is a table sorted by command index for the command indices and
  * their corresponding callbacks. This table allows easy manipulation of
  * new command addition and thier handling logic. 
  * \todo Use this table for a minimal perfect hashing rather than a 
  * binary search <i>(use gperf ?)</i>
  */
static callback_search_table the_callback_table[] = {
    {CMD0       , {no_response_preproc_abrt, no_response_postproc}},   //ly
    {CMD1       , {short_response_preproc_with_init, short_response_postproc}},    //
    {CMD2       , {long_response_preproc, long_response_postproc}},   //
    {CMD3       , {short_response_preproc, r1_r6_response_postproc}},    //
    {CMD5       , {short_response_preproc, short_response_postproc}},    //
    {CMD6       , {short_response_preproc, r1b_response_postproc}},          //select ly
    {CMD7       , {short_response_rca_preproc, r1b_response_postproc}},   //
//  {CMD8       , {short_response_block_data_preproc_noac, r1_response_read_data_postproc}},   //
    {CMD8       , {short_response_preproc, short_response_postproc}},
    {CMD9       , {long_response_rca_preproc, long_response_postproc}},   //SEND_CSD 
    {CMD11      , {short_response_stream_data_preproc, r1_response_stream_read_data_postproc}},  //VOLTAGE_SWITCH
    {CMD12      , {short_response_preproc_abrt, r1b_response_postproc}},   //
    {CMD13      , {short_response_rca_preproc, r1_response_postproc}},     //
    {CMD14      , {short_response_block_data_preproc_noac, r1_response_read_bstst_postproc}},
    {CMD15      , {no_response_preproc, no_response_postproc}},    //GO_INACTIVE_STATE
    {CMD16      , {short_response_preproc, r1_response_postproc}},    //
    {CMD17      , {short_response_block_data_preproc, r1_response_read_data_postproc}},
    {CMD18      , {short_response_block_data_preproc, r1_response_read_data_postproc}},    //
    {CMD19      , {short_response_block_write_preproc_noac, r1_response_write_bstst_postproc}},
    {CMD20      , {short_response_stream_write_preproc, r1_response_write_data_postproc}},
    {CMD23      , {short_response_preproc, r1_response_postproc}},   //
    {CMD24      , {short_response_block_write_preproc, r1_response_write_data_postproc}},
    {CMD25      , {short_response_block_write_preproc, r1_response_write_data_postproc}},   //
    {CMD32      , {short_response_preproc, r1_response_postproc}},   //
    {CMD33      , {short_response_preproc, r1_response_postproc}},   //
    {CMD35      , {short_response_preproc, r1_response_postproc}},
    {CMD36      , {short_response_preproc, r1_response_postproc}},
    {CMD38      , {short_response_preproc, r1b_response_postproc}},   //
    {CMD39      , {short_response_rca_preproc, r4_response_postproc}},
    {ACMD41     , {short_response_preproc, short_response_postproc}},   //
    {ACMD42     , {short_response_sd_app_specific_data, r1_response_postproc}},   // new add comment
    {CMD42      , {short_response_block_write_preproc, r1b_response_write_data_postproc}},   //delete
    {ACMD51     , {short_response_sd_data_app_specific_data, r1_response_read_data_postproc}},
    {CMD52      , {short_response_preproc, r5_response_postproc}},
    {CMD53      , {short_response_block_data_preproc_noac, r5_response_read_data_postproc}},
    {CMD55      , {short_response_rca_preproc, short_response_postproc}},   //
    {ACMD6      , {short_response_sd_app_specific_data, r1_response_postproc}},
    {UNADD_CMD7 , {no_response_preproc, no_response_postproc}},
    {SD_CMD8    , {short_response_preproc, short_response_postproc}},
    {SD_CMD11   , {short_response_preproc_volt_switch_stage_1, short_response_postproc_volt_switch_stage_1}},
    {WCMD53     , {short_response_block_write_preproc_noac, r5_response_write_data_postproc}}
};
//39 table

/**
  * Finds the set of callbacks for the command index.
  * Performs a binary search on the statically defined the_callback_table.
  * @param[in] cmd_index The command index.
  * \return Returns the pointer to the callbacks if found. Returns NULL if not
  * found.
  * \todo This function has to be converted to a minimal perfect hash table search.
  * \callgraph
  */
static Callbacks *emmc_lookup_callback_table(u32_t cmd_index)
{
  u32_t num_commands = (sizeof(the_callback_table) / sizeof(callback_search_table)) -1;
  u32_t left, right;
  u32_t present_index;

  left = 0;
  right = num_commands;

  while (left <= right) {
    present_index = left + (right - left) / 2;  
    if (the_callback_table[present_index].cmd_index ==  cmd_index) {

      return &(the_callback_table[present_index].the_callbacks);
    } 
        else if (cmd_index >  the_callback_table[present_index].cmd_index) {
      left = present_index + 1;
    } 
        else {
      right = present_index - 1;
    }
  }
  return NULL;
}

static void PPU_emmc_set_current_task_status(struct device *dev, u32_t slot, u32_t * resp_buffer,
              u8_t * data_buffer,
              emmc_postproc_callback
              the_completion_callback)
{
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  current_task.postproc_callback = the_completion_callback;
  current_task.resp_buffer = resp_buffer;
  printk("current_task.resp_buffer is %x \n",current_task.resp_buffer);
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  current_task.data_buffer = data_buffer;
  current_task.slot_num = slot;
  /* If the error status is not updated, it would mean that the command has 
     timed out and no response has been actually received from the card.
   */
  current_task.error_status = 0;
  current_task.aborted_command = 0;
  current_task.cmd_status = TSK_STAT_STARTED;
  current_task.bus_corruption_occured = 0;
  printk("set current_task_status  ly\n");
  return;
}

emmc_postproc_callback PPU_emmc_get_post_callback(struct device *dev, u32_t cmd_index)
{
  emmc_postproc_callback retval = NULL;
  Callbacks *the_callbacks;
  the_callbacks = emmc_lookup_callback_table(cmd_index);
  if (!the_callbacks) {
    printk("the callback post value is 0! ly \n");
    return retval;
  }

  retval = the_callbacks->postproc;
  printk("the callback postproc adders  %x  \n",retval);
  return retval;
}

emmc_preproc_callback PPU_emmc_get_pre_callback(struct device *dev, u32_t cmd_index)
{
  emmc_preproc_callback retval = NULL;
  Callbacks *the_callbacks;
  the_callbacks = emmc_lookup_callback_table(cmd_index);
  if (!the_callbacks) {
    printk("the callback pre value is 0! ly \n");
    return retval;
  }

  retval = the_callbacks->preproc;
  return retval;
}



//void emmc_callback_ppu_isr (void){ 
//}

static const struct emmc_callback_driver_api emmc_callback_ppu_drv_api_funcs = {
    .ppu_emmc_set_current_task_status = PPU_emmc_set_current_task_status,
    .ppu_emmc_get_post_callback = PPU_emmc_get_post_callback,
    .ppu_emmc_get_pre_callback = PPU_emmc_get_pre_callback,
};

static int emmc_callback_ppu_init(struct device *dev)
{
  return 0;
}

#ifdef CONFIG_EMMC_CALLBACK

//static void emmc_callback_ppu_irq_config_func_0(void);

static const struct emmc_callback_ppu_config emmc_callback_ppu_cfg_0 = {
    .emmc_callback_base_addr = PPU_EMMC_BASE,
    //.irq_config = emmc_callback_ppu_irq_config_func_0,
};

DEVICE_AND_API_INIT(emmc_callback_ppu_0, "emmc_callback_0", &emmc_callback_ppu_init,
                    NULL, &emmc_callback_ppu_cfg_0,
                    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                    &emmc_callback_ppu_drv_api_funcs);

/*static void emmc_callback_ppu_irq_config_func_0(void)
{
    IRQ_CONNECT(PPU_EMMC_IRQ,
                CONFIG_EMMC_IRQ_PRI,
                emmc_callback_ppu_isr,
                DEVICE_GET(emmc_callback_ppu_0),
                0);
    irq_enable(PPU_EMMC_IRQ);
}*/
#endif