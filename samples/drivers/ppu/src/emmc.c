#include <misc/printk.h>
#include <device.h>
#include <gpio.h>
#include <misc/util.h>
#include <emmc.h>
#include <emmc_callback.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <kernel.h>


#define EMMC_DRV_NAME "emmc_0"
#define LCD_DRV_NAME "lcd_0"

#define MEM_SIZE 8192
#define SECTOR_SIZE     512 //4 // 8 //16 // 32 //64 //128 //256 //512
#define TOT_NO_OF_SECTORS (MEM_SIZE/SECTOR_SIZE)
#define SECT_SIZE_SHIFT 9   //2 // 3 // 4 //5   //6  //7   //8   //9

#define WRITE 1
#define READ 0


#define NUM_OF_MUTLIPLE_BLOCKS  1

#define NO_OF_TEST_LOOPS  3  
//extern Card_info *the_card_info;

struct device *lcd_dev; 
current_task_status current_task;
Card_info Card_info_inst[2];
Card_info *the_card_info = Card_info_inst;
IP_status_info the_ip_status;

struct device *emmc_callback_dev;

//////////interupt handler
u32_t a;
//u32 cmd_tx_done=0;
 int interrupt_already_done ;
 int data_transfer_already_done;

static u32_t poll_demand_count = 0;
    

u32_t emmc_enumerate_the_card(struct device *dev, u32_t slot_num);

u32_t emmc_reset_sdio_card(struct device *dev, u32_t slot);

u32_t emmc_io_rw_52(struct device *dev, u32_t slot, u32_t func, u32_t address, u8_t * data,
          u32_t read_or_write, u32_t op_flags);

u32_t emmc_io_rw(struct device *dev, u32_t slot, u32_t func, u32_t address, u8_t * data,
       u32_t length, u32_t read_or_write, u32_t op_flags,
       emmc_term_function the_term_function,
       emmc_copy_function the_copy_function);

Card_Type emmc_get_card_type(struct device *dev, u32_t slot);

u32_t emmc_reset_sd_2_0_card(struct device *dev, u32_t slot);

u32_t emmc_process_SD_2_0_csd(struct device *dev, u32_t slot);

u32_t emmc_process_scr(struct device *dev, u32_t slot);

u32_t emmc_read_write_bytes(struct device *dev, u32_t slot, u32_t * resp_buffer,
            u8_t * data_buffer, u32_t start, u32_t end,
            u32_t argreg,
            emmc_copy_function the_copy_func,
            emmc_term_function
            the_term_function, u32_t read_or_write,
            u32_t custom_command,
            emmc_preproc_callback custom_pre,
            emmc_postproc_callback custom_post);

u32_t emmc_is_card_ready_for_data(struct device *dev, u32_t slot);

dma_addr_t plat_map_single(void * bus_device, u8_t * data_buffer, u32_t len1, u32_t access_type);

u32_t emmc_is_desc_free(DmaDesc *desc);

u32_t emmc_is_desc_chained(DmaDesc *desc);

u32_t emmc_set_sd_2_0_voltage_range(struct device *dev, u32_t slot);

u32_t emmc_reset_sd_card(struct device *dev, u32_t slot);

u32_t emmc_set_sd_voltage_range(struct device *dev, u32_t slot);

u32_t emmc_process_csd(struct device *dev, u32_t slot);

u32_t emmc_set_sd_wide_bus(struct device *dev, u32_t slot, u32_t width);

u32_t emmc_put_in_trans_state(struct device* dev, u32_t slot);

u32_t emmc_get_status_of_card(struct device *dev, u32_t slot, Card_state * status);

u32_t emmc_send_serial_command(struct device *dev, u32_t slot, u32_t cmd_index, u32_t arg,
           u32_t * response_buffer, u8_t * data_buffer,
           u32_t flags,
           emmc_preproc_callback custom_preproc,
           emmc_postproc_callback custom_postproc);

u32_t emmc_cmd_to_host(struct device *dev, u32_t slot,
       u32_t
       cmd_register,
       u32_t
       arg_register,
       u32_t *
       resp_buffer,
       u8_t *
       data_buffer,
       emmc_postproc_callback the_callback,
       u32_t flags);

u32_t emmc_set_clk_freq(struct device *dev, u32_t divider);

u32_t emmc_disable_all_clocks(struct device *dev);

u32_t emmc_enable_clocks_with_val(struct device *dev, u32_t val);

u32_t emmc_form_n_send_cmd(struct device *dev,
         u32_t  card_num,
           u32_t  cmd_index,
           u32_t  cmd_arg,
           u32_t *resp_buffer,
           u8_t  *data_buffer,
           u32_t  flags,
           emmc_postproc_callback  custom_callback,
           emmc_preproc_callback  custom_preproc);

u32_t emmc_get_cid(struct device *dev, u32_t slot);

u32_t emmc_set_sd_rca(struct device *dev, u32_t slot);

u32_t plat_reenable_upon_interrupt_timeout(struct device *dev);

u32_t emmc_last_com_status(void);

u32_t emmc_bus_corruption_present(void);

u32_t emmc_is_desc_owned_by_dma(DmaDesc *desc);

void emmc_remove_command(struct device *dev);

void emmc_send_raw_command(struct device *dev, u32_t slot, u32_t cmd, u32_t arg);

void emmc_abort_trans_work(struct device *dev, u32_t slot);

void emmc_set_data_trans_params(struct device *dev, u32_t slot, u8_t * data_buffer,  
            u32_t num_of_blocks,
            emmc_term_function
            the_term_func,
            emmc_copy_function the_copy_func,
            u32_t epoch_count, u32_t flag,
            u32_t custom_blocksize); 

void plat_unmap_single(void * bus_device,dma_addr_t desc2 ,u32_t dummy, u32_t access_type);

s32_t emmc_set_qptr(struct device *dev, u32_t Buffer1, u32_t Length1, u32_t Buffer1_Virt, u32_t Buffer2, u32_t Length2, 
        u32_t Buffer2_Virt);

void emmc_handle_standard_rinsts(void *prv_data, u32_t int_status);

void emmc_dump_descriptors(u32_t Mode);

boolean emmc_is_last_desc(DmaDesc *desc); 

void plat_delay(u32_t delay_value)
{
    volatile u32_t d;
    for(d = 0; d < delay_value; d++);
}

u32_t emmc_execute_command(struct device *dev, u32_t cmd_register, u32_t arg_register)
{
  emmc_set_register(dev, EMMC_REG_CMDARG, arg_register);
  emmc_set_register(dev, EMMC_REG_CMD, cmd_register | CMD_HOLD_REG);
  while (1) {
  if ((L_EMMC_REG_CMD & CMD_DONE_BIT) == 0) {
            printk("CMD_DONE_BIT clear. value of EMMC_REG_CMD: %x \n", L_EMMC_REG_CMD);
      break;
      }
        plat_delay(1000);
   }
  
  return 0;
}

/////////////////////////////////////////////////////////////////////
/**
  * Sends the clock only command.
  * This function loads the clock settings to the card. Does not pass
  * an messages to the card.
  *
  * \return 0 upon success. Error status upon failure.
  */
u32_t emmc_send_clock_only_cmd(struct device *dev)
{
  return emmc_execute_command(dev, CLK_ONLY_CMD |
          CMD_WAIT_PRV_DAT_BIT | CMD_HOLD_REG, 0);
}

/*
This function is a blocking function. it reads the CMD register for CMD_MAX_RETRIES to
see if the CMD_DONE_BIT is set to 0 by CIU 
*/

/*u32_t emmc_poll_cmd_register(void)
{
  while (1) {
  if ((L_EMMC_REG_CMD & CMD_DONE_BIT) == 0) {
            printk("CMD_DONE_BIT clear. value of EMMC_REG_CMD: %x \n", L_EMMC_REG_CMD);
      break;
      }
        plat_delay(1000);
   }
  
  return 0;

}*/

u32_t emmc_enumerate_card_stack(struct device *dev, u32_t num_of_cards)
{
  u32_t counter;
  /* Lets start Enumerating every card connected at the power up time */
  for (counter = 0; counter < the_ip_status.num_of_cards; counter++) {
    printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
    //printk("Enumerating slot %x\n", counter);
    /*
    Start enumeration of each card individually with initializing the card info with 
    card state empty (-1) for card_state and 
    card not connected state for enumeration state
    */
    the_card_info[counter].card_state = CARD_STATE_EMPTY;  //-1
    the_card_info[counter].enum_status = ERRCARDNOTCONN;   //1
    
    emmc_enumerate_the_card(dev, counter);
    the_ip_status.num_of_cards = 1;
  }

  return 0;
}


u32_t emmc_init_controller(struct device *dev)
{
    u32_t buffer_reg = 0; /* multipurpose buffer register */
    u32_t num_of_cards, fifo_thresh;
    s32_t retval = 0;
    memset((void *) &the_ip_status, 0, sizeof(the_ip_status));
    emmc_set_register(dev, EMMC_REG_UHS_REG_EXT,0xC1020000);   //ext_clk_div 8, clk_2x = 100Mhz, clk_in = 25Mhz, clk_drv and clk_sample delay 4 clk_2x, 
    printk(" Value of EMMC_REG_UHS_REG_EXT : %x .\n", emmc_read_register(dev, EMMC_REG_UHS_REG_EXT));
    emmc_set_bits(dev, EMMC_REG_GPIO,0x00000100);  //open cclk_in
    
    while(1){
          if(L_EMMC_REG_GPIO & 0X00000001 == 1)
                printk("clk_in open. \n");
          break;
            };

    plat_delay(100);

    /*Befoer proceeding further lets reset the host controller IP
    This can be achieved by writing 0x00000001 to CTRL register*/
    buffer_reg = 0x00000001;    //controller_reset
    emmc_set_bits(dev, EMMC_REG_CTRL,buffer_reg);
    
    plat_delay(1000);  

    /* No. of cards supported by the IP */
    buffer_reg = emmc_read_register(dev, EMMC_REG_HCON);     //hardware configuration register 
    the_ip_status.num_of_cards = 1;
  
    emmc_set_register(dev, EMMC_REG_RINTSTS, 0xffffffff);  
    buffer_reg = INTMSK_ALL_ENABLED & ~INTMSK_ACD;
    emmc_set_register(dev, EMMC_REG_INTMSK, buffer_reg);
    emmc_set_bits(dev, EMMC_REG_CTRL, INT_ENABLE);           //open globle int enable

    /* Set Data and Response timeout to Maximum Value*/
    emmc_set_register(dev, EMMC_REG_TMOUT, 0xffffffff);
    
    /* Enable the clocks to the all connected cards/drives
     - Note this command is to CIU of host controller ip
     - the command is not sent on the command bus
     - it emplys the polling method to accomplish the task
     - it also emplys wait prev data complete in CMD register 
        */
    u32_t clock_val;
    //  clock_val = (1 << the_ip_status.num_of_cards) - 1 ; 
    clock_val = 1;
    emmc_set_register(dev, EMMC_REG_CLKSRC, 0);
    emmc_set_register(dev, EMMC_REG_CLKDIV, 15);  //set clk_div 63, fout = 400k
    emmc_set_register(dev, EMMC_REG_CLKENA, clock_val);
    printk("EMMC_REG_CLKDIV is %x\n",L_EMMC_REG_CLKDIV);
    emmc_send_clock_only_cmd(dev);  //only update CLKDIV CLKSRC CLKENA

    printk("CLK updata complete.\n");

    /* Set the card Debounce to allow the CDETECT fluctuations to settle down*/ 
    emmc_set_register(dev, EMMC_REG_DEBNCE, DEFAULT_DEBNCE_VAL);

    /* update the global structure of the ip with the no of cards present at this point of time */
    the_ip_status.present_cdetect = emmc_read_register(dev, EMMC_REG_CDETECT);
    printk("emmc_cdetect is %x \n",the_ip_status.present_cdetect);
    /* Update the watermark levels to half the fifo depth
     - while reset bitsp[27..16] contains FIFO Depth
     - Setup Tx Watermark
     - Setup Rx Watermark
        */
    fifo_thresh = emmc_read_register(dev, EMMC_REG_FIFOTH);
    printk("FIFOTH read from Hardware = %08x\n",fifo_thresh);
    fifo_thresh = GET_FIFO_DEPTH(fifo_thresh) / 2;
    the_ip_status.fifo_depth = fifo_thresh * 2;
    printk("FIFO depth = %x\n", the_ip_status.fifo_depth);
    the_ip_status.fifo_threshold = fifo_thresh;
    /* Tx Watermark */
    emmc_clear_bits(dev, EMMC_REG_FIFOTH, 0xfff);
    emmc_set_bits(dev, EMMC_REG_FIFOTH, fifo_thresh);
    /* Rx Watermark */
    emmc_clear_bits(dev, EMMC_REG_FIFOTH, 0x0fff0000);
    emmc_set_bits(dev, EMMC_REG_FIFOTH, (fifo_thresh-1) << 16);

    /* Enumerate the cards connected */
    emmc_enumerate_card_stack(dev, num_of_cards);

    return retval;
}


/**
  * Enumerates the specified card.
  * This function determines the type of card in the specified slot and calls 
  * the respective enumeration function for the type of card.
  * @param[in] slot_num The slot number which has to be enumerated.
  * \return Returns 0 upon successful enumeration. Error status upon error.
  */
u32_t emmc_enumerate_the_card(struct device *dev, u32_t slot_num)
{
  Card_Type type;
  u32_t retval = 0;
  
  /*
  To start with, without knowing what kind of card/drive start enumeration with the lowest clk
  So configure the divider value accordingly. this is the function of cclk_in of host IP.
  */

    the_card_info[slot_num].divider_val = 15;

  /*Lets start with the single bit mode initially*/
  emmc_clear_bits(dev, EMMC_REG_CTYPE, ((1<<slot_num) | (1<<(slot_num + 16)))); //1bit mode

    
  /*Get the card type by connected to this particular slot and update the card information
    Lets make the enum_status zero just a hack since the function emmc_send_serial_command 
    checks for enum_status */
  the_card_info[slot_num].enum_status = 0;
    
  type = emmc_get_card_type(dev, slot_num);
  
  printk("MAHOO got card type as %x\n", type);
  the_card_info[slot_num].card_type = type;

  /*
  I dont understand why enum_status ERRCARDNOTCONN even after card detected???
  */
  the_card_info[slot_num].enum_status = ERRCARDNOTCONN;      //????ly
  
  switch (type) {
  case SD_TYPE:
    printk("SD FOUND AT SLOT %x\n", slot_num);
    retval = emmc_reset_sd_card(dev, slot_num);
    if(retval)
    printk("SD MEM reset returned the error %x\n", retval);
    break;
  case SD_MEM_2_0_TYPE:
    printk("SD2.0 FOUND AT SLOT %x\n", slot_num);
    retval =  emmc_reset_sd_2_0_card(dev, slot_num);
    if(retval == ERRENUMERATE){
    printk("ERROR in SD2.0 Enumeration: Card could be a SDSC Card\n");
            //The Host controller is programmed to work in higher frequency. But enumeration should 
            //take place in 400KHZ. Modify the CLKDIV value to SD enumerate Frequency
            //emmc_set_clk_freq(SD_FOD_DIVIDER_VALUE);
    }
    if(retval)
    printk("SD_MEM_2_0 reset returned the error %x\n", retval);
    break;
  case SDIO_TYPE:
    printk("SDIO FOUND AT SLOT %x\n", slot_num);
    retval = emmc_reset_sdio_card(dev, slot_num);
    if(retval)
    printk("SDIO reset returned the error %x\n",retval);
    break;

  case NONE_TYPE:
    printk("NO DEVICE AT SLOT %x\n", slot_num);
    retval = ERRNOTSUPPORTED;
    break;
  case ERRTYPE:
  default:
    printk("BAD CARD FOUND AT SLOT %x\n", slot_num);
    return ERRENUMERATE;
  }

  return retval;
}

u32_t emmc_reset_sdio_card(struct device *dev, u32_t slot)
{

  u32_t buffer_reg;
  u32_t retval = 0;
  u32_t clock_freq_to_set = SD_ONE_BIT_BUS_FREQ;
  u32_t io, mem, mp, f;
  u32_t resp;
  u32_t new_ocr;
  u8_t enable_data;
  int count;
  u8_t dat = 0x08;

  io = mem = mp = f = 0;
  /* Check if the card is connected */
  buffer_reg = emmc_read_register(dev, EMMC_REG_CDETECT);
  if (buffer_reg & (1 << slot)) {
    retval = ERRCARDNOTCONN;
    goto HOUSEKEEP;
  }

  emmc_set_bits(dev, EMMC_REG_CTRL, ENABLE_OD_PULLUP);
  the_card_info[slot].divider_val = SD_FOD_DIVIDER_VALUE;
  the_card_info[slot].enum_status = 0;

  /*
  Send the CMD5 with argument 0 to see whether the SDIO card responds?
  */
  if ((retval =  emmc_send_serial_command(dev, slot, CMD5, 0, &resp, NULL, 0, NULL, NULL))) {
    goto HOUSEKEEP;
  }

  /*
  Yes the card responded. Now send the OCR value to the card (to set new voltage). This 
  command is tried for CMD5_RETRY_COUNT(50) times. On PCI bus it fails So some 
  tweak in the delay incorporated.
  */
  new_ocr = resp & IO_R4_OCR_MSK;    //P203   DesignWare

  count = CMD5_RETRY_COUNT;
  while (count) {
    if ((retval =  emmc_send_serial_command(dev, slot, CMD5, new_ocr,  &resp, NULL, 0, NULL,  NULL))) {
      goto HOUSEKEEP;
    }
    if (resp & IO_R4_READY_BIT) {
      break;
    }
    plat_delay(100); /* changed from 10 */
    count--;
  }

  printk("The OCR from RESP is 0x%08x\n", resp);

  if (!count) {
    printk("Count not set voltage on the SDIO card\n");
    retval = ERRHARDWARE;
    goto HOUSEKEEP;
  }

  /*If memory present in the received R4 response set the mp flag 1 indication memory present */
  if (resp & IO_R4_MEM_PRESENT_BIT) {
    mp = 1;
  }
  /*Get the no of IO functions supported by the card from R4 response*/
  f = GET_IO_R4_NUM_FUNCS(resp);

  printk("The mp = %x and f= %x\n", mp, f);
  /*If no of functions not equal to '0' set io support flat to '1'*/
  if (f) {
    io = 1;
  }

  if (mp) {
    /* This has a memory function and hence we  initialise the memory module also */
    if ((retval = emmc_reset_sd_card(dev, slot))) {
      goto HOUSEKEEP;
    }
  } else if (io) {
    /*Set the RCA using CMD3: please note that response for CMD3 is modified R6 response for 
      SDIO cards please read the specs for details*/
    if ((retval =  emmc_send_serial_command(dev, slot, CMD3, 0, &resp, NULL, 0, NULL,  NULL))) {
      goto HOUSEKEEP;
    }
    printk("GOT RESP FOR CMD3 for sdio = 0x%08x\n", resp);


    the_card_info[slot].card_state = CARD_STATE_STBY;
    the_card_info[slot].the_rca = GET_R6_RCA(resp);
    printk("this is 222222222222222222%x\n",the_card_info[slot].the_rca);
  }

  emmc_clear_bits(dev, EMMC_REG_CTRL, ENABLE_OD_PULLUP);
  the_card_info[slot].divider_val = ONE_BIT_BUS_FREQ;
  /* Put it in command mode */
  if ((retval = emmc_send_serial_command(dev, slot, CMD7, 0, &resp, NULL, 0, NULL, NULL))) {
    return retval;
  }


  /* Enable all functions by default. You may actually  remove this code if you want to. */
  if (f) {
    /*Set CCCR ENABLE bit for all the functions*/
    dat = enable_data = (((1 << f) - 1)) << 1;
    if ((retval = emmc_io_rw_52(dev, 0, 0, CCCR_ENABLE, &enable_data, 1,0))) {
      goto HOUSEKEEP;
    }
    /* The functions have been enabled, now poll on the ready register to see
       if they actually have been enabled or not
     */
    count = 1000;/*to be consistent: all our polling are of 1000 iterations */
    dat = 0;
    while (count) {
      /*Read CCCR READY to make sure the SDIO is ready*/
      if ((retval = emmc_io_rw_52(dev, 0, 0, CCCR_READY, &dat, 0, 0))) {
        goto HOUSEKEEP;
      }

      if (enable_data & dat) {
        break;
      }
      plat_delay(100);/* changed from 50 */

      dat = 0;
      count--;
    }

    if (!count) {
      retval = ERRHARDWARE;
      goto HOUSEKEEP;
    }
  }


  /*If both memory present and io present it is SD combo type*/
  if (mp && io) {
    the_card_info[slot].card_type = SDCOMBO_TYPE;
  }
  /* read the CCCR. Now read 20 bytes */
  retval =
      emmc_io_rw(dev, slot, 0, CCCR_START,
         the_card_info[slot].the_cccr_bytes, CCCR_LENGTH,
         0, 0, NULL, NULL);

      HOUSEKEEP:
  the_card_info[slot].divider_val = clock_freq_to_set;
  the_card_info[slot].enum_status = retval;
  emmc_clear_bits(dev, EMMC_REG_CTRL, ENABLE_OD_PULLUP);
  emmc_set_clk_freq(dev, clock_freq_to_set);
  return retval;
}

/**
  * This function dumps the DWC_Mobstor register for debugging.
  * This is useful for debugging. 
  * \param[in] void.
  * \return Returns Void
  */

void emmc_dump_registers(void)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  printk("CTRL     : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_CTRL));
  printk("PWREN    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_PWREN));
  printk("CLKDIV   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_CLKDIV));
  printk("CLKSRC   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_CLKSRC));
  printk("CLKENA   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_CLKENA));
  printk("TMOUT    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_TMOUT));
  printk("CTYPE    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_CTYPE));
  printk("BLKSIZ   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_BLKSIZ));
  printk("BYTCNT   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_BYTCNT));
  printk("INTMSK   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_INTMSK));
  printk("RESP0    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_RESP0));
  printk("RESP1    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_RESP1));
  printk("RESP2    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_RESP2));
  printk("RESP3    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_RESP3));
  printk("MINTSTS  : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_MINTSTS));
  printk("RINTSTS  : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_RINTSTS));
  printk("STATUS   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_STATUS));
  printk("FIFOTH   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_FIFOTH));
  printk("CDETECT  : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_CDETECT));
  printk("WRTPRT   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_WRTPRT));
  printk("GPIO     : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_GPIO));
  printk("TCBCNT   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_TCBCNT));
  printk("TBBCNT   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_TBBCNT));
  printk("DEBNCE   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_DEBNCE));
  printk("USRID    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_USRID));
  printk("VERID    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_VERID));

  printk("BMOD     : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_BMOD));
  printk("PLDMND   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_PLDMND));
  printk("DBADDR   : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_DBADDR));
  printk("IDSTS    : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_IDSTS));
  printk("IDINTEN  : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_IDINTEN));
  printk("DSCADDR  : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_DSCADDR));
  printk("BUFADDR  : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_BUFADDR));

  printk("HCON     : %08x\n", emmc_read_register(emmc_dev, EMMC_REG_HCON));
}


u32_t emmc_io_rw_52(struct device *dev, u32_t slot, u32_t func, u32_t address, u8_t * data,
          u32_t read_or_write, u32_t op_flags)
{
  u32_t retval = 0;
  u32_t cmd52_arg = 0;
  u32_t resp, rinsts;
  current_task_status dummy_task = { 0 };


  if (emmc_read_register(dev, EMMC_REG_CDETECT) & (1 << slot)) {
    retval = ERRCARDNOTCONN;
    goto HOUSEKEEP;
  }


  if (func > 7) {
    retval = ERRADDRESSMISALIGN;
    goto HOUSEKEEP;
  }

  /* We have case for extended IO */
  if (read_or_write) {
    SET_BITS(cmd52_arg, IO_RW_RW_FLG_MSK);
    cmd52_arg |= *data;
  }

  SET_FUNC_NUM(cmd52_arg, func);


  if (address > IO_RW_REG_MAX_VAL) {
    retval = ERRADDRESSMISALIGN;
    goto HOUSEKEEP;
  }

  SET_IO_RW_REG_ADD(cmd52_arg, address);

  if (op_flags & SDIO_USE_ASYNC_READ) {
    emmc_send_raw_command(dev, slot,CMD52 | CMD_RESP_EXP_BIT, cmd52_arg);
    while (!(emmc_read_register(dev, EMMC_REG_RINTSTS) & CMD_DONE_BIT));
    rinsts = emmc_read_register(dev, EMMC_REG_RINTSTS);
    emmc_handle_standard_rinsts(&dummy_task, rinsts);
    if (dummy_task.error_status) {
      retval = dummy_task.error_status;
      goto HOUSEKEEP;
    }
    resp = emmc_read_register(dev, EMMC_REG_RESP0);
  } else {

    if ((retval =
         emmc_send_serial_command(dev, slot, CMD52,
              cmd52_arg, &resp,
              NULL, 0, NULL, NULL))) {
      goto HOUSEKEEP;
    }
  }

  *data = (u8_t) (resp & IO_RW_DATA_MSK);

      HOUSEKEEP:
  return retval;
}

void emmc_handle_standard_rinsts(void *prv_data, u32_t int_status)   //rintsts:Raw Interrupt status reg
{
  u32_t raw_int_stat = int_status;
  current_task_status *the_task_stat =  (current_task_status *) prv_data;

  if (raw_int_stat & INTMSK_CMD_DONE) {
    the_task_stat->cmd_status = TSK_COMMAND_DONE;
  }

  /** It is important to place resptimeout first 
      since there are a few commands for which a 
      timeout is not an error condition
   **/
  if (raw_int_stat & INTMSK_RTO) {                 //0x00000100
    the_task_stat->error_status = ERRRESPTIMEOUT;
  }

  if (raw_int_stat & INTMSK_EBE) {
    the_task_stat->error_status = ERRENDBITERR;
  }

  if (raw_int_stat & INTMSK_RESP_ERR) {
    /* We have a problem with the response reception */
    the_task_stat->error_status = ERRRESPRECEP;
  }

  if (raw_int_stat & INTMSK_RCRC) {
    /* We have a problem with the response reception */
    the_task_stat->error_status = ERRRESPRECEP;
  }

  /* Check the data and and fifo interrupts */
  if (raw_int_stat & INTMSK_FRUN) {
    the_task_stat->error_status = ERROVERREAD;
  }

  if (raw_int_stat & INTMSK_HTO) {
    the_task_stat->error_status = ERRUNDERWRITE;
  }

  if (raw_int_stat & INTMSK_DCRC) {
    the_task_stat->error_status = ERRDCRC;
  }

  if (raw_int_stat & INTMSK_SBE) {
    the_task_stat->error_status = ERRSTARTBIT;
  }
  return;
}

void emmc_handle_standard_idsts(void * prv_data, u32_t int_status)   //idst:internal DMA status reg
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  u32_t idsts_stat = int_status;
  current_task_status *the_task_stat =  (current_task_status *) prv_data;

    if (idsts_stat & IDMAC_FBE) {
    the_task_stat->error_status = ERRIDMACFBE;   // Fatal Bus Error
          emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_FBE);
    printk("FATAL BUS ERROR in IDMAC mode\n");
  }
    if (idsts_stat & IDMAC_DU) {
          emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_DU);
    printk("Descriptor Unavailable\n");
#ifdef IDMAC_CHAIN_MODE
    emmc_dump_descriptors(CHAINMODE);
#else
    emmc_dump_descriptors(RINGMODE);
#endif
    emmc_dump_registers();
    if(poll_demand_count < 3)    //poll Demand reg is a write only reg ,when OWN bit is not set,the host needs to write any value to resume normal descriptor fetch operation   ly  
    {     
      the_task_stat->error_status = 0;   // Descriptor Unavailable but we are doing a Poll Demand
      the_task_stat->cmd_status = TSK_STATE_POLLD;   // Descriptor Unavailable
      emmc_set_bits(emmc_dev, EMMC_REG_RINTSTS, 0xFFFFFFFF); //Clear all pending Slave Mode Interrupts
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, 0xFFFFFFFF); //Clear all pending IDMAC Mode Interrupts
      printk("error_status is %x and command_status is %x\n",the_task_stat->error_status,the_task_stat->cmd_status);
      printk("Issueing Poll-Demand %x th time\n",poll_demand_count);
      emmc_set_register(emmc_dev, EMMC_REG_PLDMND,0x01); // Since we are getting descriptor Unavailable we are writing a value to 
                 // poll demand Register to release the IDMA from Suspend State.
      poll_demand_count++;
    }
    else{
      the_task_stat->error_status = ERRIDMACDU;   // Descriptor Unavailable
      the_task_stat->cmd_status   = TSK_STAT_STARTED;   // Descriptor Unavailable

      //Reset the CTRL[0]
            emmc_set_bits(emmc_dev, EMMC_REG_CTRL,CTRL_RESET);
            plat_delay(100);
      if((emmc_read_register(emmc_dev, EMMC_REG_CTRL) & CTRL_RESET) != 0){
        printk("ERROR in resetting Controller\n");
      }
        //Reset the CTRL[2]
            emmc_set_bits(emmc_dev, EMMC_REG_CTRL,DMA_RESET);
            plat_delay(100);
      if((emmc_read_register(emmc_dev, EMMC_REG_CTRL) & DMA_RESET ) != 0){
        printk("ERROR in resetting DMA\n");
      }
      //Reset the FIFO controller CTRL[1]
            emmc_set_bits(emmc_dev, EMMC_REG_CTRL,FIFO_RESET);
            plat_delay(100);
      if((emmc_read_register(emmc_dev, EMMC_REG_CTRL) & FIFO_RESET) != 0){
        printk("ERROR in resetting FIFO_RESET\n");
      }
      //Reset the DMA engine BMOD[0]
            emmc_set_bits(emmc_dev, EMMC_REG_BMOD,BMOD_SWR);
          plat_delay(100);
      if((emmc_read_register(emmc_dev, EMMC_REG_BMOD) & BMOD_SWR) != 0){
        printk("ERROR in resetting BMOD\n");
      }
      emmc_set_bits(emmc_dev, EMMC_REG_RINTSTS, 0xFFFFFFFF); //Clear all pending Slave Mode Interrupts
      emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, 0xFFFFFFFF); //Clear all pending IDMAC Mode Interrupts
  
    }
  }
    if (idsts_stat & IDMAC_CES) {                 // Card Error Summary 
    the_task_stat->error_status = ERRIDMACCBE;   // CBE set 
          emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_CES);
    printk("Card Error Summary is Set\n");
  }
    if (idsts_stat & IDMAC_AI) {                   // Abnormal Interrupt
          emmc_set_bits(emmc_dev, EMMC_REG_IDSTS, IDMAC_AI);
    printk("Abnormal Interrupt received\n");
  }
}

/**
  * Checks a R1 response.
  *
  * @param[in] the_response The response which is to be checked.
  *
  * \return The error status if an error is found in the response. Else 0.
  */

u32_t emmc_check_r1_resp(u32_t the_response)
{
  u32_t retval = 0;
  if (the_response & R1CS_ERROR_OCCURED_MAP) {
    if (the_response & R1CS_ADDRESS_OUT_OF_RANGE) {
      retval = ERRADDRESSRANGE;
    } else if (the_response & R1CS_ADDRESS_MISALIGN) {
      retval = ERRADDRESSMISALIGN;
    } else if (the_response & R1CS_BLOCK_LEN_ERR) {
      retval = ERRBLOCKLEN;
    } else if (the_response & R1CS_ERASE_SEQ_ERR) {
      retval = ERRERASESEQERR;
    } else if (the_response & R1CS_ERASE_PARAM) {
      retval = ERRERASEPARAM;
    } else if (the_response & R1CS_WP_VIOLATION) {
      retval = ERRPROT;
    } else if (the_response & R1CS_CARD_IS_LOCKED) {
      retval = ERRCARDLOCKED;
    } else if (the_response & R1CS_LCK_UNLCK_FAILED) {
      retval = ERRCARDLOCKED;
    } else if (the_response & R1CS_COM_CRC_ERROR) {
      retval = ERRCRC;
    } else if (the_response & R1CS_ILLEGAL_COMMAND) {
      retval = ERRILLEGALCOMMAND;
    } else if (the_response & R1CS_CARD_ECC_FAILED) {
      retval = ERRECCFAILED;
    } else if (the_response & R1CS_CC_ERROR) {
      retval = ERRCCERR;   //25
    } else if (the_response & R1CS_ERROR) {
      retval = ERRUNKNOWN;
    } else if (the_response & R1CS_UNDERRUN) {
      retval = ERRUNDERRUN;
    } else if (the_response & R1CS_OVERRUN) {
      retval = ERROVERRUN;
    } else if (the_response & R1CS_CSD_OVERWRITE) {
      retval = ERRCSDOVERWRITE;
    } else if (the_response & R1CS_WP_ERASE_SKIP) {
      retval = ERRPROT;
    } else if (the_response & R1CS_ERASE_RESET) {
      retval = ERRERASERESET;
    } else if (the_response & R1CS_SWITCH_ERROR) {
      retval = ERRFSMSTATE;
    }
  }
#ifdef DEBUG
  if (retval) {
    printk("0x%08x:%x\n", the_response, retval);
  }
#endif
  return retval;
}

u32_t emmc_check_r5_resp(u32_t the_resp)
{
  u32_t retval = 0;

  if (the_resp & R5_IO_ERR_BITS) {
    if (the_resp & R5_IO_CRC_ERR) {
      retval = ERRDCRC;
    } else if (the_resp & R5_IO_BAD_CMD) {
      retval = ERRILLEGALCOMMAND;
    } else if (the_resp & R5_IO_GEN_ERR) {
      retval = ERRUNKNOWN;
    } else if (the_resp & R5_IO_FUNC_ERR) {
      retval = ERRBADFUNC;
    } else if (the_resp & R5_IO_OUT_RANGE) {
      retval = ERRADDRESSRANGE;
    }
  }
  return retval;
}

/**
  * This function dumps descriptr fields of all the descriptors in the list/chain.
  * This function is intended for debugging purpose.
  * @param[in] The mode the descriptors are organized: RING or CHAIN
  * \return returns void.
  */

void emmc_dump_descriptors(u32_t Mode)  
{
  u32_t Dcount=0;
  DmaDesc *desc = current_task.desc_head; 
  if(Mode == RINGMODE){
    while (Dcount != NO_OF_DESCCRIPTORS){
      printk("Desc_No: %02d \t desc0: %08x  desc1: %08x desc2: %08x desc3: %08x desc2_virt: %08x desc3_virt: %08x\n",                      Dcount,desc->desc0,desc->desc1,desc->desc2,desc->desc3,desc->desc2_virt,desc->desc3_virt);
      Dcount++;
      desc=desc+1;
    }
  }
  else{
    while (Dcount != NO_OF_DESCCRIPTORS){
      printk("Desc_No: %02d \t desc0: %08x desc1: %08x desc2: %08x desc3: %08x desc2_virt: %08x desc3_virt: %08x \n",                       Dcount,desc->desc0,desc->desc1,desc->desc2,desc->desc3,desc->desc2_virt,desc->desc3_virt);
      desc=(DmaDesc *)desc->desc3_virt;
      if(desc == current_task.desc_head)
        break;
      Dcount++;
    }
  }

  printk(" Next Descriptor points to %x \t desc_next = %08x \n",current_task.next,(u32_t)current_task.desc_next);
  printk(" Busy Descriptor points to %x \t desc_busy = %08x \n",current_task.busy,(u32_t)current_task.desc_busy);
  return;
}

/**
  * Get back the descriptor from DMA after data has been received/transmitted.
  * When the DMA indicates that the data is received (interrupt is generated), this function should be
  * called to get the descriptor and hence the data buffers. This function unmaps the buffer pointers
  * which was mapped for the DMA operation. With successful return from this
  * function caller gets the descriptor fields for processing.
  * @param[out] pointer to hold the status of DMA.
  * @param[out] Dma-able buffer1 pointer.
  * @param[out] pointer to hold length of buffer1 (Max is 8192).
  * @param[out] virtual pointer for buffer1.
  * @param[out] Dma-able buffer2 pointer.
  * @param[out] pointer to hold length of buffer2 (Max is 8192).
  * @param[out] virtual pointer for buffer2.
  * \return returns present rx descriptor index on success. Returns Negative value if error.
  */
s32_t emmc_get_qptr( u32_t * Status, u32_t * Buffer1, 
        u32_t * Buffer1_Virt, u32_t * Buffer2, u32_t * Buffer2_Virt)
{
    
    u32_t  busy             = current_task.busy;      // Index of Descriptor, DMA owned from the last processing
  DmaDesc *busy_desc    = current_task.desc_busy; // the descriptor to be handled
  
  if(emmc_is_desc_owned_by_dma(busy_desc))    // If Descriptor is owned by DMA, we have nothing to process
    return -1;
  
  if(Status != 0)
    *Status = busy_desc->desc0;                  // Desc0 contains the control and Status fielsds

  if(Buffer1 != 0)
    *Buffer1 =busy_desc->desc2;
  if(Buffer1_Virt != 0)
    *Buffer1_Virt = busy_desc->desc2_virt;

  if(Buffer2 != 0)
    *Buffer2 = busy_desc->desc3;
  if(Buffer2_Virt != 0)
    *Buffer2_Virt = busy_desc->desc3_virt;
                  
  current_task.busy = emmc_is_last_desc(busy_desc) ? 0 : busy + 1; 
  if(emmc_is_desc_chained(busy_desc)){
      current_task.desc_busy = (DmaDesc *)busy_desc->desc3_virt;
    if(*Buffer1_Virt != 0){
      plat_unmap_single((struct pci_dev *)current_task.bus_device, *Buffer1, 0,BIDIRECTIONAL);
      printk("(Chain mode) buffer1 %08x is given back\n",*Buffer1_Virt);
    }
  }
  else{
    current_task.desc_busy = emmc_is_last_desc(busy_desc) ? 
                         current_task.desc_head : (busy_desc + 1);
    if(*Buffer1_Virt != 0){
      plat_unmap_single((struct pci_dev *)current_task.bus_device, *Buffer1, 0,BIDIRECTIONAL);
      printk("(Ring mode) buffer1 %08x is given back\n",*Buffer1_Virt);
    }
    if(*Buffer2_Virt != 0){
      plat_unmap_single((struct pci_dev *)current_task.bus_device, *Buffer2, 0,BIDIRECTIONAL);
      printk("(Ring mode) buffer2 %08x is given back\n",*Buffer2_Virt);
    }
  }
  printk("%02d %08x %08x %08x %08x %08x %08x\n",busy,(u32_t)busy_desc,busy_desc->desc0,
          busy_desc->desc2,busy_desc->desc3,busy_desc->desc2_virt,busy_desc->desc3_virt);
  if(current_task.busy != current_task.next){
    return -1;
  }
  return(busy);
}

/**
  * Read in data from the FIFO in interrupt context.
  * This function reads in the bytes from the fifo in interrupt context. This 
  * function is called for a RXDR interrupt or a transfer complete interrupt.
  * For the RXDR interrupt, the RX_watermark/2 + 1 dwords are read from the   //RXDR: Receive FIFO data request
  * FIFO. If there is a data transfer over interrupt, we know that all remaining 
data  * is languishing in the fifo. Hence we check the status register and find 
the exact  * amount of data to read and read the same. If there is any unalighen 
bytes that may  * be needed to be read, it is also taken care of during reading 
the last double word  * from the FIFO.
  *
  * @param[in,out] the_task_status The status of the data transfer task.
  * @param[in] the_interrupt_status the flags for which particular interrupts 
have occured.  * \return Returns 0 for successful copying. Returns the error 
code upon errors.  * \todo Employ the <i>the_copy_function</i> in the task 
status for a custom data  */
u32_t emmc_read_in_data(current_task_status * the_task_status,
        u32_t the_interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  u32_t buffer_reg;
  u32_t u32s_to_read, u32s_left = 0;
        u32_t  u32s_to_read_temp;
  int task_bytes_left, last_word_check = 0, count;
  // u32 aligned_address; 
  u32_t the_block_size = the_task_status->blksize;
        u32_t data_read;
  u8_t *u8_buffer;
    
    printk("Num_of_blocks = %x blk_size = %x bytes_read = %x\n",the_task_status->num_of_blocks,the_block_size,the_task_status->num_bytes_read);

    task_bytes_left =   the_task_status->num_of_blocks * the_block_size - the_task_status->num_bytes_read;
  printk("task_bytes_left = %x\n", task_bytes_left);

  buffer_reg = L_EMMC_REG_STATUS; //Read the FIFO status register
    printk("STATUS = 0x%08x\n", buffer_reg);

  if (the_interrupt_status & INTMSK_DAT_OVER) {//IF DATA TRANSMISSION is complete
    buffer_reg = L_EMMC_REG_STATUS; //read register status
        //if FIFO is FUll, then the no of words to read out of FIFO is FIFO DEPTH number of words from FIFO
        //else Read Only the number of words available in the FIFO
    if (buffer_reg & STATUS_FIFO_FULL) {            
      u32s_to_read = the_ip_status.fifo_depth;
    } else {
      u32s_left = (GET_FIFO_COUNT(buffer_reg));
      u32s_to_read = u32s_left;
            // If the number of bytes to be read out of FIFO is not a multiple of Wods last_word_check holds the
            // byte left to read from last word
      if (u32s_to_read * FIFO_WIDTH > task_bytes_left) {
        u32s_to_read--;//Need to read one word less of our initial computation  why   ly 
        last_word_check = task_bytes_left % FIFO_WIDTH;
      }
    }
  } 
    else {  //NOW RXDR is set but Transfer is not complete........
            // u32s_left gives the number of words to be read to accomplish successful transfer
        if (the_task_status->num_of_blocks) {
          u32s_left = task_bytes_left / FIFO_WIDTH;
    } 
            else{
            u32s_left = the_ip_status.fifo_threshold + 1;
    }

        if(u32s_left < (the_ip_status.fifo_threshold)) {
      u32s_to_read = u32s_left;
    }
          else{
      u32s_to_read = the_ip_status.fifo_threshold + 1;
    }
        printk("Got u32s to read as = %x\n", u32s_to_read);
      }

  // Check if the interrupt status says RX ready 
  if ((the_interrupt_status & INTMSK_RXDR) || (the_interrupt_status & INTMSK_DAT_OVER)) {
        printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
        printk("dwords read = %x dwords to read = %x dwords left = %x\n",(the_task_status->num_bytes_read / FIFO_WIDTH),u32s_to_read, u32s_left);
        u8_buffer = (the_task_status->data_buffer +  the_task_status->num_bytes_read);
           u32s_to_read_temp = u32s_to_read; 
            for (count = 0; count < u32s_to_read_temp; count++) {
#if 0
          aligned_address = emmc_read_register(FIFODAT);
          if (task_bytes_left) {
            memcpy((void *) (u8_buffer + count * FIFO_WIDTH),(void *) &aligned_address,FIFO_WIDTH);
          }
#endif
          if (task_bytes_left) {
          //  printf("status is %x \n",L_EMMC_REG_STATUS);
                    if (GET_FIFO_COUNT( L_EMMC_REG_STATUS))    
                        {
                            data_read = emmc_read_register(emmc_dev, EMMC_REG_FIFODAT);
                          //  printf("data_read is %x \n",data_read);

                *((u32_t *) (u8_buffer +  count * 4)) = data_read;
                            //  emmc_read_register(FIFODAT);
                         }
                    else{
                            u32s_to_read--; //This was not read. So decrement u32s_to_read.
                            printk("NO more Read Possible as FIFO_COUNT is zero\n");
                    }
                        
          }
        }

//memcpy函数的功能是从源src所指的内存地址的起始位置开始拷贝n个字节到目标dest所指的内存地址的起始位置中。
//void *memcpy(void *dest, const void *src, size_t n);  
      
            if (last_word_check && task_bytes_left) {
      // There is a dangling few bytes to be read in 
                printk("Few dangling bytes to be read!!!!\n");
          buffer_reg = emmc_read_register(emmc_dev, EMMC_REG_FIFODAT);
          memcpy((void *) (the_task_status-> data_buffer + (u32s_to_read * FIFO_WIDTH)),(void *) (&buffer_reg), last_word_check);

        }
        
            if (task_bytes_left) {
          the_task_status->num_bytes_read +=   u32s_to_read * 4 + last_word_check;
        }
        
            printk("The number of read bytes = %x\n",the_task_status->num_bytes_read);

        // If there is a termination function registered,
        //   call it after every transfer. Under this condition
        //   we can stop open ended transfers
     
        if (the_task_status->the_term_function) {
          if (the_task_status->the_term_function(the_task_status->data_buffer, the_task_status->num_bytes_read)) {
            the_task_status->aborted_command = 1;
            emmc_abort_trans_work(emmc_dev, the_task_status-> slot_num);
          }
        }
          
    }
    else {
        the_task_status->error_status = ERRUNKNOWN;
        return ERRUNKNOWN;
    }

  if ((the_interrupt_status & INTMSK_DAT_OVER) && (the_task_status->num_bytes_read != (the_task_status->num_of_blocks * the_block_size))
      && (CEATA_TYPE != the_card_info[the_task_status->slot_num].card_type)) {
        emmc_abort_trans_work(emmc_dev, the_task_status->slot_num);
        return ERRHARDWARE;
    }
  return 0;
}

/**
  * Function to write out data to the data FIFO.
  * This function is called from interrupt context to write out the data into the 
  * FIFO when the TXDR  interrupt is asserted. This function writes out fifo   //TXDR: Transmit FIFO data request
  * threshold / 2 dwords into the fifo. It also checks whether the term function
  * signals a end of transfer signal. This is used in open ended transfers.
  * @param[in,out] the_task_status The status of the current transfer in progress.
  * @param[in] the_interrupt_status The value of the interrupt flags.
  * \todo Employ the copy function in the task status structures for copies.
  */
u32_t emmc_write_out_data(current_task_status * the_task_status,
          u32_t the_interrupt_status)
{
  struct device *emmc_dev;
  emmc_dev = device_get_binding("emmc_0");
  u32_t pending_dwords, dangling_dwords = 0, dwords_to_write;
  u32_t buffer = 0;
  u8_t *src_buffer;
  int count;
  /* u32 aligned_address; */
  u32_t the_block_size = the_task_status->blksize;


  /* For an open ended transfer, the number of blocks will be
     set to 0. 
   */
  if (the_task_status->num_of_blocks) {
    pending_dwords =
        (the_block_size
         * the_task_status->num_of_blocks -
         the_task_status->num_bytes_read) / FIFO_WIDTH;


    dangling_dwords =
        (the_block_size
         * the_task_status->num_of_blocks -
         the_task_status->num_bytes_read) % FIFO_WIDTH;
  } else {
    pending_dwords =
        the_ip_status.fifo_depth -
        the_ip_status.fifo_threshold;
  }


  if (!(the_interrupt_status & INTMSK_TXDR)) {
    return ERRUNKNOWN;
  }


  if (pending_dwords >=
      (the_ip_status.fifo_depth - the_ip_status.fifo_threshold)) {

    dwords_to_write =
        (the_ip_status.fifo_depth -
         the_ip_status.fifo_threshold);

    dangling_dwords = 0;
  } else {

    dwords_to_write = pending_dwords;
  }

  src_buffer =
      (the_task_status->data_buffer +
       the_task_status->num_bytes_read);

  for (count = 0; count < dwords_to_write; count++) {
#if 0
    if (pending_dwords) {
      memcpy((void *) &aligned_address,
             (void *) (src_buffer + count * FIFO_WIDTH),
             FIFO_WIDTH);
    }
    emmc_set_register(FIFODAT, aligned_address);
#endif
    emmc_set_register(emmc_dev, EMMC_REG_FIFODAT,*((u32_t *) (src_buffer + count * FIFO_WIDTH)));
  }

  if (dangling_dwords) {
    buffer = 0;
    memcpy((void *) &buffer, (void *) src_buffer,
           dangling_dwords);
    emmc_set_register(emmc_dev, EMMC_REG_FIFODAT, buffer);
  }


  printk("Number of bytes written out is %x (TBB = %x TCB = %x)\n",
       the_task_status->num_bytes_read,
       emmc_read_register(emmc_dev, EMMC_REG_TBBCNT),
       emmc_read_register(emmc_dev, EMMC_REG_TCBCNT));

  if (pending_dwords) {
    the_task_status->num_bytes_read +=
        (dangling_dwords + dwords_to_write * FIFO_WIDTH);
  }

  printk("Number of bytes written out is %x (TBB = %x TCB = %x)\n",
       the_task_status->num_bytes_read,
       emmc_read_register(emmc_dev, EMMC_REG_TBBCNT),
       emmc_read_register(emmc_dev, EMMC_REG_TCBCNT));


  /* Now call the callback to check if we want to stop the transfer
   * of the bytes.
   */
  if (the_task_status->the_term_function) {
    if ((the_task_status->
         the_term_function(the_task_status->
               data_buffer,
               the_task_status->num_bytes_read)) &&
        (the_card_info[the_task_status->slot_num].card_type !=
         CEATA_TYPE)) {
      emmc_abort_trans_work(emmc_dev, the_task_status->
              slot_num);
    }
  }


  return 0;
}

u32_t emmc_io_rw(struct device *dev, u32_t slot, u32_t func, u32_t address, u8_t * data,
       u32_t length, u32_t read_or_write, u32_t op_flags,
       emmc_term_function the_term_function,
       emmc_copy_function the_copy_function)
{
  u32_t cmd53_arg = 0;
  u32_t retval = 0;
  u32_t resp_buff[4];
  u32_t cmd_to_send;

  if (emmc_read_register(dev, EMMC_REG_CDETECT) & (1 << slot)) {
    retval = ERRCARDNOTCONN;
    goto HOUSEKEEP;
  }


  if (func > 7) {
    retval = ERRADDRESSMISALIGN;
    goto HOUSEKEEP;
  }

  if (read_or_write) {
    SET_BITS(cmd53_arg, IO_RW_RW_FLG_MSK);
  }

  SET_FUNC_NUM(cmd53_arg, func);

  if (op_flags & CMD53_USE_BLOCK_IO) {
    u32_t length_to_set;
    length_to_set = (length / CMD53_GET_BLKSIZ(op_flags));
    SET_BITS(cmd53_arg, IO_RW53_BMODE_MSK);
    SET_BITS(cmd53_arg, length_to_set);
  } else {
    UNSET_BITS(cmd53_arg, IO_RW53_BMODE_MSK);
    SET_BITS(cmd53_arg, length);
  }

  /* If we are accesing a fifo inside the card
   */
  if (op_flags & CMD53_FIFO_WRITE_FLAG) {
    UNSET_BITS(cmd53_arg, IO_RW53_OP_CODE_MSK);
  } else {
    SET_BITS(cmd53_arg, IO_RW53_OP_CODE_MSK);
  }

  if (address > IO_RW_REG_MAX_VAL) {
    retval = ERRADDRESSMISALIGN;
    goto HOUSEKEEP;
  }

  SET_IO_RW_REG_ADD(cmd53_arg, address);

  if (length > 0x1ff) {
    retval = ERRADDRESSMISALIGN;
    goto HOUSEKEEP;
  }

  cmd_to_send = (read_or_write) ? WCMD53 : CMD53;

  retval =
      emmc_read_write_bytes(dev, slot, resp_buff, data, 0,
              length, cmd53_arg,
              the_copy_function,
              the_term_function,
              read_or_write,
              cmd_to_send | CUSTCOM_DONT_CMD16 |
              CUSTCOM_DONTSTDBY |
              CUSTCOM_STREAM_RW |
              CUSTCOM_DONT_TRANS, NULL, NULL);
      HOUSEKEEP:
  return retval;
}

/**
  * Determine the card type in the slot.
  * This function determines emmc_set_sd_wide_busthe card type in the slot. The steps for doing so
  * are as follows:
  * -# Send a CMD5 to the slot. If a response is received, it is a SDIO card
  * -# Send ACMD41 + CMD55 combo to the slot. If a response is received, 
  * it is a SD card.
  * -# Get it to TRANS state and fire a CMD60. If a reply is got, it is a
  * CE-ATA
  * device
  * -# If not, it is a MMC card.
  *
  * @param[in] slot The index of the slot in which the card is in.
  * \return Returns the card type found.
  */
Card_Type emmc_get_card_type(struct device *dev, u32_t slot)
{
  u32_t buffer_reg, retval;
  u32_t resp_buffer[4];
  
  /*Read the CDETECT bit 0 => card connected. Note this is not true for CEATA so you find a hack in the emmc_read_register() */
  buffer_reg = emmc_read_register(dev, EMMC_REG_CDETECT);
  if ((buffer_reg & (1 << slot))) {    // 0 represent presence of card
    return NONE_TYPE;
  }
  /*
  Clear the CTYPE register bit for of IP. This bit indicates whether the card connected is 8/4/1 bit card
  */
  emmc_clear_bits(dev, EMMC_REG_CTYPE, (1 << slot));
  
  /*Send the CMD5 to see whether the card is SDIO type. If success declare a SDIO*/
  
  printk("Sending CMD5 to slot %x\n", slot);
  retval = emmc_send_serial_command(dev, slot, CMD5, 0, NULL, NULL, 0, NULL, NULL);
   
  if (!retval) {
    /* Found an SDIO card */
    printk("2 SDIO_TYPE CArd in the slot \n" );
    return SDIO_TYPE;
  }

  printk("retval = %x\n", retval);//25
  if (retval != ERRRESPTIMEOUT) {
    //return ERRTYPE;
    printk("3 ERRTYPE CArd in the slot \n" );

  }
  else {
    printk("CMD5 has timed out...\n");
  }
   
  /* If we have reached here the card is not SDIO type => so it is MEM type
     Issue cmd0 before proceeding to detect SDmem as per SPEC. 
         */
    printk("Sending SD_CMD0 to slot %x\n", slot);


  if ((retval = emmc_send_serial_command(dev, slot, CMD0, 0, NULL, NULL, 0,  NULL, NULL))) {
    printk("  After send cmd0 %d\n", retval);
  }
  

  /*
   Before Sending ACMD41 send CMD8 SEND_IF_COND to the card to see if it is an SD2.0 type
   Card checks the validity of operating condition by analyzing the argument of CMD8
   Host checks the validity by analyzing the response of CMD8. Note that argumnet passed is
     VHS=1 => 2.7 to 3.3 volts and Check pattern = 0xAA 
  */  
  printk("Sending CMD8 to slot %x\n", slot);
  retval = emmc_send_serial_command(dev, slot, CMD8, 0x000001AA, resp_buffer, NULL, 0,  NULL, NULL);

  if (!retval) {
     /* Found an SD_2.0 Card */
    printk("CMD8 received the response:%x\n",resp_buffer[0]);
    return SD_MEM_2_0_TYPE;
  }
  printk("retval = %x\n", retval);
  if (retval != ERRRESPTIMEOUT) {
    return ERRTYPE;
  }
  else {
    printk("CMD8 has timed out !!!!!!\n");
  }

  /*
  Since CMD8 has timed out it is not SD_MEM_2_0_TYPE..... Continue your card detection procedure..
  */

  /* Lets Issue ACMD41 to see whether it is SDMEM. CMD55 should preced andy ACMD command
     If nonzero response to CMD55 => the card is not an SD type so move to detect whether it is MMC?    
  */
  
  printk("Sending CMD55 to slot %x\n", slot);
  if ((retval = emmc_send_serial_command(dev, slot, CMD55, 0, NULL, NULL, 0, NULL, NULL))) {
    printk("CMD55  failied!!!!!!!\n");
  }
   
  /*
  CMD55 is successful, so send ACMD41 to get the OCR from SD card. If success Declare a SD card
  */
  printk("Sending ACMD41 to slot %x\n", slot);
  retval =  emmc_send_serial_command(dev, slot, ACMD41, 0, NULL, NULL, 0, NULL, NULL);
  if (!retval) {
    /* Found an SD card */
    return SD_TYPE;
  }
  if (retval != ERRRESPTIMEOUT) {
    return ERRTYPE;
  }
  else {
    printk("ACMD41 has timed out...\n");
  }
}

u32_t emmc_reset_sd_2_0_card(struct device *dev, u32_t slot)
{

    u32_t buffer_reg;
    u32_t retval = 0;
    u32_t clock_freq_to_set = SD_ONE_BIT_BUS_FREQ;
    u32_t resp_buffer[4];

    /* Check if the card is connected? */
    buffer_reg = emmc_read_register(dev, EMMC_REG_CDETECT);
    if (buffer_reg & (1 << slot)) {
      return ERRCARDNOTCONN;
    }
    
    /* Clear out the state information for the card status. We know the card is SD_2_0_TYPE*/
    memset((void *) (the_card_info + slot), 0, sizeof(Card_info));    //Q1:Should alloc inner space?? ly
    the_card_info[slot].card_type = SD_MEM_2_0_TYPE;


    /* Fod the clock and OD the bussince we start enumerating now */
    the_card_info[slot].divider_val = 15;
    /* Reset the card. Since we really dont know as to from where the call has been made*/
    if ((retval = emmc_send_serial_command(dev, slot, CMD0, 0, NULL, NULL, 0, NULL, NULL))) {
      goto HOUSEKEEP;                  //CMD0 : GO_IDLE_STATE
    }
    printk("Sending SD_CMD8 to slot %x\n", slot);
    retval = emmc_send_serial_command(dev, slot, SD_CMD8, 0x000001AA, resp_buffer, NULL, 0,  NULL, NULL);
    
    if (!retval) {
      /* Found an SD_2.0 Card */
      printk("SD_CMD8 received the response:%x\n",resp_buffer[0]);
    }
    else {
      printk("SD_CMD8 has timed out...\n");
      return ERRTYPE;
    }

    /* CMD8 is successful => card is in idle state */
    the_card_info[slot].card_state = CARD_STATE_IDLE;
    /* Now in IDLE state. Set the SD2.0  voltage range on the card. Uses CMD55 and ACMD41 */
    if ((retval = emmc_set_sd_2_0_voltage_range(dev ,slot))) {
      goto HOUSEKEEP;
    }
    /* Now in READY state Now extract the CID this uses CMD2. SD2.0 alos uses same cid command */
    if ((retval = emmc_get_cid(dev, slot))) {
      goto HOUSEKEEP;
    }

    /* Now in IDENT state. Set the RCA. this uses CMD3. SD2.0 also used same rca setting command*/
    if ((retval = emmc_set_sd_rca(dev, slot))) {
      goto HOUSEKEEP;
    }
    /*Clk freq is set to one bit freq. */
    the_card_info[slot].divider_val = clock_freq_to_set;

    /*Now in STBY stage. So we get the CSD Register and store it. this uses CMD9 */
    #if 1
    if ((retval = emmc_process_SD_2_0_csd(dev, slot))) {
      /* Switch off the card */
      goto HOUSEKEEP;
    }
    printk("1Card capacity = %x Kbytes\n", the_card_info[slot].card_size);
    #endif

    /* If compiled with SD_SET_WIDE_BUS flag then do the needful*/
      printk("-> Get SCR \n");
    if ((retval = emmc_process_scr(dev, slot))) {
      /* Switch off the card */
      goto HOUSEKEEP;
    }

    printk("-> Set sd wide bus \n");
    printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
    if ((retval = emmc_set_sd_wide_bus(dev, slot, 4))) {
      retval = 0;
      goto HOUSEKEEP;
    }

    // If DDR operation is required, check for DDR support from the card.
    // If support exist program the host controller to operate in DDR mode.
    printk("3Card capacity = %x Kbytes\n", the_card_info[slot].card_size);
    HOUSEKEEP:
    the_card_info[slot].enum_status = retval;
    return retval;
}

u32_t emmc_process_SD_2_0_csd(struct device *dev, u32_t slot)
{

  u32_t buffer_reg, retval;
  u32_t read_block_size, write_block_size;
  u32_t card_size;
  u32_t blocknr, blocklen;
  printk("muweilin\n");
  buffer_reg = emmc_read_register(dev, EMMC_REG_CDETECT);
  if (buffer_reg & (1 << slot)) {
    return ERRCARDNOTCONN;
  }


  if (CARD_STATE_STBY != the_card_info[slot].card_state) {
    printk("This is in the wrong FSM state for CSD fetch %x\n",the_card_info[slot].card_state);
    return ERRFSMSTATE;
  }

  if ((retval = emmc_send_serial_command(dev, slot, CMD9, 0, the_card_info[slot].the_csd, NULL, 0, NULL, NULL))) {
    return retval;
  }
  /* The CSD is in the bag */

  /* Store the largest TAAC and NSAC. We will use these for calculating the data timeout latency. */
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  printk("**************CSD of SD2.0**********************\n");
  printk("0x%08x 0x%08x 0x%08x 0x%08x\n",
         the_card_info[slot].the_csd[0],
         the_card_info[slot].the_csd[1],
         the_card_info[slot].the_csd[2],
         the_card_info[slot].the_csd[3]);
  printk("**************CSD of SD2.0**********************\n");


  if (CSD_TAAC(the_card_info[slot].the_csd) != 0x0E)
  return ERRHARDWARE;   
  else {
  printk("This is SD2.0 CARD since TAAC is %x\n",CSD_TAAC(the_card_info[slot].the_csd));  
  }

  if (CSD_NSAC(the_card_info[slot].the_csd) != 0x00)
  return ERRHARDWARE;   
  else {
  printk("This is SD2.0 CARD since NSAC is %x\n",CSD_NSAC(the_card_info[slot].the_csd));  
  }
  
  if(CSD_READ_BL_LEN(the_card_info[slot].the_csd) != 0x09)
  return ERRHARDWARE;   
  else {
  printk("For SD2.0  READ_BL_LEN is 512\n");
  read_block_size  = 1 << (CSD_READ_BL_LEN((the_card_info[slot].the_csd)));
  }
  
  if(CSD_WRT_BL_LEN(the_card_info[slot].the_csd) != 0x09)
  return ERRHARDWARE;   
  else {
  printk("For SD2.0  WRITE_BL_LEN is 512\n");
  write_block_size = 1 << (CSD_WRT_BL_LEN((the_card_info[slot].the_csd)));
  }

  /* go through section 5.3.3 of Physical layer specifications of  SD2.0
    to compute the cared capacity 
   */
  blocknr = ((CSD_C_SIZE_SD_2_0(the_card_info[slot].the_csd) & 0x3fffff) + 1);
  blocklen = read_block_size;
  card_size = blocknr * blocklen;
  
  /* read/write block size */
  the_card_info[slot].card_write_blksize = 512;
  the_card_info[slot].orig_card_read_blksize = 512;
  the_card_info[slot].card_read_blksize = 512;
  the_card_info[slot].orig_card_write_blksize = 512;
  /* Set the card capacity Note the capacity is in Kilo bytes*/
  the_card_info[slot].card_size = card_size;

  printk("Card capacity = %x Kbytes\n", the_card_info[slot].card_size);
  printk("Read Block size = %x bytes\n",the_card_info[slot].card_read_blksize);
  printk("Write Block size = %xbytes\n",the_card_info[slot].card_write_blksize);
  return 0;
}

u32_t emmc_process_scr(struct device *dev, u32_t slot)
{
  u32_t retval;
  u32_t resp_buffer[4];
  u32_t buff;

  u8_t *the_data_buffer;

  the_data_buffer = the_card_info[slot].the_scr_bytes;
  the_card_info[slot].the_scr[0] = 0;
  the_card_info[slot].the_scr[1] = 0;
  printk("***************Before reading SCR************\n");
  printk("0x%08x 0x%08x\n", the_card_info[slot].the_scr[0],the_card_info[slot].the_scr[1]);
  
  if ((retval =  emmc_read_write_bytes(dev, slot, resp_buffer,
               the_data_buffer, 0, 64 / 8, 0,
               NULL, NULL, 0,
               ACMD51 | 4 <<
               CUSTOM_BLKSIZE_SHIFT |
               CUSTCOM_DONTSTDBY, NULL, NULL))) {
    return retval;
  }

  printk("***************SCR************\n");
  printk("0x%08x 0x%08x\n", the_card_info[slot].the_scr[0],the_card_info[slot].the_scr[1]);
  printk("***************SCR************\n");
  return 0;
}

u32_t emmc_read_write_bytes(struct device *dev, u32_t slot, u32_t * resp_buffer,
            u8_t * data_buffer, u32_t start, u32_t end,
            u32_t argreg,
            emmc_copy_function the_copy_func,
            emmc_term_function
            the_term_function, u32_t read_or_write,
            u32_t custom_command,
            emmc_preproc_callback custom_pre,
            emmc_postproc_callback custom_post)
{
  u32_t retval = 0, retval2 = 0;
  u32_t num_of_blocks, command_to_send, num_of_primary_dwords = 0, the_block_size;
  u32_t *src_buffer;
  int count;
  Card_state the_state;
  u32_t arg_to_send;
        s32_t desc_available_status;
        u32_t dma_len1;
        dma_addr_t dma_addr1;
  printk("custom_command = 0x%08x\n", custom_command);
  /* Check if the card is inserted */
  if (emmc_read_register(dev, EMMC_REG_CDETECT) & (1 << slot)) {
    return ERRCARDNOTCONN;
  }


  /* Set the block size pertinent to the type of operation
   */
 
  if (CUSTCOM_STREAM_RW & custom_command) {  //CUSTCOM_STREAM_RW : the transfer is a stream transfer
    if (the_card_info[slot].card_type != SDIO_TYPE) {
      if ((start & 0x3) || (end & 0x3)) {//Check if the start and end addresses are aligned. 
        return ERRADDRESSMISALIGN;
      }
    }
    the_block_size = end - start;
  } else if (CUSTOM_BLKSIZE(custom_command)) {
    the_block_size =
        1 << ((CUSTOM_BLKSIZE(custom_command) - 1));
        printk("CUSTOM :the the_block_size %d\n",the_block_size);
  } else if (read_or_write) {
    the_block_size = the_card_info[slot].card_write_blksize;
    printk("write : the the_block_size %d\n",the_block_size);
  } else {
    the_block_size = the_card_info[slot].card_read_blksize;
    printk("read: the the_block_size %d\n",the_block_size);
  }



  if (!the_block_size) {
    retval = ERRADDRESSRANGE;
    goto HOUSEKEEP;
  }
if(CMD42 == (custom_command & 0x3f)){/*This hack is required as CMD42 block size is not equal to write_block_size*/
        the_block_size = CUSTOM_BLKSIZE(custom_command);
        printk("the block size for CMD42 is %x\n",the_block_size);
}

  printk("start = %x, end = %x, size = %x, block size = %x\n",
       start, end, the_card_info[slot].card_size, the_block_size);

  if (!(custom_command & CUSTCOM_STREAM_RW)) {    //that is necessary??   ly
    if ((start % the_block_size)
        || (end % the_block_size)) {
      printk("1 misalign start and end addr.\n");
      return ERRADDRESSRANGE;
    }
  }


  if (the_card_info[slot].card_type != SDIO_TYPE) {
    if ((end /
         (read_or_write ? the_card_info[slot].
          card_write_blksize : the_card_info[slot].
          card_read_blksize)) >
        the_card_info[slot].card_size) {
      return ERRADDRESSRANGE;
    }
  }

  if (start > end) {
    return ERRADDRESSRANGE;
  }

  num_of_blocks = (end - start) / the_block_size;

  printk("NUM OF BLOCKS = %x\n", num_of_blocks);
  if ((num_of_blocks & 0x0000ffff) != num_of_blocks) {
    return ERRADDRESSRANGE;
  }

  /* One cannot have an open ended transfer with 
     no term function specified.
   */
  if ((!the_term_function) && (start == end)) {   //if start == end and not send cmd12 ,return ERR  ly
    return ERRADDRESSRANGE;
  }

  if (!(CUSTCOM_DONT_TRANS & custom_command)) {
    /* Check if the card is in standby state and put in trans
       state.
     */
    if ((retval = emmc_put_in_trans_state(dev, slot))) {
      printk("%x TRANS STATE FAILED\n", retval);
      goto HOUSEKEEP;
    }
  }
     
     //the_card_info[slot].divider_val = 3;
  /* Set the block len *///since we use a sdhc card, cmd16 is useless.
  if ((!(custom_command & CUSTCOM_DONT_CMD16))
      && (!(custom_command & CUSTCOM_STREAM_RW))) {
    if ((retval =
         emmc_send_serial_command(dev,slot, CMD16,      //CMD16 : SET_BLOCKLEN command   ly
              the_block_size,
              resp_buffer,
              NULL, 0, NULL, NULL))) {
      printk("send cmd16 retval error %d\n",retval );
      goto HOUSEKEEP;
    }
  }

  /* Reset internal FIFO */
  emmc_reset_fifo(dev);

  /* Clear the interrupts */
  emmc_set_bits(dev, EMMC_REG_RINTSTS, 0xfffe);


  if (!(custom_command & CUSTCOM_DONT_BLKSIZ)) {
    emmc_set_register(dev, EMMC_REG_BLKSIZ, the_block_size);
  }

  emmc_set_register(dev, EMMC_REG_BYTCNT, (end - start));

  /* Depending on a read or a write, CMD18/CMD25 will be sent out. 
   * Also, the FIFO will be filled to its brim for the card to 
   * read as soon as it receives the CMD25
   */
  printk("READ OR WRITE = %x\n", read_or_write);
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  if (read_or_write) {                  //write 
    command_to_send = CMD25;
               /*Some of MMC and SD cards won't allow Single block write using CMD25. So use CMD24 when number of blocks is equal to 1*/
                if(num_of_blocks == 0x01)
                        command_to_send = CMD24;    //P90 ly 

    if(the_ip_status.idmac_enabled == 1){
      // Don't have to fill the fifo in IDMA mode  
    }
    else{
      /* We also need to put in the intial data into the 
         FIFO. For the first time we fill up the complete 
         FIFO.  
       */
      src_buffer = (u32_t *) data_buffer;
      if (((end - start) / FIFO_WIDTH) >=
          the_ip_status.fifo_depth) {
        /* Write fifo_depth dwords */
        num_of_primary_dwords = the_ip_status.fifo_depth;
      } 
      else{
        /* Just write complete buffer out */
        num_of_primary_dwords = (end - start) / FIFO_WIDTH;
      }

      if (0 == (end - start)) {
        num_of_primary_dwords = the_ip_status.fifo_depth;
      } 
      else if ((end - start) && (0 == num_of_primary_dwords)) {
        num_of_primary_dwords = 1;
      }
      /* Now copy out the actual bytes */
      for (count = 0; count < num_of_primary_dwords; count++) {
        emmc_set_register(dev, EMMC_REG_FIFODAT, *src_buffer++);
      }
    }
  } 
  else{
    command_to_send = CMD18;
    /*Some of MMC and SD cards won't allow Single block read using CMD18. So use CMD17 when number of blocks is equal to 1*/
                if(num_of_blocks == 0x01)
                        command_to_send = CMD17;
  }

  if (custom_command & CUSTCOM_COMMAND_MSK) {
    command_to_send = custom_command & CUSTCOM_COMMAND_MSK;    //extract the custom command 
  }


  /* The CMD23 is an optional feature which might be used for 
     block sized data transfers. The auto stop bit and the 
     BYTCNT size register however suffices for the host controller.
   */     //use CMD23 to set the block number before firing the data command   ly
  if (custom_command & CUSTCOM_DO_CMD23) {
    /* Do a command 23 to set the num of blocks */
    if ((retval =
         emmc_send_serial_command(dev,slot, CMD23, (end - start)/the_block_size,
              resp_buffer, NULL, 0, NULL, NULL))) {
      goto HOUSEKEEP;
    }
  }

  printk("num_of_primary_dwords = %x\n", num_of_primary_dwords);
  /* Set the transfer parameters */
  emmc_set_data_trans_params(dev, slot, data_buffer,
               num_of_blocks,
               the_term_function,
               the_copy_func,
               num_of_primary_dwords *
               FIFO_WIDTH, read_or_write,
               the_block_size);

  if(the_ip_status.idmac_enabled == 1){

    // Setup the Descriptor for the Read Transfer
    // The platform dependent allocation of DMA-able memory
    dma_len1  = emmc_read_register(dev, EMMC_REG_BYTCNT);// the number of bytes to be transmitted/received is already programmed in BYTCNT
    printk("BYTCNT read is %08x\n",dma_len1);
    if(dma_len1 >MAX_BUFF_SIZE_IDMAC){
        printk("Driver cannot handle the buffer lengths greater than %x\n",MAX_BUFF_SIZE_IDMAC);
        return ERRNOTSUPPORTED; //Driver cannot handle the memory greater than 8k bytes (13 bits in lenght field)     
    }
    dma_addr1  = plat_map_single((current_task.bus_device), data_buffer, dma_len1,BIDIRECTIONAL );
    desc_available_status = emmc_set_qptr(dev, dma_addr1, dma_len1,(u32_t) data_buffer,0,0,0);
    if(desc_available_status < 0){
      printk("No Free Descriptor available\n");
      return ERRNOMEMORY;
    }
  }

  /* Before we kick off the floating of the data on the data lines
     if it is a write function we check whether the card is ready 
     to receive data 
   */
  if (read_or_write && (the_card_info[slot].card_type != SDIO_TYPE)) {
    if ((retval = emmc_is_card_ready_for_data(dev, slot))) {
      goto HOUSEKEEP;
    }
  }

  /* Now kick off the the data read/write command. This command will 
   * will schedule this context back in again after the complete
   * transfer.
   */
  if (argreg) {
    arg_to_send = argreg;
  } else {
    arg_to_send = start;     
  }
  printk("command_to_send is %d\n",command_to_send );
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  if ((retval =
       emmc_send_serial_command(dev, slot, command_to_send,
            arg_to_send, resp_buffer,
            data_buffer, end - start,
            custom_pre, custom_post))) {
    goto HOUSEKEEP;
  }


  /*
     This generally occurs due to clocking issues. When the card 
     might be running in non-highspeed mode, and the clock is being
     driven at frequencies outside its reach, bus error might occur.
   */
  if (emmc_bus_corruption_present()) {
    printk("BUS CORRUPTION HAD OCCURED !!!!!\n");
    if (read_or_write) {
      retval = ERRENDBITERR;
    } 
    else {
      retval = ERRDCRC;
    }
  }


  /* Wait till the busy bit in the status register gets
     de-asserted for data writes
   */

  if (read_or_write) {
    while ((L_EMMC_REG_STATUS) &  STATUS_DAT_BUSY_BIT);
  }

#if 1
/*This is required when we disable the auto_stop_bit in CMD register and send the CMD12 from the driver directly.
  Some of the MMC cards and SD card misbehave (block skip problem) when auto_stop_bit is set in the CMD register.
  So Auto_stop_bit is disabled and the CMD12 (Stop CMD is sent by Host/driver) as a work around for multiblock
  read and write operation.
*/
if((command_to_send == CMD25) || (command_to_send == CMD18))
{
        retval = emmc_send_serial_command(dev, slot, CMD12, 0,resp_buffer,NULL,0,NULL,NULL);
}
#endif


     HOUSEKEEP:
  /* This value will have to be changed with the 
     processor/bus speed. Configure it for the slowest 
     frequency which might be used on the CMD line
   */
#define DIS_STATE_TRIES 100
  if (!(custom_command & CUSTCOM_DONTSTDBY)) {   //not to go to standby state after the data transfer
    if (read_or_write) {
      /* If it was a write operation, it is possible
         that the card might be in the programming state,
         so we wait for some time for it to go into 
         disconnect/trans state
       */
      for (count = 0; count < DIS_STATE_TRIES; count++) {
        if ((retval2 = emmc_get_status_of_card(dev, slot, &the_state))){
          return retval2;
        }
        if (CARD_STATE_PRG != the_state)
          break;
      }
      if (DIS_STATE_TRIES == count) {
        return ERRHARDWARE;
      }
    }

    retval2 = emmc_put_in_trans_state(dev, slot);

    /* Hit the card with an unaddressed CMD7 */
    retval2 =  emmc_send_serial_command(dev, slot, UNADD_CMD7,
             0, resp_buffer,NULL, 0, NULL, NULL);

    if (retval2) {
      return retval2;
    }
  }

  the_card_info[slot].card_state = CARD_STATE_STBY;
  return retval;
}

u32_t emmc_bus_corruption_present(void)
{
  return current_task.bus_corruption_occured;
}

u32_t emmc_is_card_ready_for_data(struct device *dev, u32_t slot)
{
  u32_t resp_buffer, retval = 0;
  int count;

  for (count = 0; count < READY_FOR_DATA_RETRIES; count++) {
    if ((retval =
         emmc_send_serial_command(dev, slot, CMD13, 0,     //CMD13: SET_STATUS
              &resp_buffer,
              NULL, 0, NULL, NULL))) {
      return retval;
    }
    if (resp_buffer & R1CS_READY_FOR_DATA) {
      break;
    }
    plat_delay(1);
  }

  if (READY_FOR_DATA_RETRIES == count) {
    return ERRCARDNOTREADY;
  }

  return 0;
}

void plat_unmap_single(void * bus_device,dma_addr_t desc2 ,u32_t dummy, u32_t access_type)
{
  printk("UNLOCKING --plat_unmap_single---\n");
#if 0
   if(access_type == TO_DEVICE)
    pci_unmap_single((struct pci_dev *)(bus_device), desc2, 0,PCI_DMA_TODEVICE );
   if(access_type == FROM_DEVICE)
    pci_unmap_single((struct pci_dev *)(bus_device), desc2, 0,PCI_DMA_FROMDEVICE );
   if(access_type == BIDIRECTIONAL)
    pci_unmap_single((struct pci_dev *)(bus_device), desc2, 0,PCI_DMA_BIDIRECTIONAL);
#endif
}

dma_addr_t plat_map_single(void * bus_device, u8_t * data_buffer, u32_t len1, u32_t access_type)
{
  printk("UNLOCKING --plat_map_single---\n");

   dma_addr_t dma_addr = 0;
#if 0
   if(access_type == TO_DEVICE)
    dma_addr =  pci_map_single((struct pci_dev *)(bus_device), data_buffer, len1,PCI_DMA_TODEVICE );
   if(access_type == FROM_DEVICE)
    dma_addr = pci_map_single((struct pci_dev *)(bus_device), data_buffer, len1,PCI_DMA_FROMDEVICE );
   if(access_type == BIDIRECTIONAL)
    dma_addr = pci_map_single((struct pci_dev *)(bus_device), data_buffer, len1,PCI_DMA_BIDIRECTIONAL);
#endif
   return dma_addr;
}

void emmc_set_data_trans_params(struct device *dev, u32_t slot, u8_t * data_buffer,  
            u32_t num_of_blocks,
            emmc_term_function
            the_term_func,
            emmc_copy_function the_copy_func,
            u32_t epoch_count, u32_t flag,
            u32_t custom_blocksize)               // callback
{
  emmc_clear_bits(dev, EMMC_REG_CTRL, INT_ENABLE);
  printk("EPOCH COUNT = %u\n", epoch_count);
  current_task.num_of_blocks = num_of_blocks;
  current_task.the_copy_function = the_copy_func;
  current_task.the_term_function = the_term_func;
  current_task.slot_num = slot;
  current_task.data_buffer = data_buffer;
  current_task.num_bytes_read = epoch_count;
  current_task.blksize = custom_blocksize;
  emmc_set_bits(dev, EMMC_REG_CTRL, INT_ENABLE);
}

s32_t emmc_set_qptr(struct device *dev, u32_t Buffer1, u32_t Length1, u32_t Buffer1_Virt, u32_t Buffer2, u32_t Length2, 
        u32_t Buffer2_Virt)
{
    u32_t  next      = current_task.next;
  DmaDesc *desc  = current_task.desc_next;


  emmc_set_register(dev, EMMC_REG_DBADDR,0x20030000);

  if(!emmc_is_desc_free(desc))
    return -1;
    
  if(emmc_is_desc_chained(desc)){
    desc->desc1       = ((Length1 << DescBuf1SizeShift) & DescBuf1SizMsk);
    desc->desc2       = Buffer1;
    desc->desc2_virt  = Buffer1_Virt;
    desc->desc0      |= DescOwnByDma | DescFirstDesc | DescLastDesc; // We handle only one buffer per descriptor

    current_task.next      = emmc_is_last_desc(desc) ? 0 : next + 1;
      current_task.desc_next = (DmaDesc *)desc->desc3_virt; //desc3_virt contains the next descriptor address
  }
  else{
    desc->desc1       = (((Length1 << DescBuf1SizeShift) & DescBuf1SizMsk) | 
                 ((Length2 << DescBuf2SizeShift) & DescBuf2SizMsk));
    desc->desc2       = Buffer1;
    desc->desc2_virt  = Buffer1_Virt;
    desc->desc3       = Buffer2;
    desc->desc3_virt  = Buffer2_Virt;
    desc->desc0      |= DescOwnByDma | DescFirstDesc | DescLastDesc; // We handle only one buffer per descriptor

    current_task.next      = emmc_is_last_desc(desc) ? 0 : next + 1;
      current_task.desc_next = emmc_is_last_desc(desc) ?
                         current_task.desc_head : desc + 1; // if last: assign head as next 
  }
  printk("%02d %08x %08x %08x %08x %08x %08x %08x\n",next,(u32_t)desc,desc->desc0,desc->desc1,desc->desc2,
          desc->desc3,desc->desc2_virt,desc->desc3_virt);
  
  return next;
} 

u32_t emmc_is_desc_free(DmaDesc *desc)
{
  if((desc->desc0 & DescOwnByDma) == 0)
    return 1; //if OWN bit is not set Descriptor is free
  else
    return 0; //if OWN bit is still set Desccriptor is owned by DMA   
}

u32_t emmc_is_desc_chained(DmaDesc *desc)
{
  if((desc->desc0 & DescSecAddrChained) == DescSecAddrChained)
    return 1;
  else
    return 0;
}

u32_t emmc_set_sd_2_0_voltage_range(struct device *dev, u32_t slot)
{
  u32_t retval = 0, resp_buffer = 0, new_ocr = 0;
  int count = CMD1_RETRY_COUNT;

  /* Is the card connected ? */
  if (!(CARD_PRESENT(dev, slot))) {
    return ERRCARDNOTCONN;
  }

  /* Check if it is in the correct state */
  if (the_card_info[slot].card_state != CARD_STATE_IDLE) {
    return ERRFSMSTATE;
  }

  new_ocr = OCR_27TO36 | OCR_POWER_UP_STATUS | OCR_HCS;   //OCR_HCS: card is High Capacity SD card ly
    
    // OCR_XPC is used to check on SDXC Power Control. OCR_S18R is used to check Voltage Switching Support
    new_ocr = new_ocr | OCR_XPC | OCR_S18R;

  count = ACMD41_RETRY_COUNT;
  while (count) {
    if ((retval =  emmc_send_serial_command(dev, slot, CMD55, 0, NULL, NULL, 0, NULL, NULL))) {
      return retval;
    }
       //ACMD41 :send host capacity support info and ask card seed its OCR reg value   ly
    retval = emmc_send_serial_command(dev, slot, ACMD41, new_ocr, &resp_buffer, NULL, 0, NULL, NULL);
    printk("GOT OCR AS = 0x%08x\n", resp_buffer);
    if ((resp_buffer & OCR_POWER_UP_STATUS) && (!retval)) {
      /* The power up process for the card is over */
      break;
    }

    --count;
    plat_delay(100);
  }

  if (0 == count) {
    printk("Giving up on trying to set voltage after %x retries\n", CMD1_RETRY_COUNT);
    the_card_info[slot].card_state = CARD_STATE_INA;
    return ERRHARDWARE;
  } else {
    if ((resp_buffer & OCR_27TO36) != OCR_27TO36) {
      printk("Set voltage differs from OCR. Aborting");
      the_card_info[slot].card_state = CARD_STATE_INA;
      return ERRHARDWARE;
    }
    printk("SENT OCR = 0x%08x GOT OCR = 0x%08x\n", new_ocr, resp_buffer);
  }
  if(resp_buffer & OCR_CCS){
    printk("The card responded with CCS set\n");
  }
  else{
    printk("The card responded with CCS Un-set... The card could be a SDSC card\n");
    return ERRENUMERATE;    
  }  
    // If it is SDHC card, check whether SD3.0 card by verifying the Voltage switching support.
  if(resp_buffer & OCR_S18R){
    printk("The card supports Voltage switching\n");
    printk("The Card Found is SD MEMORY 3.0 Type\n");
    the_card_info[slot].card_type = SD_MEM_3_0_TYPE;
    the_card_info[slot].is_volt_switch_supported = 1; //Set a flag indicating that voltage switching supported.
    }
    the_card_info[slot].card_state = CARD_STATE_READY;
    return retval;
}

u32_t emmc_is_desc_owned_by_dma(DmaDesc *desc)
{
  if((desc->desc0 & DescOwnByDma) == DescOwnByDma) 
    return 1; //if OWN bit is set, Descriptor is owned by DMA
  else
    return 0; //if OWN bit is not set, Descriptor is not owned by DMA   
    
}

boolean emmc_is_last_desc(DmaDesc *desc)
{
  if(((desc->desc0 & DescEndOfRing) == DescEndOfRing) || (u32_t)current_task.desc_head == desc->desc3_virt)
    return 1;
  else 
    return 0;
}

u32_t emmc_reset_sd_card(struct device *dev, u32_t slot)
{

    u32_t buffer_reg;
    u32_t retval = 0;
    u32_t clock_freq_to_set = SD_ONE_BIT_BUS_FREQ;
#ifdef SD_SET_HIGH_SPEED
       Card_state the_state;
       u32_t resp_buffer;
#endif


  /* Check if the card is connected? */
  buffer_reg = emmc_read_register(dev, EMMC_REG_CDETECT);
  if (buffer_reg & (1 << slot)) {
    return ERRCARDNOTCONN;
  }
    /* Clear out the state information for the card status. We know the card is SD_TYPE*/
    memset((void *) (the_card_info + slot), 0, sizeof(Card_info));
    the_card_info[slot].card_type = SD_TYPE;


    /* Fod the clock and OD the bussince we start enumerating now */
    printk("Setting divider value to %x(%x)\n",SD_FOD_DIVIDER_VALUE, slot);
    emmc_set_bits(dev, EMMC_REG_CTRL, ENABLE_OD_PULLUP);    //SD reset need enable open_drain pullup  ly
    the_card_info[slot].divider_val = SD_FOD_DIVIDER_VALUE;
    /* Reset the card. Since we really dont know as to from where the call has been made*/
    if ((retval = emmc_send_serial_command(dev, slot, CMD0, 0, NULL, NULL, 0, NULL, NULL))) {
    goto HOUSEKEEP;                       //CMD0:GO_IDLE_STATE command ly 
  }

    /* CMD0 is successful => card is in idle state */
    the_card_info[slot].card_state = CARD_STATE_IDLE;
    /* Now in IDLE state. Set the sd voltage range on the card. Uses CMD55 and ACMD41 */
    if ((retval = emmc_set_sd_voltage_range(dev, slot))) {
      goto HOUSEKEEP;
    }

    /* Now in READY state Now extract the CID this uses CMD2 */
    if ((retval = emmc_get_cid(dev, slot))) {
    goto HOUSEKEEP;
  }

    /* Now in IDENT state. Set the RCA. this uses CMD3*/
    if ((retval = emmc_set_sd_rca(dev, slot))) {
    goto HOUSEKEEP;
  }

    /*Clk freq is set to one bit freq. */
    the_card_info[slot].divider_val = clock_freq_to_set;
    emmc_clear_bits(dev, EMMC_REG_CTRL, ENABLE_OD_PULLUP);

    /*Now in STBY stage. So we get the CSD Register and store it. this uses CMD9 */
    if ((retval = emmc_process_csd(dev, slot))) {
    /* Switch off the card */
    goto HOUSEKEEP;
  }
    HOUSEKEEP:      //if return is 0 : command success   ly
    the_card_info[slot].divider_val = clock_freq_to_set;
    the_card_info[slot].enum_status = retval;
    emmc_set_clk_freq(dev, clock_freq_to_set);
    emmc_clear_bits(dev, EMMC_REG_CTRL, ENABLE_OD_PULLUP);
    return retval;
}

u32_t emmc_set_sd_voltage_range(struct device *dev, u32_t slot)
{
  u32_t retval = 0, resp_buffer = 0, new_ocr = 0;
  int count = CMD1_RETRY_COUNT;

  /* Is the card connected ? */
  if (!(CARD_PRESENT(dev, slot))) {
    printk("The card is not connected..\n");
    return ERRCARDNOTCONN;
  }

  /* Check if it is in the correct state */
  if (the_card_info[slot].card_state != CARD_STATE_IDLE) {
    return ERRFSMSTATE;
  }

  new_ocr = OCR_27TO36 | OCR_POWER_UP_STATUS;

  count = ACMD41_RETRY_COUNT;
  while (count) {
    if ((retval =
         emmc_send_serial_command(dev, slot, CMD55, 0, NULL, NULL, 0, NULL, NULL))) {
      return retval;
    }


    retval = emmc_send_serial_command(dev, slot, ACMD41, new_ocr, &resp_buffer, NULL, 0, NULL, NULL);
    printk("GOT OCR AS = 0x%08x\n", resp_buffer);
    if ((resp_buffer) && (!retval)) {
      /* The power up process for the card is over */
      break;
    }

    --count;
    plat_delay(1000); /*1 GB Kingston Micro SD card needs more time. so changed from 100 to 1000*/
  }

  if (0 == count) {
    printk("Giving up on trying to set voltage after %x retries\n", CMD1_RETRY_COUNT);
    the_card_info[slot].card_state = CARD_STATE_INA;
    return ERRHARDWARE;
  } else {
    if ((new_ocr & OCR_27TO36) != OCR_27TO36) {
      printk("Set voltage differs from OCR. Aborting");
      the_card_info[slot].card_state = CARD_STATE_INA;
      return ERRHARDWARE;
    }
    printk("SENT OCR = 0x%08x GOT OCR = 0x%08x\n", new_ocr, resp_buffer);
  }
  the_card_info[slot].card_state = CARD_STATE_READY;
  return retval;
}

u32_t emmc_set_sd_rca(struct device *dev, u32_t slot)
{
  u32_t buffer_reg, resp_buffer, retval = 0;

  /* Check if the card is connected */
  buffer_reg = emmc_read_register(dev, EMMC_REG_CDETECT);
  if (buffer_reg & (1 << slot)) {
    return ERRCARDNOTCONN;
  }

  if (CARD_STATE_IDENT != the_card_info[slot].card_state) {
    return ERRFSMSTATE;
  }

  if ((retval =
       emmc_send_serial_command(dev, slot, CMD3, 0,
            &resp_buffer, NULL, 0, NULL,
            NULL))) {
    return retval;
  }
  printk("GOT RESP FOR CMD3 = 0x%08x\n", resp_buffer);
    
  the_card_info[slot].card_state = CARD_STATE_STBY;
  the_card_info[slot].the_rca = GET_R6_RCA(resp_buffer);
  printk("this is 222222222222222222 = %08x\n",the_card_info[slot].the_rca);
  return 0;
}

u32_t emmc_process_csd(struct device *dev, u32_t slot)
{
  u32_t buffer_reg, retval;
  u32_t read_block_size, write_block_size;
  u32_t card_size;
  u32_t blocknr, blocklen;

  buffer_reg = emmc_read_register(dev, EMMC_REG_CDETECT);
  if (buffer_reg & (1 << slot)) {
    return ERRCARDNOTCONN;
  }


  if (CARD_STATE_STBY != the_card_info[slot].card_state) {
    printk("This is in the wrong FSM state for CSD fetch %x\n",the_card_info[slot].card_state);
    return ERRFSMSTATE;
  }
  if ((retval = emmc_send_serial_command(dev, slot, CMD9, 0, the_card_info[slot].the_csd, NULL, 0, NULL, NULL))) {
    return retval;
  }
  /* The CSD is in the bag */

  /* Store the largest TAAC and NSAC. We will use these for calculating the data timeout latency. */
  printk("**************CSD**********************\n");
  printk("0x%08x 0x%08x 0x%08x 0x%08x\n",
         the_card_info[slot].the_csd[0],
         the_card_info[slot].the_csd[1],
         the_card_info[slot].the_csd[2],
         the_card_info[slot].the_csd[3]);
  printk("**************CSD**********************\n");

  //TAAC :define the asynchronous part of the data access time
  //NSAC :define the worst case for the clock-dependant factor of the data access time
  if (CSD_TAAC(the_card_info[slot].the_csd) > the_ip_status.max_TAAC_value) {
    the_ip_status.max_TAAC_value =  CSD_TAAC(the_card_info[slot].the_csd);
  }

  if (CSD_NSAC(the_card_info[slot].the_csd) > the_ip_status.max_NSAC_value) {
    the_ip_status.max_NSAC_value = CSD_NSAC(the_card_info[slot].the_csd);
  }

  read_block_size  = 1 << (CSD_READ_BL_LEN((the_card_info[slot].the_csd)));
  write_block_size = 1 << (CSD_WRT_BL_LEN((the_card_info[slot].the_csd)));

  /* See section 5.3 of the 4.1 revision of the MMC specs for
     an explanation for the calculation of these values
   */          //P147    version 1.0
  blocknr = (CSD_C_SIZE(the_card_info[slot].the_csd) + 1) * (1 << (CSD_C_SIZE_MULT(the_card_info[slot].the_csd) + 2));
  blocklen = read_block_size;
  card_size = blocknr * blocklen;
  /* read/write block size */
  the_card_info[slot].card_write_blksize = (write_block_size > 512) ? 512 : write_block_size;
  the_card_info[slot].orig_card_read_blksize = read_block_size;
  the_card_info[slot].card_read_blksize = (read_block_size > 512) ? 512 : read_block_size;
  the_card_info[slot].orig_card_write_blksize = write_block_size;
  /* Set the card capacity */
  the_card_info[slot].card_size = blocknr;

  printk("Card capacity = %x bytes\n", card_size);
  printk("Read Block size = %x bytes\n",the_card_info[slot].card_read_blksize);
  printk("Write Block size = %x bytes\n",the_card_info[slot].card_write_blksize);
  return 0;
}

u32_t emmc_set_sd_wide_bus(struct device *dev, u32_t slot, u32_t width)
{
  u32_t retval, arg;
  u32_t resp;

  if (4 == width) {
    arg = 2;
  } else if (0 == width) {
    arg = 0;
  } else {
    return ERRILLEGALCOMMAND;
  }

    /* Check if the card is in standby state and put in trans
       state.
     */
  if ((retval = emmc_put_in_trans_state(dev, slot))) {
    printk("%x TRANS STATE FAILED\n", retval);
    return retval;
  }
    printk("Clr card detect on dat3. \n");
    if ((retval =
       emmc_send_serial_command(dev, slot, ACMD42, 0, &resp, NULL, 0, NULL, NULL))) {
             printk("WE HAVE RETVAL = %x\n", retval);
       return retval;
  }

  if ((retval =
       emmc_send_serial_command(dev, slot, ACMD6, arg, &resp, NULL, 0, NULL, NULL))) {
             printk("WE HAVE RETVAL = %x\n", retval);
       return retval;
  }

  /* now set the width of the bus */
    emmc_clear_bits(dev, EMMC_REG_CTYPE, (1 << slot) || (1 << (slot + 16)));
  if (4 == width) {
    printk("Setting Host controller to operate in 4 bit mode\n");
    emmc_set_bits(dev, EMMC_REG_CTYPE, (1 << slot));
  }

  return 0;
}

u32_t emmc_put_in_trans_state(struct device* dev, u32_t slot)
{
  u32_t retval = 0, resp_buffer;
  Card_state the_state;

  if (emmc_read_register(dev, EMMC_REG_CDETECT) & (1 << slot)) {
    return ERRCARDNOTCONN;
  }

//send cmd13 to get the status  
#if 1         
  if ((retval = emmc_get_status_of_card(dev, slot, &the_state))) {
    printk("Getting status of card borked out !\n");
    return retval;
  }

  /* If the card already is 
     in the trans state, our work 
     here is done 
   */
  if (CARD_STATE_TRAN == the_state) {
    the_card_info[slot].card_state = CARD_STATE_TRAN;
    return 0;
  }

  if (CARD_STATE_STBY != the_state) {
    printk("The card state = %x, erroring out\n", the_state);
    return ERRFSMSTATE;
  }
#endif
  the_card_info[slot].card_state = CARD_STATE_STBY;

  printk("CMD13 pass and run CMD7 to change state...");
  /* Now send the command to send to standby state */
  if ((retval =
       emmc_send_serial_command(dev, slot, CMD7, 0,
            &resp_buffer, NULL, 0,
            NULL, NULL))) {

    return retval;
  }

  /* This puts the card into trans */
  the_card_info[slot].card_state = CARD_STATE_TRAN;

  return retval;

}

u32_t emmc_get_status_of_card(struct device *dev, u32_t slot, Card_state * status)
{
  u32_t resp_buffer, retval = 0;

  /* Pick up the status from a R1 response */
  if ((retval =
       emmc_send_serial_command(dev, slot, CMD13, 0,
            &resp_buffer, NULL, 0,
            NULL, NULL))) {
    return retval;
  }

  /* We have R1 in the resp buffer , we now check the 
     status of the card. If it is not in a a standby
     state we exit. 
   */
  *status = (Card_state) R1CS_CURRENT_STATE(resp_buffer);
  return 0;
}

u32_t emmc_get_cid(struct device *dev, u32_t slot)
{
  u32_t buffer_reg, retval = 0;
  int count;
  char product_name[7];
  int product_revision[2];
  int month, year;
  /* Check if the card is connected */
  buffer_reg = emmc_read_register(dev, EMMC_REG_CDETECT);
  if (buffer_reg & (1 << slot)) {
    return ERRCARDNOTCONN;
  }


  if (CARD_STATE_READY != the_card_info[slot].card_state) {
    return ERRFSMSTATE;
  }

  count = CMD2_RETRY_COUNT;
  while (count) {
    retval =
        emmc_send_serial_command(dev, slot,
             CMD2,
             0,
             (the_card_info
              [slot].
              the_cid), NULL, 0, NULL,
             NULL);
    if (!retval) {
      break;
    }
    count--;
    plat_delay(100);
  }

  if (0 == count) {
    printk("FAILED TO GET CID OF THE CARD !!\n");
    return ERRHARDWARE;
  }

  else {
    printk
        ("CID = 0x%08x  0x%08x  0x%08x  0x%08x\n",
         the_card_info[slot].the_cid[0],
         the_card_info[slot].the_cid[1],
         the_card_info[slot].the_cid[2],
         the_card_info[slot].the_cid[3]);
  }

  /* printk out some  informational message about the card
     always makes for good eye candy
   */
  for (count = 5; count > -1; count--) {
    product_name[5 - count] =
        the_card_info[slot].the_cid_bytes[count + 7];
  }
  product_name[count] = 0;
  product_revision[0] = the_card_info[slot].the_cid_bytes[6] & 0x0f;
  product_revision[1] =
      (the_card_info[slot].the_cid_bytes[6] & 0xf0) >> 4;
  month = (the_card_info[slot].the_cid_bytes[1] & 0xf0) >> 4;
  year = (the_card_info[slot].the_cid_bytes[1] & 0x0f) + 1997;
  printk("Found Card %s Rev %x.%x (%x/%x)\n", product_name, product_revision[1], product_revision[0], month, year);
  the_card_info[slot].card_state = CARD_STATE_IDENT;
  return 0;
}

void plat_disable_interrupts(struct device *dev, int_register * buffer){

  /* We could disable all interrupts on the board or we could disable only the
     host controller interrupt. Disabling the host control interrupt would be 
     a better idea. 
   */
  emmc_clear_bits(dev, EMMC_REG_CTRL, INT_ENABLE);
}

/**
  * Enable the host controller interrupt.
  * @param[in] buffer Buffer pointer to restore the interrupt mask which might have been removed when plat_disable_interrupts was called.
  * \return Returns void.
  *   
  */
void plat_enable_interrupts(struct device *dev, int_register * buffer)
{
  emmc_set_bits(dev, EMMC_REG_CTRL, INT_ENABLE);
}

u32_t emmc_send_serial_command(struct device *dev, u32_t slot, u32_t cmd_index, u32_t arg,
           u32_t * response_buffer, u8_t * data_buffer,
           u32_t flags,
           emmc_preproc_callback custom_preproc,
           emmc_postproc_callback custom_postproc)
{

  int_register reg;
  u32_t status,buffer1,buffer1_virt,buffer2,buffer2_virt;
  u32_t retval = 0, retval2 = 0;
  s32_t delay_for_command_done_check = 10000;

  /*
  If card is not connected or the enum_status not zero return saying card not connected
  */
  if (emmc_read_register(dev, EMMC_REG_CDETECT) & (1 << slot)) {
    printk("Card not connected\n");
    return ERRCARDNOTCONN;
    
  }
  
  if (the_card_info[slot].enum_status) {
    printk("send serial command's responds is ERRCARDNOTCONN \n");  //not in
    return ERRCARDNOTCONN;
  }


  /* Disable the host controller interrupt before sending the command on the line to the card
     Enable the interrupts once the command sent to the card */
  plat_disable_interrupts(dev, &reg);


  retval = emmc_form_n_send_cmd(dev, slot, cmd_index, arg,
            response_buffer, data_buffer,
            flags, custom_postproc,
            custom_preproc);

  plat_enable_interrupts(dev, &reg);

  if (retval) {   //retval=0
    return retval;  
  }


  /* Put myself to be scheduled from either interrupt context or from the timeout.
     I am following this paradigm since I am thinking in Unix space. I am however 
     presuming that this will hold true for any other kernel.
     Now this thread gets blocked till we receive an interrupt or timeouts
   */
   
  if ((plat_reenable_upon_interrupt_timeout(dev))) {     //timeout   lly
    printk("CMD%x did not receive any interrupt(1) even after %x seconds\n",cmd_index,CMD_RESP_TIMEOUT);
    retval = ERRTIMEROUT;  //37
  }

  /* If there is a size for the data supplied in the flags, wait for it
     TODO: Calculate the latency of the transfer.
   */
  

  if (flags && (!(emmc_last_com_status()))) {

    if ((ERRTIMEROUT == retval) && (the_card_info[slot].card_type != CEATA_TYPE)) {
      emmc_remove_command(dev);/*Lets make the command status as TASK_STAT_ABSENT*/
      emmc_abort_trans_work(dev, slot);
      plat_delay(200);/*This delay is required as the above function doesnot block*/
    }

  }

  /*
     We have been scheduled back again. Now check the error status of the command. For non zero error status 
     set command status as TSK_STAT_ABSENT 
   */

  if ((retval2 = emmc_last_com_status())) {
    emmc_remove_command(dev);
  } 
  
  printk("Leaving (%x %x %d)\n", retval, retval2, cmd_index);
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);

  return (retval ? retval : retval2);
}

void emmc_remove_command(struct device *dev)
{
  emmc_clear_bits(dev, EMMC_REG_CTRL, INT_ENABLE);
  current_task.cmd_status = TSK_STAT_ABSENT;
  emmc_set_bits(dev, EMMC_REG_CTRL, INT_ENABLE);
}

u32_t emmc_last_com_status(void)
{
  return current_task.error_status;
}

u32_t plat_reenable_upon_interrupt_timeout(struct device *dev)
{

    u32_t retval=1;
    int_register i;


    while(1){
            if(interrupt_already_done)
            {    
                  printk(" Enter if is %d\n",interrupt_already_done);
                  retval=0;  
                  break;
                }
                
            }
    plat_disable_interrupts(dev, &i);
    interrupt_already_done = 0;
    plat_enable_interrupts(dev, &i);

if (retval){
    return 1;
      printk("interrupt_already_done is zero:%x\n",interrupt_already_done);
      }
  else
    return 0;
}

void emmc_abort_trans_work(struct device *dev, u32_t slot)
{
  /* Send a raw command */
  emmc_send_raw_command(dev, slot, CMD12 | CMD_RESP_EXP_BIT | CMD_ABRT_CMD_BIT, 0);

}

void emmc_send_raw_command(struct device *dev, u32_t slot, u32_t cmd, u32_t arg)
{
  u32_t buff_cmd;
  buff_cmd = cmd | CMD_DONE_BIT;
  SET_CARD_NUM(buff_cmd, slot);
  printk("SENDING RAW Command 0x%08x:0x%08x\n", buff_cmd, arg);
  emmc_execute_command(dev, buff_cmd, arg);
  return;
}

u32_t emmc_form_n_send_cmd(struct device *dev,
         u32_t  card_num,
           u32_t  cmd_index,
           u32_t  cmd_arg,
           u32_t *resp_buffer,
           u8_t  *data_buffer,
           u32_t  flags,
           emmc_postproc_callback  custom_callback,
           emmc_preproc_callback  custom_preproc)
{

  u32_t cmd_register = 0;
  emmc_postproc_callback post_callback = NULL;
  emmc_preproc_callback preproc_fn = NULL;
  u32_t arg_register = cmd_arg;
  /* First time make the previous divider a maximum value.*/
  static u32_t previous_divider = 0xffffffff;
  /* whether post_callback is from table or custom_callback? */

  struct device *emmc_callback_dev;
  emmc_callback_dev = device_get_binding("emmc_callback_0");

  if (!custom_callback) {
    post_callback = emmc_get_post_callback(emmc_callback_dev, cmd_index);
  } else {
    post_callback = custom_callback;

  }

  if (!post_callback) {
    printk("ERRNOTSUPPORTED! CMD%x Command is not supported!\n", cmd_index);
    return ERRNOTSUPPORTED;

  }

  /* whether preproc_callback is from table or custom_preproc? */
  if (!custom_preproc) {
    preproc_fn = emmc_get_pre_callback(emmc_callback_dev, cmd_index);
  } else {
    preproc_fn = custom_preproc;
  }

  /*execute the preproc_fn to prepare the command: this function populates 
    the cmd_register and arg_register*/
  if (preproc_fn) {
    preproc_fn(card_num, cmd_index, &cmd_register, &arg_register);
  } else {
    printk("CMD%x Command is not supported!\n", cmd_index);
    return ERRNOTSUPPORTED;
  }

  /* Set the frequency for the card and enable the clocks for appropriate cards */
  if (previous_divider != the_card_info[card_num].divider_val) {
    previous_divider = the_card_info[card_num].divider_val;
    printk("previous_divider is %x\n",previous_divider);
    emmc_set_clk_freq(dev, the_card_info[card_num].divider_val);
  }
  current_task.command_index = cmd_index; 
  return emmc_cmd_to_host(dev, card_num, cmd_register, arg_register,resp_buffer, data_buffer,post_callback, flags);
}

/**
  * This sends the command after taking the lock. The lock is released from ISR.
  * This forms the CMD and CMDARG and calls emmc_execute_command which sends 
  * the command and polls for CMD_DONE_BIT to see if it is 0
  * For IDMAC mode of operation, setup the IDMAC related registers, and
  * setup the flag idma_mode_on to indicate the ISR that the present transfer
  * uses IDMAC flow rather than Slave mode flow.
  */

u32_t emmc_cmd_to_host(struct device *dev, u32_t slot,
       u32_t
       cmd_register,
       u32_t
       arg_register,
       u32_t *
       resp_buffer,
       u8_t *
       data_buffer,
       emmc_postproc_callback the_callback,
       u32_t flags)
{
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  printk("Sending Command : 0x%08x:0x%08x\n", cmd_register, arg_register);
  //struct device *emmc_callback_dev;
  //emmc_callback_dev = device_get_binding("emmc_callback_0");

  /* update the task status with call back, response buffer, 
     data buffer,error_status,cmd_status,bus_corruption_occured,...
  */
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  emmc_set_current_task_status(emmc_callback_dev, slot, resp_buffer, data_buffer, the_callback);
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  /*Set the CMD_DONE_BIT to initiate the command sending, CIU clears this bit */
  SET_BITS(cmd_register, CMD_DONE_BIT);    //cmd start info
  printk("the_ip_status.num_of_cards %x\n", the_ip_status.num_of_cards);
  printk("CMD: %08x CMDARG:%08x\n", cmd_register,arg_register);
  /* Execute the command and wait for the command to execute */
  return emmc_execute_command(dev, cmd_register, arg_register);
}

/**
  * Sets the divider for the clock in CIU.
  * This function sets a particular divider to the clock.
  * @param[in] divider The divider value.
  * \return 0 upon success. Error code upon failure.
  */
u32_t emmc_set_clk_freq(struct device *dev, u32_t divider)
{
    u32_t orig_emmc_CLKENA;
    u32_t retval;

    if (divider > MAX_DIVIDER_VALUE) {
      return 0xffffffff;
    }

    /* To make sure we dont disturb enable/disable settings of the cards*/
    orig_emmc_CLKENA = emmc_read_register(dev, EMMC_REG_CLKENA);

    /* Disable all clocks before changing frequency the of card clocks */
    if ((retval = emmc_disable_all_clocks(dev))) {
      return retval;
    }
    printk("disable ok..\n");
    /* Program the clock divider in our case it is divider 0 */
    emmc_clear_bits(dev, EMMC_REG_CLKDIV, MAX_DIVIDER_VALUE);      
    emmc_set_bits(dev, EMMC_REG_CLKDIV, divider);
    printk("EMMC_REG_CLKDIV is %x\n",L_EMMC_REG_CLKDIV);
    /*Send the command to CIU using emmc_send_clock_only_cmd and enable the clocks in emmc_CLKENA register */
    if ((retval = emmc_send_clock_only_cmd(dev))) {         //////////////   
      emmc_enable_clocks_with_val(dev, orig_emmc_CLKENA);
      return retval;
  }

  return emmc_enable_clocks_with_val(dev, orig_emmc_CLKENA);
}

u32_t emmc_disable_all_clocks(struct device *dev)
{
  emmc_set_register(dev, EMMC_REG_CLKENA, 0);
  return emmc_send_clock_only_cmd(dev);    //CIU not return 0 so divsion is not complement    lly
}

u32_t emmc_enable_clocks_with_val(struct device *dev, u32_t val)
{
  emmc_set_register(dev, EMMC_REG_CLKENA, val);
  return emmc_send_clock_only_cmd(dev);
}

u32_t emmc_read_write_multiple_blocks(struct device *dev, u32_t slot, u32_t start_sect,
          u32_t num_of_sects, u8_t * buffer,
          u32_t read_or_write, u32_t sect_size)
{
  u32_t retval = 0;
  u32_t resp_buff[4];
  u32_t read_or_write_to_send;
  u32_t sect_size_in_bytes;
  sect_size_in_bytes = 1 << (sect_size - 1); //1<<9
    printk("12Card capacity = %x Kbytes\n", the_card_info[slot].card_size);
  if (read_or_write) {
    read_or_write_to_send = sect_size_in_bytes * num_of_sects;
  } else {
    read_or_write_to_send = 0;
  }

  if (start_sect + num_of_sects > the_card_info[slot].card_size) {
    retval = ERRADDRESSRANGE;
    goto HOUSEKEEP;
  }

  printk("Card capacity = %x Kbytes\n", the_card_info[slot].card_size);
  retval =  emmc_read_write_bytes(dev, slot, resp_buff, buffer,
                start_sect * sect_size_in_bytes, //start 
                start_sect * sect_size_in_bytes + sect_size_in_bytes * num_of_sects, //end
                0, NULL, NULL,
                read_or_write_to_send,
                sect_size << CUSTOM_BLKSIZE_SHIFT, //10<<12=0xa000
                NULL,
                NULL);
#if 1
  {
    s32_t i = 0;
    printk("read_or_write = %d\n",read_or_write_to_send);
      {
      for(i = 0; i< ((start_sect*sect_size_in_bytes+sect_size_in_bytes*num_of_sects)-(start_sect*sect_size_in_bytes)); i=i+32)
        printk("DATA[%04d . . .] = %08x %08x %08x %08x %08x %08x %08x %08x\n",
          i,*((u32_t*)(buffer+i)),*((u32_t*)(buffer+i+4)),*((u32_t*)(buffer+i+8)),*((u32_t*)(buffer+i+12)),
            *((u32_t*)(buffer+i+16)),*((u32_t*)(buffer+i+20)),*((u32_t*)(buffer+i+24)),*((u32_t*)(buffer+i+28)));
      }
  }
#endif

    HOUSEKEEP:
    return retval;
}


int main(void)
{
    printk("This is Zephyr EMMC_PPU Driver Test...\n");
    printk("Find device...\n");

    struct device *emmc_dev;

    emmc_dev = device_get_binding(EMMC_DRV_NAME);
    if(!emmc_dev)
    {
        printk("Cannot find %s!\n", EMMC_DRV_NAME);
        //return;
    }
    
    printk("Find %s!\n", EMMC_DRV_NAME);

    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    if(!gpio_dev)
    {
        printk("Cannot find %s!\n", "gpio0");
        return 0;
    }
    printk("Find gpio0!\n");

    lcd_dev = device_get_binding(LCD_DRV_NAME);
    if(!lcd_dev)
    {
        printk("Cannot find %s!\n", LCD_DRV_NAME);
        //return;
    }
    
    printk("Find %s!\n", LCD_DRV_NAME);

    emmc_callback_dev = device_get_binding("emmc_callback_0");

    //u32_t cmd_tx_done;
    u32_t retval = 0;

    gpio_pin_configure(gpio_dev, 2, GPIO_DIR_OUT);
    gpio_pin_write(gpio_dev, 2, 0);

    PPU_IER |= (1<< 15); //enable emmc interrupt 

    retval = emmc_init_controller(emmc_dev);
    if(retval) {
        printk("Could not initialize the host controller IP \n");
        return retval;
}

u32_t ret,REG;
/************************************************************************************************ 
This test is Erase function validataion for MMC and SD cards.
@paramerter slot
@parameter  Erase_block_start address/sector index (For MMC4.2 and SD2.0 this is sector index for other cards it is byte address)
@parameter  Erase_block_End address/sector index (For MMC4.2 and SD2.0 this is sector index for other cards it is byte address)
************************************************************************************************/ 

#if 1
{
     int count,ii,jj,i;
     u8_t *buff_write1;
     u8_t *buff_read1;
     u8_t data_to_write;


    buff_write1  = (u8_t *)0x50010000;      //8192
    buff_read1   = (u8_t *)0x50020000;
    printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~buff_write1 is %x\n", buff_write1);
        memset((void *)buff_read1 ,0x00,MEM_SIZE);
        memset((void *)buff_write1,0x00,MEM_SIZE);
        printk("TESTING  write and read for MMC/SD device\n");
        data_to_write = 0xF0;//IDMAC testing
  for(ii = 0; ii<NO_OF_TEST_LOOPS;)
  {

      memset((void *)buff_read1,0x00,MEM_SIZE);
      memset((void*)(buff_write1),data_to_write,SECTOR_SIZE);
      memset((void*)(buff_write1+SECTOR_SIZE),0xaa,(MEM_SIZE-SECTOR_SIZE));
       #if 1
          gpio_pin_write(gpio_dev, 2, 0);
          ret =  emmc_read_write_multiple_blocks(emmc_dev, 0, ii,NUM_OF_MUTLIPLE_BLOCKS , buff_write1, WRITE,(SECT_SIZE_SHIFT + 1));
          if (ret) {
                  printk( "WRITE FAILED with %u value at %d write operation\n",ret,ii);
                  break;
          }
       
    plat_delay(10000);
      #endif  
          ret =  emmc_read_write_multiple_blocks(emmc_dev, 0, ii, NUM_OF_MUTLIPLE_BLOCKS  , buff_read1, READ,(SECT_SIZE_SHIFT + 1));
          if (ret) {
                  printk("READ FAILED with %u value at ii = %d\n",ret,ii);
                  break;
    }

       #if 1

          if ((memcmp(buff_read1,buff_write1,( NUM_OF_MUTLIPLE_BLOCKS  * 512)) == 0)) {
                  printk( "TEST SUCCESS for %d WRITE/READ Operation \n",ii);
          }
          else {
                  printk( "TEST FAILURE for %d  WRITE/READ operation \n",ii);
                  for(i =0; i<(NUM_OF_MUTLIPLE_BLOCKS*SECTOR_SIZE);i++)
                        printk("%02x",buff_read1[i]);
            printk("\n");
                  break;
            }
     #endif
    
    ii = ii + NUM_OF_MUTLIPLE_BLOCKS;
  }
  printk("READ_WRITE TEST COMPLETED\n");
}
# endif

//int_disable();
  
return 0;

}