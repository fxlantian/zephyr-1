/*
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <board.h>
#include <ann.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <gpio.h>
#include <i2c.h>
#include <spi.h>
#include <misc/printk.h>
#include "ann_ppu.h"

/* convenience defines */
#define DEV_ANN_CFG(dev)                           \
        ((struct ann_ppu_config * const)(dev)->config->config_info)
#define DEV_ANN(dev)                               \
        ((volatile struct ann_ppu_t *)(DEV_ANN_CFG(dev))->ann_base_addr)
#define SPI_START_TRANSACTION(trans_type, csnum)   \
        (((1 << (csnum + 8)) & 0xF00) | ((1 << trans_type) & 0xFF))

volatile int Isr_Type;
volatile int count;


static void PPU_init_pro(int im_depth, int weight_depth, int bias_depth, int DMA_src_addr,  int block_size)
{
    int ready = 1;
    ready = *(volatile int *) (ANN_REG_STATUS_RUN);
    if (!(ready & 0x03)) // not run ot init
    {
         *(volatile int*) (ANN_REG_IM_DEPTH      )  = im_depth;
         *(volatile int*) (ANN_REG_WEIGTH_DEPTH  )  = weight_depth;
         *(volatile int*) (ANN_REG_BIAS_DEPTH    )  = bias_depth;
         *(volatile int*) (ANN_REG_DMA_SRC_ADDR  )  = DMA_src_addr;
         *(volatile int*) (ANN_REG_DMA_BLOCK_INFO)  = block_size; 
         *(volatile int*) (ANN_REG_SIG_DEPTH     )  = 0x200;
         *(volatile int*) (ANN_REG_CTR) = 0x06 ;
    }
}

static void PPU_npu_pro(int npu_datain_depth, int npu_detaout_depth, int DMA_src_addr, int DMA_dst_addr, int block_size )
{
    int ready = 1;
    ready = *(volatile int *) (ANN_REG_STATUS_RUN);
    if (!(ready & 0x03)) // not run ot init
    {
         *(volatile int*) (ANN_REG_NPU_DATAIN_DEPTH )  = npu_datain_depth;
         *(volatile int*) (ANN_REG_NPU_DATAOUT_DEPTH)  = npu_detaout_depth;
         *(volatile int*) (ANN_REG_DMA_SRC_ADDR     )  = DMA_src_addr    ;
         *(volatile int*) (ANN_REG_DMA_DST_ADDR     )  = DMA_dst_addr    ;
         *(volatile int*) (ANN_REG_DMA_BLOCK_INFO   )  =  block_size;
         *(volatile int*) (ANN_REG_CTR) = 0x05;      
    }
}

static void LCD_WR_REG(u8_t da)
{
    struct device *gpio_dev = device_get_binding("gpio0");
    gpio_pin_write(gpio_dev, 30, 0);
    int cmd_reg;
    cmd_reg = da << 24;
    *(volatile int*) (SPI1_REG_SPICMD) = cmd_reg;
    *(volatile int*) (SPI1_REG_SPIADR) = 0;
    *(volatile int*) (SPI1_REG_SPILEN) = (8 & 0x3F) | ((0 << 8) & 0x3F00);
    *(volatile int*) (SPI1_REG_STATUS) = SPI_START_TRANSACTION(SPI_CMD_WR, SPI_CSN0);
}

static void LCD_WR_DATA8(u8_t da)
{
    struct device *gpio_dev = device_get_binding("gpio0");
    gpio_pin_write(gpio_dev, 30, 1);
    int cmd_reg;
    cmd_reg = da << 24;
    *(volatile int*) (SPI1_REG_SPICMD) = cmd_reg;
    *(volatile int*) (SPI1_REG_SPIADR) = 0;
    *(volatile int*) (SPI1_REG_SPILEN) = (8 & 0x3F) | ((0 << 8) & 0x3F00);
    *(volatile int*) (SPI1_REG_STATUS) = SPI_START_TRANSACTION(SPI_CMD_WR, SPI_CSN0);
}

static void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{ 
   LCD_WR_REG(0x2a);//sety
   LCD_WR_DATA8(x1>>8);
   LCD_WR_DATA8(x1);
   LCD_WR_DATA8(x2>>8);
   LCD_WR_DATA8(x2);

   LCD_WR_REG(0x2b);//setx
   LCD_WR_DATA8(y1>>8);
   LCD_WR_DATA8(y1);
   LCD_WR_DATA8(y2>>8);
   LCD_WR_DATA8(y2);

   LCD_WR_REG(0x2C);    //wram
}

static void PPU_sample_image(struct device *dev, volatile unsigned char* image_addr, unsigned char* image_sampled)
{
    u16_t i,j;
    u16_t k=0;      
    //image_addr->320*240 yuv 4:2:2
    volatile unsigned char* image_addr_ver = image_addr + 2 * 640;
    for(i=0;i<28;i++)
    {
      image_addr_ver = image_addr + i * 640 * 8 + 2 * 10; //pick a pixel every 8 line
      for(j=0;j<28;j++)
      {
        image_sampled[i*28 + j] = *image_addr_ver;
        image_addr_ver += 16;
      }
    }
}

static void PPU_normalization_image(struct device *dev ,volatile unsigned char* image_sampled)
{
    volatile int i,j,ver;
    u8_t max=0;
    u8_t min=255; 
    float temp[28*28];
    ver=0;
    for(i=0;i<28;i++)
    {
      for(j=0;j<28;j++)
      {
        if(image_sampled[ver]>90)
            image_sampled[ver] = 0;
        else
            image_sampled[ver] = 255;
        ver++;
      }
    }
    Address_set( 50, 50, 106 - 1, 106 - 1);
    unsigned char* image_addr_ver = image_sampled;
    unsigned char* image_addr_ver1 = image_sampled;
    for(i=0;i<28;i++)
    {
      for(j=0;j<28;j++)
      {
        LCD_WR_DATA8(*(image_addr_ver));
        LCD_WR_DATA8(*(image_addr_ver));
        LCD_WR_DATA8(*(image_addr_ver));

        LCD_WR_DATA8(*(image_addr_ver));
        LCD_WR_DATA8(*(image_addr_ver));
        LCD_WR_DATA8(*(image_addr_ver));
        image_addr_ver++;
      }

      for(j=0;j<28;j++)
      {
        LCD_WR_DATA8(*(image_addr_ver1));
        LCD_WR_DATA8(*(image_addr_ver1));
        LCD_WR_DATA8(*(image_addr_ver1));

        LCD_WR_DATA8(*(image_addr_ver1));
        LCD_WR_DATA8(*(image_addr_ver1));
        LCD_WR_DATA8(*(image_addr_ver1));
        image_addr_ver1++;
      }
    }
}

static void PPU_qfix_image(volatile unsigned char* image_addr, unsigned char* dst_image)
{
    int i,j;
    int ver = 0;  
     for(i=0;i<28;i++)
    {
      for(j=0;j<28;j++)
      {
          dst_image[ver] = image_addr[ver] >> 1;
          ver++;
      }
    }
}

void ann_ppu_isr (void){ 
    int a=0;
    int npu_on=0;
    a = *(volatile int*)(ANN_REG_STATUS_END);
    printk("a is %x\n", a);
    *(volatile int*) (ANN_REG_CTR) = 0x08;
    PPU_ICP |= (1<<16);

    if(a==1){      //init_ram is complete
        PPU_IER |= (1<< 17); //enable camera interrupt
        PPU_EER |= (1<< 17); //enable camera interrupt
        count++; 
    }
    else if(a==2){ //computing is complete
        count=2;
    }
}

static const struct ann_driver_api ann_ppu_drv_api_funcs = {
    .ppu_sample_image = PPU_sample_image,
    .ppu_normalization_image = PPU_normalization_image,
    .ppu_qfix_image = PPU_qfix_image,
    .ppu_npu_pro = PPU_npu_pro,
    .ppu_init_pro = PPU_init_pro,
};

static int ann_ppu_init(struct device *dev)
{
  return 0;
}

#ifdef CONFIG_ANN

static void ann_ppu_irq_config_func_0(void);

static const struct ann_ppu_config ann_ppu_cfg_0 = {
    .ann_base_addr = PPU_ANN_BASE,
    .irq_config = ann_ppu_irq_config_func_0,
};

DEVICE_AND_API_INIT(ann_ppu_0, "ann_0", &ann_ppu_init,
                    NULL, &ann_ppu_cfg_0,
                    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                    &ann_ppu_drv_api_funcs);

static void ann_ppu_irq_config_func_0(void)
{
    IRQ_CONNECT(PPU_ANN_IRQ,
                CONFIG_ANN_IRQ_PRI,
                ann_ppu_isr,
                DEVICE_GET(ann_ppu_0),
                0);
    irq_enable(PPU_ANN_IRQ);
}
#endif