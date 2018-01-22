/*
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdio.h>
#include <board.h>
#include <cam.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <gpio.h>
#include <spi.h>
#include <misc/printk.h>
#include "lcd_ppu.h"
#include "oledfont.h"
#include "bmp.h"
//#include "spi_ppu.h"

/* convenience defines */
/*#define DEV_CFG(dev)                            \
    ((const struct cam_ppu_config * const)(dev)->config->config_info)
#define DEV_SPI_CFG(dev)                           \
        ((struct spi_ppu_config * const)(dev)->config->config_info)*/
#define DEV_LCD_DATA(dev)                           \
    ((struct cam_ppu_data * const)(dev)->driver_data)
#define SPI_START_TRANSACTION(trans_type, csnum)   \
        (((1 << (csnum + 8)) & 0xF00) | ((1 << trans_type) & 0xFF))


void delay_ms(volatile int delay)
{
    volatile int i;
    delay = delay * 400;
    for(i = 0; i < delay; i++);
}

static void PPU_OLED_RST_Set(struct device *dev1)
{
    struct device *gpio_dev = dev1;
    //gpio_dev = device_get_binding("GPIO_NAME");
    gpio_pin_write(gpio_dev, 31, 1);
}

static void PPU_OLED_RST_Clr(struct device *dev1)
{
    struct device *gpio_dev = dev1;
    //gpio_dev = device_get_binding("GPIO_NAME");
    gpio_pin_write(gpio_dev, 31, 0);
}

static void PPU_OLED_DC_Set(struct device *dev1)
{
    struct device *gpio_dev = dev1;
    //gpio_dev = device_get_binding("GPIO_NAME");
    gpio_pin_write(gpio_dev, 30, 1);
}

static void PPU_OLED_DC_Clr(struct device *dev1)
{
    struct device *gpio_dev = dev1;
    //gpio_dev = device_get_binding("GPIO_NAME");
    gpio_pin_write(gpio_dev, 30, 0);
}

static void PPU_LCD_WR_DATA(struct device *dev1, u32_t da)
{
    PPU_OLED_DC_Set(dev1);
    int cmd_reg;
    cmd_reg = da << 8;
    *(volatile int*) (SPI1_REG_SPICMD) = cmd_reg;
    *(volatile int*) (SPI1_REG_SPIADR) = 0;
    *(volatile int*) (SPI1_REG_SPILEN) = (24 & 0x3F) | ((0 << 8) & 0x3F00);
    *(volatile int*) (SPI1_REG_STATUS) = SPI_START_TRANSACTION(SPI_CMD_WR, SPI_CSN0);
}

static void PPU_LCD_WR_REG(struct device *dev1, u8_t da)
{
    PPU_OLED_DC_Clr(dev1);
    int cmd_reg;
    cmd_reg = da << 24;
    *(volatile int*) (SPI1_REG_SPICMD) = cmd_reg;
    *(volatile int*) (SPI1_REG_SPIADR) = 0;
    *(volatile int*) (SPI1_REG_SPILEN) = (8 & 0x3F) | ((0 << 8) & 0x3F00);
    *(volatile int*) (SPI1_REG_STATUS) = SPI_START_TRANSACTION(SPI_CMD_WR, SPI_CSN0);
}

static void PPU_LCD_WR_DATA8(struct device *dev1, u8_t da)
{
    PPU_OLED_DC_Set(dev1);
    int cmd_reg;
    cmd_reg = da << 24;
    *(volatile int*) (SPI1_REG_SPICMD) = cmd_reg;
    *(volatile int*) (SPI1_REG_SPIADR) = 0;
    *(volatile int*) (SPI1_REG_SPILEN) = (8 & 0x3F) | ((0 << 8) & 0x3F00);
    *(volatile int*) (SPI1_REG_STATUS) = SPI_START_TRANSACTION(SPI_CMD_WR, SPI_CSN0);
}

static void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2, struct device *gpio_dev)
{ 
   PPU_LCD_WR_REG(gpio_dev, 0x2a);//sety
   PPU_LCD_WR_DATA8(gpio_dev, x1>>8);
   PPU_LCD_WR_DATA8(gpio_dev, x1);
   PPU_LCD_WR_DATA8(gpio_dev, x2>>8);
   PPU_LCD_WR_DATA8(gpio_dev, x2);
  
   PPU_LCD_WR_REG(gpio_dev, 0x2b);//setx
   PPU_LCD_WR_DATA8(gpio_dev, y1>>8);
   PPU_LCD_WR_DATA8(gpio_dev, y1);
   PPU_LCD_WR_DATA8(gpio_dev, y2>>8);
   PPU_LCD_WR_DATA8(gpio_dev, y2);

   PPU_LCD_WR_REG(gpio_dev, 0x2C);    //wram
}

static void cam_ppu_lcd_clear(u32_t color)
{
    u32_t i,j;
    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");    
    Address_set(0,0,LCD_H-1,LCD_W-1, gpio_dev);
    PPU_OLED_DC_Set(gpio_dev);
    for(i=0;i<LCD_W;i++)
     {
      for (j=0;j<LCD_H;j++)
       {
            PPU_LCD_WR_DATA(gpio_dev, color);
        }

      }
}

static void cam_ppu_display_shade(struct device *dev, u32_t x1)
{
    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    u32_t i,j; 
    u32_t pixel;
    u8_t b;
    Address_set(x1,10,x1 + 34, 309, gpio_dev);
    for(i = 0; i < 300; i++){
       b=255;
       for (j=0;j<35;j++){  
        if(b>100)
        b-=15;
        else if(b>6)
        b-=5;
        pixel = ((b << 16) | 0x0000);
        PPU_LCD_WR_DATA(gpio_dev, pixel);
        }
    }
}

static void cam_ppu_display_image(struct device *dev, u32_t x1, u32_t y1, u32_t x2, u32_t y2, volatile unsigned char* image_addr)
{
    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    u32_t i,j,xl,yl; 
    u32_t pixel;
    Address_set(x1,y1,x2,y2, gpio_dev);
    //Address_set(0,10,99,109);
    xl = x2 - x1;
    yl = y2 - y1;
    for(i=0;i<yl;i++){
       for (j=0;j<xl;j++){    
    pixel = (((*(image_addr)) << 16) | ((*(image_addr + 1)) << 8) | (*(image_addr + 2)));
        //LCD_WR_DATA8(*image_addr);
        //LCD_WR_DATA8(*(image_addr + 1));
    PPU_LCD_WR_DATA(gpio_dev, pixel);
    image_addr += 3;
        }
    }
}

static void cam_ppu_display_cam(struct device *dev, volatile unsigned char* image_addr)
{
    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    u16_t i,j;        
    //u32 pixel;
    Address_set(0,0,LCD_H-1,LCD_W-1,gpio_dev);
    PPU_OLED_DC_Set(gpio_dev);
    volatile unsigned char* image_addr_ver = image_addr;
    for(i=0;i<320;i++)
     {
    image_addr_ver = image_addr + i*1280;
      for (j=0;j<480;j++)
       {
        //pixel = ((*(image_addr_ver + 1) << 3) << 16) | ((((*image_addr_ver) << 5) | ((*(image_addr_ver + 1) & 0xE0) >> 3))<< 8) | (((*image_addr_ver) & 0xF8));
        
        PPU_LCD_WR_DATA8(gpio_dev, (*(image_addr_ver + 1) << 3));
               PPU_LCD_WR_DATA8(gpio_dev, (((*image_addr_ver) << 5) | ((*(image_addr_ver + 1) & 0xE0) >> 3)));
        PPU_LCD_WR_DATA8(gpio_dev, ((*image_addr_ver) & 0xF8));

        //LCD_WR_DATA(pixel);

        image_addr_ver+=2;               
        }

      }
}

 static void cam_ppu_printf_lcd(const char* image_addr, struct device *dev)
{
    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    if(dis_line > 300)
    {   
        LCD_Fill(gpio_dev, 150,0,479,319,WHITE);
        dis_line = 15;
    }
    LCD_ShowString(150,dis_line,image_addr, gpio_dev);  //cam
    //LCD_ShowString(0,dis_line,image_addr, gpio_dev);  //shell
    dis_line += 20;
}

void LCD_Fill(struct device *gpio_dev, u32_t xsta,u32_t ysta,u32_t xend,u32_t yend,u32_t color)
{          
    u32_t i,j; 
    Address_set(xsta,ysta,xend,yend, gpio_dev);      //设置光标位置 
    for(i=ysta;i<=yend;i++)
    {                                                           
        for(j=xsta;j<=xend;j++)PPU_LCD_WR_DATA(gpio_dev, color);//设置光标位置      
    }                           
} 

void LCD_ShowString(u32_t x,u32_t y,const u8_t *p, struct device *gpio_dev)
{         
    while(*p!='\0')
    {       
        if(x>LCD_H-16){x=0;y+=16;}
        if(y>LCD_W-16){y=x=0;cam_ppu_lcd_clear(BACK_COLOR);
        }
        LCD_ShowChar(gpio_dev, x,y,*p,0);
        x+=8;
        p++;
    }  
}

void LCD_ShowChar(struct device *gpio_dev, u32_t x,u32_t y,u8_t num,u8_t mode)
{ 
    u8_t temp;
    u8_t pos,t;
    u32_t x0=x;
    u32_t colortemp=POINT_COLOR;      
    if(x>LCD_H-16||y>LCD_W-16)return;       
    //设置窗口         
    num=num-' ';//得到偏移后的值
    Address_set(x,y,x+8-1,y+16-1,gpio_dev);      //设置光标位置 
    if(!mode) //非叠加方式
    {
        for(pos=0; pos<16;pos++)
        { 
            temp=aasc2_1608[(u32_t)num*16+pos];         //调用1608字体
            for(t=0;t<8;t++)
            {                 
                if(temp&0x01)POINT_COLOR=colortemp;
                else POINT_COLOR=BACK_COLOR;
                PPU_LCD_WR_DATA(gpio_dev, POINT_COLOR);   
                temp>>=1; 
                x++;
            }
            x=x0;
            y++;
        }   
    }else//叠加方式
    {
        for(pos=0;pos<16;pos++)
        {
            temp=aasc2_1608[(u32_t)num*16+pos];         //调用1608字体
            for(t=0;t<8;t++)
            {                 
                if(temp&0x01)LCD_DrawPoint(gpio_dev, x+t,y+pos);//画一个点     
                temp>>=1; 
            }
        }
    }
    POINT_COLOR=colortemp;
}

static void LCD_DrawLine(u32_t x1, u32_t y1, u32_t x2, u32_t y2)
{ 
    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    u32_t t; 
    u32_t xerr=0,yerr=0,delta_x,delta_y,distance; 
    u32_t incx,incy,uRow,uCol; 

    delta_x=x2-x1; //计算坐标增量 
    delta_y=y2-y1; 
    uRow=x1; 
    uCol=y1; 
    if(delta_x>0)incx=1; //设置单步方向 
    else if(delta_x==0)incx=0;//垂直线 
    else {incx=-1;delta_x=-delta_x;} 
    if(delta_y>0)incy=1; 
    else if(delta_y==0)incy=0;//水平线 
    else{incy=-1;delta_y=-delta_y;} 
    if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
    else distance=delta_y; 
    for(t=0;t<=distance+1;t++ )//画线输出 
    {  
        LCD_DrawPoint(gpio_dev,uRow,uCol);//画点 
        xerr+=delta_x ; 
        yerr+=delta_y ; 
        if(xerr>distance) 
        { 
            xerr-=distance; 
            uRow+=incx; 
        } 
        if(yerr>distance) 
        { 
            yerr-=distance; 
            uCol+=incy; 
        } 
    }  
}    
//画矩形
static void cam_lcd_drawrectangle(struct device *dev, u32_t x1, u32_t y1, u32_t x2, u32_t y2)
{
    LCD_DrawLine(x1,y1,x2,y1);
    LCD_DrawLine(x1,y1,x1,y2);
    LCD_DrawLine(x1,y2,x2,y2);
    LCD_DrawLine(x2,y1,x2,y2);
} 

void LCD_DrawPoint(struct device *gpio_dev, u32_t x,u32_t y)
{
    Address_set(x,y,x,y, gpio_dev);//设置光标位置 
    PPU_LCD_WR_DATA(gpio_dev, POINT_COLOR);       
} 

static int cam_ppu_lcd_config(struct device *dev, struct device *dev1)
{
    gpio_pin_configure(dev1, 30, GPIO_DIR_OUT);
    gpio_pin_configure(dev1, 31, GPIO_DIR_OUT);
    gpio_pin_write(dev1, 30, 1);
    gpio_pin_write(dev1, 31, 1);
    *(volatile int*)(SPI1_REG_CLKDIV) = 0x01;
    PPU_OLED_RST_Set(dev1);
    delay_ms(20);
    PPU_OLED_RST_Clr(dev1);
    delay_ms(120);
    PPU_OLED_RST_Set(dev1);
    delay_ms(120);
    PPU_LCD_WR_REG(dev1, 0XF2);
    PPU_LCD_WR_DATA8(dev1, 0x18);
    PPU_LCD_WR_DATA8(dev1, 0xA3);
    PPU_LCD_WR_DATA8(dev1, 0x12);
    PPU_LCD_WR_DATA8(dev1, 0x02);
    PPU_LCD_WR_DATA8(dev1, 0XB2);
    PPU_LCD_WR_DATA8(dev1, 0x12);
    PPU_LCD_WR_DATA8(dev1, 0xFF);
    PPU_LCD_WR_DATA8(dev1, 0x10);
    PPU_LCD_WR_DATA8(dev1, 0x00);
    PPU_LCD_WR_REG(dev1, 0XF8);
    PPU_LCD_WR_DATA8(dev1, 0x21);
    PPU_LCD_WR_DATA8(dev1, 0x04);
    PPU_LCD_WR_REG(dev1, 0X13);
    PPU_LCD_WR_REG(dev1, 0x36);
    PPU_LCD_WR_DATA8(dev1, 0x60);
    PPU_LCD_WR_REG(dev1, 0xB4);
    PPU_LCD_WR_DATA8(dev1, 0x02);
    PPU_LCD_WR_REG(dev1, 0xB6);
    PPU_LCD_WR_DATA8(dev1, 0x02);
    PPU_LCD_WR_DATA8(dev1, 0x22);
    PPU_LCD_WR_REG(dev1, 0xC1);
    PPU_LCD_WR_DATA8(dev1, 0x41);
    PPU_LCD_WR_REG(dev1, 0xC5);
    PPU_LCD_WR_DATA8(dev1, 0x00);
    PPU_LCD_WR_DATA8(dev1, 0x18);
    PPU_LCD_WR_REG(dev1, 0x3a);
    PPU_LCD_WR_DATA8(dev1, 0x66);
    delay_ms(50);
    PPU_LCD_WR_REG(dev1, 0xE0);
    PPU_LCD_WR_DATA8(dev1, 0x0F);
    PPU_LCD_WR_DATA8(dev1, 0x1F);
    PPU_LCD_WR_DATA8(dev1, 0x1C);
    PPU_LCD_WR_DATA8(dev1, 0x0C);
    PPU_LCD_WR_DATA8(dev1, 0x0F);
    PPU_LCD_WR_DATA8(dev1, 0x08);
    PPU_LCD_WR_DATA8(dev1, 0x48);
    PPU_LCD_WR_DATA8(dev1, 0x98);
    PPU_LCD_WR_DATA8(dev1, 0x37);
    PPU_LCD_WR_DATA8(dev1, 0x0A);
    PPU_LCD_WR_DATA8(dev1, 0x13);
    PPU_LCD_WR_DATA8(dev1, 0x04);
    PPU_LCD_WR_DATA8(dev1, 0x11);
    PPU_LCD_WR_DATA8(dev1, 0x0D);
    PPU_LCD_WR_DATA8(dev1, 0x00);
    PPU_LCD_WR_REG(dev1, 0xE1);
    PPU_LCD_WR_DATA8(dev1, 0x0F);
    PPU_LCD_WR_DATA8(dev1, 0x32);
    PPU_LCD_WR_DATA8(dev1, 0x2E);
    PPU_LCD_WR_DATA8(dev1, 0x0B);
    PPU_LCD_WR_DATA8(dev1, 0x0D);
    PPU_LCD_WR_DATA8(dev1, 0x05);
    PPU_LCD_WR_DATA8(dev1, 0x47);
    PPU_LCD_WR_DATA8(dev1, 0x75);
    PPU_LCD_WR_DATA8(dev1, 0x37);
    PPU_LCD_WR_DATA8(dev1, 0x06);
    PPU_LCD_WR_DATA8(dev1, 0x10);
    PPU_LCD_WR_DATA8(dev1, 0x03);
    PPU_LCD_WR_DATA8(dev1, 0x24);
    PPU_LCD_WR_DATA8(dev1, 0x20);
    PPU_LCD_WR_DATA8(dev1, 0x00);
    PPU_LCD_WR_REG(dev1, 0x11);
    delay_ms(120);
    PPU_LCD_WR_REG(dev1, 0x29);
    PPU_LCD_WR_REG(dev1, 0x2C);
    cam_ppu_lcd_clear(BLACK);
    return 0;
}

static const struct cam_driver_api cam_ppu_drv_api_funcs = {
    .ppu_lcd_config = cam_ppu_lcd_config,
    .ppu_display_image = cam_ppu_display_image,
    .ppu_display_shade = cam_ppu_display_shade,
    .ppu_display_cam = cam_ppu_display_cam,
    .ppu_printf_lcd = cam_ppu_printf_lcd,
    .ppu_lcd_drawrectangle = cam_lcd_drawrectangle,
};


static int lcd_ppu_init(struct device *dev)
{
  /*struct cam_ppu_data *data = dev->driver_data;
  struct spi_config *config = &data->config;
  struct device *spi;

  spi = device_get_binding("SPI_1");
  if (!spi) {
    return -ENODEV;
  }
  config->dev = spi;
  config->frequency = 500000;
  config->operation = SPI_OP_MODE_MASTER | SPI_LINES_SINGLE;
  config->slave = 0;
  config->cs = NULL;*/

  return 0;
}


//static struct cam_ppu_data cam_data;

DEVICE_AND_API_INIT(cam_ppu_0, "lcd_0", &lcd_ppu_init,
                    NULL, NULL,
                    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                    &cam_ppu_drv_api_funcs);

//#endif /* CONFIG_PPU_SPI_0 */
