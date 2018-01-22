#include <misc/printk.h>
#include <device.h>
#include <cam.h>
#include <gpio.h>
#include <spi.h>
#include <misc/util.h>
#include "testANN.h"
#include <ann.h>
#include <stdio.h>
#include <stdlib.h>

#define ANN_DRV_NAME "ann_0"
#define LCD_DRV_NAME "lcd_0"

#define LCD_W 320
#define LCD_H 480
#define WHITE            0xFCFCFC
#define BLACK            0x000000     
#define BLUE             0xFC0000  
#define RED              0x0000FC
#define GREEN            0x00FC00
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define MAGENTA          0xF81F
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40 //棕色
#define BRRED            0XFC07 //棕红色
#define GRAY             0X8430 //灰色

#define FRAME1_ADDR       0x32020000
#define FRAME2_ADDR       0x33040000
#define FRAME3_ADDR       0x34060000

volatile int count = 0;
int* Dst_Addr;
volatile int Isr_Type;
unsigned char image_sampled[28*28];
unsigned char image_normal[28*28];

u8_t cmd = 0x9F;
u8_t cmd1 = 0x06;
u8_t cmd2 = 0x71;
u32_t addr = 0x80000410;
u32_t data = 0;

// macro constants
#define   SET_FM1_INT   1
#define UNSET_FM1_INT   0

#define   SET_FM2_INT   1
#define UNSET_FM2_INT   0

#define   SET_FM3_INT   1
#define UNSET_FM3_INT   0

#define   SET_RQFUL_INT   1
#define UNSET_RQFUL_INT   0

#define   SET_RQOVF_INT   1
#define UNSET_RQOVF_INT   0

#define   SET_PROERR_INT   1
#define UNSET_PROERR_INT   0

//u32_t dis_line = 15;
volatile int Isr_Type;
u32_t BACK_COLOR, POINT_COLOR;
struct device *lcd_dev;

void check_result(char* Dst_Addr)
{
    lcd_dev = device_get_binding(LCD_DRV_NAME);
    int test_i;
    signed char res=0;
    signed char big_last=0;
    signed char big_num=0;
    
    for(test_i=0;test_i<10;test_i++)
    {
      res = *(Dst_Addr + test_i);
      printk("%02x ",res);
      if(res > big_last){
        big_num = test_i;
        big_last = res;
      }
    }
    printk("big_num:%x\n",big_num);
    if(big_num==0)
     printf_lcd("NUMBER: 0", lcd_dev);
    else if(big_num==1)
     printf_lcd("NUMBER: 1", lcd_dev);
   else if(big_num==2)
     printf_lcd("NUMBER: 2", lcd_dev);
   else if(big_num==3)
     printf_lcd("NUMBER: 3", lcd_dev);
   else if(big_num==4)
     printf_lcd("NUMBER: 4", lcd_dev);
   else if(big_num==5)
     printf_lcd("NUMBER: 5", lcd_dev);
   else if(big_num==6)
     printf_lcd("NUMBER: 6", lcd_dev);
   else if(big_num==7)
     printf_lcd("NUMBER: 7", lcd_dev);
   else if(big_num==8)
     printf_lcd("NUMBER: 8", lcd_dev);
   else if(big_num==9)
     printf_lcd("NUMBER: 9", lcd_dev);
}

void main(void)
{
    printk("This is Zephyr ANN_PPU Driver Test...\n");
    printk("Find device...\n");

    struct device *ann_dev;

    ann_dev = device_get_binding(ANN_DRV_NAME);
    if(!ann_dev)
    {
        printk("Cannot find %s!\n", ANN_DRV_NAME);
        //return;
    }
    
    printk("Find %s!\n", ANN_DRV_NAME);

    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    if(!gpio_dev)
    {
        printk("Cannot find %s!\n", "gpio0");
        return;
    }
    printk("Find gpio0!\n");

    lcd_dev = device_get_binding(LCD_DRV_NAME);
    if(!lcd_dev)
    {
        printk("Cannot find %s!\n", LCD_DRV_NAME);
        //return;
    }
    
    printk("Find %s!\n", LCD_DRV_NAME);

    int ret;
    ret = lcd_config(lcd_dev, gpio_dev);
    if(ret) {
        printk("LCD config error\n");
        //return;
    }
    printk("LCD config success\n");

    BACK_COLOR=BLACK;
    POINT_COLOR=WHITE;
    LCD_DrawRectangle(lcd_dev,49, 49, 106, 106);

    printk("Zephyr LCD_PPU Driver Test Successed.....\n");

    //Dst_Addr = (int*)malloc(sizeof(int) * NPU_DATAOUT_DEPTH0);
    Dst_Addr = (int*)0x50010000;
    printk("this is %x\n", Dst_Addr);
    struct device *sccb_dev = device_get_binding("sccb");
    if(!sccb_dev) {
        printk("Cannot find sccb\n");
    }
    printk("find sccb0!!\n");

    SCCB_init(sccb_dev);
    SCCB_WriteByte(sccb_dev, 0x12, 0x80);
    SCCB_WriteByte(sccb_dev, 0x11, 0x01);
    SCCB_WriteByte(sccb_dev, 0x0d, 0x00);
    SCCB_WriteByte(sccb_dev, 0x12, 0x40);

    SCCB_ReadByte(sccb_dev, 0x12);
    SCCB_ReadByte(sccb_dev, 0x0d);
    printk("Zephyr SCCB Driver Test Successed.....\n");

    struct device *cam_dev = device_get_binding("cam_0");
    if(!cam_dev) {
        printk("Cannot find cam_0\n");
    }
    printk("find cam_0!!\n");
    Isr_Type = 0;
    camctl_init(cam_dev);
    printk("Camera init done!!!\n");

    //int_enable();
    PPU_IER |= (1<< 16); //enable ann interrupt
    init_pro(ann_dev, IM_DEPTH0, WEIGTH_DEPTH0, BIAS_DEPTH0, (int)&Ann_Config, DMA_BLOCK_INFO0);
    printk("ANN init done!!!\n");

    camctl_start(cam_dev);
    while(1){
       if(Isr_Type != 0)
        {
           if(Isr_Type == 1) {
                sample_image(ann_dev, (volatile unsigned char*)FRAME1_ADDR, image_sampled);
                normalization_image(ann_dev, image_sampled);
                qfix_image(ann_dev, image_sampled, (char*)&data_input);
                npu_pro(ann_dev, NPU_DATAIN_DEPTH0, NPU_DATAOUT_DEPTH0, (int)&data_input,  (int)Dst_Addr, NPU_DMA_BLOCK_INFO0 ); 
                while(count!=2) {
                  }
                count=0;
                check_result((char*)Dst_Addr);
            }
            else if(Isr_Type == 2){
                sample_image(ann_dev, (volatile unsigned char*)FRAME2_ADDR,image_sampled);
                normalization_image(ann_dev, image_sampled);
                qfix_image(ann_dev, image_sampled, (char*)&data_input);
                npu_pro(ann_dev, NPU_DATAIN_DEPTH0, NPU_DATAOUT_DEPTH0, (int)&data_input,  (int)Dst_Addr, NPU_DMA_BLOCK_INFO0 ); 
                while(count!=2) {
                 }
                count=0;
                check_result((char*)Dst_Addr);
            }
            else if(Isr_Type == 3){
                sample_image(ann_dev, (volatile unsigned char*)FRAME3_ADDR,image_sampled);
                normalization_image(ann_dev, image_sampled);
                qfix_image(ann_dev, image_sampled, (char*)&data_input);
                npu_pro(ann_dev, NPU_DATAIN_DEPTH0, NPU_DATAOUT_DEPTH0, (int)&data_input,  (int)Dst_Addr, NPU_DMA_BLOCK_INFO0 ); 
                while(count!=2) {
                  }
                count=0;
                check_result((char*)Dst_Addr);
            }
        Isr_Type = 0;
        camctl_int_enable(SET_FM1_INT, SET_FM2_INT, SET_FM3_INT, UNSET_RQFUL_INT, UNSET_RQOVF_INT, SET_PROERR_INT);
        camctl_start(cam_dev); //open the camera
        }
    }
  return 0;
}
