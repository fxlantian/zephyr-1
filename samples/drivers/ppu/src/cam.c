#include <misc/printk.h>
#include <device.h>
#include <cam.h>
#include <gpio.h>
#include <spi.h>
#include <misc/util.h>
#include "oledfont.h"
#include "bmp.h"
//#include <int.h>

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

void waste_time(volatile int delay);

struct spi_config spi0 = {
    .frequency = 5000000,
    .operation = SPI_OP_MODE_MASTER | SPI_LINES_SINGLE,
    .slave = 0,
    .cs = NULL,
};


void main(void)
{
    printk("This is Zephyr CAM_PPU Driver Test...\n");
    printk("Find device...\n");

    struct device *lcd_dev;

    lcd_dev = device_get_binding(LCD_DRV_NAME);
    if(!lcd_dev)
    {
        printk("Cannot find %s!\n", LCD_DRV_NAME);
        //return;
    }
    
    printk("Find %s!\n", LCD_DRV_NAME);

    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    if(!gpio_dev)
    {
        printk("Cannot find %s!\n", "gpio0");
        return;
    }
    printk("Find gpio0!\n");

    int ret;
    ret = lcd_config(lcd_dev, gpio_dev);
    if(ret) {
        printk("LCD config error\n");
        //return;
    }
    printk("LCD config success\n");

    BACK_COLOR=BLACK;
    POINT_COLOR=WHITE;
    display_shade(lcd_dev, 110);
    display_image(lcd_dev, 5, 100, 105 - 1, 200 - 1, (volatile unsigned char*)logo);
    printf_lcd("ICT WuSystem!", lcd_dev);
    POINT_COLOR = GREEN;
    printf_lcd("      U", lcd_dev);                                                      
    printf_lcd("  UUUUUUUU       UUUUUUUU    UUU    UUU", lcd_dev);
    printf_lcd(" U    U   U       U      U    U      U", lcd_dev);
    printf_lcd("U     U    U      U      U    U      U", lcd_dev);
    printf_lcd("U     U    U      U     U     U      U", lcd_dev);
    printf_lcd(" UU   U   UU      U Chip      U      U", lcd_dev);
    printf_lcd("   UUUUUUU        U     5G    U      U", lcd_dev);
    printf_lcd("      U   RISC-V  U      AI   U      U", lcd_dev);
    printf_lcd("      UU         UUUUUUU  IOT  UUUUUU", lcd_dev);
    POINT_COLOR=WHITE;
    printk("Zephyr LCD_PPU Driver Test Successed.....\n");

    struct device *sccb_dev = device_get_binding("sccb");
    if(!sccb_dev) {
        printk("Cannot find sccb\n");
    }
    printk("find sccb0!!\n");

    SCCB_init(sccb_dev);
    SCCB_WriteByte(sccb_dev, 0x12, 0x80);
    SCCB_WriteByte(sccb_dev, 0x11, 0x01);
    SCCB_WriteByte(sccb_dev, 0x0d, 0x00);
    SCCB_WriteByte(sccb_dev, 0x12, 0x06);

    SCCB_ReadByte(sccb_dev, 0x18);
    SCCB_ReadByte(sccb_dev, 0x1A);
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
    PPU_IER |= (1<< 17); //enable camera interrupt
    PPU_EER |= (1<< 17); //enable camera interrupt
    printk("INT_CAM init done!!!\n");

    camctl_start(cam_dev);
    while(1){
     if(Isr_Type != 0)
    {
       printk("Get in ISR, Service Type: %d\n", Isr_Type);
           if(Isr_Type == 1) {
                display_cam(lcd_dev, (volatile unsigned char*)FRAME1_ADDR);
        }
       else if(Isr_Type == 2){
               display_cam(lcd_dev, (volatile unsigned char*)FRAME2_ADDR);
        }
       else if(Isr_Type == 3){
               display_cam(lcd_dev, (volatile unsigned char*)FRAME3_ADDR);
        }
          Isr_Type = 0;
          camctl_int_enable(cam_dev, SET_FM1_INT, SET_FM2_INT, SET_FM3_INT, UNSET_RQFUL_INT, UNSET_RQOVF_INT, SET_PROERR_INT);
          camctl_start(cam_dev);   //open the camera
    }
}
  return 0;
}

void waste_time(volatile int delay)
{
    volatile int i;
    delay = delay * 400;
    for(i = 0; i < delay; i++);
}