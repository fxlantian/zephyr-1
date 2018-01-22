#include <misc/printk.h>
#include <device.h>
#include <pwm.h>
#include <misc/util.h>

#define PWM_DRV_NAME "pwm_0"
//#define GPIO_FUNC 1
//#define GPIO_OUT_PIN 1
//#define GPIO_INT_PIN 2
void waste_time(void);

void main(void)
{
    printk("This is Zephyr PWM_PPU Driver Test...\n");
    printk("Find device...\n");

    struct device *pwm_dev;

    pwm_dev = device_get_binding(PWM_DRV_NAME);
    if(!pwm_dev)
    {
        printk("Cannot find %s!\n", PWM_DRV_NAME);
        //return;
    }
    
    printk("Find %s!\n", PWM_DRV_NAME);

    int ret;
    ret = pwm_pin_set_cycles(pwm_dev, PWM_DRV_NAME, 1000000, 25000);
    if(ret) {
        printk("SPI transceive error\n");
        return;
    }
    printk("SPI transceive success\n");

    while (1){
    pwm_pin_set_cycles(pwm_dev, PWM_DRV_NAME, 1000000, 25000);
    waste_time();
    pwm_pin_set_cycles(pwm_dev, PWM_DRV_NAME, 1000000, 50000);
    waste_time();
    pwm_pin_set_cycles(pwm_dev, PWM_DRV_NAME, 1000000, 75000);
    waste_time();
    pwm_pin_set_cycles(pwm_dev, PWM_DRV_NAME, 1000000, 100000);
    waste_time();
    pwm_pin_set_cycles(pwm_dev, PWM_DRV_NAME, 1000000, 125000);
    waste_time();
}
   return 0;
}

void waste_time(void) {
     volatile int i;
     for(i = 0; i < 300000; i++);
  //  for(i = 0; i < 300; i++) asm volatile("nop");
 }


