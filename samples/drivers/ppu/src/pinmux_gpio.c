#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <pinmux.h>
#include <gpio.h>
#include <misc/util.h>

#define PINMUX_DRV_NAME "pinmux_0"
#define GPIO_FUNC 1
#define GPIO_OUT_PIN 1
#define GPIO_INT_PIN 2
void waste_time(void);
void gpio_callback(struct device *port,
		   struct gpio_callback *cb, u32_t pins)
{
	printk("gpio0 triggered\n");
}

static struct gpio_callback gpio_cb;

void main(void)
{
    printk("Find device...\n");

    struct device *pinmux_dev;

    pinmux_dev = device_get_binding(PINMUX_DRV_NAME);
    if(!pinmux_dev)
    {
        printk("Cannot find %s!\n", PINMUX_DRV_NAME);
        return;
    }
    
    printk("Find %s!\n", PINMUX_DRV_NAME);

    pinmux_pin_set(pinmux_dev, GPIO_OUT_PIN, GPIO_FUNC);
    pinmux_pin_set(pinmux_dev, 3, GPIO_FUNC);
    printk("success\n");
    u32_t func0 = 0;
    u32_t func1 = 0;
    
    pinmux_pin_get(pinmux_dev, GPIO_OUT_PIN, &func0);
    pinmux_pin_get(pinmux_dev, 3, &func1);

    if(func0 && func1)
        printk("pin function set success\n");
    else
        printk("pin function set failed\n");   

    
    struct device *gpio_dev;
    gpio_dev = device_get_binding("gpio0");
    if(!gpio_dev)
    {
        printk("Cannot find %s!\n", "gpio0");
        return;
    }

    printk("Find gpio0!\n");

    int ret = gpio_pin_configure(gpio_dev, GPIO_OUT_PIN, GPIO_DIR_OUT);
    if(ret)
    {
        printk("Error configuring gpio pin0\n");
        return;
    }
   
    //ret = gpio_pin_configure(gpio_dev, GPIO_INT_PIN, (GPIO_DIR_IN | GPIO_INT |
    //                                                  GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH));
    //if(ret)
    //{
    //    printk("Error configuring gpio pin1\n");
    //    return;
    //}

    //gpio_init_callback(&gpio_cb, gpio_callback, BIT(GPIO_INT_PIN));

    //ret = gpio_add_callback(gpio_dev, &gpio_cb);
    //if (ret) {
 	//printk("Cannot setup callback!\n");
    //}

    //ret = gpio_pin_enable_callback(gpio_dev, GPIO_INT_PIN);
    //if (ret) {
	//printk("Error enabling callback!\n");
    //}
 
    int toggle =1;
    while(1)
    {
        ret = gpio_pin_write(gpio_dev, GPIO_OUT_PIN, toggle);
        if(ret)
        {
            printk("Error setting gpio\n");
            return;
        }
        if(toggle) toggle = 0;
        else       toggle = 1;
        //k_sleep(MSEC_PER_SEC);
        waste_time();
    }
}

void waste_time(void) {
     volatile int i;
     for(i = 0; i < 300000; i++);
  //  for(i = 0; i < 300; i++) asm volatile("nop");
 }









