#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <pinmux.h>
#include <i2c.h>
#include <misc/util.h>

#define I2C_FUNC 0

static union dev_config i2c_cfg = {
	.raw = 0,
	.bits = {
		.use_10_bit_addr = 0,
		.is_master_device = 0,
		.speed = I2C_SPEED_STANDARD,
	},
};

void main()
{
    printk("This is Zephyr I2C_PPU Driver test... \n");
    struct device *i2c_dev = device_get_binding("i2c0");
    if(!i2c_dev) {
        printk("Cannot find i2c0\n");
    }
    printk("find i2c0!!\n");

    int ret;
    ret = i2c_configure(i2c_dev, i2c_cfg.raw);
    if (ret) {
	printk("I2C config failed\n");
	return;
    }
    printk("configure over\n");

//    i2c_transfer(i2c_dev, NULL, 0, 0x00);
//    printk("i2c transceive over\n");
 
//    unsigned char datas[20] = {0};
//    unsigned char rdatas[20] = {0};
   
//    for(int i = 0; i < 10; i++)
//        datas[i] = i;

//    i2c_write(i2c_dev, datas, 16, 0xA0);
//    printk("data send over\n");

//    i2c_read(i2c_dev, rdatas, 16, 0xA1);
//    printk("read data over\n");

    
    unsigned char datas[40] = {0};
    unsigned char rdatas[40] = {0};
    unsigned char rrdatas[40] = {0};

    datas[0] = 0x00;
//    datas[1] = 0x00;

    for(int i = 0; i < 12; i++)
        datas[i+1] = 2*(i + 1);

   
    i2c_write(i2c_dev, datas, 13, 0xA0);
    printk("data send over\n");

    i2c_write(i2c_dev, NULL, 0, 0xA0);
    printk("acknowledge poll over\n");

    i2c_write(i2c_dev, datas, 1, 0xA0);
    printk("read data addr send over\n");
    
    i2c_read(i2c_dev, rdatas, 12, 0xA1);
    printk("read data over\n");

    for(int i = 0; i < 12; i++)
    {
        printk("data %d is: %d\n", 2*(i+1), rdatas[i]);
    }


    for(int i = 0; i < 16; i++)
        datas[i+1] = 3*(i + 1);

    i2c_write(i2c_dev, datas, 17, 0xA0);
    printk("2data send over\n");

    i2c_write(i2c_dev, NULL, 0, 0xA0);
    printk("acknowledge poll over\n");

    i2c_write(i2c_dev, datas, 1, 0xA0);
    printk("2read data addr send over\n");
    
    i2c_read(i2c_dev, rrdatas, 17, 0xA1);
    printk("2read data over\n");

    for(int i = 0; i < 16; i++)
    {
        printk("2data %d is: %d\n", 3*(i+1), rrdatas[i]);
    }

}


