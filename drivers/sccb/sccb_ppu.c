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
#include <i2c.h>
#include <misc/printk.h>

/* convenience defines */
/*#define DEV_CFG(dev)                            \
    ((const struct cam_ppu_config * const)(dev)->config->config_info)
#define DEV_SPI_CFG(dev)                           \
        ((struct spi_ppu_config * const)(dev)->config->config_info)*/
//#define DEV_DATA(dev)                           \
    ((struct cam_ppu_data * const)(dev)->driver_data)

static union dev_config i2c_cfg = {
    .raw = 0,
    .bits = {
        .use_10_bit_addr = 0,
        .is_master_device = 0,
        .speed = I2C_SPEED_STANDARD,
    },
};

static u8_t PPU_sccb_write(struct device *dev, u8_t WriteAddress, u8_t SendByte)
{
    unsigned char datas[40] = {0};
    datas[0] = WriteAddress;
    datas[1] = SendByte;
    struct device *i2c_dev = device_get_binding("i2c0");
    i2c_write(i2c_dev, datas, 2, 0x42);
    return 1;
}

static void PPU_sccb_read(struct device *dev, u8_t ReadAddress)
{
    u8_t rdatas[40] = {0};
    unsigned char datas[40] = {0};
    datas[0] = ReadAddress;
    struct device *i2c_dev = device_get_binding("i2c0");
    i2c_write(i2c_dev, NULL, 0, 0x42);
    i2c_write(i2c_dev, datas, 1, 0x42);
    i2c_read(i2c_dev, rdatas, 1, 0x43);
    for(int i = 0; i < 1; i++)
    {
        printk("SCCB receive is: %x\n", rdatas[i]);
    }
}

static void PPU_sccb_init(struct device *dev)
{
    printk("i2c init for SCCB.\n");
    struct device *i2c_dev = device_get_binding("i2c0");
    int ret;
    ret = i2c_configure(i2c_dev, i2c_cfg.raw);
    if (ret) {
    printk("I2C config failed\n");
    return;
    }
    printk("configure over\n");
}

static const struct cam_driver_api sccb_ppu_drv_api_funcs = {
    .ppu_sccb_init = PPU_sccb_init,
    .ppu_sccb_write = PPU_sccb_write,
    .ppu_sccb_read = PPU_sccb_read,
};


static int sccb_ppu_init(struct device *dev)
{
  return 0;
}

DEVICE_AND_API_INIT(sccb_ppu_0, "sccb", &sccb_ppu_init,
                    NULL, NULL,
                    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                    &sccb_ppu_drv_api_funcs);
