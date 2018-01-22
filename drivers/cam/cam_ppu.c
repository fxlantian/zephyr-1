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
#include "cam_ppu.h"

/* convenience defines */
#define DEV_CAM_CFG(dev)                           \
        ((struct cam_ppu_config * const)(dev)->config->config_info)
#define DEV_CAM(dev)                               \
        ((volatile struct cam_ppu_t *)(DEV_CAM_CFG(dev))->cam_base_addr)

volatile int Isr_Type;


void camctl_int_enable(struct device *dev, unsigned int fm1_int, unsigned int fm2_int, unsigned int fm3_int, unsigned int rqful_int, unsigned int rqovf_int, unsigned int err_int)
{
    volatile struct cam_ppu_t *cam = DEV_CAM(dev);
    int val = 0;

    if(fm1_int !=0)
        val = SET_FRAME1_INT_EN(val);

    if(fm2_int !=0)
        val = SET_FRAME2_INT_EN(val);

    if(fm3_int !=0)
        val = SET_FRAME3_INT_EN(val);

    if(rqful_int !=0)
        val = SET_RQFUL_INT_EN(val);

    if(rqovf_int !=0)
        val = SET_RQOVF_INT_EN(val);

    if(err_int !=0)
        val = SET_PROERR_INT_EN(val);
    *(volatile int*) (CAMCTL_INT_ENABLE) = val;
    //cam->cam_ien = val;
}

void camctl_int_disable(struct device *dev, unsigned int fm1_int, unsigned int fm2_int, unsigned int fm3_int, unsigned int rqful_int, unsigned int rqovf_int, unsigned int err_int)
{
    volatile struct cam_ppu_t *cam = DEV_CAM(dev);
    int val = 0;
    //val = cam->cam_ien;
    val = *(volatile int*) (CAMCTL_INT_ENABLE);

    if(fm1_int == UNSET_FM1_INT)
        val = UNSET_FRAME1_INT_EN(val);

    if(fm2_int == UNSET_FM2_INT)
        val = UNSET_FRAME2_INT_EN(val);

    if(fm3_int == UNSET_FM3_INT)
        val = UNSET_FRAME3_INT_EN(val);

    if(rqful_int == UNSET_RQFUL_INT)
        val = UNSET_RQFUL_INT_EN(val);

    if(rqovf_int == UNSET_RQOVF_INT)
        val = UNSET_RQOVF_INT_EN(val);

    if(err_int == UNSET_PROERR_INT)
        val = UNSET_PROERR_INT_EN(val);
    //cam->cam_ien = val;
    *(volatile int*) (CAMCTL_INT_ENABLE) = val;
}

static void PPU_cam_start(struct device *dev)
{
    volatile struct cam_ppu_t *cam = DEV_CAM(dev);
    int val;
    val = *(volatile int*) (CAMCTL_CTRL_REG);
    //val = cam->cam_ctrl;
    val = ENABLE_CAPTURE(val);
    //cam->cam_ctrl = val;
    *(volatile int*) (CAMCTL_CTRL_REG) = val;
}

static void PPU_cam_stop(struct device *dev)
{
    volatile struct cam_ppu_t *cam = DEV_CAM(dev);
    int val;
    camctl_int_disable(dev, UNSET_FM1_INT, UNSET_FM2_INT, UNSET_FM3_INT, UNSET_RQFUL_INT, UNSET_RQOVF_INT, UNSET_PROERR_INT);
    val = *(volatile int*) (CAMCTL_CTRL_REG);
    //val = cam->cam_ctrl;
    val = DISABLE_CAPTURE(val);
    *(volatile int*) (CAMCTL_CTRL_REG) = val;
    //cam->cam_ctrl = val;
}

static void PPU_cam_init(struct device *dev)
{
    volatile struct cam_ppu_t *cam = DEV_CAM(dev);
    *(volatile int*) (CAMCTL_FRAME1_ADDR)   = FRAME1_ADDR;
    *(volatile int*) (CAMCTL_FRAME2_ADDR)   = FRAME2_ADDR;
    *(volatile int*) (CAMCTL_FRAME3_ADDR)   = FRAME3_ADDR;
    //cam->cam_frame1_addr   = FRAME1_ADDR;
    //cam->cam_frame2_addr   = FRAME2_ADDR;
    //cam->cam_frame3_addr   = FRAME3_ADDR;
  
    camctl_int_enable(dev, SET_FM1_INT, SET_FM2_INT, SET_FM3_INT, SET_RQFUL_INT, SET_RQOVF_INT, SET_PROERR_INT);
}

void cam_ppu_isr(void)
{
    struct device *cam_dev = device_get_binding("cam_0");
    struct device *dev = cam_dev;
    volatile struct cam_ppu_t *cam = DEV_CAM(dev);
    int val;

    PPU_cam_stop(cam_dev);    //close the camera
    PPU_ICP |= (1 << 17);
    val = *(volatile int*) (CAMCTL_STATUS);
    //val = cam->cam_sta;

    if(DATAOK_FRAME1(val) == 1) {
        Isr_Type = 1;
        val = CLR_FRAME1_INT(val);
    }

    if(DATAOK_FRAME2(val) == 1) {
        Isr_Type = 2;
        val = CLR_FRAME2_INT(val);
    }

    if(DATAOK_FRAME3(val) == 1) {
        Isr_Type = 3;
        val = CLR_FRAME3_INT(val);
    }

    if(RQ_FUL(val) == 1) {
        Isr_Type = 4;
        val = CLR_RQFUL_INT(val);
    }

    if(RQ_OVF(val) == 1) {
        Isr_Type = 5;
        val = CLR_RQOVF_INT(val);
    }

    if(PROTOCAL_ERR0(val) == 1) {
        Isr_Type = 6;
        val = CLR_PROERR0_INT(val);
    }

    if(PROTOCAL_ERR1(val) == 1) {
        Isr_Type = 7;
        val = CLR_PROERR1_INT(val);
    }
    printk("val:%d\n", val);
    *(volatile int*) (CAMCTL_STATUS) = val;
    //cam->cam_sta = val;
    printk("Status reg:%d\n", *(volatile int*) (CAMCTL_STATUS));
}

static const struct cam_driver_api cam_ppu_drv_api_funcs = {
    .ppu_camctl_init = PPU_cam_init,
    .ppu_camctl_start = PPU_cam_start,
};


static int cam_ppu_init(struct device *dev)
{
  return 0;
}

static void cam_ppu_irq_config_func_0(void);

static const struct cam_ppu_config cam_ppu_cfg_0 = {
    .cam_base_addr = PPU_CAM_BASE,
    .irq_config = cam_ppu_irq_config_func_0,
};

DEVICE_AND_API_INIT(cam_ppu_0, "cam_0", &cam_ppu_init,
                    NULL, &cam_ppu_cfg_0,
                    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                    &cam_ppu_drv_api_funcs);

static void cam_ppu_irq_config_func_0(void)
{
    IRQ_CONNECT(PPU_CAM_IRQ,
                CONFIG_CAM_IRQ_PRI,
                cam_ppu_isr,
                DEVICE_GET(cam_ppu_0),
                0);
    irq_enable(PPU_CAM_IRQ);
}
