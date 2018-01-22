/*
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <board.h>
#include <pwm.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <misc/printk.h>
#include "pwm_ppu.h"

#define CHANNEL_LENGTH 4

/* convenience defines */
#define DEV_CFG(dev)                            \
    ((const struct pwm_ppu_config * const)(dev)->config->config_info)
#define DEV_DATA(dev)                           \
    ((struct pwm_ppu_data * const)(dev)->driver_data)

/*
 * Set the period and pulse width for a PWM pin.
 *
 * Parameters
 * dev: Pointer to PWM device structure
 * pwm: PWM channel to set
 * period_cycles: Period (in timer count)
 * pulse_cycles: Pulse width (in timer count).
 *
 * return 0, or negative errno code
 */
static int pwm_ppu_pin_set(struct device *dev, u32_t pwm,
                 u32_t period_cycles, u32_t pulse_cycles)
{
    //struct pwm_ppu_data *data = DEV_DATA(dev);
    //volatile struct pwm_ppu_t * pwm_0 = DEV_SPI(dev);
    //printk("successs\n");
    const struct pwm_ppu_config * const cfg = DEV_CFG(dev);
    u32_t channel;
    channel = (pwm - 1)*CHANNEL_LENGTH;

    if (period_cycles == 0 || pulse_cycles > period_cycles) {
        return -EINVAL;
    }

    /* configure channel */
    PPWM0_CTRL = (PPWM0_CTRL & ~(0x7 << 3)) | (0 & 0x7) << 3;

    PPWM0_TIMER = 0;
    PPWM0_CTRL = PPWM0_CTRL | 0x1;

    PPWM0_LOAD = period_cycles;
    PPWM0_CMP = pulse_cycles;
/*while(1)
{
    printk("PWM over.....\n");
  for(i = 0; i < 3000000; i++) // 0
  pulse_cycles = pulse_cycles + 25000;
  PPWM0_CMP = pulse_cycles;

  for(i = 0; i < 3000000; i++) // 45
  pulse_cycles = pulse_cycles + 25000;
  PPWM0_CMP = pulse_cycles;

  for(i = 0; i < 3000000; i++) // 90
  pulse_cycles = pulse_cycles + 25000;
  PPWM0_CMP = pulse_cycles;

  for(i = 0; i < 3000000; i++) // 135
  pulse_cycles = pulse_cycles + 25000;
  PPWM0_CMP = pulse_cycles;
  //printf("PWM over.....\n");

}*/
    return 0;
}

static const struct pwm_driver_api pwm_ppu_drv_api_funcs = {
    .pin_set = pwm_ppu_pin_set,
};


static int pwm_ppu_init(struct device *dev)
{
    return 0;
}


#ifdef CONFIG_PWM_PPU_0
static struct pwm_ppu_data pwm_ppu_dev_data_0 = {
    /* Default case */
    .pwm_prescaler = 10000,
};

static const struct pwm_ppu_config pwm_ppu_dev_cfg_0 = {
    .pwm_base = PPU_PWM_BASE,
};
#endif /* CONFIG_PWM_PPU_0 */

/*DEVICE_AND_API_INIT(pwm_ppu_0, "pwm_0",
            pwm_ppu_init,
            &pwm_ppu_dev_data_0, &pwm_ppu_dev_cfg_0,
            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
            &pwm_ppu_drv_api_funcs);*/

DEVICE_AND_API_INIT(pwm_ppu_0, "pwm_0",
            pwm_ppu_init,
            NULL, NULL,
            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
            &pwm_ppu_drv_api_funcs);

