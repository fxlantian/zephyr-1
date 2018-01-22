/*
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Header file for the PPU PWM driver.
 */
#ifndef __PWM_PPU_H__
#define __PWM_PPU_H__

#ifdef __cplusplus
extern "C" {
#endif

#define PWM0_LOAD         0x00
#define PWM0_CMP          0x04
#define PWM0_CTRL         0x08
#define PWM0_TIMER        0x0C

#define __PPWM__(a) *(volatile int*) (PPU_PWM_BASE + a)

#define PPWM0_LOAD  __PPWM__(PWM0_LOAD)
#define PPWM0_CMP   __PPWM__(PWM0_CMP)
#define PPWM0_CTRL  __PPWM__(PWM0_CTRL)
#define PPWM0_TIMER __PPWM__(PWM0_TIMER)


//#define PPWM0_LOAD  (PPU_PWM_BASE + 0x00)
//#define PPWM0_CMP   (PPU_PWM_BASE + 0x04)
//#define PPWM0_CTRL  (PPU_PWM_BASE + 0x08)
//#define PPWM0_TIMER (PPU_PWM_BASE + 0x0C)


/** Configuration data */
struct pwm_ppu_config {
    u32_t pwm_base;
    u32_t num_ports;
    /* clock subsystem driving this peripheral */
    //struct ppu_pclken pclken;
};

/** Runtime driver data */
struct pwm_ppu_data {
    /* PWM peripheral handler */
    //TIM_HandleTypeDef hpwm;
    /* Prescaler for PWM output clock
     * Value used to divide the TIM clock.
     * Min = 0x0000U, Max = 0xFFFFU
     */
    u32_t pwm_prescaler;
    /* clock device */
    //struct device *clock;
};

#ifdef __cplusplus
}
#endif

#endif /* __PWM_PPU_H__ */