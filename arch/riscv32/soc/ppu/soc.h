/*
 * Copyright (c) 2016 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the ppu core
 */

#ifndef __PPU_SOC_H_
#define __PPU_SOC_H_

/* CSR Registers */
#define PPU_MESTATUS              0x7C0 /* Machine Exception Status Register */
#define PPU_LPSTART0              0x7B0 /* Hardware Loop 0 Start Register */
#define PPU_LPEND0                0x7B1 /* Hardware Loop 0 End Register */
#define PPU_LPCOUNT0              0x7B2 /* Hardware Loop 0 Count Register */
#define PPU_LPSTART1              0x7B4 /* Hardware Loop 1 Start Register */
#define PPU_LPEND1                0x7B5 /* Hardware Loop 1 End Register */
#define PPU_LPCOUNT1              0x7B6 /* Hardware Loop 1 Count Register */

/* IRQ numbers */
#define PPU_EMMC_IRQ 	          15 /* EMMC/SDIO   */
#define PPU_ANN_IRQ    	          16 /* NeuroNetwork */
#define PPU_CAM_IRQ               17 /* Camera */
#define PPU_AHB_IRQ               18 /* AHB bus */
#define PPU_I2C_1_IRQ   	      19 /* I2C Controller 1 */
#define PPU_UART_1_IRQ    	      20 /* Uart Controller 1 */
#define PPU_SPI_1_0_IRQ           21 /* SPI Controller 1 #0 end of transmission */
#define PPU_SPI_1_1_IRQ           22 /* SPI Controller 1 #1 R/T finished  */
#define PPU_I2C_0_IRQ             23 /* I2C Controller 0 */
#define PPU_UART_0_IRQ            24 /* Uart Controller 0 */
#define PPU_GPIO_0_IRQ            25 /* GPIO Controller */
#define PPU_SPI_0_0_IRQ           26 /* SPI Controller 0 #0 end of transmission */
#define PPU_SPI_0_1_IRQ           27 /* SPI Controller 0 #1 R/T finished */
#define PPU_TIMER_A_OVERFLOW_IRQ  28 /* Timer Overflow A */
#define PPU_TIMER_A_CMP_IRQ       29 /* Timer Output Cmp A */
#define PPU_TIMER_B_OVERFLOW_IRQ  30 /* Timer Overflow B */
#define PPU_TIMER_B_CMP_IRQ       31 /* Timer Output Cmp B */

/* min value to consider as IRQ in MCAUSE register */
#define PPU_MIN_IRQ               PPU_EMMC_IRQ

/* Exception numbers */
#define PPU_ECALL_EXP             11 /* ECALL Instruction */

/*
 * SOC-specific MSTATUS related info
 */
/* MSTATUS register to save/restore upon interrupt/exception/context switch */
#define SOC_MSTATUS_REG            PPU_MESTATUS

#define SOC_MSTATUS_IEN            (1 << 0) /* Machine Interrupt Enable bit */

/*
 * Default MSTATUS register value to restore from stack
 * upon scheduling a thread for the first time
 */
#define SOC_MSTATUS_DEF_RESTORE    SOC_MSTATUS_IEN

/* SOC-specific MCAUSE bitfields */
#define SOC_MCAUSE_EXP_MASK        0x1F           /* Exception code Mask */
#define SOC_MCAUSE_ECALL_EXP       PPU_ECALL_EXP /* ECALL exception number */

/* SOC-Specific EXIT ISR command */
#define SOC_ERET                   eret

/* UART configuration */
#define UART_NS16550_PORT_0_BASE_ADDR     0x1A100000
#define UART_NS16550_PORT_0_CLK_FREQ      50000000  /* 50MHz */
#define UART_NS16550_PORT_0_IRQ           PPU_UART_0_IRQ

/* GPIO configuration */
#define PPU_GPIO_0_BASE           0x1A101000
#define PPU_SPI_0_BASE            0x1A102000
#define PPU_I2C_0_BASE            0x1A105000
#define PPU_SPI_1_BASE            0x1A108000

/* PINMAX*/
#define PPU_IOMUX_CFG_BASE         0x1A106020

/* IRQ configuration */
#define PPU_IRQ_BASE              0x1A104000

#define PPU_IER_ADDR  (PPU_IRQ_BASE + 0x00) /* IRQ Enable Register */
#define PPU_IPR_ADDR  (PPU_IRQ_BASE + 0x04) /* IRQ Pending Register */
#define PPU_ISP_ADDR  (PPU_IRQ_BASE + 0x08) /* IRQ Set Pending Register */
#define PPU_ICP_ADDR  (PPU_IRQ_BASE + 0x0C) /* IRQ Clear Pending Register */
#define PPU_EER_ADDR  (PPU_IRQ_BASE + 0x10) /* Event Enable Register */
#define PPU_EPR_ADDR  (PPU_IRQ_BASE + 0x14) /* Event Pending Register */
#define PPU_ESP_ADDR  (PPU_IRQ_BASE + 0x18) /* Event Set Pending Register */
#define PPU_ECP_ADDR  (PPU_IRQ_BASE + 0x1C) /* Event Clear Pending Register */
#define PPU_SCR_ADDR  (PPU_IRQ_BASE + 0x20) /* Sleep Control Register */

/* Timer configuration */
#define PPU_TIMER_A_BASE          0x1A103000
#define PPU_TIMER_B_BASE          0x1A103010

/*
 * Zephyr-SDK makes use a the latest generic riscv32 toolchain, which
 * encodes the wfi opcode as 0x10500073. Pulpino does not understand
 * this opcode and will generate a fault upon execution. Pulpino core
 * implementation is based on a previous RISC-V ISA specification and
 * expects the wfi opcode to be encoded as 0x10200073. In new toolchain,
 * 0x10200073 opcode is used to represent the sret opcode. Hence,
 * when compiled with a generic riscv32 toolchain, define wfi by sret
 * in assembly code.
 */
#if defined(CONFIG_RISCV_GENERIC_TOOLCHAIN)
#define wfi                        sret
#endif

#ifndef _ASMLANGUAGE
#include <irq.h>

/* Register Access MACRO */
#define PPU_REG(x)                (*((volatile unsigned int *)(x)))

/* Interrupt Registers */
#define PPU_IER                   PPU_REG(PPU_IER_ADDR)
#define PPU_IPR                   PPU_REG(PPU_IPR_ADDR)
#define PPU_ISP                   PPU_REG(PPU_ISP_ADDR)
#define PPU_ICP                   PPU_REG(PPU_ICP_ADDR)
#define PPU_EER                   PPU_REG(PPU_EER_ADDR)
#define PPU_EPR                   PPU_REG(PPU_EPR_ADDR)
#define PPU_ESP                   PPU_REG(PPU_ESP_ADDR)
#define PPU_ECP                   PPU_REG(PPU_ECP_ADDR)
#define PPU_SCR                   PPU_REG(PPU_SCR_ADDR)


#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
void soc_interrupt_init(void);
#endif

/*
 * when a generic riscv32 toolchain is used replaced wfi by sret
 * in inline assembly. Explanation given above.
 */
#if defined(CONFIG_RISCV_GENERIC_TOOLCHAIN)
#define SOC_WFI                    __asm__ volatile("sret")
#else
#define SOC_WFI                    __asm__ volatile("wfi")
#endif

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE             CONFIG_DTCM_BASE_ADDRESS
#define RISCV_RAM_SIZE             CONFIG_DTCM_SIZE

#endif /* !_ASMLANGUAGE */

#endif /* __PULPINO_SOC_H_ */
