/*
 * Copyright (c) 2016 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <toolchain.h>

/* imports */
GTEXT(__reset)
GTEXT(__irq_wrapper)

/*
 * addr 0x00000000 - 0x00000038 are not used
 * in IVT. Hence, set them to nop.
 *
 * Call __irq_wrapper to handle all interrupts/exceptions/faults/ECALL
 *
 * ECALL is used to handle context switching of threads, as well as
 * IRQ offloading (when enabled).
 *
 * Interrupt Line 15: EMMC/SDIO IRQ 0x0000003C
 * Interrupt Line 16: ANN IRQ 0x00000040
 * Interrupt Line 17: Camera IRQ 0x00000044
 * Interrupt Line 18: AHB IRQ 0x00000048
 * Interrupt Line 19: I2C1 IRQ 0x0000004C
 * Interrupt Line 20: Uart1 IRQ 0x00000050
 * Interrupt Line 21: SPI Master1 0 0x00000054
 * Interrupt Line 22: SPI Master1 1 0x00000058
 * Interrupt Line 23: I2C0 IRQ 0x0000005C
 * Interrupt Line 24: UART0 IRQ 0x00000060
 * Interrupt Line 25: GPIO IRQ 0x00000064
 * Interrupt Line 26: SPI Master0 0 0x00000068
 * Interrupt Line 27: SPI Master0 1 0x0000006C
 * Interrupt Line 28: Timer A Overflow 0x00000070
 * Interrupt Line 29: Timer A Output Cmp 0x00000074
 * Interrupt Line 30: Timer B Overflow 0x00000078
 * Interrupt Line 31: Timer B Output Cmp 0x0000007C
 *
 * RESET 0x00000080 - call __reset

 * Illegal Instruction Exception 0x00000084
 * ECALL Instruction 0x00000088
 * Invalid Memory Access 0x0000008C
 */
SECTION_FUNC(vectors, vinit)
	.option norvc;

	/* nop addr 0x00000000 - 0x00000038 */
	.org 0x00
	.rept 15
	nop
	.endr

	/* Call __irq_wrapper for all interrupts */
	.org 0x3C
	.rept 17
	jal x0, __irq_wrapper
	.endr

	/* Call __reset for reset vector */
	.org 0x80
	jal x0, __reset

	/* Illegal instruction */
	jal x0, __irq_wrapper

	/* ECALL (system call) */
	jal x0, __irq_wrapper

	/* Invalid memory access */
	jal x0, __irq_wrapper
