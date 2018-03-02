# Makefile for EMMC drivers
ccflags-y +=-I$(srctree)/drivers
ccflags-$(CONFIG_EMMC_QMSI) += -I$(CONFIG_QMSI_INSTALL_PATH)/include

obj-$(CONFIG_EMMC_PCA9685)	+= emmc_pca9685.o
obj-$(CONFIG_EMMC_DW)		+= emmc_dw.o
obj-$(CONFIG_EMMC_QMSI)		+= emmc_qmsi.o
obj-$(CONFIG_EMMC_K64_FTM)	+= emmc_k64_ftm.o
obj-$(CONFIG_EMMC_STM32)		+= emmc_stm32.o
obj-$(CONFIG_EMMC_NRF5_SW)       += emmc_nrf5_sw.o
obj-$(CONFIG_EMMC_PPU)       += emmc_ppu.o
