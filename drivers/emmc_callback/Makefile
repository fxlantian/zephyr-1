# Makefile for EMMC_CALLBACK drivers
ccflags-y +=-I$(srctree)/drivers
ccflags-$(CONFIG_EMMC_CALLBACK_QMSI) += -I$(CONFIG_QMSI_INSTALL_PATH)/include

obj-$(CONFIG_EMMC_CALLBACK_PCA9685)	+= emmc_callback_pca9685.o
obj-$(CONFIG_EMMC_CALLBACK_DW)		+= emmc_callback_dw.o
obj-$(CONFIG_EMMC_CALLBACK_QMSI)		+= emmc_callback_qmsi.o
obj-$(CONFIG_EMMC_CALLBACK_K64_FTM)	+= emmc_callback_k64_ftm.o
obj-$(CONFIG_EMMC_CALLBACK_STM32)		+= emmc_callback_stm32.o
obj-$(CONFIG_EMMC_CALLBACK_NRF5_SW)       += emmc_callback_nrf5_sw.o
obj-$(CONFIG_EMMC_CALLBACK_PPU)       += emmc_callback_ppu.o
