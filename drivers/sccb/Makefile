# Makefile for SCCB drivers
ccflags-y +=-I$(srctree)/drivers
ccflags-$(CONFIG_SCCB_QMSI) += -I$(CONFIG_QMSI_INSTALL_PATH)/include

obj-$(CONFIG_SCCB_PCA9685)	+= sccb_pca9685.o
obj-$(CONFIG_SCCB_DW)		+= sccb_dw.o
obj-$(CONFIG_SCCB_QMSI)		+= sccb_qmsi.o
obj-$(CONFIG_SCCB_K64_FTM)	+= sccb_k64_ftm.o
obj-$(CONFIG_SCCB_STM32)		+= sccb_stm32.o
obj-$(CONFIG_SCCB_NRF5_SW)       += sccb_nrf5_sw.o
obj-$(CONFIG_SCCB_PPU)       += sccb_ppu.o
