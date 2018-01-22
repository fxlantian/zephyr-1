# Makefile for ANN drivers
ccflags-y +=-I$(srctree)/drivers
ccflags-$(CONFIG_ANN_QMSI) += -I$(CONFIG_QMSI_INSTALL_PATH)/include

obj-$(CONFIG_ANN_PCA9685)	+= ann_pca9685.o
obj-$(CONFIG_ANN_DW)		+= ann_dw.o
obj-$(CONFIG_ANN_QMSI)		+= ann_qmsi.o
obj-$(CONFIG_ANN_K64_FTM)	+= ann_k64_ftm.o
obj-$(CONFIG_ANN_STM32)		+= ann_stm32.o
obj-$(CONFIG_ANN_NRF5_SW)       += ann_nrf5_sw.o
obj-$(CONFIG_ANN_PPU)       += ann_ppu.o
