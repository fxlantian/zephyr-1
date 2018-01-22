# Makefile for CAM drivers
ccflags-y +=-I$(srctree)/drivers
ccflags-$(CONFIG_CAM_QMSI) += -I$(CONFIG_QMSI_INSTALL_PATH)/include

obj-$(CONFIG_CAM_PCA9685)	+= cam_pca9685.o
obj-$(CONFIG_CAM_DW)		+= cam_dw.o
obj-$(CONFIG_CAM_QMSI)		+= cam_qmsi.o
obj-$(CONFIG_CAM_K64_FTM)	+= cam_k64_ftm.o
obj-$(CONFIG_CAM_STM32)		+= cam_stm32.o
obj-$(CONFIG_CAM_NRF5_SW)       += cam_nrf5_sw.o
obj-$(CONFIG_CAM_PPU)       += cam_ppu.o
