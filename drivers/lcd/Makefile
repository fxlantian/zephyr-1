# Makefile for LCD drivers
ccflags-y +=-I$(srctree)/drivers
ccflags-$(CONFIG_LCD_QMSI) += -I$(CONFIG_QMSI_INSTALL_PATH)/include

obj-$(CONFIG_LCD_PCA9685)	+= lcd_pca9685.o
obj-$(CONFIG_LCD_DW)		+= lcd_dw.o
obj-$(CONFIG_LCD_QMSI)		+= lcd_qmsi.o
obj-$(CONFIG_LCD_K64_FTM)	+= lcd_k64_ftm.o
obj-$(CONFIG_LCD_STM32)		+= lcd_stm32.o
obj-$(CONFIG_LCD_NRF5_SW)       += lcd_nrf5_sw.o
obj-$(CONFIG_LCD_PPU)       += lcd_ppu.o
