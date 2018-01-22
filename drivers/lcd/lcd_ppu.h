#ifndef _LCD_PPU_H
#define _LCD_PPU_H

#define LCD_W 320
#define LCD_H 480
#define WHITE            0xFCFCFC
#define BLACK            0x000000     
#define BLUE             0xFC0000  
#define RED              0x0000FC
#define GREEN            0x00FC00
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define MAGENTA          0xF81F
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40 //棕色
#define BRRED            0XFC07 //棕红色
#define GRAY             0X8430 //灰色

#define SPI_CMD_RD    0
#define SPI_CMD_WR    1
#define SPI_CMD_QRD   2
#define SPI_CMD_QWR   3
#define SPI_CMD_SWRST 4

#define SPI_CSN0 0
#define SPI_CSN1 1
#define SPI_CSN2 2
#define SPI_CSN3 3

#define SPI1_REG_STATUS                ( PPU_SPI_1_BASE + 0x00 )
#define SPI1_REG_CLKDIV                ( PPU_SPI_1_BASE + 0x04 )
#define SPI1_REG_SPICMD                ( PPU_SPI_1_BASE + 0x08 )
#define SPI1_REG_SPIADR                ( PPU_SPI_1_BASE + 0x0C )
#define SPI1_REG_SPILEN                ( PPU_SPI_1_BASE + 0x10 )
#define SPI1_REG_SPIDUM                ( PPU_SPI_1_BASE + 0x14 )
#define SPI1_REG_TXFIFO                ( PPU_SPI_1_BASE + 0x18 )
#define SPI1_REG_RXFIFO                ( PPU_SPI_1_BASE + 0x20 )
#define SPI1_REG_INTCFG                ( PPU_SPI_1_BASE + 0x24 )
#define SPI1_REG_INTSTA                ( PPU_SPI_1_BASE + 0x28 )

u32_t dis_line = 15;
u32_t BACK_COLOR, POINT_COLOR;

#define GPIO_NAME "gpio0"

//#endif /* CONFIG_SPI_LEGACY_API */

#endif
