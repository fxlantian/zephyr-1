#ifndef ANNCTL_H
#define ANNCTL_H

#define INIT_START  0x06
#define NPU_START   0x05 
#define CLEAR_INT   0x08
#define STOP_EN     0x10


#define ANN_REG_CTR               PPU_ANN_BASE
#define ANN_REG_INT_EN            ( PPU_ANN_BASE + 0x04 )
#define ANN_REG_DMA_SRC_ADDR      ( PPU_ANN_BASE + 0x08 )
#define ANN_REG_DMA_DST_ADDR      ( PPU_ANN_BASE + 0x0C )
#define ANN_REG_DMA_BLOCK_INFO    ( PPU_ANN_BASE + 0x10 )
#define ANN_REG_WEIGTH_DEPTH      ( PPU_ANN_BASE + 0x14 )
#define ANN_REG_BIAS_DEPTH        ( PPU_ANN_BASE + 0x18 )
#define ANN_REG_IM_DEPTH          ( PPU_ANN_BASE + 0x1C )
#define ANN_REG_SIG_DEPTH         ( PPU_ANN_BASE + 0x20 )
#define ANN_REG_NPU_DATAIN_DEPTH  ( PPU_ANN_BASE + 0x24 )
#define ANN_REG_NPU_DATAOUT_DEPTH ( PPU_ANN_BASE + 0x28 )
#define ANN_REG_STATUS_END        ( PPU_ANN_BASE + 0x2C )
#define ANN_REG_STATUS_RUN        ( PPU_ANN_BASE + 0x30 )

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

#define SPI_CMD_RD    0
#define SPI_CMD_WR    1
#define SPI_CMD_QRD   2
#define SPI_CMD_QWR   3
#define SPI_CMD_SWRST 4

#define SPI_CSN0 0
#define SPI_CSN1 1
#define SPI_CSN2 2
#define SPI_CSN3 3

typedef void (*irq_config_func_t)(struct device *dev);


struct ann_ppu_config {
    u32_t ann_base_addr;
    irq_config_func_t irq_config;
};

#endif
