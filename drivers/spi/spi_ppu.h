#ifndef _SPI_PPU_H
#define _SPI_PPU_H


#define SPI_QPI    1
#define SPI_NO_QPI 0

#define SPI_CMD_RD    0
#define SPI_CMD_WR    1
#define SPI_CMD_QRD   2
#define SPI_CMD_QWR   3
#define SPI_CMD_SWRST 4

#define SPI_CSN0 0
#define SPI_CSN1 1
#define SPI_CSN2 2
#define SPI_CSN3 3


#define SPI_REG_STATUS                ( PPU_SPI_0_BASE + 0x00 )
#define SPI_REG_CLKDIV                ( PPU_SPI_0_BASE + 0x04 )
#define SPI_REG_SPICMD                ( PPU_SPI_0_BASE + 0x08 )
#define SPI_REG_SPIADR                ( PPU_SPI_0_BASE + 0x0C )
#define SPI_REG_SPILEN                ( PPU_SPI_0_BASE + 0x10 )
#define SPI_REG_SPIDUM                ( PPU_SPI_0_BASE + 0x14 )
#define SPI_REG_TXFIFO                ( PPU_SPI_0_BASE + 0x18 )
#define SPI_REG_RXFIFO                ( PPU_SPI_0_BASE + 0x20 )
#define SPI_REG_INTCFG                ( PPU_SPI_0_BASE + 0x24 )
#define SPI_REG_INTSTA                ( PPU_SPI_0_BASE + 0x28 )


typedef void (*irq_config_func_t)(struct device *dev);

struct spi_ppu_t {
    volatile int status;
    volatile int clk_div;
    volatile int spi_cmd;
    volatile int spi_adr;
    volatile int spi_len;
    volatile int spi_dum;
    volatile int tx_fifo;
    volatile int null;
    volatile int rx_fifo;
    volatile int int_cfg;
    volatile int int_sta;
};

struct spi_ppu_config {
    u32_t spi_base_addr;
    irq_config_func_t irq_config;
};
#ifdef CONFIG_SPI_LEGACY_API

struct spi_ppu_data {
    struct k_sem device_sync_sem;
    u32_t dfs;
    void *tx_buf;
    u32_t tx_len;
    void *rx_buf;
    u32_t rx_len;
};

#else

#include "spi_context.h"
struct spi_ppu_data {
    struct spi_context ctx;
};

#endif /* CONFIG_SPI_LEGACY_API */

#endif
