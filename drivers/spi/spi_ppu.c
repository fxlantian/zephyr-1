#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

#include <misc/util.h>
#include <kernel.h>
#include <board.h>
#include <errno.h>
#include <spi.h>
#include "spi_ppu.h"
#include <misc/printk.h>

#define DEV_SPI_CFG(dev)                           \
        ((struct spi_ppu_config * const)(dev)->config->config_info)
#define DEV_SPI_DATA(dev)                          \
        ((struct spi_ppu_data * const)(dev)->driver_data)
#define DEV_SPI(dev)                               \
        ((volatile struct spi_ppu_t *)(DEV_SPI_CFG(dev))->spi_base_addr)

#define SPI_PPU_CLK_DIVIDER(spi_clk_hz)            \
        (((CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / (2 * spi_clk_hz)) - 1) & 0xFFFF)
#define SPI_START_TRANSACTION(trans_type, csnum)   \
        (((1 << (csnum + 8)) & 0xF00) | ((1 << trans_type) & 0xFF))

static void spi_ppu_transmit(struct device *dev);
static void spi_ppu_recieve(struct device *dev);

static void spi_ppu_isr(struct device *dev)
{
    struct spi_ppu_data *data = DEV_SPI_DATA(dev);
    volatile struct spi_ppu_t *spi = DEV_SPI(dev);
    int status = 0;

    PPU_ICP |= 1 << 26; 
    status = spi->int_sta;

    
    spi_context_complete(&data->ctx, 0);     

}

static int spi_ppu_configure(struct spi_config *config)
{
    struct device *dev = config->dev;
    struct spi_ppu_data *data = DEV_SPI_DATA(dev);
    volatile struct spi_ppu_t *spi = DEV_SPI(dev);

    if(spi_context_configured(&data->ctx, config)){
        return 0;
    }
    
    spi->clk_div = SPI_PPU_CLK_DIVIDER(config->frequency);   //set the clock divider

    data->ctx.config = config;

    return 0;

}

static void spi_ppu_transmit(struct device *dev)
{
    struct spi_ppu_data *data = DEV_SPI_DATA(dev);
    volatile struct spi_ppu_t *spi = DEV_SPI(dev);

    volatile int reg_data = 0;
    volatile int datalen = 0;
    volatile int num_words = 0;
    volatile int i = 0;
    volatile unsigned char prime_addr[8] = {0};

    //when spi_write() or spi_transcieve(), put the first 8 byte data to cmd and addr registers to send out,
    //so that the csn will not go back up between cmd, addr, and data.
    while(data->ctx.tx_len && (i < 8)) {
        if(data->ctx.tx_len + i > 8) {
            break;
        }

        if(data->ctx.tx_len > 1) {
            int temp = data->ctx.tx_len;
            while(temp) {
                prime_addr[i + temp - 1] = (*(u8_t *)(data->ctx.tx_buf++));
                temp--;
            }
            i += data->ctx.tx_len;
        }
        else {
            prime_addr[i] = (*(u8_t *)(data->ctx.tx_buf));
            i++;
        }
        
        data->ctx.tx_len = 1;                   //clear tx_len
        spi_context_update_tx(&data->ctx, 1);   //get next buffer address and length
    }

    //set the command register and address register
    spi->spi_cmd = ((prime_addr[0] & 0xFF) << 24) | ((prime_addr[1] & 0xFF) << 16) | ((prime_addr[2] & 0xFF) << 8) | (prime_addr[3] & 0xFF);
    spi->spi_adr = ((prime_addr[4] & 0xFF) << 24) | ((prime_addr[5] & 0xFF) << 16) | ((prime_addr[6] & 0xFF) << 8) | (prime_addr[7] & 0xFF);

    if(i <= 4) {                                                 //if i <= 4, put the data in cmd register
        spi->spi_len = i * 8;
    } else {
        spi->spi_len = 0x20 | ((((i - 4) * 8) << 8) & 0x3F00);   //if 4<=i<=8, put low 4 byte data in cmd register, high 4 byte or less data in addr register
    }
    spi->spi_dum = 0x00;                                         //set the dummy cycles to 0 initially


    //if data waiting to send out is less than 8 byte, or there is no need to wait to recieve data after that, start transaction here.
    if((i < 8) && (!data->ctx.tx_len) && (!data->ctx.rx_len)) {
        spi->status = SPI_START_TRANSACTION(SPI_CMD_WR, SPI_CSN0);
        while((spi->status & 0xFFFF) != 1);
        return;
    }

    while(data->ctx.tx_len)
    {
        if(data->ctx.tx_buf) {
            reg_data = spi->spi_len;
            datalen = data->ctx.tx_len * 8;
            spi->spi_len = ((datalen << 16) & 0xFFFF0000) | (reg_data & 0xFFFF);  //set data length
 
            num_words = (datalen >> 5) & 0x7FF;
            if((datalen & 0x1F) != 0)
                num_words++;

            volatile int *tx_data = (volatile int *)(data->ctx.tx_buf);           //get the data buffer
            spi->status = SPI_START_TRANSACTION(SPI_CMD_WR, SPI_CSN0);            //start transaction

            for(i = 0; i < num_words; i++) {
                while(((spi->status >> 24) & 0xFF) >= 8);
                spi->tx_fifo = tx_data[i];                                        //fill the tx fifo
            }
            while((spi->status & 0xFFFF) != 1);

            spi->spi_cmd = 0x00;
            spi->spi_adr = 0x00;
            spi->spi_len = 0x00;
            spi->spi_dum = 0x00;                                     //clear registers
        }
        else {
            spi->spi_dum = (data->ctx.tx_len << 16) & 0xFFFF0000;    //if tx_buf is NULL, set write dummy cycles = tx_len
        }
       
        data->ctx.tx_len = 1;
        spi_context_update_tx(&data->ctx, 1);                        //clear tx_len, get next buffer and length
    }
}

static void spi_ppu_recieve(struct device *dev)
{
    struct spi_ppu_data *data = DEV_SPI_DATA(dev);
    volatile struct spi_ppu_t *spi = DEV_SPI(dev);

    volatile int reg_data = 0;
    volatile int datalen = 0;
    volatile int num_words = 0;
    volatile int i = 0;

    reg_data = spi->spi_len;

    while(data->ctx.rx_len)
    {
        if(data->ctx.rx_buf) {

            datalen = data->ctx.rx_len * 8;
            spi->spi_len = ((datalen << 16) & 0xFFFF0000) | (reg_data & 0xFFFF);

            num_words = (datalen >> 5) & 0x7FF;
            if((datalen & 0x1F) != 0)
                num_words++;

            volatile int *rx_data = (volatile int *)(data->ctx.rx_buf);
            spi->status = SPI_START_TRANSACTION(SPI_CMD_RD, SPI_CSN0);

            for(i = 0; i < num_words; i++) {
                while(((spi->status >> 16) & 0xFF) == 0);
                rx_data[i] = spi->rx_fifo;                 //get data from rx fifo                    
            }
            while((spi->status & 0xFFFF) != 1);
            
            spi->spi_cmd = 0x00;
            spi->spi_adr = 0x00;
            spi->spi_len = 0x00;
            spi->spi_dum = 0x00;
        }
        else {
            spi->spi_dum = data->ctx.rx_len & 0xFFFF;     //if rx_buf is NULL, set read dummy cycles = rx_len
        }
        data->ctx.rx_len = 1;
        spi_context_update_rx(&data->ctx, 1);             //clear rx_len, get the next buffer and length of the buffer 
    }
}

static int spi_ppu_release(struct spi_config *config)
{
    struct device *dev = config->dev;
    struct spi_ppu_data *data = DEV_SPI_DATA(dev);
    spi_context_unlock_unconditionally(&data->ctx);       //unlock the mutex
    return 0; 
}

static int transceive(struct spi_config *config,
                      const struct spi_buf *tx_bufs, u32_t tx_count,
                      struct spi_buf *rx_bufs, u32_t rx_count,
                      bool asynchronous, struct k_poll_signal *signal)
{
    struct device *dev = config->dev;
    struct spi_ppu_data *data = DEV_SPI_DATA(dev);
    volatile struct spi_ppu_t *spi = DEV_SPI(dev);
    
    int ret;

    if(!tx_count && !rx_count) {
        return 0;
    }

    spi_context_lock(&data->ctx, asynchronous, signal);

    ret = spi_ppu_configure(config);
    if(ret) {
        return ret;
    }

    spi_context_buffers_setup(&data->ctx, tx_bufs, tx_count, rx_bufs, rx_count, 1);

#ifdef CONFIG_SPI_PPU_INTERRUPT
    spi->int_cfg |= (0x3) << 29;
    spi_context_wait_for_completion(&data->ctx);
#else
    if(tx_count) {            //send data
        spi_ppu_transmit(dev);
    }
    if(rx_count){             //recieve data
        spi_ppu_recieve(dev);
    }
#endif
    spi_context_complete(&data->ctx, 0);

    spi_context_release(&data->ctx, 0);
    return 0; 
}

static int spi_ppu_transceive(struct spi_config *config,
                      const struct spi_buf *tx_bufs, u32_t tx_count,
                      struct spi_buf *rx_bufs, u32_t rx_count)
{
    return transceive(config, tx_bufs, tx_count, rx_bufs, rx_count, false, NULL);
}

#ifdef CONFIG_POLL
static int spi_ppu_transceive_async(struct spi_config *config,
                      const struct spi_buf *tx_bufs, u32_t tx_count,
                      struct spi_buf *rx_bufs, u32_t rx_count,
                      struct k_poll_signal *async)
{
    return transceive(config, tx_bufs, tx_count, rx_bufs, rx_count, true, async);
}
#endif

static const struct spi_driver_api spi_ppu_driver = {
    .transceive = spi_ppu_transceive,
#ifdef CONFIG_POLL
    .transceive_async = spi_ppu_transceive_async,
#endif
    .release = spi_ppu_release,
};

static int spi_ppu_init(struct device *dev)
{
    struct spi_ppu_data *data = DEV_SPI_DATA(dev);
    struct spi_ppu_config *cfg = DEV_SPI_CFG(dev);

#ifdef CONFIG_SPI_PPU_INTERRUPT
    cfg->irq_config();
#endif
    spi_context_unlock_unconditionally(&data->ctx);
    return 0;
}


#ifdef CONFIG_SPI_0

static void spi_ppu_irq_config_func_0(void);

static const struct spi_ppu_config spi_ppu_cfg_0 = {
    .spi_base_addr = PPU_SPI_0_BASE,
    .irq_config = spi_ppu_irq_config_func_0,
};

static struct spi_ppu_data spi_ppu_dev_data_0 = {
    SPI_CONTEXT_INIT_LOCK(spi_ppu_dev_data_0, ctx),
    SPI_CONTEXT_INIT_SYNC(spi_ppu_dev_data_0, ctx),
};

DEVICE_AND_API_INIT(spi_ppu_0, CONFIG_SPI_0_NAME, spi_ppu_init,
                    &spi_ppu_dev_data_0, &spi_ppu_cfg_0,
                    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
                    &spi_ppu_driver);

static void spi_ppu_irq_config_func_0(void)
{
    IRQ_CONNECT(PPU_SPI_0_0_IRQ,
                CONFIG_SPI_0_IRQ_PRI,
                spi_ppu_isr,
                DEVICE_GET(spi_ppu_0),
                0);
    irq_enable(PPU_SPI_0_0_IRQ);

}

#endif /* CONFIG_PPU_SPI_0 */


#ifdef CONFIG_SPI_1

static void spi_ppu_irq_config_func_1(void);

static const struct spi_ppu_config spi_ppu_cfg_1 = {
    .spi_base_addr = PPU_SPI_1_BASE,
    .irq_config = spi_ppu_irq_config_func_1,
};

static struct spi_ppu_data spi_ppu_dev_data_1 = {
    SPI_CONTEXT_INIT_LOCK(spi_ppu_dev_data_1, ctx),
    SPI_CONTEXT_INIT_SYNC(spi_ppu_dev_data_1, ctx),
};

DEVICE_AND_API_INIT(spi_ppu_1, CONFIG_SPI_1_NAME, spi_ppu_init,
                    &spi_ppu_dev_data_1, &spi_ppu_cfg_1,
                    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
                    &spi_ppu_driver);

static void spi_ppu_irq_config_func_1(void)
{
    IRQ_CONNECT(PPU_SPI_1_0_IRQ,
                CONFIG_SPI_1_IRQ_PRI,
                spi_ppu_isr,
                DEVICE_GET(spi_ppu_1),
                0);
    irq_enable(PPU_SPI_1_0_IRQ);

}

#endif /* CONFIG_PPU_SPI_1 */

