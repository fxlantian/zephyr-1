#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <pinmux.h>
#include <spi.h>
#include <misc/util.h>

#define SPI_NAME "SPI_0"
#define SPI_FUNC 0

#define BUF_SIZE 17

u8_t cmd = 0x9F;
u8_t cmd1 = 0x06;
u8_t cmd2 = 0x71;
u32_t addr = 0x80000410;
u32_t data = 0;

struct spi_config spi0 = {
    .frequency = 500000,
    .operation = SPI_OP_MODE_MASTER | SPI_LINES_SINGLE,
    .slave = 0,
    .cs = NULL,
};

int flash_get_cr1() {
    int status;
    u8_t rdcr1_cmd = 0x35;

    struct spi_buf tx[] = {
        {
            .buf = &rdcr1_cmd,
            .len = 1,
        },
    };
    struct spi_buf rx[] = {
        {
            .buf = &status,
            .len = 1,
        },
    };
    spi_transceive(&spi0, tx, 1, rx, 1);
    return status;
}

int flash_get_wip() {
    int status;
    u8_t rdcr1_cmd = 0x05;

    struct spi_buf tx[] = {
        {
            .buf = &rdcr1_cmd,
            .len = 1,
        },
    };
    struct spi_buf rx[] = {
        {
            .buf = &status,
            .len = 1,
        },
    };
    spi_transceive(&spi0, tx, 1, rx, 1);
    return status & 0x1;
}

void flash_sector_erase_parameter(u32_t addr) {
    while(flash_get_wip());
    u8_t wren_cmd = 0x06;
    u8_t p4e_cmd = 0x20;
    u32_t erase_addr = addr;
    struct spi_buf tx[] = {
        {
            .buf = &wren_cmd,
            .len = 1,
        },
        {
            .buf = &p4e_cmd,
            .len = 1,
        },
        {
            .buf = &erase_addr,
            .len = 3,
        },
    };

    spi_write(&spi0, tx, 1); 
    spi_write(&spi0, &tx[1], 2); 
}

void flash_write(u32_t addr, int* buffer, u32_t buf_len) {
    while(flash_get_wip());
    u8_t wren_cmd = 0x06;
    u8_t pp_cmd = 0x02;
    u32_t waddr = addr;
    
    struct spi_buf tx[] = {
        {
            .buf = &wren_cmd,
            .len = 1,
        },
        {
            .buf = &pp_cmd,
            .len = 1,
        },
        {
            .buf = &waddr,
            .len = 3,
        },
        {
            .buf = buffer,
            .len = buf_len,
        },
    };
    spi_write(&spi0, tx, 1); 
    spi_write(&spi0, &tx[1], 3); 
}

void flash_read(u32_t addr, int* buffer, u32_t buf_len) {
    while(flash_get_wip());
    u8_t read_cmd = 0x13;
    u32_t raddr = addr;
    struct spi_buf tx[] = {
        {
            .buf = &read_cmd,
            .len = 1,
        },
        {
            .buf = &raddr,
            .len = 4,
        },
    };
    struct spi_buf rx[] = {
        {
            .buf = buffer,
            .len = buf_len,
        },
    };
    spi_transceive(&spi0, tx, 2, rx, 1);
}

void main()
{
    printk("This is Zephyr SPI_PPU Driver Test...\n");

    struct spi_buf tx_bufs[] = {
        {
            .buf = &cmd,
            .len = 1,
        },
        {
            .buf = &cmd1,
            .len = 1,
        },
        {
            .buf = &cmd2,
            .len = 1,
        },
        {
            .buf = &addr,
            .len = 4,
        },
    };
    struct spi_buf rx_bufs[] = {
        {
            .buf = &data,
            .len = 4,
        },
    };

    spi0.dev = device_get_binding(SPI_NAME);
    if(!spi0.dev) {
        printk("Cannot find %s!\n", SPI_NAME);
        return;
    }
    else {
        printk("find spi0!\n");
    }

    int ret;
    ret = spi_transceive(&spi0, tx_bufs, 1, rx_bufs, 1);
    if(ret) {
        printk("SPI transceive error\n");
        return;
    }

    printk("flash id: %x\n!", data);

    int cr1 = flash_get_cr1();
    if((cr1 & (1 << 2)) != 0) {
        printk("Read Flash Error!!!\n");
        return;
    }
    printk("CR1: %x\n", cr1);
    spi_write(&spi0, &tx_bufs[1], 1);
    spi_write(&spi0, &tx_bufs[2], 2);

    printk("Flash erasing...\n");
    flash_sector_erase_parameter(0x00);
    while(flash_get_wip());
    printk("Flash erased\n");
     
    int flash_data[10] = {0x1234, 0x2345, 0x3456, 0x4567, 0x5678, 0x6789, 0x7890, 0x8901, 0x9012, 0x0123};
    int rdata[10] = {0};

    flash_write(0x00, flash_data, 40);
    flash_read(0x00, rdata, 40);
 
    printk("SPI transceive over\n");
    printk("Flash read data back: %x\n", rdata[0]);
    printk("Flash read data back: %x\n", rdata[1]);
    printk("Flash read data back: %x\n", rdata[2]);
    printk("Flash read data back: %x\n", rdata[3]);
    printk("Flash read data back: %x\n", rdata[4]);
    printk("Flash read data back: %x\n", rdata[5]);
    printk("Flash read data back: %x\n", rdata[6]);
    printk("Flash read data back: %x\n", rdata[7]);
    printk("Flash read data back: %x\n", rdata[8]);
    printk("Flash read data back: %x\n", rdata[9]);
}


