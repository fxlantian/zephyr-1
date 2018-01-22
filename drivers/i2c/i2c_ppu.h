#ifndef _I2C_PPU_H
#define _I2C_PPU_H

#define I2C_START       0x80
#define I2C_STOP        0x40
#define I2C_READ        0x20
#define I2C_WRITE       0x10
#define I2C_CLR_INT     0x01
#define I2C_START_READ  0xA0
#define I2C_STOP_READ   0x60
#define I2C_STOP_READ_NACK   0x68
#define I2C_START_WRITE 0x90
#define I2C_STOP_WRITE  0x50

#define I2C_CTR_EN        0x80 // enable only
#define I2C_CTR_INTEN     0x40 // interrupt enable only
#define I2C_CTR_EN_INTEN  0xC0 // enable i2c and interrupts

#define I2C_STATUS_RXACK 0x80
#define I2C_STATUS_BUSY  0x40
#define I2C_STATUS_AL    0x20
#define I2C_STATUS_TIP   0x02
#define I2C_STATUS_IF    0x01

typedef void (*irq_config_func_t)(struct device *dev);

struct i2c_ppu_t {
    volatile int pre; //prescale
    volatile int ctr; //controller
    volatile int rx;
    volatile int status;
    volatile int tx;
    volatile int cmd;
};

struct i2c_ppu_config {
    u32_t i2c_base_addr;
    irq_config_func_t irq_config;
};

struct i2c_ppu_data {
    struct k_sem device_sync_sem;
    union dev_config dev_cfg; 
};

#endif


