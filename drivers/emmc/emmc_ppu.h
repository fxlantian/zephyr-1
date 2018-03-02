#ifndef EMMCCTL_H
#define EMMCCTL_H



typedef void (*irq_config_func_t)(struct device *dev);


struct emmc_ppu_config {
    u32_t emmc_base_addr;
    irq_config_func_t irq_config;
};

#endif
