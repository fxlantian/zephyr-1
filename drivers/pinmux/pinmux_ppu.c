
#include <misc/__assert.h>
#include <misc/util.h>
#include <soc.h>
#include <toolchain.h>
#include <device.h>
#include <pinmux.h>
#include <sys_io.h>

#define IOCFG_REG(Func) \
        (PPU_IOMUX_CFG_BASE + 0x4 * (u32_t)(Func / 4))
        
static int pinmux_ppu_init(struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
} 

static int pinmux_ppu_set(struct device *dev, u32_t pin,
                          u32_t func)
{
    const u32_t iocfg_reg = IOCFG_REG(pin);
    u32_t conf = sys_read32(iocfg_reg);

    if((func == 2) || (func == 5) || (func == 1 && pin < 4))
        conf |= 1 << ((pin % 4) * 8);
    else
        conf &= ~(1 << ((pin % 4) * 8)); 

    sys_write32(conf, iocfg_reg);
    
    return 0;
}

static int pinmux_ppu_get(struct device *dev, u32_t pin,
                          u32_t *func)
{
    const u32_t iocfg_reg = IOCFG_REG(pin);
    u32_t conf = sys_read32(iocfg_reg);

    *func = (conf >> ((pin % 4) * 8)) & 0x1;
    return 0;
}

const struct pinmux_driver_api pinmux_ppu_driver_api = {
    .set = pinmux_ppu_set,
    .get = pinmux_ppu_get
};

DEVICE_AND_API_INIT(pinmux_ppu_0, CONFIG_PINMUX_PPU_0_NAME,
                    pinmux_ppu_init, NULL, NULL,
                    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                    &pinmux_ppu_driver_api);










                 
