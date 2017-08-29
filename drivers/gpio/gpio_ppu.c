#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <misc/util.h>

#include "gpio_utils.h"

typedef void (*ppu_cfg_func_t)(void);

/* ppu GPIO register-set structure */
struct gpio_ppu_t {
	u32_t paddir;
	u32_t padin;
	u32_t padout;
	u32_t inten;
	u32_t inttype0;
	u32_t inttype1;
	u32_t intstatus;
};

struct gpio_ppu_config {
	u32_t              gpio_base_addr;
	ppu_cfg_func_t    gpio_cfg_func;
};

struct gpio_ppu_data {
	/* list of callbacks */
	sys_slist_t cb;
};

/* Helper Macros for GPIO */
#define DEV_GPIO_CFG(dev)						\
	((const struct gpio_ppu_config * const)(dev)->config->config_info)
#define DEV_GPIO(dev)							\
	((volatile struct gpio_ppu_t *)(DEV_GPIO_CFG(dev))->gpio_base_addr)
#define DEV_GPIO_DATA(dev)					\
	((struct gpio_ppu_data *)(dev)->driver_data)

static void gpio_ppu_irq_handler(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct gpio_ppu_data *data = DEV_GPIO_DATA(dev);
	volatile struct gpio_ppu_t *gpio = DEV_GPIO(dev);

	_gpio_fire_callbacks(&data->cb, dev, gpio->intstatus);
}

/**
 * @brief Configure pin
 *
 * @param dev Device structure
 * @param access_op Access operation
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_ppu_config(struct device *dev,
			       int access_op,
			       u32_t pin,
			       int flags)
{
	volatile struct gpio_ppu_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin > 31)
		return -EINVAL;

	/* Configure pin as gpio */
//	PPU_PADMUX |= (PPU_PAD_GPIO << pin);

	/* Configure gpio direction */
	if (flags & GPIO_DIR_OUT)
		gpio->paddir |= BIT(pin);
	else
		gpio->paddir &= ~BIT(pin);

	/*
	 * Configure interrupt if GPIO_INT is set.
	 * Here, we just configure the gpio interrupt behavior,
	 * we do not enable/disable interrupt for a particular
	 * gpio.
	 * Interrupt for a gpio is:
	 * 1) enabled only via a call to gpio_ppu_enable_callback.
	 * 2) disabled only via a call to gpio_ppu_disabled_callback.
	 */
	if (!(flags & GPIO_INT))
		return 0;

	/*
	 * Interrupt cannot be set for GPIO_DIR_OUT
	 */
	if (flags & GPIO_DIR_OUT)
		return -EINVAL;

	/* Double edge trigger not supported */
	if (flags & GPIO_INT_DOUBLE_EDGE)
		return -ENOTSUP;

	/* Edge or Level triggered ? */
	if (flags & GPIO_INT_EDGE)
		gpio->inttype1 |= BIT(pin);
	else
		gpio->inttype1 &= ~BIT(pin);

	/* Level High/Rising Edge ? */
	if (flags & GPIO_INT_ACTIVE_HIGH)
		gpio->inttype0 &= ~BIT(pin);
	else
		gpio->inttype0 |= BIT(pin);

	return 0;
}

/**
 * @brief Set the pin
 *
 * @param dev Device struct
 * @param access_op Access operation
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_ppu_write(struct device *dev,
			      int access_op,
			      u32_t pin,
			      u32_t value)
{
	volatile struct gpio_ppu_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (value)
		gpio->padout |= BIT(pin);
	else
		gpio->padout &= ~BIT(pin);

	return 0;
}

/**
 * @brief Read the pin
 *
 * @param dev Device struct
 * @param access_op Access operation
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_ppu_read(struct device *dev,
			     int access_op,
			     u32_t pin,
			     u32_t *value)
{
	volatile struct gpio_ppu_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	/*
	 * If gpio is configured as output,
	 * read gpio value from padout register,
	 * otherwise read gpio value from padin register
	 */
	if (gpio->paddir & BIT(pin))
		*value = !!(gpio->padout & BIT(pin));
	else
		*value = !!(gpio->padin & BIT(pin));

	return 0;
}

static int gpio_ppu_manage_callback(struct device *dev,
					struct gpio_callback *callback,
					bool set)
{
	struct gpio_ppu_data *data = DEV_GPIO_DATA(dev);

	_gpio_manage_callback(&data->cb, callback, set);

	return 0;
}

static int gpio_ppu_enable_callback(struct device *dev,
					int access_op,
					u32_t pin)
{
	volatile struct gpio_ppu_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	/* Enable interrupt for pin */
	gpio->inten |= BIT(pin);

	return 0;
}

static int gpio_ppu_disable_callback(struct device *dev,
					 int access_op,
					 u32_t pin)
{
	volatile struct gpio_ppu_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	/* Disable interrupt for pin */
	gpio->inten &= ~BIT(pin);

	return 0;
}

static const struct gpio_driver_api gpio_ppu_driver = {
	.config              = gpio_ppu_config,
	.write               = gpio_ppu_write,
	.read                = gpio_ppu_read,
	.manage_callback     = gpio_ppu_manage_callback,
	.enable_callback     = gpio_ppu_enable_callback,
	.disable_callback    = gpio_ppu_disable_callback,
};

/**
 * @brief Initialize a GPIO controller
 *
 * Perform basic initialization of a GPIO controller
 *
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpio_ppu_init(struct device *dev)
{
	const struct gpio_ppu_config *cfg = DEV_GPIO_CFG(dev);

	cfg->gpio_cfg_func();

	return 0;
}

static void gpio_ppu_cfg_0(void);

static const struct gpio_ppu_config gpio_ppu_config0 = {
	.gpio_base_addr    = PPU_GPIO_0_BASE,
	.gpio_cfg_func     = gpio_ppu_cfg_0,
};

static struct gpio_ppu_data gpio_ppu_data0;

DEVICE_AND_API_INIT(gpio_ppu_0, "gpio0", gpio_ppu_init,
		    &gpio_ppu_data0, &gpio_ppu_config0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_ppu_driver);

static void gpio_ppu_cfg_0(void)
{
	IRQ_CONNECT(PPU_GPIO_0_IRQ,
		    0,
		    gpio_ppu_irq_handler,
		    DEVICE_GET(gpio_ppu_0),
		    0);
	irq_enable(PPU_GPIO_0_IRQ);
}
