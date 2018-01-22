#ifndef __CAM_H__
#define __CAM_H__

/**
 * @brief CAM Interface
 * @defgroup cam_interface CAM Interface
 * @ingroup io_interfaces
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_ACCESS_BY_PIN	0
#define CAM_ACCESS_ALL		1

#include <errno.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>

/**
 * @typedef cam_config_t
 * @brief Callback API upon configuration
 * See @a cam_pin_configure() for argument description
 */
typedef int (*cam_display_shade_t)(struct device *dev, u32_t x1);

/**
 * @typedef cam_set_values_t
 * @brief Callback API upon setting PIN values
 * See @a cam_pin_set_values() for argument description
 */
typedef int (*cam_sccb_init)(struct device *dev);

/**
 * @typedef cam_set_duty_cycle_t
 * @brief Callback API upon setting the duty cycle
 * See @a cam_pin_set_duty_cycle() for argument description
 */
typedef int (*cam_display_cam)(struct device *dev, volatile unsigned char* image_addr);

/**
 * @typedef cam_set_phase_t
 * @brief Callback API upon setting the phase
 * See @a cam_pin_set_phase() for argument description
 */
typedef int (*cam_display_image_t)(struct device *dev, u32_t x1, u32_t y1, u32_t x2, u32_t y2, volatile unsigned char* image_addr);

/**
 * @typedef cam_set_period_t
 * @brief Callback API upon setting the period
 * See @a cam_pin_set_period() for argument description
 */
typedef int (*cam_printf_lcd_t)(const char* image_addr, struct device *dev);

/**
 * @typedef cam_pin_set_t
 * @brief Callback API upon setting the pin
 * See @a cam_pin_set_cycles() for argument description
 */
typedef int (*cam_lcd_config)(struct device *dev, struct device *dev1);

/**
 * @typedef cam_get_cycles_per_sec_t
 * @brief Callback API upon getting cycles per second
 * See @a cam_get_cycles_per_sec() for argument description
 */
typedef int (*cam_sccb_write)(struct device *dev, u8_t WriteAddress, u8_t SendByte);

typedef int (*cam_sccb_readbyte)(struct device *dev, u8_t ReadAddress);

typedef int (*cam_ppu_camctl_init)(struct device *dev);

typedef int (*cam_ppu_camctl_start)(struct device *dev);

typedef int (*cam_ppu_lcd_drawrectangle)(struct device *dev, u32_t x1, u32_t y1, u32_t x2, u32_t y2);

/** @brief CAM driver API definition. */
struct cam_driver_api {
	cam_display_shade_t ppu_display_shade;   //
	cam_sccb_init ppu_sccb_init;        
	cam_printf_lcd_t ppu_printf_lcd;         //     
	cam_display_cam ppu_display_cam;
	cam_display_image_t ppu_display_image;   //
	cam_lcd_config ppu_lcd_config;           //
	cam_sccb_write ppu_sccb_write;        //
	cam_sccb_readbyte ppu_sccb_read;
	cam_ppu_camctl_init ppu_camctl_init;
	cam_ppu_camctl_start ppu_camctl_start;
	cam_ppu_lcd_drawrectangle ppu_lcd_drawrectangle;
};
static inline int lcd_config(struct device *dev, struct device *dev1)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_lcd_config(dev, dev1);
}

static inline int display_shade(struct device *dev, u32_t color)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_display_shade(dev, color);
}

static inline int display_cam(struct device *dev, volatile unsigned char* image_addr)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_display_cam(dev, image_addr);
}

static inline int display_image(struct device *dev, u32_t x1, u32_t y1, u32_t x2, u32_t y2, volatile unsigned char* image_addr)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_display_image(dev, x1, y1, x2, y2, image_addr);
}

static inline int printf_lcd(const char* image_addr, struct device *dev)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_printf_lcd(image_addr, dev);
}

static inline int LCD_DrawRectangle(struct device *dev, u32_t x1, u32_t y1, u32_t x2, u32_t y2)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_lcd_drawrectangle(dev,x1, y1, x2, y2);
}

static inline int SCCB_init(struct device *dev)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_sccb_init(dev);
}

static inline int SCCB_WriteByte(struct device *dev, u8_t WriteAddress, u8_t SendByte)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_sccb_write(dev, WriteAddress, SendByte);
}

static inline int SCCB_ReadByte(struct device *dev, u8_t ReadAddress)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_sccb_read(dev, ReadAddress);
}

static inline int camctl_init(struct device *dev)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_camctl_init(dev);
}

static inline int camctl_start(struct device *dev)
{
	struct cam_driver_api *api;

	api = (struct cam_driver_api *)dev->driver_api;
	return api->ppu_camctl_start(dev);
}


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __CAM_H__ */
