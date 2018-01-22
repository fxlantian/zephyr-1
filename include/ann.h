#ifndef __ANN_H__
#define __ANN_H__

/**
 * @brief ANN Interface
 * @defgroup ann_interface ANN Interface
 * @ingroup io_interfaces
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define ANN_ACCESS_BY_PIN	0
#define ANN_ACCESS_ALL		1

#include <errno.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>

/**
 * @typedef ann_config_t
 * @brief Callback API upon configuration
 * See @a ann_pin_configure() for argument description
 */
typedef int (*ann_normalization_image)(struct device *dev, volatile u8_t* image_sampled);


/**
 * @typedef ann_set_duty_cycle_t
 * @brief Callback API upon setting the duty cycle
 * See @a ann_pin_set_duty_cycle() for argument description
 */
typedef int (*ann_qfix_image)(volatile u8_t* image_addr, u8_t* dst_image);

typedef int (*ann_npu_pro)(int npu_datain_depth, int npu_detaout_depth, int DMA_src_addr, int DMA_dst_addr, int block_size);

/**
 * @typedef ann_pin_set_t
 * @brief Callback API upon setting the pin
 * See @a ann_pin_set_cycles() for argument description
 */
typedef int (*ann_sample_image)(struct device *dev, volatile u8_t* image_addr, u8_t* image_sampled);



typedef int (*ann_ppu_init_pro)(int im_depth, int weight_depth, int bias_depth, int DMA_src_addr,  int block_size);

/** @brief ANN driver API definition. */
struct ann_driver_api {        
	ann_npu_pro ppu_npu_pro;         //     
	ann_qfix_image ppu_qfix_image;
	ann_normalization_image ppu_normalization_image;   //
	ann_sample_image ppu_sample_image;           //
	ann_ppu_init_pro ppu_init_pro;
};
static inline int sample_image(struct device *dev, volatile u8_t* image_addr, u8_t* image_sampled)
{
	struct ann_driver_api *api;

	api = (struct ann_driver_api *)dev->driver_api;
	return api->ppu_sample_image(dev, image_addr, image_sampled);
}

static inline int normalization_image(struct device *dev, volatile u8_t* image_sampled)
{
	struct ann_driver_api *api;

	api = (struct ann_driver_api *)dev->driver_api;
	return api->ppu_normalization_image(dev, image_sampled);
}

static inline int qfix_image(struct device *dev, volatile u8_t* image_addr, u8_t* dst_image)
{
	struct ann_driver_api *api;

	api = (struct ann_driver_api *)dev->driver_api;
	return api->ppu_qfix_image(image_addr, dst_image);
}

static inline int npu_pro(struct device *dev, int npu_datain_depth, int npu_detaout_depth, int DMA_src_addr, int DMA_dst_addr, int block_size)
{
	struct ann_driver_api *api;

	api = (struct ann_driver_api *)dev->driver_api;
	return api->ppu_npu_pro(npu_datain_depth, npu_detaout_depth, DMA_src_addr,  DMA_dst_addr, block_size);
}

static inline int init_pro(struct device *dev, int im_depth, int weight_depth, int bias_depth, int DMA_src_addr,  int block_size)
{
	struct ann_driver_api *api;

	api = (struct ann_driver_api *)dev->driver_api;
	return api->ppu_init_pro(im_depth, weight_depth, bias_depth, DMA_src_addr, block_size);
}


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __ANN_H__ */
