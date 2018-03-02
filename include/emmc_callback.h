#ifndef __EMMC_CALLBACK_H__
#define __EMMC_CALLBACK_H__

/**
 * @brief EMMC_CALLBACK Interface
 * @defgroup emmc_callback_interface EMMC_CALLBACK Interface
 * @ingroup io_interfaces
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define EMMC_ACCESS_BY_PIN	0
#define EMMC_ACCESS_ALL		1

#include <errno.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>
#include "emmc_callback.h"
#include <string.h>


typedef void (*emmc_postproc_callback)(void *,u32_t *) ;
typedef u32_t (*emmc_term_function)(u8_t* buffer, u32_t bytes_read);
typedef void* (*emmc_copy_function)(u32_t *,u32_t *,u32_t);

typedef void (*emmc_postproc_callback)(void *,u32_t *) ;
typedef void (*emmc_preproc_callback)(u32_t,u32_t,u32_t*,u32_t*);

typedef struct {
	emmc_preproc_callback preproc;
	emmc_postproc_callback postproc;
} Callbacks;

typedef struct {
	u32_t cmd_index;
	Callbacks the_callbacks;
} callback_search_table;


typedef unsigned long dma_addr_t;

typedef int boolean ;
typedef unsigned int int_register ;

/* Status register bits */
#define STATUS_DAT_BUSY_BIT	    0x00000200

#define R4_RESP_ERROR_BIT	           0x00010000
#define CMD39_WRITE_REG		           0x00008000

/* Internal DMAC Status Register IDSTS Bit Definitions
 * Internal DMAC Interrupt Enable Register Bit Definitions */
#define  IDMAC_AI			     0x00000200   // Abnormal Interrupt Summary Enable/ Status                                       9
#define  IDMAC_NI    		   0x00000100   // Normal Interrupt Summary Enable/ Status                                         8
#define  IDMAC_CES				 0x00000020   // Card Error Summary Interrupt Enable/ status                                     5
#define  IDMAC_DU				   0x00000010   // Descriptor Unavailabe Interrupt Enable /Status                                  4
#define  IDMAC_FBE				 0x00000004   // Fata Bus Error Enable/ Status                                                   2
#define  IDMAC_RI				   0x00000002   // Rx Interrupt Enable/ Status                                                     1
#define  IDMAC_TI				   0x00000001   // Tx Interrupt Enable/ Status                                                     0

/* Definitions for cmd status */
#define TSK_STAT_STARTED	1
#define TSK_STATE_READRESP	2
#define TSK_STATE_READDAT	3
#define TSK_STATE_WRITEDAT	4
#define TSK_COMMAND_DONE	5
#define TSK_COMMAND_ABORTING	6
#define TSK_STATE_POLLD   	7


#define TSK_STAT_ABSENT		0


#define DATA_RECV	0
#define DATA_SEND	1

/* Standard MMC commands (3.1)           type  argument     response */
/* class 1 */
#define CMD0    0   /* MMC_GO_IDLE_STATE        bc                    */
#define CMD1    1   /* MMC_SEND_OP_COND         bcr  [31:0]  OCR  R3  */
#define CMD2    2   /* MMC_ALL_SEND_CID         bcr               R2  */
#define CMD3    3   /* MMC_SET_RELATIVE_ADDR    ac   [31:16] RCA  R1  */
#define CMD4    4   /* MMC_SET_DSR              bc   [31:16] RCA      */

#define CMD5    5   /* SDIO_SEND_OCR            ??   ??               */

#define CMD6    6   /* HSMMC_SWITCH             ac                R1  */
                    /* For ACMD6:SET_BUS_WIDTH  ??   ??               */

#define CMD7    7   /* MMC_SELECT_CARD          ac   [31:16] RCA  R1  */
#define CMD8    8   /* HSMMC_SEND_EXT_CSD       adtc [31:16] RCA  R1  */
#define CMD9    9   /* MMC_SEND_CSD             ac   [31:16] RCA  R2  */
#define CMD10   10  /* MMC_SEND_CID             ac   [31:16] RCA  R2  */
#define CMD11   11  /* MMC_READ_DAT_UNTIL_STOP  adtc [31:0]  dadr R1  */
#define CMD12   12  /* MMC_STOP_TRANSMISSION    ac                R1b */
#define CMD13   13  /* MMC_SEND_STATUS          ac   [31:16] RCA  R1  */
#define ACMD13  13  /* SD_STATUS                ac   [31:2] Stuff,
                                                     [1:0]Buswidth  R1*/
#define CMD14   14  /* HSMMC_BUS_TESTING        adtc [31:16] stuff R1 */
#define CMD15   15  /* MMC_GO_INACTIVE_STATE    ac   [31:16] RCA  */
#define CMD19   19  /* HSMMC_BUS_TESTING        adtc [31:16] stuff R1 */

/* class 2 */
#define CMD16   16  /* MMC_SET_BLOCKLEN         ac   [31:0] blkln R1  */
#define CMD17   17  /* MMC_READ_SINGLE_BLOCK    adtc [31:0] dtadd R1  */
#define CMD18   18  /* MMC_READ_MULTIPLE_BLOCK  adtc [31:0] dtadd R1  */

/* class 3 */
#define CMD20   20  /* MMC_WRITE_DAT_UNTIL_STOP adtc [31:0] dtadd R1  */

/* class 4 */
#define CMD23   23  /* MMC_SET_BLOCK_COUNT      adtc [31:0] dtadd R1  */
#define CMD24   24  /* MMC_WRITE_BLOCK          adtc [31:0] dtadd R1  */
#define CMD25   25  /* MMC_WRITE_MULTIPLE_BLOCK adtc              R1  */
#define CMD26   26  /* MMC_PROGRAM_CID          adtc              R1  */
#define CMD27   27  /* MMC_PROGRAM_CSD          adtc              R1  */

/* class 6 */
#define CMD28   28  /* MMC_SET_WRITE_PROT       ac   [31:0] dtadd R1b */
#define CMD29   29  /* _CLR_WRITE_PROT          ac   [31:0] dtadd R1b */
#define CMD30   30  /* MMC_SEND_WRITE_PROT      adtc [31:0] wpdtaddr R1  */

/* class 5 */
#define CMD32   32  /* SD_ERASE_GROUP_START    ac   [31:0] dtadd  R1  */
#define CMD33   33  /* SD_ERASE_GROUP_END      ac   [31:0] dtaddr R1  */

#define CMD35   35  /* MMC_ERASE_GROUP_START    ac   [31:0] dtadd  R1  */
#define CMD36   36  /* MMC_ERASE_GROUP_END      ac   [31:0] dtaddr R1  */
#define CMD38   38  /* MMC_ERASE                ac                 R1b */

/* class 9 */
#define CMD39   39  /* MMC_FAST_IO              ac   <Complex>     R4  */
#define CMD40   40  /* MMC_GO_IRQ_STATE         bcr                R5  */

#define ACMD41  41  /* SD_SEND_OP_COND          ??                 R1  */
#define ACMD42  42  /* SET_CLR_CARD_DETECT      ac                 R1  */

/* class 7 */
#define CMD42   42  /* MMC_LOCK_UNLOCK          adtc               R1b */

#define ACMD51  51  /* SEND_SCR                 adtc               R1  */

#define CMD52   52  /* SDIO_RW_DIRECT           ??                 R5  */
#define CMD53   53  /* SDIO_RW_EXTENDED         ??                 R5  */

/* class 8 */
#define CMD55   55  /* MMC_APP_CMD              ac   [31:16] RCA   R1  */
#define CMD56   56  /* MMC_GEN_CMD              adtc [0] RD/WR     R1b */

// For CE-ATA Drive
#define CMD60 60
#define CMD61 61

#define SDIO_RESET  100  //To differentiate CMD52 for IORESET and other rd/wrs.
#define SDIO_ABORT  101  //To differentiate CMD52 for IO ABORT and other rd/wrs.


#define UNADD_OFFSET  200
#define UNADD_CMD7      207
#define WCMC52        252
#define WCMD53        253
#define WCMD60        260
#define WCMD61        261
#define ACMD6         206
#define SD_CMD8       208  /*This is added to support SD 2.0 (SDHC) cards*/
#define SD_CMD11      211  /*This is added to support SDXC Voltage Switching*/



typedef int (*emmc_set_current_task_status_t)(struct device *dev, u32_t slot, u32_t * resp_buffer,
              u8_t * data_buffer,
              emmc_postproc_callback
              the_completion_callback);

typedef int (*emmc_get_post_callback_t)(struct device *dev, u32_t cmd_index);

typedef int (*emmc_get_pre_callback_t)(struct device *dev, u32_t cmd_index);

/** @brief EMMC_CALLBACK driver API definition. */
struct emmc_callback_driver_api {
	//emmc_callback_clear_bits_t ppu_emmc_clear_bits;
	emmc_get_pre_callback_t ppu_emmc_get_pre_callback;
	emmc_get_post_callback_t ppu_emmc_get_post_callback;
	emmc_set_current_task_status_t ppu_emmc_set_current_task_status;
	//ppu_init_pro_t ppu_init_pro;
};

/**
  * This function updates the current_task structure just before issuing the command.
  * This function complements the emmc_set_data_trans_params() function. The differnce being this function is called
  * following the emmc_set_data_trans_params() for all data transfer commands, but for non-data transfer commands, only
  * this function is called but not the emmc_set_data_trans_params() function.
  * @param dev Pointer to the device structure for the driver instance.
  * @param[in] slot number to which card is connected.
  * @param[in] pointer to response buffer.
  * @param[in] pointer to data buffer.
  * @param[in] function pointer to completion callback routine.
  * \return returns void.
  * \callgraph
  */
static inline int emmc_set_current_task_status(struct device *dev, u32_t slot, u32_t * resp_buffer,
              u8_t * data_buffer,
              emmc_postproc_callback
              the_completion_callback)
{
	struct emmc_callback_driver_api *api;

	api = (struct emmc_callback_driver_api *)dev->driver_api;
	return api->ppu_emmc_set_current_task_status(dev, slot, resp_buffer, data_buffer, the_completion_callback);
}

/** Looks up the table for a pre processing callback for the cmd.
  * This function looks up the has table of function pointers to locate the
  * appropriate postprocessing callback for the index.
  * @param[in] cmd_index. The command which is to be sent on the bus.
  * \return The function pointer to the pre processing function.
  */
 static inline int emmc_get_pre_callback(struct device *dev, u32_t cmd_index)
{
	struct emmc_callback_driver_api *api;

	api = (struct emmc_callback_driver_api *)dev->driver_api;
	return api->ppu_emmc_get_pre_callback(dev, cmd_index);
}

/** Looks up the table for a post processing callback for the cmd.
  * This function looks up the has table of function pointers to locate the
  * appropriate postprocessing callback for the index.
  * @param dev Pointer to the device structure for the driver instance.
  * @param[in] cmd_index	The command which is to be sent on the bus.
  * \return The function pointer to the post processing function.
  * \callgraph
  */
static inline int emmc_get_post_callback(struct device *dev, u32_t cmd_index)
{
	struct emmc_callback_driver_api *api;

	api = (struct emmc_callback_driver_api *)dev->driver_api;
	return api->ppu_emmc_get_post_callback(dev, cmd_index);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __EMMC_CALLBACK_H__ */