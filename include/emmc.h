#ifndef __EMMC_H__
#define __EMMC_H__

/**
 * @brief EMMC Interface
 * @defgroup emmc_interface EMMC Interface
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

//extern Card_info Card_info_inst[2];
//extern Card_info *the_card_info = Card_info_inst;

#define SD_SET_WIDE_BUS
#define PROCESSOR_CLK       50000     //In KH. 50MHz.

#define SD_MAX_OPFREQUENCY  25000 
#define MMC_MAX_OPFREQUENCY 20000 

#define  FIFO_DEPTH      8
#define  SD_MMC_IRQ      5         //IRQ number for core.

#define MAX_CARDS        16 //SD_MMC mode
#define MAX_HSMMC_CARDS  2  //HSMMC sepcific constant.
#define MAX_HSSD_CARDS   2  //HSSD sepcific constant.

#ifdef PCI_CARD_CIU
#define CIU_CLK       50000
#else
#define CIU_CLK       50000 /* In KHz. 24 MHz */
#endif
#define SD_MAX_OPFREQUENCY  25000       
#define MMC_MAX_OPFREQUENCY 20000

#define  CARD_OCR_RANGE  0x00300000 /* 3.2-3.4V bits 20,21 set. */
#define  SD_VOLT_RANGE   0x80ff8000 /* 2.7-3.6V bits 20,21 set. */
#define  MMC_33_34V      0x80ff8000 /* 2.7-3.6V bits 20,21 set. */

#define  FIFO_DEPTH      8
#define  SD_MMC_IRQ      5  /* IRQ number for core. */

#define MMC_FREQDIVIDER    CIU_CLK/(MMC_MAX_OPFREQUENCY+1)
#define SD_FREQDIVIDER     CIU_CLK/(SD_MAX_OPFREQUENCY+1)

#define DIVIDER_FOR_FREQ(x) ((CIU_CLK)/((x)*2))

#define MAX_THRESHOLD  0x0fff
#define MIN_THRESHOLD  100

#define FOD_VALUE      400  /* 400 KHz */
#define SD_FOD_VALUE   125  /* 125 KHz */
#define FOD_VALUE_KHZ  125  /* 300 KHz */
#define MMC_FOD_VALUE  125  /* 125 KHz */

#define MMC_FOD_DIVIDER_VALUE   ((CIU_CLK/(MMC_FOD_VALUE*2))+1)
#define SD_FOD_DIVIDER_VALUE  ((CIU_CLK/(SD_FOD_VALUE*2))+1)
#define ONE_BIT_BUS_FREQ        35
#define SD_ONE_BIT_BUS_FREQ     ONE_BIT_BUS_FREQ
#define HIGH_SPEED_FREQ   DIVIDER_FOR_FREQ((CIU_CLK/2))
#define SD_HIGH_SPEED_FREQ  DIVIDER_FOR_FREQ((CIU_CLK/2))
#define FOD_DIVIDER_VALUE (CIU_CLK/(FOD_VALUE*2))+1

#if 1
#define 	EMMC_REG_CTRL          (PPU_EMMC_BASE + 0x0 )     /** Control */
#define 	EMMC_REG_PWREN  	   (PPU_EMMC_BASE + 0x4 ) 	  /** Power-enable */
#define 	EMMC_REG_CLKDIV 	   (PPU_EMMC_BASE + 0x8 ) 	  /** Clock divider */
#define 	EMMC_REG_CLKSRC 	   (PPU_EMMC_BASE + 0xC ) 	  /** Clock source */
#define 	EMMC_REG_CLKENA 	   (PPU_EMMC_BASE + 0x10 )    /** Clock enable */
#define 	EMMC_REG_TMOUT  	   (PPU_EMMC_BASE + 0x14 )    /** Timeout */
#define 	EMMC_REG_CTYPE  	   (PPU_EMMC_BASE + 0x18 )    /** Card type */
#define 	EMMC_REG_BLKSIZ 	   (PPU_EMMC_BASE + 0x1C )    /** Block Size */
#define 	EMMC_REG_BYTCNT 	   (PPU_EMMC_BASE + 0x20 )    /** Byte count */
#define 	EMMC_REG_INTMSK 	   (PPU_EMMC_BASE + 0x24 )    /** Interrupt Mask */
#define 	EMMC_REG_CMDARG 	   (PPU_EMMC_BASE + 0x28 )    /** Command Argument */
#define 	EMMC_REG_CMD    	   (PPU_EMMC_BASE + 0x2C )    /** Command */
#define 	EMMC_REG_RESP0  	   (PPU_EMMC_BASE + 0x30 )    /** Response 0 */
#define 	EMMC_REG_RESP1  	   (PPU_EMMC_BASE + 0x34 )    /** Response 1 */
#define 	EMMC_REG_RESP2  	   (PPU_EMMC_BASE + 0x38 )    /** Response 2 */
#define 	EMMC_REG_RESP3  	   (PPU_EMMC_BASE + 0x3C )    /** Response 3 */
#define 	EMMC_REG_MINTSTS	   (PPU_EMMC_BASE + 0x40 )    /** Masked interrupt status */
#define 	EMMC_REG_RINTSTS	   (PPU_EMMC_BASE + 0x44 )    /** Raw interrupt status */
#define 	EMMC_REG_STATUS 	   (PPU_EMMC_BASE + 0x48 )    /** Status */
#define 	EMMC_REG_FIFOTH 	   (PPU_EMMC_BASE + 0x4C )    /** FIFO threshold */
#define 	EMMC_REG_CDETECT	   (PPU_EMMC_BASE + 0x50 )    /** Card detect */
#define 	EMMC_REG_WRTPRT 	   (PPU_EMMC_BASE + 0x54 )    /** Write protect */
#define 	EMMC_REG_GPIO   	   (PPU_EMMC_BASE + 0x58 )    /** General Purpose IO */
#define 	EMMC_REG_TCBCNT 	   (PPU_EMMC_BASE + 0x5C )    /** Transferred CIU byte count */
#define 	EMMC_REG_TBBCNT 	   (PPU_EMMC_BASE + 0x60 )    /** Transferred host/DMA to/from byte count */
#define 	EMMC_REG_DEBNCE 	   (PPU_EMMC_BASE + 0x64 )    /** Card detect debounce */
#define 	EMMC_REG_USRID  	   (PPU_EMMC_BASE + 0x68 )    /** User ID */
#define 	EMMC_REG_VERID  	   (PPU_EMMC_BASE + 0x6C )    /** Version ID */
#define 	EMMC_REG_HCON   	   (PPU_EMMC_BASE + 0x70 )    /** Hardware Configuration */
#define 	EMMC_REG_UHSREG 	   (PPU_EMMC_BASE + 0x74 )    /** Reserved */  
#define 	EMMC_REG_BMOD	       (PPU_EMMC_BASE + 0x80 )    /** Bus mode Register */
#define 	EMMC_REG_PLDMND	   	   (PPU_EMMC_BASE + 0x84 )    /** Poll Demand */
#define 	EMMC_REG_DBADDR		   (PPU_EMMC_BASE + 0x88 )    /** Descriptor Base Address */
#define 	EMMC_REG_IDSTS	 	   (PPU_EMMC_BASE + 0x8C )    /** Internal DMAC Status */
#define 	EMMC_REG_IDINTEN	   (PPU_EMMC_BASE + 0x90 )    /** Internal DMAC Interrupt Enable */
#define 	EMMC_REG_DSCADDR	   (PPU_EMMC_BASE + 0x94 )    /** Current Host Descriptor Address */
#define 	EMMC_REG_BUFADDR	   (PPU_EMMC_BASE + 0x98 )    /** Current Host Buffer Address */
#define     EMMC_REG_UHS_REG_EXT   (PPU_EMMC_BASE + 0x108)    /** FIFO data read write */
 
#define	 	EMMC_REG_FIFODAT	   (PPU_EMMC_BASE + 0x200 )   /** FIFO data read write */
#endif

#define   L_EMMC_REG_CTRL        *(volatile u32_t*) (PPU_EMMC_BASE + 0x0 )    /** Control */
#define   L_EMMC_REG_PWREN       *(volatile u32_t*) (PPU_EMMC_BASE + 0x4 )    /** Power-enable */
#define   L_EMMC_REG_CLKDIV      *(volatile u32_t*) (PPU_EMMC_BASE + 0x8 )    /** Clock divider */
#define   L_EMMC_REG_CLKSRC      *(volatile u32_t*) (PPU_EMMC_BASE + 0xC )    /** Clock source */
#define   L_EMMC_REG_CLKENA      *(volatile u32_t*) (PPU_EMMC_BASE + 0x10 )    /** Clock enable */
#define   L_EMMC_REG_TMOUT       *(volatile u32_t*) (PPU_EMMC_BASE + 0x14 )    /** Timeout */
#define   L_EMMC_REG_CTYPE       *(volatile u32_t*) (PPU_EMMC_BASE + 0x18 )    /** Card type */
#define   L_EMMC_REG_BLKSIZ      *(volatile u32_t*) (PPU_EMMC_BASE + 0x1C )    /** Block Size */
#define   L_EMMC_REG_BYTCNT      *(volatile u32_t*) (PPU_EMMC_BASE + 0x20 )    /** Byte count */
#define   L_EMMC_REG_INTMSK      *(volatile u32_t*) (PPU_EMMC_BASE + 0x24 )    /** Interrupt Mask */
#define   L_EMMC_REG_CMDARG      *(volatile u32_t*) (PPU_EMMC_BASE + 0x28 )    /** Command Argument */
#define   L_EMMC_REG_CMD         *(volatile u32_t*) (PPU_EMMC_BASE + 0x2C )    /** Command */
#define   L_EMMC_REG_RESP0       *(volatile u32_t*) (PPU_EMMC_BASE + 0x30 )    /** Response 0 */
#define   L_EMMC_REG_RESP1       *(volatile u32_t*) (PPU_EMMC_BASE + 0x34 )    /** Response 1 */
#define   L_EMMC_REG_RESP2       *(volatile u32_t*) (PPU_EMMC_BASE + 0x38 )    /** Response 2 */
#define   L_EMMC_REG_RESP3       *(volatile u32_t*) (PPU_EMMC_BASE + 0x3C )    /** Response 3 */
#define   L_EMMC_REG_MINTSTS     *(volatile u32_t*) (PPU_EMMC_BASE + 0x40 )    /** Masked interrupt status */
#define   L_EMMC_REG_RINTSTS     *(volatile u32_t*) (PPU_EMMC_BASE + 0x44 )    /** Raw interrupt status */
#define   L_EMMC_REG_STATUS      *(volatile u32_t*) (PPU_EMMC_BASE + 0x48 )    /** Status */
#define   L_EMMC_REG_FIFOTH      *(volatile u32_t*) (PPU_EMMC_BASE + 0x4C )    /** FIFO threshold */
#define   L_EMMC_REG_CDETECT     *(volatile u32_t*) (PPU_EMMC_BASE + 0x50 )    /** Card detect */
#define   L_EMMC_REG_WRTPRT      *(volatile u32_t*) (PPU_EMMC_BASE + 0x54 )    /** Write protect */
#define   L_EMMC_REG_GPIO        *(volatile u32_t*) (PPU_EMMC_BASE + 0x58 )    /** General Purpose IO */
#define   L_EMMC_REG_TCBCNT      *(volatile u32_t*) (PPU_EMMC_BASE + 0x5C )    /** Transferred CIU byte count */
#define   L_EMMC_REG_TBBCNT      *(volatile u32_t*) (PPU_EMMC_BASE + 0x60 )    /** Transferred host/DMA to/from byte count */
#define   L_EMMC_REG_DEBNCE      *(volatile u32_t*) (PPU_EMMC_BASE + 0x64 )    /** Card detect debounce */
#define   L_EMMC_REG_USRID       *(volatile u32_t*) (PPU_EMMC_BASE + 0x68 )    /** User ID */
#define   L_EMMC_REG_VERID       *(volatile u32_t*) (PPU_EMMC_BASE + 0x6C )    /** Version ID */
#define   L_EMMC_REG_HCON        *(volatile u32_t*) (PPU_EMMC_BASE + 0x70 )    /** Hardware Configuration */
#define   L_EMMC_REG_UHSREG      *(volatile u32_t*) (PPU_EMMC_BASE + 0x74 )    /** Reserved */  
#define   L_EMMC_REG_BMOD        *(volatile u32_t*) (PPU_EMMC_BASE + 0x80 )    /** Bus mode Register */
#define   L_EMMC_REG_PLDMND      *(volatile u32_t*) (PPU_EMMC_BASE + 0x84 )    /** Poll Demand */
#define   L_EMMC_REG_DBADDR      *(volatile u32_t*) (PPU_EMMC_BASE + 0x88 )    /** Descriptor Base Address */
#define   L_EMMC_REG_IDSTS       *(volatile u32_t*) (PPU_EMMC_BASE + 0x8C )    /** Internal DMAC Status */
#define   L_EMMC_REG_IDINTEN     *(volatile u32_t*) (PPU_EMMC_BASE + 0x90 )    /** Internal DMAC Interrupt Enable */
#define   L_EMMC_REG_DSCADDR     *(volatile u32_t*) (PPU_EMMC_BASE + 0x94 )    /** Current Host Descriptor Address */
#define   L_EMMC_REG_BUFADDR     *(volatile u32_t*) (PPU_EMMC_BASE + 0x98 )    /** Current Host Buffer Address */
#define   L_EMMC_REG_UHS_REG_EXT *(volatile u32_t*) (PPU_EMMC_BASE + 0x108)   /** FIFO data read write */
 
#define   L_EMMC_REG_FIFODAT     *(volatile u32_t*) (PPU_EMMC_BASE + 0x200)   /** FIFO data read write */

/* CMD Register Defines */
#define CMD_VOLT_SW_BIT         0x10000000
#define CMD_RESP_EXP_BIT	       0x00000040
#define CMD_RESP_LENGTH_BIT	    0x00000080
#define CMD_CHECK_CRC_BIT	    0x00000100
#define CMD_DATA_EXP_BIT	    0x00000200
#define CMD_RW_BIT		        0x00000400
#define	CMD_TRANSMODE_BIT	    0x00000800
#define CMD_SENT_AUTO_STOP_BIT	0x00001000
#define CMD_WAIT_PRV_DAT_BIT	0x00002000
#define CMD_ABRT_CMD_BIT	    0x00004000
#define CMD_SEND_INIT_BIT 	    0x00008000
#define CMD_SEND_CLK_ONLY	    0x00200000
#define CMD_READ_CEATA		    0x00400000
#define CMD_CCS_EXPECTED	    0x00800000
#define CMD_HOLD_REG          0x20000000
//#ifdef EMULATE_BOOT
#define CMD_ENABLE_BOOT         0x01000000
#define CMD_EXP_BOOT_ACK        0x02000000
#define CMD_DISABLE_BOOT        0x04000000
#define CMD_BOOT_MODE		    0x08000000
//#endif
#define CMD_DONE_BIT		    0x80000000

/* Interrupt mask defines */
#define INTMSK_CDETECT          0x00000001
#define INTMSK_RESP_ERR         0x00000002
#define INTMSK_CMD_DONE         0x00000004
#define INTMSK_DAT_OVER         0x00000008
#define INTMSK_TXDR             0x00000010
#define INTMSK_RXDR             0x00000020
#define INTMSK_RCRC             0x00000040
#define INTMSK_DCRC             0x00000080
#define INTMSK_RTO              0x00000100
#define INTMSK_DTO              0x00000200
#define INTMSK_HTO              0x00000400
#define INTMSK_VSI              INTMSK_HTO   // VSI => Voltage Switch Interrupt
#define INTMSK_FRUN             0x00000800
#define INTMSK_HLE              0x00001000
#define INTMSK_SBE              0x00002000
#define INTMSK_ACD              0x00004000
#define INTMSK_EBE              0x00008000


/*SDIO interrupts are catered from bit 15 through 31*/
#define INTMSK_SDIO_INTR        0xffff0000
#define INTMSK_SDIO_CARD(x)     (1<<(16+x))
#define INTMSK_ALL_ENABLED      0xffffffff

/* Misc defines */
#define VOLT_SWITCH_TIMEOUT_5   1    /* 1 Second is the time kept for Voltage switching to Happen*/
#define VOLT_SWITCH_TIMEOUT_1   1    /* 1 Second is the time kept for Voltage switching to Happen*/
#define CMD_RESP_TIMEOUT  11   /*25 changed from 25 to 11 => wait for 11 seconds: CEATA time out is 10 seconds*/
#define CMD_MAX_RETRIES   1000 /*changed from 100 to be consistent with all polling iterations -- Manju */

#define CCCR_START    0x00
#define CCCR_ENABLE   0x02
#define CCCR_READY    0x03
#define CCCR_BUS_INT_CTRL 0x07
#define CCCR_BUS_SUSPEND  0x0c
#define CCCR_FUNC_SELECT  0x0d
#define CCCR_HIGH_SPEED         0x13
#define CCCR_LENGTH   0x14
#define CCCR_SUSPEND_SUPPORTED  0x08
#define CCCR_BUS_IN_USE   0x01
#define CCCR_EXEC_FLAGS   0x0e
#define CCCR_SUSPEND_BIT  0x02

#define NO_OF_DESCCRIPTORS   8

#define R5_IO_ERR_BITS  0x0000cd00

#define R5_IO_CRC_ERR 0x00008000
#define R5_IO_BAD_CMD 0x00004000
#define R5_IO_GEN_ERR 0x00000800
#define R5_IO_FUNC_ERR  0x00000200
#define R5_IO_OUT_RANGE 0x00000100

/* Control register definitions */
#define CTRL_RESET	           0x00000001
#define FIFO_RESET             0x00000002
#define DMA_RESET	           0x00000004
#define INT_ENABLE	           0x00000010
#define DMA_ENABLE	           0x00000020
#define READ_WAIT	           0x00000040
#define SEND_IRQ_RESP	       0x00000080
#define ABRT_READ_DATA         0x00000100
#define SEND_CCSD	           0x00000200
#define SEND_AS_CCSD	       0x00000400
#define ENABLE_OD_PULLUP       0x01000000

/* Misc Defines */
#define HCON_NUM_CARDS(x)       ((((x&0x3E)>>1))+1)
#define DEFAULT_DEBNCE_VAL      0x0FFFFF
#define GET_FIFO_DEPTH(x)       ((((x)&0x0FFF0000)>>16)+1)
#define FIFO_WIDTH              4		/* in bytes */
#define MAX_DIVIDER_VALUE	    0xff
#define CLK_ONLY_CMD	        0x80200000
#define SET_RCA(x,y)	        ((x)|=(y<<16))

#define SET_BITS(x,y)	        ((x)|=(y))          // Set y bits in x
#define UNSET_BITS(x,y)	        ((x)&=(~(y)))       // Unset y bits in x

#define CARD_PRESENT(x,y)	        (!((emmc_read_register(x, EMMC_REG_CDETECT))&(1<<y)))
#define SET_CARD_NUM(x,y)       ((x)|= ((y)<<16))
#define SET_CMD_INDEX(x,y)      ((x)|= (y&0x3f))

/* Bus Mode Register Bit Definitions */
#define  BMOD_SWR     0x00000001  // Software Reset: Auto cleared after one clock cycle                                0
#define  BMOD_FB    0x00000002  // Fixed Burst Length: when set SINGLE/INCR/INCR4/INCR8/INCR16 used at the start     1 
#define  BMOD_DE    0x00000080  // Idmac Enable: When set IDMAC is enabled                                           7
#define  BMOD_DSL_MSK   0x0000007C  // Descriptor Skip length: In Number of Words                                      6:2 
#define  BMOD_DSL_Shift   2         // Descriptor Skip length Shift value
#define  BMOD_DSL_ZERO          0x00000000  // No Gap between Descriptors
#define  BMOD_DSL_TWO           0x00000008  // 2 Words Gap between Descriptors
#define  BMOD_PBL   0x00000400  // MSIZE in FIFOTH Register 


/* Error Codes */
#define ERRNOERROR              0
#define	ERRCARDNOTCONN	     	1
#define ERRCMDNOTSUPP		    2
#define ERRINVALIDCARDNUM    	3
#define ERRRESPTIMEOUT	    	4
#define ERRCARDNOTFOUND  		5
#define ERRCMDRETRIESOVER   	6
#define	ERRCMDINPROGRESS    	7
#define	ERRNOTSUPPORTED	    	8
#define ERRRESPRECEP		    9
#define ERRENUMERATE    		10
#define	ERRHARDWARE      		11
#define ERRNOMEMORY     		12
#define ERRFSMSTATE	        	14
#define ERRADDRESSRANGE  		15
#define ERRADDRESSMISALIGN	    16
#define ERRBLOCKLEN     		17
#define ERRERASESEQERR   		18
#define ERRERASEPARAM	    	19
#define ERRPROT	        		20
#define	ERRCARDLOCKED    		21
#define ERRCRC		        	22
#define	ERRILLEGALCOMMAND   	23
#define ERRECCFAILED     		24
#define ERRCCERR         		25
#define	ERRUNKNOWN	        	26
#define ERRUNDERRUN      		27
#define ERROVERRUN	        	28
#define ERRCSDOVERWRITE	    	29
#define ERRERASERESET	    	30
#define ERRDATATIMEOUT   		31
#define ERRUNDERWRITE	    	32
#define ERROVERREAD	        	33
#define ERRENDBITERR     		34
#define ERRDCRC		        	35
#define ERRSTARTBIT	        	36
#define ERRTIMEROUT	        	37
#define ERRCARDNOTREADY	     	38
#define ERRBADFUNC	            39
#define ERRPARAM        		40
#define ERRNOTFOUND      		41
#define ERRWRTPRT	        	42


#define ERRIDMACFBE	        	51
#define ERRIDMACDU	        	52
#define ERRIDMACCBE	        	53
#define ERRIDMACPOLLD	        	54
#define ERRIDMACMISC	        	61


#define CMD_MAX_RETRIES		1000 /*changed from 100 to be consistent with all polling iterations -- Manju */

// #ifdef IDMAC_SUPPORT
#define MAX_BUFF_SIZE_IDMAC    8192
#define CTRL_USE_IDMAC         0x02000000  
#define CTRL_IDMAC_RESET       0x00000004    
// #endif

/** 
  * This structure contains the information for the IP which is being used.
  * It maintains certain state information for the IP and also so the border
  * values for the IP which will need to be referred at a later time. 
  */
typedef struct IP_Curent_Status_Info_Tag {
    u32_t operating_mode;   	       /* 1=MMC only,2=SD/MMC. Read only. Not settable */
    u32_t idmac_enabled;             /* 1=IDMA is configured in the hardware         */

    u32_t total_cards;	           /* The total cards on the system                */
    u32_t total_MMC_cards;	       /* Counter for MMC cards                        */
    u32_t total_SD_cards;	           /* Counter for SD cards                         */
    u32_t total_SDIO_cards;	       /* Counter for SDIO cards                       */

    boolean command_path_busy;	   /* TRUE when the command path is in use         */
    boolean data_path_busy;	       /* TRUE when the data path is in use            */

    u32_t max_TAAC_value;	           /* Maximum TAAC value of all connected cards    */
    u32_t max_NSAC_value;	           /* Maximum NSAC value for all connected cards   */

    u32_t max_operating_freq;	       /* Maximum operating frequency.  
				                    * This is calculated and stored after
				                    * timeout values are calculated 
				                    * during initialisation.                        */
    u32_t min_divider_val;	       /* Minimum of the divider values of clk sources. */

    u32_t fifo_depth;		           /* The fifo depth of the IP                      */
    u32_t fifo_threshold;	           /* the fifo threshold which is being used        */
	
    u32_t num_of_cards;	           /* Total number of cards the IP has been 
				                    * configured for                                */
    u32_t present_cdetect;	       /* CDETECT register after the last enumeration   */
    
} IP_status_info;

/*
 *
 * DMA Descriptor Structure
 * The descriptor is of 4 words (32 bits each), but our structrue contains 6 words, where
 * last two words to hold the virtual address of the contents pointed to by desc1 and desc2
 * These two pointers are only for driver's usage, hardware is concerned about these two pointers.
 * The DSL (Descriptor Skip Length) should be programmed properly to incorporate these two extra words.
*/
typedef struct DmaDescStruct {
    u32_t desc0;     /* control and status information of descriptor */
    u32_t desc1;     /* buffer sizes                                 */
    u32_t desc2;     /* physical address of the buffer 1             */  
    u32_t desc3;     /* physical address of the buffer 2             */  
        
    u32_t desc2_virt;  /* virtual address of the buffer  1             */
    u32_t desc3_virt;  /* virtual address of the buffer  2             */ 
}DmaDesc;

/*
One has to maintain proper state information of the card in order to have proper
functionality. Please refer MMC specs for the state information. Note irq is not
considered as the state. btst(Bus test state) of mmc spec is not considered in the driver
*/
typedef enum {
  CARD_STATE_EMPTY = -1,
  CARD_STATE_IDLE = 0,
  CARD_STATE_READY = 1,
  CARD_STATE_IDENT = 2,
  CARD_STATE_STBY = 3,
  CARD_STATE_TRAN = 4,
  CARD_STATE_DATA = 5,
  CARD_STATE_RCV = 6,
  CARD_STATE_PRG = 7,
  CARD_STATE_DIS = 8,
  CARD_STATE_INA = 9
}Card_state;

/*The supported card enumeration types. At present following cards are supported.
Note that once the HSMMC ad HSSD cards come in to picture the enum elements may
increase
*/
typedef enum {
  SD_TYPE = 0,
  SDIO_TYPE = 1,
  MMC_TYPE = 2,
  CEATA_TYPE = 3,
  SDCOMBO_TYPE = 4,
  NONE_TYPE = 5,
  ERRTYPE = 6,
  SD_MEM_2_0_TYPE = 7,
  MMC_4_2_TYPE = 8,
  MMC_4_3_TYPE = 9,
  SD_MEM_3_0_TYPE = 10,
  MMC_4_4_TYPE = 11

}Card_Type;

typedef struct {
	Card_Type card_type;
  	Card_state card_state;
  	union {
    	u32_t csd_dwords[4];
    	u8_t csd_bytes[16];
 	} csd_union;
	union {
		u32_t cid_dwords[4];
		u8_t cid_bytes[16];
	} cid_union;

	union {
		u32_t extcsd_dwords[128];
		u8_t extcsd_bytes[512];
	} extcsd_union;

	union {
		u32_t scr_dwords[2];
		u8_t scr_bytes[8];
	} scr_union;

	u8_t the_cccr_bytes[CCCR_LENGTH];

	u32_t the_rca;
	u32_t card_write_blksize;
	u32_t card_read_blksize;
	u32_t orig_card_write_blksize;
	u32_t orig_card_read_blksize;
	u32_t card_size;
	u32_t divider_val;
	u32_t enum_status;
	u32_t nien;
	u32_t card_bus_width;              // This is added for MMC4.4 support
#ifdef EMULATE_BOOT 
	u32_t is_emmc;              // Indicates whether it is an emmc or MMC4.3 card
	boot_params_t boot_params;      // boot parameters applicable only if card is bootable
#endif
    u32_t is_esd;                    // Indicates whether it is an eSD card.
    u32_t is_volt_switch_supported;  // Indicator to tell whether this SD card supports voltage switching
    u32_t volt_switch_done;          // If this bit is set, the card is operating in low voltage mode => 1.8V

	u32_t mmc_hs_ddr_3v_18v;       // To indicate whether MMC cards support DDR operation or not
	u32_t mmc_hs_ddr_12v;
} Card_info;

//extern Card_info Card_info_inst[2];
//extern Card_info *the_card_info = Card_info_inst;

typedef struct {

    /** the bus structure: This is required for the PCI DMA-able memory allocation
     */
	void * bus_device;   

	/** The error status of the command. 
    * 0 means that there is no error. 
    */
	u32_t error_status;

	/** The callback for the function when command is completed.  
   	*/
  	emmc_postproc_callback postproc_callback;

  	/** The array of dwords which stores the response for the  command. 
   	*  If set to NULL, the response is discarded,
   	*/
  	u32_t *resp_buffer;

  	/** The data buffer for a data command. It is ignored for
   	*  non data commands. used in Slave mode of operation
   	*/
  	u8_t *data_buffer;
    
    u32_t idma_mode_on;              /* This flag indicates the command in question 
                                      is handled with idmac interrupts */
    /** When IDMAC is supported the bit is set to indicate the ISR that,
     *  it is DMA mode of operation not the Slave mode!     
     */
    u32_t slave_mode_int_mask;       /* To keep slave mode interrupts off in DMA opn */
    u32_t idmac_mode_int_mask;       /* Interrupt mask for IDMAC operation           */

    u32_t desc_mode;                 /* 1=>Ring mode 0=> Chain mode                  */
    u32_t desc_count;                /* Total number of Descriptors in the list      */

    DmaDesc *desc_head;            /* Descriptor List Head i.e First Descriptor    */
    dma_addr_t desc_head_dma;      /* Descriptor List Head i.e First Descriptor    */

    u32_t next;                      /* index of Free Descriptor, owned by Driver    */
    u32_t busy;                      /* index of Busy Descriptor, owned by DMA       */
    
    DmaDesc *desc_busy;        /* First Descriptor to be handled by DMA        */
  	DmaDesc *desc_next;        /* First Free Descriptor, owned by Driver       */


  	/** The state of the command in progress. */
  	u32_t cmd_status;

  	/** The number of blocks of to be read/written. */
  	u32_t num_of_blocks;

  	/** The number of bytes already read/written. */
  	u32_t num_bytes_read;

  	/** The user specified memory transfer function.*/
  	emmc_copy_function the_copy_function;

  	/** The termination check function for stream read writes. It is periodically called 
         *  during an open transfer. If this function returns TRUE, the transfer is stopped.
   	*/
  	emmc_term_function the_term_function;

  	/** The slot in which the target card is inserted in. */
  	u32_t slot_num;

  	/** This flag is set if a data command got aborted. */
  	u32_t aborted_command;

  	/** A bus corruption had occured during the data transfer. */
  	u32_t bus_corruption_occured;

  	/** The block size for the current data exchange */
  	u32_t blksize;
#ifdef EMULATE_BOOT 
  	/* Indicates booting is in progress */
  	u32_t booting_in_progress;    // When set to 1 indicates booting is in progress and this should be used  
#endif  
  	u32_t command_index;

	//  struct work_struct scheduled_work; //2.6.30    //ly
} current_task_status;

#define GET_BITS_BETWEEN(x,y,z) ((((z)[y>>5])>>(x&0x1f))&((1<<(y-x+1))-1))


#define SET_BITS_BETWEEN(v,x,y,z) ((z)[y>>5])|=((v&((1<<(y-x+1))-1))<<(x&0x1f))
#define CLEAR_BITS_BETWEEN(v,x,y,z) ((z)[y>>5])&=(~((v&((1<<(y-x+1))-1))<<(x&0x1f)))

/*************** Begin CSD Register Defines **************************/
#define CSD_CSD_STRUCTURE(x)  GET_BITS_BETWEEN(126,127,x)
#define CSD_SPEC_VERS(x)  GET_BITS_BETWEEN(122,125,x)
#define CSD_TAAC(x)   GET_BITS_BETWEEN(112,119,x)
#define CSD_NSAC(x)   GET_BITS_BETWEEN(104,111,x)
#define CSD_TRAN_SPEED(x) GET_BITS_BETWEEN(96,103,x)
#define CSD_CCC(x)    GET_BITS_BETWEEN(84,95,x)
#define CSD_READ_BL_LEN(x)  GET_BITS_BETWEEN(80,83,x)
#define CSD_READ_BL_PARTIAL(x)  GET_BITS_BETWEEN(79,79,x)
#define CSD_WRT_BLK_MISALIGN(x) GET_BITS_BETWEEN(78,78,x)
#define CSD_RD_BLK_MISALIGN(x)  GET_BITS_BETWEEN(77,77,x)
#define CSD_DSR_IMP(x)    GET_BITS_BETWEEN(76,76,x)
#define CSD_VDD_R_CURR_MIN(x) GET_BITS_BETWEEN(59,61,x)
#define CSD_VDD_R_CURR_MAX(x) GET_BITS_BETWEEN(56,58,x)
#define CSD_VDD_W_CURR_MIN(x) GET_BITS_BETWEEN(53,55,x)
#define CSD_VDD_W_CURR_MAX(x) GET_BITS_BETWEEN(50,52,x)
#define CSD_C_SIZE_MULT(x)  GET_BITS_BETWEEN(47,49,x)
#define CSD_ERASE_GRP_SZ(x) GET_BITS_BETWEEN(42,46,x)
#define CSD_ERASE_GRP_MULT(x) GET_BITS_BETWEEN(37,41,x)
#define CSD_WP_GRP_SIZE(x)  GET_BITS_BETWEEN(32,36,x)
#define CSD_WP_GRP_ENABLE(x)  GET_BITS_BETWEEN(31,31,x)
#define CSD_DEFAULT_ECC(x)  GET_BITS_BETWEEN(29,30,x)
#define CSD_R2W_FACTOR(x) GET_BITS_BETWEEN(26,28,x)
#define CSD_WRT_BL_LEN(x) GET_BITS_BETWEEN(22,25,x)
#define CSD_WRT_BL_PARTIAL(x) GET_BITS_BETWEEN(21,21,x)
#define CSD_CONT_PROT_APP(x)  GET_BITS_BETWEEN(16,16,x)
#define CSD_FILE_FORMAT_GRP(x)  GET_BITS_BETWEEN(15,15,x)
#define CSD_COPY_FLAG(x)  GET_BITS_BETWEEN(15,15,x)
#define CSD_PERM_WRITE_PROT(x)  GET_BITS_BETWEEN(13,13,x)
#define CSD_TEMP_WRITE_PROT(x)  GET_BITS_BETWEEN(12,12,x)
#define CSD_FILE_FORMAT(x)  GET_BITS_BETWEEN(10,11,x)
#define CSD_ECC_CODE(x)   GET_BITS_BETWEEN(8,9,x)
#define CSD_CRC(x)    GET_BITS_BETWEEN(1,7,x)

#define CSD_ERASE_BLK_EN(x)     GET_BITS_BETWEEN(46,46,x)
#define CSD_SECTOR_SIZE(x)      GET_BITS_BETWEEN(39,45,x)

#define CSD_C_SIZE(x)           CSD_C_SIZE_INLINE(x)
#define CSD_C_SIZE_SD_2_0(x)    CSD_C_SIZE_SD_2_0_INLINE(x)

static inline u32_t CSD_C_SIZE_INLINE(u32_t * csd_array)
{
  u32_t bits_62_to_63, bits_64_to_73;
  bits_62_to_63 = GET_BITS_BETWEEN(62, 63, csd_array);
  bits_64_to_73 = GET_BITS_BETWEEN(64, 73, csd_array);
  return (bits_62_to_63 | (bits_64_to_73 << 2));
}


static inline u32_t CSD_C_SIZE_SD_2_0_INLINE(u32_t * csd_array)
{
  u32_t bits_48_to_63, bits_64_to_69;
  bits_48_to_63 = GET_BITS_BETWEEN(48, 63, csd_array);
  bits_64_to_69 = GET_BITS_BETWEEN(64, 69, csd_array);
  return (bits_48_to_63 | (bits_64_to_69 << 16));
}


/* Retry counts */
#define CMD1_RETRY_COUNT   1000 /*changed from 50 Just to be cautious--Manju */
//#define CMD1_RETRY_COUNT   2 /*changed from 50 Just to be cautious--Manju */
#define ACMD41_RETRY_COUNT 1000 /*changed from 50 Just to be cautious--Manju */
#define CMD2_RETRY_COUNT   1000 /*changed from 50 Just to be cautious--Manju */
#define CMD5_RETRY_COUNT   1000 /*changed from 50 Just to be cautious--Manju */

/////////////////////////                ////////////////////////////
///                       erase   core                            ///
////////////////////////                 ////////////////////////////

#define the_cid cid_union.cid_dwords
#define the_csd csd_union.csd_dwords
#define the_cid_bytes cid_union.cid_bytes
#define the_csd_bytes csd_union.csd_bytes
#define the_extcsd_bytes extcsd_union.extcsd_bytes
#define the_extcsd extcsd_union.extcsd_dwords
#define the_scr scr_union.scr_dwords
#define the_scr_bytes scr_union.scr_bytes

/* Define Card status bits (R1) */
#define R1CS_ADDRESS_OUT_OF_RANGE       0x80000000
#define R1CS_ADDRESS_MISALIGN       0x40000000
#define R1CS_BLOCK_LEN_ERR          0x20000000
#define R1CS_ERASE_SEQ_ERR              0x10000000
#define R1CS_ERASE_PARAM              0x08000000
#define R1CS_WP_VIOLATION           0x04000000
#define R1CS_CARD_IS_LOCKED             0x02000000
#define R1CS_LCK_UNLCK_FAILED       0x01000000
#define R1CS_COM_CRC_ERROR            0x00800000
#define R1CS_ILLEGAL_COMMAND        0x00400000
#define R1CS_CARD_ECC_FAILED        0x00200000
#define R1CS_CC_ERROR             0x00100000
#define R1CS_ERROR                  0x00080000
#define R1CS_UNDERRUN             0x00040000
#define R1CS_OVERRUN              0x00020000
#define R1CS_CSD_OVERWRITE            0x00010000
#define R1CS_WP_ERASE_SKIP            0x00008000
#define R1CS_RESERVED_0             0x00004000
#define R1CS_ERASE_RESET            0x00002000
#define R1CS_CURRENT_STATE_MASK       0x00001e00
#define R1CS_READY_FOR_DATA           0x00000100
#define R1CS_SWITCH_ERROR           0x00000080
#define R1CS_RESERVED_1             0x00000040
#define R1CS_APP_CMD              0x00000020
#define R1CS_RESERVED_2             0x00000010
#define R1CS_APP_SPECIFIC_MASK        0x0000000c
#define R1CS_MANUFAC_TEST_MASK        0x00000003

#define R1CS_ERROR_OCCURED_MAP        0xfdffa080

#define R1CS_CURRENT_STATE(x)       (((x)&R1CS_CURRENT_STATE_MASK)>>9)

#define GET_FIFO_COUNT(x)             (((x)&0x3ffe0000)>>17)
#define GET_R6_RCA(x)                 (((x)&0xffff0000)>>16)
#define STATUS_FIFO_FULL              0x00000008

#define READY_FOR_DATA_RETRIES          20

/* Operation Conditions Register (OCR) Register Definition */
#define OCR_POWER_UP_STATUS            0x80000000
#define OCR_ACCESSMODE_SECTOR          0x40000000 /*This is to indicate the secor addressing for MMC4.2 High capacity MMC cards */
#define OCR_RESERVED_1               0x7f000000
#define OCR_27TO36                   0x00ff8000
#define OCR_20TO26                   0x00007f00
#define OCR_165TO195               0x00000010
#define OCR_RESERVED_2               0x0000007f
#define MMC_MOBILE_VOLTAGE             OCR_165TO195

#define OCR_CCS                    0x40000000 /*This is sent by card to indicate it is high capcity SD card*/
#define OCR_HCS                    OCR_CCS    /*This is sent by host querying whether card is high capacity?*/

#define OCR_FB                         0x20000000  /* Fast Boot bit reserved for eSD */
#define OCR_XPC                        0x10000000  /* OCR_XPC used to check on SDXC Power Control. If 0 => Power Saving, If 1 => Maximum Performance */
#define OCR_S18R                       0x01000000  /* Switching to 1.8V Request 0 => Use current Signal Voltage, 1 => Switch to 1.8V Signal Voltage */



#define CMD53_FIFO_WRITE_FLAG 0x00000001
#define SDIO_USE_ASYNC_READ 0x00000002
#define CMD53_USE_BLOCK_IO  0x00000004
#define CMD53_GET_BLKSIZ(x) (((x)&0xffff0000)>>16)  
#define FUNC_0      0

#define IO_RW_REG_MAX_VAL (IO_RW_REG_ADD_MSK >> 9)
#define SET_IO_RW_REG_ADD(x,v)  ((x)|=(v<<9))

#define GET_IO_RW_DATA(x) ((x)&0x000000ff)
#define GET_IO_RW_REG_ADD(x)  (((x)&0x03fffe00)>>9)
#define GET_IO_RW_RAW_FLAG(x) (((x)&0x08000000)>>27)
#define GET_IO_RW_FUNC_NUM(x) (((x)&0x70000000)>>28)
#define GET_IO_RW_RW_FLG(x) (((x)&0x80000000)>>31)

#define SET_IO_RW_DATA(v,x) (v|=((x)&0x000000ff))
#define SET_IO_RW_RAW_FLAG(v,x) ((v|=((x)<<27)))
#define SET_IO_RW_FUNC_NUM(v,x) ((v|=((x)<<28)))
#define SET_IO_RW_RW_FLG(v,x) ((v|=((x)<<31)))

#define IO_R4_OCR_MSK   0x00ffffff
#define IO_R4_MEM_PRESENT_BIT 0x08000000
#define IO_R4_NUM_FUNCS_MSK 0x70000000
#define IO_R4_READY_BIT   0x80000000

#define GET_IO_R4_NUM_FUNCS(x)  (((x)&IO_R4_NUM_FUNCS_MSK)>>28)
#define SET_FUNC_NUM(x,v) SET_BITS(x,(v<<28))


/******** CMD52 SDIO IO_RW_DIRECT arg *********************/

#define IO_RW_DATA_MSK    0x000000ff
#define IO_RW_UNUSED1_MSK 0x00000100
#define IO_RW_REG_ADD_MSK 0x03fffe00
#define IO_RW_UNUSED2_MSK 0x04000000
#define IO_RW_RAW_FLAG_MSK  0x08000000
#define IO_RW_FUNC_NUM_MSK  0x70000000
#define IO_RW_RW_FLG_MSK  0x80000000

#define IO_RW53_BMODE_MSK 0x08000000
#define IO_RW53_OP_CODE_MSK 0x04000000
#define IO_RW53_BYTECNT_MSK 0x000001ff

/** 
  * Flag in emmc_read_write_bytes custom command instructing the function
  * not to put the card into the trans state before firing the command.
  */
#define CUSTCOM_DONT_TRANS  0x10000000

/** 
  * Flag in emmc_read_write_bytes custom command instructing the function to
  * use CMD23 to set the block number before firing the data command.
  */
#define CUSTCOM_DO_CMD23  0x20000000

/** 
  * Flag in emmc_read_write_bytes custom command instructing the function not to
  * to go to standby state after the data transfer.
  */
#define CUSTCOM_DONTSTDBY 0x40000000

/**
  * Flag in emmc_read_write_bytes custom command instructing the function not to
  * issue a CMD16 for setting the blocksize before the data command is fired.
  */
#define CUSTCOM_DONT_CMD16  0x01000000

/**
  * Flag in emmc_read_write_bytes custom command instructing the function
  * that the transfer is a stream transfer
  */
#define CUSTCOM_STREAM_RW 0x02000000

/**
  * Flag in emmc_read_write_bytes custom command instructing the function
  * not to populate the BLKSIZ register expecting calling function will do the
  * needful.
  */
#define CUSTCOM_DONT_BLKSIZ 0x80000000


#define CUSTCOM_COMMAND_MSK 0x00000fff
#define CUSTOM_BLKSIZE_MSK  0x000ff000

#define CUSTOM_BLKSIZE_SHIFT  12

#define CUSTOM_BLKSIZE(x) (((x&CUSTOM_BLKSIZE_MSK))>>CUSTOM_BLKSIZE_SHIFT)


enum DmaDescriptorDES0    // Control and status word of DMA descriptor DES0 
{
     DescOwnByDma          = 0x80000000,   /* (OWN)Descriptor is owned by DMA engine              31   */
     DescCardErrSummary    = 0x40000000,   /* Indicates EBE/RTO/RCRC/SBE/DRTO/DCRC/RE             30   */
     DescEndOfRing         = 0x00000020,   /* A "1" indicates End of Ring for Ring Mode           05   */
     DescSecAddrChained    = 0x00000010,   /* A "1" indicates DES3 contains Next Desc Address     04   */
     DescFirstDesc         = 0x00000008,   /* A "1" indicates this Desc contains first            03
                                              buffer of the data                                       */
     DescLastDesc          = 0x00000004,   /* A "1" indicates buffer pointed to by this this      02
                                              Desc contains last buffer of Data                        */
     DescDisInt            = 0x00000002,   /* A "1" in this field disables the RI/TI of IDSTS     01
                                              for data that ends in the buffer pointed to by 
                                              this descriptor                                          */     
};

enum DmaDescriptorDES1    // Buffer's size field of Descriptor
{
     DescBuf2SizMsk       = 0x03FFE000,    /* Mask for Buffer2 Size                            25:13   */
     DescBuf2SizeShift    = 13,            /* Shift value for Buffer2 Size                             */
     DescBuf1SizMsk       = 0x00001FFF,    /* Mask for Buffer1 Size                            12:0    */
     DescBuf1SizeShift    = 0,             /* Shift value for Buffer2 Size                             */
};

enum DescMode
{
  RINGMODE  = 0x00000001,
  CHAINMODE = 0x00000002,
};

enum BufferMode
{
  SINGLEBUF = 0x00000001,
  DUALBUF   = 0x00000002,
};
enum DmaAccessType
{
    TO_DEVICE       = 0x00000001,
    FROM_DEVICE     = 0x00000002,
    BIDIRECTIONAL   = 0x00000003,
};


/************************************************************************/
/**
 * @typedef emmc_config_t
 * @brief Callback API upon configuration
 * See @a emmc_pin_configure() for argument description
 */
typedef int (*emmc_read_register_def)(struct device *dev, u32_t reg);


/**
 * @typedef emmc_set_duty_cycle_t
 * @brief Callback API upon setting the duty cycle
 * See @a emmc_pin_set_duty_cycle() for argument description
 */
typedef int (*emmc_set_bits_def)(struct device *dev, u32_t reg, u32_t val);

typedef int (*emmc_clear_bits_def)(struct device *dev, u32_t reg, u32_t val);

/**
 * @typedef emmc_pin_set_t
 * @brief Callback API upon setting the pin
 * See @a emmc_pin_set_cycles() for argument description
 */
typedef int (*emmc_set_register_def)(struct device *dev, u32_t reg, u32_t val);



typedef int (*emmc_reset_fifo_def)(struct device *dev);

/** @brief EMMC driver API definition. */
struct emmc_driver_api {
	emmc_clear_bits_def ppu_emmc_clear_bits;
	emmc_set_bits_def ppu_emmc_set_bits;
	emmc_read_register_def ppu_emmc_read_register;
	emmc_set_register_def ppu_emmc_set_register;
	emmc_reset_fifo_def ppu_emmc_reset_fifo;
};
static inline int emmc_set_register(struct device *dev, u32_t reg, u32_t val)
{
	struct emmc_driver_api *api;

	api = (struct emmc_driver_api *)dev->driver_api;
	return api->ppu_emmc_set_register(dev, reg, val);
}

static inline int emmc_read_register(struct device *dev, u32_t reg)
{
	struct emmc_driver_api *api;

	api = (struct emmc_driver_api *)dev->driver_api;
	return api->ppu_emmc_read_register(dev, reg);
}

static inline int emmc_set_bits(struct device *dev, u32_t reg, u32_t val)
{
	struct emmc_driver_api *api;

	api = (struct emmc_driver_api *)dev->driver_api;
	return api->ppu_emmc_set_bits(dev, reg, val);
}

static inline int emmc_clear_bits(struct device *dev, u32_t reg, u32_t val)
{
	struct emmc_driver_api *api;

	api = (struct emmc_driver_api *)dev->driver_api;
	return api->ppu_emmc_clear_bits(dev, reg, val);
}

static inline int emmc_reset_fifo(struct device *dev)
{
	struct emmc_driver_api *api;

	api = (struct emmc_driver_api *)dev->driver_api;
	return api->ppu_emmc_reset_fifo(dev);
}


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __EMMC_H__ */
