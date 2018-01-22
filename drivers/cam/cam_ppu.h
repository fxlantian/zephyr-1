#ifndef CAMCTL_H
#define CAMCTL_H

// macro functions
#define DATAOK_FRAME1(REG)  ( REG       & 0x1 )
#define DATAOK_FRAME2(REG)  ((REG >> 1) & 0x1 )
#define DATAOK_FRAME3(REG)  ((REG >> 2) & 0x1 )
#define RQ_FUL(REG)         ((REG >> 3) & 0x1 )
#define RQ_OVF(REG)         ((REG >> 4) & 0x1 )
#define PROTOCAL_ERR0(REG)  ((REG >> 5) & 0x1 )
#define PROTOCAL_ERR1(REG)  ((REG >> 6) & 0x1 )
#define CUR_FRAME(REG)      ((REG >> 7) & 0x3 )

#define CLR_FRAME1_INT(REG)   ( REG | 0x1 )
#define CLR_FRAME2_INT(REG)   ( REG | 0x2 )
#define CLR_FRAME3_INT(REG)   ( REG | 0x4 )
#define CLR_RQFUL_INT(REG)    ( REG | 0x8 )
#define CLR_RQOVF_INT(REG)    ( REG | 0x10 )
#define CLR_PROERR0_INT(REG)  ( REG | 0x20 )
#define CLR_PROERR1_INT(REG)  ( REG | 0x40 )

#define SET_FRAME1_INT_EN(REG)  ( REG | 0x1 )
#define SET_FRAME2_INT_EN(REG)  ( REG | 0x2 )
#define SET_FRAME3_INT_EN(REG)  ( REG | 0x4 )
#define SET_RQFUL_INT_EN(REG)   ( REG | 0x8 )
#define SET_RQOVF_INT_EN(REG)   ( REG | 0x10 )
#define SET_PROERR_INT_EN(REG)  ( REG | 0x60 )

#define UNSET_FRAME1_INT_EN(REG)  ( REG & ~0x1 )
#define UNSET_FRAME2_INT_EN(REG)  ( REG & ~0x2 )
#define UNSET_FRAME3_INT_EN(REG)  ( REG & ~0x4 )
#define UNSET_RQFUL_INT_EN(REG)   ( REG & ~0x8 )
#define UNSET_RQOVF_INT_EN(REG)   ( REG & ~0x10 )
#define UNSET_PROERR_INT_EN(REG)  ( REG & ~0x60 )

#define ENABLE_CAPTURE(REG)     ( REG | 0x1 )
//#define DISABLE_CAPTURE(REG)    ( REG & 0xffffffe )
#define DISABLE_CAPTURE(REG)    ( REG & ~0x1 )

// macro constants
#define   SET_FM1_INT   1
#define UNSET_FM1_INT   0

#define   SET_FM2_INT   1
#define UNSET_FM2_INT   0

#define   SET_FM3_INT   1
#define UNSET_FM3_INT   0

#define   SET_RQFUL_INT   1
#define UNSET_RQFUL_INT   0

#define   SET_RQOVF_INT   1
#define UNSET_RQOVF_INT   0

#define   SET_PROERR_INT   1
#define UNSET_PROERR_INT   0

#define FRAME1_ADDR       0x32020000
#define FRAME2_ADDR       0x33040000
#define FRAME3_ADDR       0x34060000

#define CAMCTL_FRAME1_ADDR        ( PPU_CAM_BASE + 0x00)
#define CAMCTL_FRAME2_ADDR        ( PPU_CAM_BASE + 0x04)
#define CAMCTL_FRAME3_ADDR        ( PPU_CAM_BASE + 0x08)

#define CAMCTL_STATUS             ( PPU_CAM_BASE + 0x14)
#define CAMCTL_INT_ENABLE         ( PPU_CAM_BASE + 0x18)
#define CAMCTL_CTRL_REG           ( PPU_CAM_BASE + 0x1C)

typedef void (*irq_config_func_t)(struct device *dev);

struct cam_ppu_t {
    volatile int cam_frame1_addr;
    volatile int cam_frame2_addr;
    volatile int cam_frame3_addr;
    volatile int cam_ien;
    volatile int cam_ctrl;
    volatile int cam_sta;
};

struct cam_ppu_config {
    u32_t cam_base_addr;
    irq_config_func_t irq_config;
};

#endif
