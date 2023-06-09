#ifndef APP_QMA6110P_H
#define APP_QMA6110P_H

#include "nrf_twi_mngr.h"

#define TSK_QMA6110P_STACK_SIZE           (64)	// the unit is word, 4-bytes
#define TSK_QMA6110P_PRIORITY             (1)

#define QMA6110P_ADDR                     (0x12) // ad0 grounded in schematic

#define QMA6110P_REG_CHIP_ID      (0x00)
#define QMA6110P_REG_DXL          (0x01)
#define QMA6110P_REG_DXM          (0x02)
#define QMA6110P_REG_DYL          (0x03)
#define QMA6110P_REG_DYM          (0x04)
#define QMA6110P_REG_DZL          (0x05)
#define QMA6110P_REG_DZM          (0x06)
#define QMA6110P_REG_STEP_CNTL    (0x07)
#define QMA6110P_REG_STEP_CNTH    (0x08)
#define QMA6110P_REG_INT_ST0      (0x09)
#define QMA6110P_REG_INT_ST1      (0x0a)
#define QMA6110P_REG_INT_ST2      (0x0b)
#define QMA6110P_REG_INT_ST3      (0x0c)
#define QMA6110P_REG_INT_ST4      (0x0d)
#define QMA6110P_REG_FIFO_ST      (0x0e)
#define QMA6110P_REG_FSR          (0x0f)
#define QMA6110P_REG_BW           (0x10)
#define QMA6110P_REG_PM           (0x11)
#define QMA6110P_REG_STEP_CONF0   (0x12)
#define QMA6110P_REG_STEP_CONF1   (0x13)
#define QMA6110P_REG_STEP_CONF2   (0x14)
#define QMA6110P_REG_STEP_CONF3   (0x15)
#define QMA6110P_REG_INT_EN0      (0x16)
#define QMA6110P_REG_INT_EN1      (0x17)
#define QMA6110P_REG_INT_EN2      (0x18)
#define QMA6110P_REG_INT_MAP0     (0x19)
#define QMA6110P_REG_INT_MAP1     (0x1a)
#define QMA6110P_REG_INT_MAP2     (0x1b)
#define QMA6110P_REG_INT_MAP3     (0x1c)
#define QMA6110P_REG_STEP_CFG0    (0x1d)
#define QMA6110P_REG_STEP_CFG1    (0x1e)
#define QMA6110P_REG_STEP_CFG2    (0x1f)
#define QMA6110P_REG_INTPIN_CONF  (0x20)
#define QMA6110P_REG_INT_CFG      (0x21)
#define QMA6110P_REG_RAISE_WAKE0  (0x22)
#define QMA6110P_REG_RAISE_WAKE1  (0x23)
#define QMA6110P_REG_RAISE_WAKE2  (0x24)
#define QMA6110P_REG_RAISE_WAKE3  (0x25)
#define QMA6110P_REG_RAISE_WAKE4  (0x26)
#define QMA6110P_REG_OS_CUST_X    (0x27)
#define QMA6110P_REG_OS_CUST_Y    (0x28)
#define QMA6110P_REG_OS_CUST_Z    (0x29)
#define QMA6110P_REG_TAPL         (0x2a)
#define QMA6110P_REG_TAPH         (0x2b)
#define QMA6110P_REG_MOT_CFG0     (0x2c)
#define QMA6110P_REG_MOT_CFG1     (0x2d)
#define QMA6110P_REG_MOT_CFG2     (0x2e)
#define QMA6110P_REG_MOT_CFG3     (0x2f)
#define QMA6110P_REG_RST_MOT      (0x30)
#define QMA6110P_REG_FIFO_WM      (0x31)
#define QMA6110P_REG_ST           (0x32)
#define QMA6110P_REG_INERNAL      (0x33)
#define QMA6110P_REG_YTH_YZTHSEL  (0x34)
#define QMA6110P_REG_XTH_ZTH      (0x35)
#define QMA6110P_REG_S_RESET      (0x36)
#define QMA6110P_REG_IMAGE0       (0x37)
#define QMA6110P_REG_IMAGE1       (0x38)
#define QMA6110P_REG_IMAGE2       (0x39)
#define QMA6110P_REG_IMAGE3       (0x3a)
#define QMA6110P_REG_IMAGE4       (0x3b)
#define QMA6110P_REG_IMAGE5       (0x3c)
#define QMA6110P_REG_IMAGE6       (0x3d)
#define QMA6110P_REG_FIFO_CFGL    (0x3e)
#define QMA6110P_REG_FIFO_CFGH    (0x3f)

#define QMA6110P_NUMBER_OF_REGS   64



#define QMA6110P_READ(p_reg_addr, p_buffer, byte_cnt) \
    NRF_TWI_MNGR_WRITE(QMA6110P_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
    NRF_TWI_MNGR_READ (QMA6110P_ADDR, p_buffer,   byte_cnt, 0)

/*
 * This file defines m601z interface.
 */
void app_qma6110p_freertos_init(void);

#endif
