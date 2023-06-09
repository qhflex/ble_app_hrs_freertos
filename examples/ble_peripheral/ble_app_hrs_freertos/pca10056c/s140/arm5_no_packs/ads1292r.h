#ifndef APP_ADS1292R_H
#define APP_ADS1292R_H

#define TSK_ADS1292R_STACK_SIZE           (64)	// the unit is word, 4-bytes
#define TSK_ADS1292R_PRIORITY             (1)

#define ADS1292R_REG_ID                   (0x00)
#define ADS1292R_REG_CFG1                 (0x01)
#define ADS1292R_REG_CFG2                 (0x02)
#define ADS1292R_REG_LOFF                 (0x03)
#define ADS1292R_REG_CH1SET               (0x04)
#define ADS1292R_REG_CH2SET               (0x05)
#define ADS1292R_REG_RLD_SENS             (0x06)
#define ADS1292R_REG_LOFF_SENS            (0x07)
#define ADS1292R_REG_LOFF_STAT            (0x08)
#define ADS1292R_REG_RESP1                (0x09)
#define ADS1292R_REG_RESP2                (0x0a)
#define ADS1292R_REG_GPIO                 (0x0b)

#endif
