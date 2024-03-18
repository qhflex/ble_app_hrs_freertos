#ifndef APP_ADS1292R_H
#define APP_ADS1292R_H

/****************************************************************/
/* ADS1x9x COMMAND DESCRIPTION and definations */
/****************************************************************/
 // System Commands
 #define ADS1292R_CMD_WAKEUP    0x02      // Wake-up from standby mode
 #define ADS1292R_CMD_STANDBY   0x04      // Enter standby mode
 #define ADS1292R_CMD_RESET     0x06      // Reset the device
 #define ADS1292R_CMD_START     0x08      // Start/restart (synchronize) conversions
 #define ADS1292R_CMD_STOP      0x0A      // Stop conversion
 
// Data Read Commands
 #define ADS1292R_CMD_RDATAC    0x10      // Enable Read Data Continuous mode.
                                          // This mode is the default mode at power-up.
 #define ADS1292R_CMD_SDATAC    0x11      // Stop Read Data Continuously mode
 #define ADS1292R_CMD_RDATA     0x12      // Read data by command; supports multiple read back.
 
 // Register Read Commands
 #define ADS1292R_CMD_RREG      0x20      // Read n nnnn registers starting at address r rrrr
                                          // first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
 #define ADS1292R_CMD_WREG      0x40      // Write n nnnn registers starting at address r rrrr
                                          // first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define TSK_ADS1292R_STACK_SIZE           (1024)	// the unit is word, 4-bytes
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

void app_ads1292r_freertos_init(void);

#endif
