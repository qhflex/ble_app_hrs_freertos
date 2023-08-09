#ifndef APP_MAX86141_H
#define APP_MAX86141_H

#include <stdint.h>
#include "nrf_drv_spi.h"
#include "nrf_spi_mngr.h"

#define TAGLIST_SIZE            16

//MAX86141 Registry addresses
#define REG_OP_WRITE            0x00
#define REG_OP_READ             0xFF

#define REG_INT_STAT_1         (0x00)      //Interrupt Status 1
#define REG_INT_STAT_2         (0x01)      //Interrupt Status 2
#define REG_INT_EN_1           (0x02)      //Interrupt Enable 1
#define REG_INT_EN_2           (0x03)      //Interrupt Enable 2
#define REG_FIFO_WR_PTR        (0x04)      //FIFO Buffer Write Pointer
#define REG_FIFO_RD_PTR        (0x05)      //FIFO Buffer Read Pointer
#define REG_OVF_COUNTER        (0x06)      //Over Flow Counter
#define REG_FIFO_DATA_COUNT    (0x07)      //FIFO Data Counter
#define REG_FIFO_DATA          (0x08)      //FIFO Data Register

#define REG_FIFO_CONFIG_1      (0x09)      //FIFO Configuration 1
#define REG_FIFO_CONFIG_2      (0x0A)      //FIFO Configuration 2
#define REG_SYSCTRL            (0x0D)      //System Control

//Photoplethysmogram (PPG) registers
#define REG_PPG_SYNC_CTRL      (0x10)      //PPG Sync Control
#define REG_PPG_CONFIG_1       (0x11)      //PPG Configuration Settings Group 1
#define REG_PPG_CONFIG_2       (0x12)      //PPG Configuration Settings Group 2
#define REG_PPG_CONFIG_3       (0x13)      //PPG Configuration Settings Group 3

#define REG_PROX_INTR_THRESH   (0x14)      //Prox Interrupt Threshold
#define REG_PD_BIAS            (0x15)      //Photo Diode Bias
#define REG_PICKET_FENCE       (0x16)      //Picket Fence Settings

#define REG_LED_SEQ_1          (0x20)      //LED Sequence 1
#define REG_LED_SEQ_2          (0x21)      //LED Sequence 2
#define REG_LED_SEQ_3          (0x22)      //LED Sequence 3

#define REG_LED1_PA            (0x23)      //LED 1 Pulse Amplitude
#define REG_LED2_PA            (0x24)      //LED 2 Pulse Amplitude
#define REG_LED3_PA            (0x25)      //LED 3 Pulse Amplitude
#define REG_LED4_PA            (0x26)      //LED 4 Pulse Amplitude
#define REG_LED5_PA            (0x27)      //LED 5 Pulse Amplitude
#define REG_LED6_PA            (0x28)      //LED 6 Pulse Amplitude
#define REG_LED_PILOT_PA       (0x29)      //LED Pilot Pulse Amplitude
#define REG_LED_RANGE_1        (0x2A)      //LED Amplitude Range 1
#define REG_LED_RANGE_2        (0x2B)      //LED Amplitude Range 2

//Hi resolution DAC settings for each LED.
#define REG_S1_HI_RES_DAC1     (0x2C)
#define REG_S2_HI_RES_DAC1     (0x2D)
#define REG_S3_HI_RES_DAC1     (0x2E)
#define REG_S4_HI_RES_DAC1     (0x2F)
#define REG_S5_HI_RES_DAC1     (0x30)
#define REG_S6_HI_RES_DAC1     (0x31)

#define REG_S1_HI_RES_DAC2     (0x32)
#define REG_S2_HI_RES_DAC2     (0x33)
#define REG_S3_HI_RES_DAC2     (0x34)
#define REG_S4_HI_RES_DAC2     (0x35)
#define REG_S5_HI_RES_DAC2     (0x36)
#define REG_S6_HI_RES_DAC2     (0x37)

//Die-temp registers
#define REG_TEMP_CONFIG        (0x40)
#define REG_TEMP_INTR          (0x41)
#define REG_TEMP_FRAC          (0x42)

//SHA256 registers
#define REG_SHA_CMD            (0xF0)
#define REG_SHA_CONFIG         (0xF1)

//Memory registers
#define REG_MEM_CTRL           (0xF2)
#define REG_MEM_IDX            (0xF3)
#define REG_MEM_DATA           (0xF4)
#define REG_PART_ID            (0xFF)

typedef struct __attribute__((packed)) reg_intstat1  // 0x00
{
  unsigned int pwr_rdy          : 1;
  unsigned int vdd_oor          : 1;
  unsigned int die_temp_dry     : 1;
  unsigned int led_compb        : 1;
  unsigned int prox_int         : 1;
  unsigned int alc_ovf          : 1;
  unsigned int data_rdy         : 1;
  unsigned int a_full           : 1;
} reg_intstat1_t;

typedef struct __attribute__((packed)) reg_intstat2  // 0x01
{
  unsigned int sha_done         : 1;
  unsigned int resv             : 7;
} reg_intstat2_t;

typedef struct __attribute__((packed)) reg_inten1    // 0x02
{
  unsigned int resv             : 1;
  unsigned int vdd_oor_en       : 1;
  unsigned int die_temp_rdy_en  : 1;
  unsigned int led_compb_en     : 1;
  unsigned int prox_int_en      : 1;
  unsigned int alc_ovf_en       : 1;
  unsigned int data_rdy_en      : 1;
  unsigned int a_full_en        : 1;
} reg_inten1_t;

typedef struct __attribute__((packed)) reg_inten2    // 0x03
{
  unsigned int sha_done_en      : 1;
  unsigned int resv             : 7;
} reg_inten2_t;

typedef struct __attribute__((packed)) reg_fifocfg1  // 0x09
{
  unsigned int fifo_a_full      : 7;
  unsigned int resv             : 1;
} reg_fifocfg1_t;

typedef struct __attribute__((packed)) reg_fifocfg2  // 0x0a
{
  unsigned int resv1            : 1;
  unsigned int fifo_ro          : 1;
  unsigned int a_full_type      : 1;
  unsigned int fifo_stat_clr    : 1;
  unsigned int flush_fifo       : 1;
  unsigned int resv2            : 3;
} reg_fifocfg2_t;

typedef struct __attribute__((packed)) reg_sysctrl   // 0x0d
{
  unsigned int reset      : 1;
  unsigned int shdn       : 1;
  unsigned int lp_mode    : 1;
  unsigned int single_ppg : 1;
  unsigned int resv       : 4;
} reg_sysctrl_t;

typedef struct __attribute__((packed)) reg_ppgcfg1
{
  unsigned int ppg_tint     : 2;
  unsigned int ppg1_adc_rge : 2;
  unsigned int ppg2_adc_rge : 2;
  unsigned int add_offset   : 1;
  unsigned int alc_disable  : 1;
} reg_ppgcfg1_t;

typedef struct __attribute__((packed)) reg_ppgcfg2
{
  unsigned int smp_ave      : 3;
  unsigned int ppg_sr       : 5;
} reg_ppgcfg2_t;

typedef struct __attribute__((packed)) reg_ppgcfg3
{
  unsigned int burst_en     : 1;
  unsigned int burst_rate   : 2;
  unsigned int resv         : 2;
  unsigned int dig_filt_sel : 1;
  unsigned int led_setlng   : 2;
} reg_ppgcfg3_t;

typedef struct __attribute__((packed)) reg_pdbias
{
  unsigned int pdbias1      : 3;
  unsigned int resv1        : 1;
  unsigned int pdbias2      : 3;
  unsigned int resv2        : 1;
} reg_pdbias_t;

typedef struct __attribute__((packed)) reg_ledrge
{
  unsigned int led14_rge    : 2;
  unsigned int led25_rge    : 2;
  unsigned int led36_rge    : 2;
  unsigned int resv         : 2;
} reg_ledrge_t;


typedef struct __attribute__((packed)) reg_ledseq
{
  unsigned int ledc135      : 4;
  unsigned int ledc246      : 4;
} reg_ledseq_t;

typedef struct __attribute__((packed)) max86141_cfg
{
  reg_inten1_t    inten1;
  reg_fifocfg1_t  fifocfg1;
  reg_fifocfg2_t  fifocfg2;
  reg_sysctrl_t   sysctrl;
  reg_ppgcfg1_t   ppgcfg1;
  reg_ppgcfg2_t   ppgcfg2;
  reg_ppgcfg3_t   ppgcfg3;
  reg_pdbias_t    pdbias;
  uint8_t         led1pa;
  uint8_t         led2pa;
  uint8_t         led3pa;
  uint8_t         led4pa;
  uint8_t         led5pa;
  uint8_t         led6pa;
  reg_ledrge_t    ledrge1;
  reg_ledrge_t    ledrge2;
  reg_ledseq_t    ledseq1;
  reg_ledseq_t    ledseq2;
  reg_ledseq_t    ledseq3;
} max86141_cfg_t;

typedef struct max86141_ctx
{
  int ledModeSize;
  int num_photo_diodes;       // number of photo diodes, 1 for single, otherwise dual
  int *ledMode;
  int number_leds;
  int intensity_led;
  int sample_average;
  int sample_rate;
  int pulse_width;
  int adcRange;

  const max86141_cfg_t *        p_maxcfg;

  char tags[TAGLIST_SIZE];

  /* for spi transfer */
  nrf_drv_spi_config_t          spicfg;
  uint8_t                       txbuf[3];
  uint8_t                       rxbuf[3];
  nrf_spi_mngr_transfer_t       xfer;
} max86141_ctx_t;

void app_max86141_freertos_init(void);

#endif // APP_MAX86141_H
