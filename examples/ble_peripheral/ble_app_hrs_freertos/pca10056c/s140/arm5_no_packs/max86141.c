#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nrfx_gpiote.h"

#include "nrf_log.h"
#include "nrf_assert.h"

#include "nrf_spi_mngr.h"

#include "nrf_uart.h"
#include "app_uart.h"
#include "app_fifo.h"

#include "usbcdc.h"
#include "max86141.h"

#define PRINT_REGISTERS                     0
#define WRITE_READBACK                      0

#define TSK_MAX86141_STACK_SIZE             64
#define TSK_MAX86141_PRIORITY               1

#define MAX86141_SPI_INSTANCE_ID            2

#define MAX86141A_CS_PIN
#define MAX86141A_CK_PIN
#define MAX86141A_DI_PIN
#define MAX86141A_DO_PIN

#define MAX86141B_CS_PIN
#define MAX86141B_CK_PIN
#define MAX86141B_DI_PIN
#define MAX86141B_DO_PIN

#define MAX_PENDING_TRANSACTIONS            5

NRF_SPI_MNGR_DEF(m_max86141_spi_mngr, MAX_PENDING_TRANSACTIONS, MAX86141_SPI_INSTANCE_ID);

/* this value only determine when the interrupt is fired */
// #define FIFO_READ_SAMPLES            108
#define FIFO_READ_SAMPLES       60 // 60    // 60 * 3 + 2 should not exceed 255 (uint8_t of spi rx buf size)
#define SPI_FIFO_RX_SIZE        ((FIFO_READ_SAMPLES * 3) + 2)

#define REGVAL(x)               (*((uint8_t *)&x))
#define CFGVAL(x)               REGVAL(ctx->p_maxcfg->x)

const static uint8_t spi_fifo_tx[2] = { REG_FIFO_DATA, REG_OP_READ };
static uint8_t spi_fifo_rx[SPI_FIFO_RX_SIZE] = {0};
static uint8_t * buf = &spi_fifo_rx[2];

static void spo2_init_packet(void);
static int spo2_type_pos;
static int spo2_pkt_len_pos;
static int spo2_payload_pos;
static int spo2_seq_num_pos;
static int spo2_fifo_begin_pos;
static int spo2_crc_pos;

/*
 * This is a simple and fast crc originally from ublox gps protocol
 * u-blox8-M8_RecerverDescrProtSpec_UBX-12003221.pdf
 */
static void simple_crc(uint8_t * buf, uint8_t * cka, uint8_t * ckb)
{
    int num = cka - buf;
    *cka = 0;
    *ckb = 0;
    for (int i = 0; i < num; i++)
    {
        *cka = *cka + buf[i];
        *ckb = *ckb + *cka;
    }
}

const static uint16_t global_tlv_len = 9;

typedef struct reg_tlv_def {
    uint8_t reg;
    uint16_t num;
} local_tlv_def_t;

const local_tlv_def_t reg_tlvs[4] = {
    { 0x0D, 1 },
    { 0x10, 6 },
    { 0x20, 3 },
    { 0x23, 9 },
};

const uint16_t reg_tlv_len = 3 * 4 + 1 + 6 + 3 + 9; // 12 + 19 = 31
const uint16_t fifo_tlv_len = 3 + FIFO_READ_SAMPLES * 3; // = 183;

const uint16_t payload_len = global_tlv_len + reg_tlv_len + fifo_tlv_len; // 9 + 31 + 183 = 223

static uint8_t spo2_packet[237] = {
    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xD5, // preamble
    0x01, 0x01,                                     // type
    0xdf, 0x00,                                     // length
    0x00                                            // remaining 223 byte data and 2 byte crc
};

const static nrf_spi_mngr_transfer_t max86141_fifo_xfers[] =
{
  NRF_SPI_MNGR_TRANSFER(spi_fifo_tx, 2, spi_fifo_rx,SPI_FIFO_RX_SIZE),
};

static void read_fifo(max86141_ctx_t * ctx)
{
    ret_code_t err_code;
    err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, &ctx->spicfg, max86141_fifo_xfers, 1, NULL);
    APP_ERROR_CHECK(err_code);
}

const static max86141_cfg_t spo2_maxcfg = {
  .inten1 = {
    .a_full_en      = 1,    // enable fifo
  },
  .fifocfg1 = {
    .fifo_a_full  = (128 - FIFO_READ_SAMPLES),
  },
  .fifocfg2 = {
    .fifo_ro      = 1,      // drop old samples when fifo full
  },
  .sysctrl = {
    .single_ppg   = 1,      // only one channle is used
  },
  .ppgcfg1 = {
//  .ppg2_adc_rge = 2,
    .ppg1_adc_rge = 2,      // 0b00  4.097uA (full scale)
                            // 0b01  8.192uA
                            // 0b10 16.384uA
                            // 0b11 32.768uA <-

    .ppg_tint     = 2,      // pulse width = tint + tsetlng + 0.5uS = 129.8uS
                            // if tsetlng = 01 (6uS, default) then pw = 123.8uS
                            // as in the value provided in pseudo code
                            //
                            // 0b00 integration time is  14.8uS <-
                            // 0b01 integration time is  29.4uS
                            // 0b10 integration time is  58.7uS
                            // 0b11 integration time is 117.3uS
  },
  .ppgcfg2 = {
    .ppg_sr       = 0x05,   // 0x00 25sps
                            // 0x01 50sps   
                            // 0x02 84sps
                            // 0x03 100sps
                            // 0x04 200sps
                            // 0x05 400sps
                            // 0x11 1024sps

    .smp_ave      = 3,      // 0b000 (0) 1  <- (no sample averaging)
                            // 0b001 (1) 2
                            // 0b010 (2) 4
                            // 0b011 (3) 8
  },
  .ppgcfg3 = {
    .led_setlng   = 2,      // 0b00  4.0uS
                            // 0b01  6.0uS
                            // 0b10  8.0uS
                            // 0b11 12.0us  <-
  },
  .pdbias = {
//  .pdbias2      = 1,
    .pdbias1      = 1,      // 0-64pF, smallest
  },
  .ledrge1 = {
    .led25_rge    = 0,      // 0b00  31mA   <-
    .led14_rge    = 0,      // 0b01  62mA
                            // 0b10  93mA
                            // 0b11 124mA
  },
  .led1pa         = 0x28,   // lsb =  0.12 when rge is 00 (0b00)
                            //        0.24 when rge is 01 (0b01)
                            //        0.36 when rge is 02 (0b10) <-
                            //        0.48 when rge is 03 (0b11)
                            // rge = 0, pa = 0x40 => 0.12 * 64 = 7.68mA

  .led2pa         = 0x38,
  .ledseq1 = {
    .ledc135      = 1,      // 0001 LED1
    .ledc246      = 2,      // 0010 LED2
  },
};

// 'a' for SpO2 (blood oxygen saturation)
static max86141_ctx_t spo2_ctx = {
  .p_maxcfg = &spo2_maxcfg,
};
static max86141_ctx_t ctx_b = {0};

static TaskHandle_t m_max86141_thread = NULL;

static void write_reg(max86141_ctx_t * ctx, uint8_t addr, uint8_t data_in);
static uint8_t read_reg(max86141_ctx_t * ctx, uint8_t addr);
static void print_registers(void);


static void write_reg(max86141_ctx_t * ctx, uint8_t addr, uint8_t data_in)
{
    ret_code_t err_code;

    ctx->txbuf[0] = addr;
    ctx->txbuf[1] = REG_OP_WRITE;
    ctx->txbuf[2] = data_in;

    err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, &ctx->spicfg, &ctx->xfer, 1, NULL);
    APP_ERROR_CHECK(err_code);

    if (WRITE_READBACK)
    {
        uint8_t data_out = read_reg(ctx, addr);
        NRF_LOG_INFO("reg %02x: write %02x, read %02x", addr, data_in, data_out);
    }
}

static uint8_t read_reg(max86141_ctx_t * ctx, uint8_t addr)
{
  ret_code_t err_code;

  ctx->txbuf[0] = addr;
  ctx->txbuf[1] = REG_OP_READ;
  ctx->txbuf[2] = 0;

  err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, &ctx->spicfg, &ctx->xfer, 1, NULL);
  APP_ERROR_CHECK(err_code);
  return ctx->rxbuf[2];
}

/*
 * This function sets
 * 1. Photo diode
 *    single or dual, pulse width, adc range, sample average, sample rate
 *    settling time, bias
 * 2. led range, drive current, low power mode
 * 3. fifo configuration
 * 4. led exposure timing (sequence)
 */
void max86141_init(max86141_ctx_t * ctx)
{
  /*
    Pseudo code from max86141 datasheet

    DEVICE OPEN
    START;
    // AFE Initialization
    WRITE 0x1 to RESET[0];                          // Soft Reset (Register 0x0D[0])
    DELAY 1ms;
    READ Interrupt Status 1;                        // Clear Interrupt (Register 0x00)
    READ Interrupt Status 2; 			            // Clear Interrupt (Register 0x01)
    WRITE 0x1 to SHDN[0];                           // Shutdown (Register 0x0D[1])
    WRITE 0x3 to PPG_TINT[1:0]; 		            // Pulse Width = 123.8ms (Register 0x11[1:0])
    WRITE 0x2 to PPG1_ADC_RGE1:0]; 		            // ADC Range = 16uA (Register 0x11[3:2])
    WRITE 0x2 to PPG2_ADC_RGE1:0]; 		            // ADC Range = 16uA (Register 0x11[3:2])

    // For MAX86141 when used in Dual Channel only
    WRITE 0x0 to SMP_AVE[2:0]; 		                // Sample Averaging = 1 (Register 0x12[2:0])
    WRITE 0x00 to PPG_SR[4:0]; 		                // Sample Rate = 25sps (Register 0x12[7:3])
    WRITE 0x3 to LED_SETLNG[1:0]; 		            // LED Settling Time = 12ms (Register 0x13[7:6])
    WRITE 0x01 to PDBIAS1[2:0]; 		            // PD 1 Biasing for Cpd = 0~65pF (Register 0x15[2:0])
    WRITE 0x01 to PDBIAS2[2:0]; 		            // PD 1 Biasing for Cpd = 0~65pF (Register 0x15[2:0])

    // For MAX86141 when used in Dual Channel only
    WRITE 0x3 to LED1_RGE[1:0]; 		            // LED Driver 1 Range = 124mA (Register 0x15[2:0])
    WRITE 0x3 to LED2_RGE[1:0]; 		            // LED Driver 2 Range = 124mA (Register 0x15[2:0])
    WRITE 0x20 to LED1_DRV[1:0]; 		            // LED 1 Drive Current = 15.36mA (Register 0x23[7:0])
    WRITE 0x20 to LED2_DRV[1:0]; 		            // LED 2 Drive Current = 15.36mA (Register 0x24[7:0])
    WRITE 0x1 to LP_Mode[0]; 			            // Low Power mode enabled

    // FIFO Configuration
    WRITE 0x10 to FIFO_A_FULL[6:0]; 		        // FIFO INT triggered condition (Register 0x09[6:0])
    WRITE 0x1 to FIFO_RO; 			                // FIFO Roll Over enabled (Register 0x0A[1])
    WRITE 0x1 to A_FULL_EN; 			            // FIFO_A_FULL interrupt enabled (Register 0x02[7])
    WRITE 0x1 to LEDC1[3:0]; 			            // LED1 exposure configured in time slot 1
    WRITE 0x2 to LEDC2[3:0]; 			            // LED2 exposure configured in time slot 1
    WRITE 0x0 to LEDC3[3:0];
    WRITE 0x0 to LEDC4[3:0];
    WRITE 0x0 to LEDC5[3:0];
    WRITE 0x0 to LEDC6[3:0];
    WRITE 0x0 to SHDN[0]; 			                // Start Sampling STOP; */

    write_reg(ctx, REG_SYSCTRL, 0x01);              // 0b00000001);    // Soft Reset
    vTaskDelay(2);

    // write_reg(REG_PICKET_FENCE,0b11000000);
    // write_reg(REG_PICKET_FENCE,0b00000000);

    /* Clear interrupts. */
    read_reg(ctx, REG_INT_STAT_1);                  // Clear Interrupt
    read_reg(ctx, REG_INT_STAT_2);                  // Clear Interrupt

    reg_sysctrl_t sysctrl = ctx->p_maxcfg->sysctrl;
    sysctrl.shdn = 1;
    write_reg(ctx, REG_SYSCTRL,       REGVAL(sysctrl) );  // 0b00000010);    // Shutdown

    write_reg(ctx, REG_PPG_CONFIG_1,  CFGVAL(ppgcfg1) );
    write_reg(ctx, REG_PPG_CONFIG_2,  CFGVAL(ppgcfg2) );
    write_reg(ctx, REG_PPG_CONFIG_3,  CFGVAL(ppgcfg3) );
    write_reg(ctx, REG_PD_BIAS,       CFGVAL(pdbias)  );

    write_reg(ctx, REG_LED_RANGE_1,   CFGVAL(ledrge1) );  // 0x2A, not 0x15
    write_reg(ctx, REG_LED_RANGE_2,   CFGVAL(ledrge2) );  // 0x2B, not 0x15
    write_reg(ctx, REG_LED1_PA,       CFGVAL(led1pa)  );
    write_reg(ctx, REG_LED2_PA,       CFGVAL(led2pa)  );

    write_reg(ctx, REG_FIFO_CONFIG_1, CFGVAL(fifocfg1));
    write_reg(ctx, REG_FIFO_CONFIG_2, CFGVAL(fifocfg2));

    write_reg(ctx, REG_INT_EN_1,      CFGVAL(inten1)  );

    write_reg(ctx, REG_LED_SEQ_1,     CFGVAL(ledseq1) );
    write_reg(ctx, REG_LED_SEQ_2,     CFGVAL(ledseq2) );
    write_reg(ctx, REG_LED_SEQ_3,     CFGVAL(ledseq3) );

    // split into another function
    // sysctrl.shdn = 0;
    // write_reg(ctx, REG_SYSCTRL,       REGVAL(sysctrl) );
}

static void max86141_run(max86141_ctx_t * ctx)
{
    uint8_t sysctrl = read_reg(ctx, REG_SYSCTRL);
    sysctrl &= ~(0x02);    // SHDN @ bit 1, clear this bit
    write_reg(ctx, REG_SYSCTRL, sysctrl);
}

static void device_data_read_single_2LEDs(max86141_ctx_t * ctx) {
  int i;

  // uint8_t buf [FIFO_READ_SAMPLES*3];    //(128 - FIFO_A_FULL[6:0]) samples, 3 byte/channel
  uint8_t tag1[FIFO_READ_SAMPLES/2];       //(128 - FIFO_A_FULL[6:0])/2channels samples
  uint8_t tag2[FIFO_READ_SAMPLES/2];       //(128 - FIFO_A_FULL[6:0])/2channels samples
  int     led1[FIFO_READ_SAMPLES/2];       //(128 - FIFO_A_FULL[6:0])/2channels samples
  int     led2[FIFO_READ_SAMPLES/2];       //(128 - FIFO_A_FULL[6:0])/2channels samples

  uint8_t sampleCnt = read_reg(ctx, REG_FIFO_DATA_COUNT);          // sampleCnt should be the same value as FIFO_READ_SAMPLES

  //Read FIFO
  // ReadFifo(buf, sampleCnt * 3);
  read_fifo(ctx);

  for ( i = 0; i < sampleCnt/2  /*channels*/ ; i++ ) {
    tag1[i] = (buf[i*6+0] >> 3) & 0x1f;
    led1[i] = ((buf[i*6+0] << 16) | (buf[i*6+1] << 8) | (buf[i*6+2])) & 0x7ffff;
    tag2[i] = (buf[i*6+3] >> 3) & 0x1f;
    led2[i] = ((buf[i*6+3] << 16) | (buf[i*6+4] << 8) | (buf[i*6+5])) & 0x7ffff;
  }
}

static void device_data_read_dual_2LEDs(max86141_ctx_t * ctx) {
  int i;

  uint8_t buf   [FIFO_READ_SAMPLES*3];       //(128 - FIFO_A_FULL[6:0]) samples, 3 byte/channel
  uint8_t tag1A [FIFO_READ_SAMPLES/4];       //(128 - FIFO_A_FULL[6:0])/4channels samples
  uint8_t tag1B [FIFO_READ_SAMPLES/4];       //(128 - FIFO_A_FULL[6:0])/4channels samples
  uint8_t tag2A [FIFO_READ_SAMPLES/4];       //(128 - FIFO_A_FULL[6:0])/4channels samples
  uint8_t tag2B [FIFO_READ_SAMPLES/4];       //(128 - FIFO_A_FULL[6:0])/4channels samples
  int     led1A [FIFO_READ_SAMPLES/4];
  int     led1B [FIFO_READ_SAMPLES/4];
  int     led2A [FIFO_READ_SAMPLES/4];
  int     led2B [FIFO_READ_SAMPLES/4];
  // ReadReg(REG_FIFO_DATA_COUNT, &sampleCnt);            // sampleCnt should be the same value as FIFO_READ_SAMPLES

  uint8_t sampleCnt = read_reg(ctx, REG_FIFO_DATA_COUNT);          // sampleCnt should be the same value as FIFO_READ_SAMPLES
  APP_ERROR_CHECK_BOOL(sampleCnt == FIFO_READ_SAMPLES);

  // Read FIFO
  // ReadFifo(buf, sampleCnt * 3);
  read_fifo(ctx);

  for ( i = 0; i < sampleCnt/4/*channels*/; i++ ) {
    tag1A[i] = ( buf[i*12+0] >>  3) & 0x1f;
    led1A[i] = ((buf[i*12+0] << 16) | (buf[i*12+ 1] << 8) | (buf[i*12+ 2])) & 0x7ffff; // LED1, PD1

    tag1B[i] = ( buf[i*12+3] >>  3) & 0x1f;
    led1B[i] = ((buf[i*12+3] << 16) | (buf[i*12+ 4] << 8) | (buf[i*12+ 5])) & 0x7ffff; // LED1, PD2

    tag2A[i] = ( buf[i*12+6] >>  3) & 0x1f;
    led2A[i] = ((buf[i*12+6] << 16) | (buf[i*12+ 7] << 8) | (buf[i*12+ 8])) & 0x7ffff; // LED2, PD1

    tag2B[i] = ( buf[i*12+9] >>  3) & 0x1f;
    led2B[i] = ((buf[i*12+9] << 16) | (buf[i*12+10] << 8) | (buf[i*12+11])) & 0x7ffff; // LED2, PD2
  }
}

static void extract_tags(bool single_ppg,
                         unsigned int ledc1,
                         unsigned int ledc2,
                         unsigned int ledc3,
                         unsigned int ledc4,
                         unsigned int ledc5,
                         unsigned int ledc6,
                         char tags[TAGLIST_SIZE])
{
  char str[8] = {0};
  str[0] = ledc1;
  str[1] = ledc2;
  str[2] = ledc3;
  str[3] = ledc4;
  str[4] = ledc5;
  str[5] = ledc6;

  size_t len = strlen(str);
  memset(tags, 0, TAGLIST_SIZE);
  for (int i = 0; i < len; i++)
  {
    switch(str[i])
    {
      case 0x00:
        return;
      case 0x01:  // LED1
        if (single_ppg)
        {
          tags[i] = 1;
        }
        break;
      case 0x02:  // LED2
        if (single_ppg)
        {
          tags[i] = 2;
        }
        break;
      case 0x03:  // LED3
        if (single_ppg)
        {
          tags[i] = 0;
        }
        break;
      default:
        break;
    }
  }
}

static void print_registers()
{
    NRF_LOG_INFO(" int status 1: 0x%02x", read_reg(&spo2_ctx, 0x00));
    NRF_LOG_INFO(" int status 2: 0x%02x", read_reg(&spo2_ctx, 0x01));
    NRF_LOG_INFO(" int enable 1: 0x%02x", read_reg(&spo2_ctx, 0x02));
    NRF_LOG_INFO(" int enable 2: 0x%02x", read_reg(&spo2_ctx, 0x03));
    NRF_LOG_INFO(" fifo write p: 0x%02x", read_reg(&spo2_ctx, 0x04));
    NRF_LOG_INFO("  fifo read p: 0x%02x", read_reg(&spo2_ctx, 0x05));
    NRF_LOG_INFO(" overflow cnt: 0x%02x", read_reg(&spo2_ctx, 0x06));
    NRF_LOG_INFO("fifo data cnt: 0x%02x", read_reg(&spo2_ctx, 0x07));
    NRF_LOG_INFO("fifo data reg: 0x%02x", read_reg(&spo2_ctx, 0x08));
    NRF_LOG_INFO("  fifo conf 1: 0x%02x", read_reg(&spo2_ctx, 0x09));
    NRF_LOG_INFO("  fifo conf 2: 0x%02x", read_reg(&spo2_ctx, 0x0a));
    NRF_LOG_INFO("     sys ctrl: 0x%02x", read_reg(&spo2_ctx, 0x0d));
    NRF_LOG_INFO("PPG sync ctrl: 0x%02x", read_reg(&spo2_ctx, 0x10));
    NRF_LOG_INFO("   PPG conf 1: 0x%02x", read_reg(&spo2_ctx, 0x11));
    NRF_LOG_INFO("   PPG conf 2: 0x%02x", read_reg(&spo2_ctx, 0x12));
    NRF_LOG_INFO("   PPG conf 3: 0x%02x", read_reg(&spo2_ctx, 0x13));
    NRF_LOG_INFO(" Prox int thr: 0x%02x", read_reg(&spo2_ctx, 0x14));
    NRF_LOG_INFO(" Pho dio bias: 0x%02x", read_reg(&spo2_ctx, 0x15));
    NRF_LOG_INFO(" Picket fence: 0x%02x", read_reg(&spo2_ctx, 0x16));
    NRF_LOG_INFO(" LED seq reg1: 0x%02x", read_reg(&spo2_ctx, 0x20));
    NRF_LOG_INFO(" LED seq reg2: 0x%02x", read_reg(&spo2_ctx, 0x21));
    NRF_LOG_INFO(" LED seq reg3: 0x%02x", read_reg(&spo2_ctx, 0x22));
    NRF_LOG_INFO("      LED1 pa: 0x%02x", read_reg(&spo2_ctx, 0x23));
    NRF_LOG_INFO("      LED2 pa: 0x%02x", read_reg(&spo2_ctx, 0x24));
    NRF_LOG_INFO("      LED3 pa: 0x%02x", read_reg(&spo2_ctx, 0x25));
    NRF_LOG_INFO("      LED4 pa: 0x%02x", read_reg(&spo2_ctx, 0x26));
    NRF_LOG_INFO("      LED5 pa: 0x%02x", read_reg(&spo2_ctx, 0x27));
    NRF_LOG_INFO("      LED6 pa: 0x%02x", read_reg(&spo2_ctx, 0x28));
    NRF_LOG_INFO(" LED pilot pa: 0x%02x", read_reg(&spo2_ctx, 0x29));
    NRF_LOG_INFO("  LED range 1: 0x%02x", read_reg(&spo2_ctx, 0x2a));
    NRF_LOG_INFO("  LED range 2: 0x%02x", read_reg(&spo2_ctx, 0x2b));
    NRF_LOG_INFO("S1 hires dac1: 0x%02x", read_reg(&spo2_ctx, 0x2c));
    NRF_LOG_INFO("S2 hires dac1: 0x%02x", read_reg(&spo2_ctx, 0x2d));
    NRF_LOG_INFO("S3 hires dac1: 0x%02x", read_reg(&spo2_ctx, 0x2e));
    NRF_LOG_INFO("S4 hires dac1: 0x%02x", read_reg(&spo2_ctx, 0x2f));
    NRF_LOG_INFO("S5 hires dac1: 0x%02x", read_reg(&spo2_ctx, 0x30));
    NRF_LOG_INFO("S6 hires dac1: 0x%02x", read_reg(&spo2_ctx, 0x31));
    NRF_LOG_INFO("S1 hires dac2: 0x%02x", read_reg(&spo2_ctx, 0x32));
    NRF_LOG_INFO("S2 hires dac2: 0x%02x", read_reg(&spo2_ctx, 0x33));
    NRF_LOG_INFO("S3 hires dac2: 0x%02x", read_reg(&spo2_ctx, 0x34));
    NRF_LOG_INFO("S4 hires dac2: 0x%02x", read_reg(&spo2_ctx, 0x35));
    NRF_LOG_INFO("S5 hires dac2: 0x%02x", read_reg(&spo2_ctx, 0x36));
    NRF_LOG_INFO("S6 hires dac2: 0x%02x", read_reg(&spo2_ctx, 0x37));
    NRF_LOG_INFO("die temp conf: 0x%02x", read_reg(&spo2_ctx, 0x40));
    NRF_LOG_INFO(" die temp int: 0x%02x", read_reg(&spo2_ctx, 0x41));
    NRF_LOG_INFO("die temp frac: 0x%02x", read_reg(&spo2_ctx, 0x42));
    NRF_LOG_INFO("      sha cmd: 0x%02x", read_reg(&spo2_ctx, 0xf0));
    NRF_LOG_INFO("     sha conf: 0x%02x", read_reg(&spo2_ctx, 0xf1));
    NRF_LOG_INFO("     mem ctrl: 0x%02x", read_reg(&spo2_ctx, 0xf2));
    NRF_LOG_INFO("    mem index: 0x%02x", read_reg(&spo2_ctx, 0xf3));
    NRF_LOG_INFO("     mem data: 0x%02x", read_reg(&spo2_ctx, 0xf4));
    NRF_LOG_INFO("      part id: 0x%02x", read_reg(&spo2_ctx, 0xff));
}

static void spo2_packet_init(bool log)
{
    int pos = 0;   // 8 preamble, 2 type, 2 length;

    spo2_packet[pos++] = 0x55;  // preamble
    spo2_packet[pos++] = 0x55;
    spo2_packet[pos++] = 0x55;
    spo2_packet[pos++] = 0x55;
    spo2_packet[pos++] = 0x55;
    spo2_packet[pos++] = 0x55;
    spo2_packet[pos++] = 0x55;
    spo2_packet[pos++] = 0xD5;

    spo2_type_pos = pos;
    if (log) 
    {
        NRF_LOG_INFO("  crc begin pos: %d", pos);
    }

    spo2_packet[pos++] = 0x01;  // type
    spo2_packet[pos++] = 0x01;

    spo2_pkt_len_pos = pos;
    if (log)
    {
        NRF_LOG_INFO(" pkt length pos: %d", pos);
    }

    spo2_packet[pos++] = 0xdf;  // packet length (223)
    spo2_packet[pos++] = 0x00;

    spo2_payload_pos = pos;
    if (log)
    {
        NRF_LOG_INFO("pkt payload pos: %d", pos);
    }

    // global tlv
    spo2_packet[pos++] = 0xff;  // type
    spo2_packet[pos++] = 0x06;  // length
    spo2_packet[pos++] = 0x00;
    spo2_packet[pos++] = 0x01;  // sensor id, max86141
    spo2_packet[pos++] = 0x00;
    spo2_packet[pos++] = 0x00;  // global tlv version
    spo2_packet[pos++] = 0x00;  // instance id

    spo2_seq_num_pos = pos;
    if (log)
    {
        NRF_LOG_INFO("    seq num pos: %d", pos);
    }

    spo2_packet[pos++] = 0x00;  // sequence id
    spo2_packet[pos++] = FIFO_READ_SAMPLES; // number of samples

    int num_reg_tlvs = 4;
    for (int i = 0; i < 4; i++)
    {
        uint8_t reg = reg_tlvs[i].reg;
        uint16_t num = reg_tlvs[i].num;

        spo2_packet[pos++] = reg;
        spo2_packet[pos++] = (uint8_t)(num & 0xff);
        spo2_packet[pos++] = (uint8_t)(num >> 8);

        for (uint16_t j = 0; j < num; j++)
        {
            spo2_packet[pos++] = read_reg(&spo2_ctx, reg + j);
        }
    }

    spo2_packet[pos++] = REG_FIFO_DATA;

    uint16_t fifo_size = FIFO_READ_SAMPLES * 3;
    spo2_packet[pos++] = (uint8_t)(fifo_size & 0x00ff);
    spo2_packet[pos++] = (uint8_t)(fifo_size >> 8);

    spo2_fifo_begin_pos = pos;
    spo2_crc_pos = pos + fifo_size;
    if (log)
    {
        NRF_LOG_INFO(" fifo begin pos: %d", pos);
        NRF_LOG_INFO("        crc pos: %d", spo2_crc_pos);
    }

    uint16_t payload_len = spo2_fifo_begin_pos + fifo_size - spo2_payload_pos;
    if (log)
    {
        NRF_LOG_INFO("    payload len: %d", payload_len);
    }

    spo2_packet[spo2_pkt_len_pos] = (uint8_t)(payload_len & 0x00ff);
    spo2_packet[spo2_pkt_len_pos + 1] = (uint8_t)(payload_len >> 8);

    APP_ERROR_CHECK_BOOL(pos + fifo_size + 2 == 237);
}

static void spo2_ctx_init()
{
    spo2_ctx.xfer.p_tx_data = spo2_ctx.txbuf;
    spo2_ctx.xfer.tx_length = 3;
    spo2_ctx.xfer.p_rx_data = spo2_ctx.rxbuf;
    spo2_ctx.xfer.rx_length = 3;

    // spo2_ctx.spicfg = NRF_DRV_SPI_DEFAULT_CONFIG;
    spo2_ctx.spicfg.ss_pin        = 8;
    spo2_ctx.spicfg.sck_pin       = 11;
    spo2_ctx.spicfg.mosi_pin      = 32 + 8;
    spo2_ctx.spicfg.miso_pin      = 32 + 9;
    spo2_ctx.spicfg.orc           = 0xff;
    spo2_ctx.spicfg.mode          = NRF_DRV_SPI_MODE_0;
    spo2_ctx.spicfg.irq_priority  = APP_IRQ_PRIORITY_LOWEST;
    spo2_ctx.spicfg.frequency     = NRF_DRV_SPI_FREQ_1M;
    spo2_ctx.spicfg.bit_order     = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

#define MAX_RED_PA      0x80
#define MAX_IR_PA       0x80

#define MIN_RED_PA      0x10
#define MIN_IR_PA       0x10
#define PA_UP_STEP      0x04
#define PA_DOWN_STEP    0x10

static void dynamic(uint32_t ir, uint32_t red)
{
    static uint8_t led1_ir  = 0;
    static uint8_t led2_red = 0;
    
    return;

    if (led1_ir == 0) {
        led1_ir = read_reg(&spo2_ctx, REG_LED1_PA);
        // TODO
    }

    if (led2_red == 0) {
        led2_red = read_reg(&spo2_ctx, REG_LED2_PA);
        // TODO
    }

    uint8_t old_led1_ir  = led1_ir;
    uint8_t old_led2_red = led2_red;

    if (red > 0x078000)
    {
        if (led2_red > MIN_RED_PA)
        {
            led2_red = (led2_red - PA_DOWN_STEP) > MIN_RED_PA ? (led2_red - PA_DOWN_STEP) : MIN_RED_PA;
            write_reg(&spo2_ctx, REG_LED2_PA, led2_red);
        }
    }
    else if (red < 0x030000)
    {
        if (led2_red < MAX_RED_PA)
        {
            led2_red = (led2_red + PA_UP_STEP) < MAX_RED_PA ? (led2_red + PA_UP_STEP) : MAX_RED_PA;
            write_reg(&spo2_ctx, REG_LED2_PA, led2_red);
        }
    }

    if (ir > 0x078000)
    {
        // NRF_LOG_INFO("ir %d is greater than 0x060000, led1ir %d", ir, led1_ir);
        if (led1_ir > MIN_IR_PA)
        {
            led1_ir = (led1_ir - PA_DOWN_STEP) > MIN_IR_PA ? (led1_ir - PA_DOWN_STEP) : MIN_IR_PA;
            write_reg(&spo2_ctx, REG_LED1_PA, led1_ir);
        }
    }
    else if (ir < 0x030000)
    {
        if (led1_ir < MAX_IR_PA)
        {
            led1_ir = (led1_ir + PA_UP_STEP) < MAX_IR_PA ? (led1_ir + PA_UP_STEP) : MAX_IR_PA;
            write_reg(&spo2_ctx, REG_LED1_PA, led1_ir);
        }
    }

    char red_sign[2] = {0};
    char ir_sign[2] = {0};

    red_sign[0] = (led2_red == old_led2_red) ? ' ' : (led2_red < old_led2_red) ? '-' : '+';
    ir_sign [0] = (led1_ir  == old_led1_ir ) ? ' ' : (led1_ir  < old_led1_ir ) ? '-' : '+';

    // NRF_LOG_INFO("red: 0x%06x %s, ir: 0x%06x %s", red, red_sign, ir, ir_sign);
}

static void max86141_task(void * pvParameters)
{
    ret_code_t err_code;

    spo2_ctx_init();    
    if (PRINT_REGISTERS) 
    {
        print_registers();
    }
    max86141_init(&spo2_ctx);
    spo2_packet_init(false);
    max86141_run(&spo2_ctx);
    
    NRF_LOG_INFO("max86141 started");

    int line = 0;
    int j = 0;
    for (int i = 0;;i++)
    {
        vTaskDelay(100);

        // uint8_t intstat1 = ;
        if (cdc_acm_port_open() && (0x80 & read_reg(&spo2_ctx, REG_INT_STAT_1)))  // A_FULL
        {
            static int fifo_count;
            read_fifo(&spo2_ctx);
            
#if defined MIMIC_ROUGU && MIMIC_ROUGU == 1
            uint32_t total_red = 0;
            uint32_t total_ir = 0;

            for (int k = 0; k < FIFO_READ_SAMPLES / 2; k++)
            {
                // int tag1 = ( buf[k*6+0] >>  3) & 0x1f;
                // int led1 = ((buf[k*6+0] << 16) | (buf[k*6+1] << 8) | (buf[k*6+2])) & 0x7ffff;
                // int tag2 = ( buf[k*6+3] >>  3) & 0x1f;
                // int led2 = ((buf[k*6+3] << 16) | (buf[k*6+4] << 8) | (buf[k*6+5])) & 0x7ffff;
                spo2_sample_t smpl;
                smpl.byte[0] = buf[k * 6 + 0] & 0x07;
                smpl.byte[1] = buf[k * 6 + 1];
                smpl.byte[2] = buf[k * 6 + 2];
                smpl.byte[3] = buf[k * 6 + 3] & 0x07;
                smpl.byte[4] = buf[k * 6 + 4];
                smpl.byte[5] = buf[k * 6 + 5];

                // in schematic, led1 is ir, led2 is red
                total_ir  += ((uint32_t)smpl.byte[0] << 16) + ((uint32_t)smpl.byte[1] << 8) + (uint32_t)smpl.byte[2];
                total_red += ((uint32_t)smpl.byte[3] << 16) + ((uint32_t)smpl.byte[4] << 8) + (uint32_t)smpl.byte[5];

                rougu_enqueue(&smpl);
            }
            
            uint32_t ir_avg = total_ir / (FIFO_READ_SAMPLES / 2);
            uint32_t red_avg = total_red / (FIFO_READ_SAMPLES / 2);
            dynamic(ir_avg, red_avg);
#else
            
#endif            
        }
    }
}

      // NRF_LOG_INFO("read fifo done %d", j);

//      for (int k = 0; k < FIFO_READ_SAMPLES / 2; k++)
//      {
//        int tag1 = ( buf[k*6+0] >>  3) & 0x1f;
//        int led1 = ((buf[k*6+0] << 16) | (buf[k*6+1] << 8) | (buf[k*6+2])) & 0x7ffff;
//        int tag2 = ( buf[k*6+3] >>  3) & 0x1f;
//        int led2 = ((buf[k*6+3] << 16) | (buf[k*6+4] << 8) | (buf[k*6+5])) & 0x7ffff;
//
//        // NRF_LOG_RAW_INFO("%d, %d, %d, %d\n", tag1, led1, tag2, led2);
//        NRF_LOG_RAW_INFO("%d, %d, %d\n", line++, led1, led2);
//      }
//      j++;

//      memcpy(&spo2_packet[spo2_fifo_begin_pos], buf, FIFO_READ_SAMPLES * 3);
//      spo2_packet[spo2_seq_num_pos] += 1;
//      simple_crc(&spo2_packet[spo2_type_pos],
//                 &spo2_packet[spo2_crc_pos],
//                 &spo2_packet[spo2_crc_pos + 1]);
//
//      NRF_LOG_INFO("seq %d, cka %d, ckb %d",
//        spo2_packet[spo2_seq_num_pos],
//        spo2_packet[spo2_crc_pos],
//        spo2_packet[spo2_crc_pos + 1]);


//      for (int k = 0; k < spo2_crc_pos + 2; k++)
//      {
//        // app_uart_put(spo2_packet[k]);
//      }


        // vTaskDelay(16);

    // static int counter = 0;
    // NRF_LOG_INFO("max86141 %d", counter++);
    
void app_max86141_freertos_init(void)
{
  BaseType_t xReturned;

  /*
  static nrf_spi_mngr_transaction_t rdatac_trans = {
  .begin_callback = NULL,
  .end_callback = spi_rdatac_end_callback,
  .p_user_data = NULL,
  .p_transfers = rdatac_xfers,
  .number_of_transfers = sizeof(rdatac_xfers) / sizeof(rdatac_xfers[0]),
  .p_required_spicfg = NULL
};
  */

  xReturned = xTaskCreate(max86141_task,
                          "max86141",
                          TSK_MAX86141_STACK_SIZE,
                          NULL,
                          TSK_MAX86141_PRIORITY,
                          &m_max86141_thread);

  if (xReturned != pdPASS)
  {
    NRF_LOG_ERROR("[max86141] task not created.");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  else
  {
    NRF_LOG_INFO("[max86141] task created.");
  }
}
