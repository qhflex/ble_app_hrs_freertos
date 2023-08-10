/*******************************************************************************
 * INCLUDES
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nrfx_gpiote.h"

#include "nrf_log.h"

#include "nrf_spi_mngr.h"

#include "usbcdc.h"
#include "max86141.h"
#include "sens-proto.h"

/**********************************************************************
 * MACROS
 */

/**********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/**********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*********************************************************************
 * PROFILE CALLBACKS
 */

/**********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * MACROS
 */
#define REGVAL(x)                           (*((uint8_t *)&x))
#define CFGVAL(x)                           REGVAL(ctx->p_maxcfg->x)

/*********************************************************************
 * CONSTANTS
 */

#define PRINT_REGISTERS                     0
#define WRITE_READBACK                      0

#define TSK_MAX86141_STACK_SIZE             64
#define TSK_MAX86141_PRIORITY               1

#define MAX86141_SPI_INSTANCE_ID            2

#define MAX86141_SPO_CS_PIN                 8
#define MAX86141_SPO_CK_PIN                 11
#define MAX86141_SPO_DI_PIN                 (32 + 8)
#define MAX86141_SPO_DO_PIN                 (32 + 9)

#define MAX86141_ABP_CS_PIN
#define MAX86141_ABP_CK_PIN
#define MAX86141_ABP_DI_PIN
#define MAX86141_ABP_DO_PIN

#define MAX_PENDING_TRANSACTIONS            5

/* this value only determine when the interrupt is fired */
#define FIFO_READ_SAMPLES                   MAX86141_NUM_OF_SAMPLES // 60 * 3 + 2 should not exceed 255 (uint8_t of spi rx buf size)
#define SPI_FIFO_RX_SIZE                    ((FIFO_READ_SAMPLES * 3) + 2)
#define SPI_FIFO_RX_SIZE_ROUGU              (2 * 3 + 2)

/*********************************************************************
 * TYPEDEFS
 */

/**********************************************************************
 * GLOBAL VARIABLES
 */

/**********************************************************************
 * LOCAL VARIABLES
 */
static TaskHandle_t m_max86141_thread = NULL;

NRF_SPI_MNGR_DEF(m_max86141_spi_mngr, MAX_PENDING_TRANSACTIONS, MAX86141_SPI_INSTANCE_ID);

const static uint8_t spi_fifo_tx[2] = { REG_FIFO_DATA, REG_OP_READ };
static uint8_t spi_fifo_rx[SPI_FIFO_RX_SIZE] = {0};
static uint8_t * buf = &spi_fifo_rx[2];

static uint8_t rougu[15] = {0};

const static nrf_spi_mngr_transfer_t max86141_fifo_xfers[] =
{
  NRF_SPI_MNGR_TRANSFER(spi_fifo_tx, 2, spi_fifo_rx, SPI_FIFO_RX_SIZE),
};

const static nrf_spi_mngr_transfer_t max86141_fifo_xfers_rougu[] =
{
    NRF_SPI_MNGR_TRANSFER(spi_fifo_tx, 2, spi_fifo_rx, 8),
};

const static max86141_cfg_t spo2_maxcfg = {
  .inten1 = {
    .a_full_en      = 1,    // enable fifo
  },
  .fifocfg1 = {
    .fifo_a_full  = (128 - FIFO_READ_SAMPLES),
  },
  .fifocfg2 = {
    .flush_fifo     = 1,    // required
    .fifo_stat_clr  = 1,    // not sure TODO
    .a_full_type    = 1,    // not sure TODO
    .fifo_ro        = 1,    // drop old samples when fifo full
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
    .led25_rge    = 0,      // 0b00  31mA
    .led14_rge    = 0,      // 0b01  62mA
                            // 0b10  93mA
                            // 0b11 124mA   <-
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

const static max86141_cfg_t abp_maxcfg = {
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
    .led25_rge    = 0,      // 0b00  31mA
    .led14_rge    = 0,      // 0b01  62mA
                            // 0b10  93mA
                            // 0b11 124mA   <-
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

static max86141_ctx_t spo2_ctx = {
    .p_maxcfg = &spo2_maxcfg,
};

static max86141_ctx_t abp_ctx = {
    .p_maxcfg = &abp_maxcfg,
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void write_reg(max86141_ctx_t * ctx, uint8_t addr, uint8_t data_in);
static uint8_t read_reg(max86141_ctx_t * ctx, uint8_t addr);
static void print_registers(max86141_ctx_t * ctx);
static void max86141_init(max86141_ctx_t * ctx);
static void max86141_run(max86141_ctx_t * ctx);
static void read_fifo(max86141_ctx_t * ctx);
static void dynamic(uint32_t ir, uint32_t red);

/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

static void read_fifo(max86141_ctx_t * ctx)
{
    ret_code_t err_code;
    err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, &ctx->spicfg, max86141_fifo_xfers, 1, NULL);
    APP_ERROR_CHECK(err_code);
}

static void read_fifo_rougu(max86141_ctx_t * ctx)
{
    ret_code_t err_code;
    err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, &ctx->spicfg, max86141_fifo_xfers_rougu, 1, NULL);
    APP_ERROR_CHECK(err_code);
}

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
static void max86141_init(max86141_ctx_t * ctx)
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

static void print_registers(max86141_ctx_t * ctx)
{
    (void)print_registers;

    NRF_LOG_INFO(" int status 1: 0x%02x", read_reg(ctx, 0x00));
    NRF_LOG_INFO(" int status 2: 0x%02x", read_reg(ctx, 0x01));
    NRF_LOG_INFO(" int enable 1: 0x%02x", read_reg(ctx, 0x02));
    NRF_LOG_INFO(" int enable 2: 0x%02x", read_reg(ctx, 0x03));
    NRF_LOG_INFO(" fifo write p: 0x%02x", read_reg(ctx, 0x04));
    NRF_LOG_INFO("  fifo read p: 0x%02x", read_reg(ctx, 0x05));
    NRF_LOG_INFO(" overflow cnt: 0x%02x", read_reg(ctx, 0x06));
    NRF_LOG_INFO("fifo data cnt: 0x%02x", read_reg(ctx, 0x07));
    NRF_LOG_INFO("fifo data reg: 0x%02x", read_reg(ctx, 0x08));
    NRF_LOG_INFO("  fifo conf 1: 0x%02x", read_reg(ctx, 0x09));
    NRF_LOG_INFO("  fifo conf 2: 0x%02x", read_reg(ctx, 0x0a));
    NRF_LOG_INFO("     sys ctrl: 0x%02x", read_reg(ctx, 0x0d));
    NRF_LOG_INFO("PPG sync ctrl: 0x%02x", read_reg(ctx, 0x10));
    NRF_LOG_INFO("   PPG conf 1: 0x%02x", read_reg(ctx, 0x11));
    NRF_LOG_INFO("   PPG conf 2: 0x%02x", read_reg(ctx, 0x12));
    NRF_LOG_INFO("   PPG conf 3: 0x%02x", read_reg(ctx, 0x13));
    NRF_LOG_INFO(" Prox int thr: 0x%02x", read_reg(ctx, 0x14));
    NRF_LOG_INFO(" Pho dio bias: 0x%02x", read_reg(ctx, 0x15));
    NRF_LOG_INFO(" Picket fence: 0x%02x", read_reg(ctx, 0x16));
    NRF_LOG_INFO(" LED seq reg1: 0x%02x", read_reg(ctx, 0x20));
    NRF_LOG_INFO(" LED seq reg2: 0x%02x", read_reg(ctx, 0x21));
    NRF_LOG_INFO(" LED seq reg3: 0x%02x", read_reg(ctx, 0x22));
    NRF_LOG_INFO("      LED1 pa: 0x%02x", read_reg(ctx, 0x23));
    NRF_LOG_INFO("      LED2 pa: 0x%02x", read_reg(ctx, 0x24));
    NRF_LOG_INFO("      LED3 pa: 0x%02x", read_reg(ctx, 0x25));
    NRF_LOG_INFO("      LED4 pa: 0x%02x", read_reg(ctx, 0x26));
    NRF_LOG_INFO("      LED5 pa: 0x%02x", read_reg(ctx, 0x27));
    NRF_LOG_INFO("      LED6 pa: 0x%02x", read_reg(ctx, 0x28));
    NRF_LOG_INFO(" LED pilot pa: 0x%02x", read_reg(ctx, 0x29));
    NRF_LOG_INFO("  LED range 1: 0x%02x", read_reg(ctx, 0x2a));
    NRF_LOG_INFO("  LED range 2: 0x%02x", read_reg(ctx, 0x2b));
    NRF_LOG_INFO("S1 hires dac1: 0x%02x", read_reg(ctx, 0x2c));
    NRF_LOG_INFO("S2 hires dac1: 0x%02x", read_reg(ctx, 0x2d));
    NRF_LOG_INFO("S3 hires dac1: 0x%02x", read_reg(ctx, 0x2e));
    NRF_LOG_INFO("S4 hires dac1: 0x%02x", read_reg(ctx, 0x2f));
    NRF_LOG_INFO("S5 hires dac1: 0x%02x", read_reg(ctx, 0x30));
    NRF_LOG_INFO("S6 hires dac1: 0x%02x", read_reg(ctx, 0x31));
    NRF_LOG_INFO("S1 hires dac2: 0x%02x", read_reg(ctx, 0x32));
    NRF_LOG_INFO("S2 hires dac2: 0x%02x", read_reg(ctx, 0x33));
    NRF_LOG_INFO("S3 hires dac2: 0x%02x", read_reg(ctx, 0x34));
    NRF_LOG_INFO("S4 hires dac2: 0x%02x", read_reg(ctx, 0x35));
    NRF_LOG_INFO("S5 hires dac2: 0x%02x", read_reg(ctx, 0x36));
    NRF_LOG_INFO("S6 hires dac2: 0x%02x", read_reg(ctx, 0x37));
    NRF_LOG_INFO("die temp conf: 0x%02x", read_reg(ctx, 0x40));
    NRF_LOG_INFO(" die temp int: 0x%02x", read_reg(ctx, 0x41));
    NRF_LOG_INFO("die temp frac: 0x%02x", read_reg(ctx, 0x42));
    NRF_LOG_INFO("      sha cmd: 0x%02x", read_reg(ctx, 0xf0));
    NRF_LOG_INFO("     sha conf: 0x%02x", read_reg(ctx, 0xf1));
    NRF_LOG_INFO("     mem ctrl: 0x%02x", read_reg(ctx, 0xf2));
    NRF_LOG_INFO("    mem index: 0x%02x", read_reg(ctx, 0xf3));
    NRF_LOG_INFO("     mem data: 0x%02x", read_reg(ctx, 0xf4));
    NRF_LOG_INFO("      part id: 0x%02x", read_reg(ctx, 0xff));
}

static void spo2_ctx_init()
{
    spo2_ctx.xfer.p_tx_data = spo2_ctx.txbuf;
    spo2_ctx.xfer.tx_length = 3;
    spo2_ctx.xfer.p_rx_data = spo2_ctx.rxbuf;
    spo2_ctx.xfer.rx_length = 3;

    // spo2_ctx.spicfg = NRF_DRV_SPI_DEFAULT_CONFIG;
    spo2_ctx.spicfg.ss_pin        = MAX86141_SPO_CS_PIN;
    spo2_ctx.spicfg.sck_pin       = MAX86141_SPO_CK_PIN;
    spo2_ctx.spicfg.mosi_pin      = MAX86141_SPO_DI_PIN;
    spo2_ctx.spicfg.miso_pin      = MAX86141_SPO_DO_PIN;
    spo2_ctx.spicfg.orc           = 0xff;
    spo2_ctx.spicfg.mode          = NRF_DRV_SPI_MODE_0;
    spo2_ctx.spicfg.irq_priority  = APP_IRQ_PRIORITY_LOWEST;
    spo2_ctx.spicfg.frequency     = NRF_DRV_SPI_FREQ_1M;
    spo2_ctx.spicfg.bit_order     = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
}

//void uart_error_handle(app_uart_evt_t * p_event)
//{
//    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_communication);
//    }
//    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
//    {
//        APP_ERROR_HANDLER(p_event->data.error_code);
//    }
//}

//#define MAX_RED_PA      0x60
//#define MAX_IR_PA       0x60

//#define MIN_RED_PA      0x10
//#define MIN_IR_PA       0x10
//#define PA_UP_STEP      0x04
//#define PA_DOWN_STEP    0x10

static void dynamic(uint32_t ir, uint32_t red)
{
    (void)dynamic;

#if 0
    static uint8_t led1_ir  = 0;
    static uint8_t led2_red = 0;

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

    NRF_LOG_INFO("red: 0x%06x %s, ir: 0x%06x %s", red, red_sign, ir, ir_sign);
#endif
}

static bool is_invalid(uint8_t msb)
{
    return ((msb >> 3) == 0x1e);
}

static void max86141_task(void * pvParameters)
{
    spo2_ctx_init();
    if (PRINT_REGISTERS)
    {
        print_registers(&spo2_ctx);
        print_registers(&abp_ctx);
    }

    max86141_init(&spo2_ctx);
    // spo2_packet_init(false);
    max86141_run(&spo2_ctx);

    NRF_LOG_INFO("max86141 started");

#if defined MIMIC_ROUGU && MIMIC_ROUGU == 1
    TickType_t xFrequency = 20;
    uint8_t count = 0;
    for (;;)
    {
        vTaskDelay(xFrequency);

        read_fifo_count:
        count = read_reg(&spo2_ctx, REG_FIFO_DATA_COUNT);

        if (count < 2) {
            NRF_LOG_INFO("[%d] data count: %d, wait a few ticks", xTaskGetTickCount(),count);
            vTaskDelay(4);
            goto read_fifo_count;
        }
        else
        {
            // NRF_LOG_INFO("[%d] data count: %d", xTaskGetTickCount(),count);
        }
        
        read_fifo_rougu(&spo2_ctx);

        rougu[0] = 0x18;
        rougu[1] = 0xff;

        /* tag 00001 LEDC1 which is IR
           tag 00010 LEDC2 which is RED
           tag 00011 LEDC3 which is GREEN */

        /* rougu protocol requires RED @ [2-4] and IR @ [5-7] */
        /* reversed data observed */
        if ((buf[0] >> 3) == 1) // if IR first
        {
            rougu[2] = buf[3] & 0x07;   // seems that rougu client not confused by tag
            rougu[3] = buf[4];
            rougu[4] = buf[5];
            rougu[5] = buf[0] & 0x07;
            rougu[6] = buf[1];
            rougu[7] = buf[2];
        }
        else // otherwise
        {
            rougu[2] = buf[0] & 0x07;
            rougu[3] = buf[1];
            rougu[4] = buf[2];
            rougu[5] = buf[3] & 0x07;
            rougu[6] = buf[4];
            rougu[7] = buf[5];
        }

        rougu[8] = rougu[2];
        rougu[9] = rougu[3];
        rougu[10] = rougu[4];
        rougu[11] = rougu[5];
        rougu[12] = rougu[6];
        rougu[13] = rougu[7];

        uint16_t sum = 0;
        for (int j = 0; j < 14; j++)
        {
            sum += (uint16_t)rougu[j];
        }

        if (sum < 256)
        {
            rougu[14] = sum;
        }
        else;
        {
            rougu[14] = ((~sum)+1) & 0xff;
        }

        if (cdc_acm_port_open())
        {
            cdc_acm_send_packet(rougu, sizeof(rougu));
        }

        xFrequency = count > 10 ? 19 : 20;
    }
#else
    TickType_t xFrequency = 21;
    uint8_t spo_count = 0;
    uint8_t abp_count = 0;
    for (;;)
    {
        vTaskDelay(xFrequency);
        
        spo_count = read_reg(&spo2_ctx, REG_FIFO_DATA_COUNT);
        if (spo_count >= 60)
        {
            
        }
        
       
    }
    
#endif

}



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
