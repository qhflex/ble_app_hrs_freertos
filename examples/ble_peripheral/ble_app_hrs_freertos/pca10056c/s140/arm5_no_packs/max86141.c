/*******************************************************************************
 * INCLUDES
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nrfx_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_spi_mngr.h"

#include "usbcdc.h"
#include "max86141.h"
#include "sens-proto.h"

/**********************************************************************
 * MACROS
 */

 #define COMBO_MODE

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

#define TSK_MAX86141_STACK_SIZE             512
#define TSK_MAX86141_PRIORITY               1

#define SPI_INSTANCE_ID                     2

#ifdef COMBO_MODE

#define MAX86141_COMBO_INT_PIN              ( 0 + 24)       // MAX-INT-P0.24
                                                            // This signal is pulled up to nRF MCU VDD
                                                            // on max side it's open-drain driven
#define MAX86141_COMBO_CS_PIN               ( 0 + 17)       // MAX-CS-P0.17
#define MAX86141_COMBO_CK_PIN               ( 0 + 14)       // MAX-SCLK-P0.14
#define MAX86141_COMBO_DI_PIN               ( 0 + 15)       // MAX-SDI-P0.15
#define MAX86141_COMBO_DO_PIN               ( 0 + 13)       // MAX-SDO-P0.13

#else

// P9 on schematic
#define MAX86141_SPO_CS_PIN                 8
#define MAX86141_SPO_CK_PIN                 11
#define MAX86141_SPO_DI_PIN                 (32 + 8)
#define MAX86141_SPO_DO_PIN                 (32 + 9)

// P6 on schematic
#define MAX86141_ABP_CS_PIN                 12
#define MAX86141_ABP_CK_PIN                 15
#define MAX86141_ABP_DI_PIN                 13
#define MAX86141_ABP_DO_PIN                 14

#endif // COMBO_MODE

#define MAX_PENDING_TRANSACTIONS            5

/* this value only determine when the interrupt is fired */
#define FIFO_READ_SAMPLES                   MAX86141_NUM_OF_SAMPLES         // 60 * 3 + 2 should not exceed 255 (uint8_t of spi rx buf size)
#define SPI_FIFO_RX_SIZE                    ((FIFO_READ_SAMPLES * 3) + 2)
#define SPI_FIFO_RX_SIZE_ROUGU              (2 * 3 + 2)

#define NUM_OF_RDBUF                        8      
#define SENS_PACKET_POOL_SIZE               2

#define RDBUF_SIZE                          ((MAX86141_NUM_OF_SAMPLES / 2) * 3 + 2)

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

static QueueHandle_t rdbuf_idle = NULL;
static QueueHandle_t rdbuf_pending = NULL;
static uint8_t rdbuf[NUM_OF_RDBUF][RDBUF_SIZE];

NRF_SPI_MNGR_DEF(m_max86141_spi_mngr, MAX_PENDING_TRANSACTIONS, SPI_INSTANCE_ID);

const static uint8_t spi_fifo_tx[2] = { REG_FIFO_DATA, REG_OP_READ };
static uint8_t spi_fifo_rx[SPI_FIFO_RX_SIZE] = {0};
static uint8_t * buf = &spi_fifo_rx[2];

#if defined MIMIC_ROUGU && MIMIC_ROUGU == 1
static uint8_t rougu[15] = {0};
#endif

#ifdef COMBO_MODE

static uint8_t combo_0x10_0x16_regs[7] = {0};
static uint8_t combo_0x20_0x2B_regs[12] = {0};

#else

static uint8_t spo_0x10_0x16_regs[7] = {0}; // for 0x10-0x16
static uint8_t spo_0x20_0x2B_regs[12] = {0};

static uint8_t abp_0x10_0x16_regs[7] = {0}; // for 0x10-0x16
static uint8_t abp_0x20_0x2B_regs[12] = {0};

#endif

const static nrf_spi_mngr_transfer_t max86141_fifo_xfers[] =
{
    NRF_SPI_MNGR_TRANSFER(spi_fifo_tx, 2, spi_fifo_rx, SPI_FIFO_RX_SIZE),
};

const static nrf_spi_mngr_transfer_t max86141_fifo_xfers_rougu[] =
{
    NRF_SPI_MNGR_TRANSFER(spi_fifo_tx, 2, spi_fifo_rx, 8), // 6 + 2
};

#ifdef COMBO_MODE

const static max86141_cfg_t combo_maxcfg = {
    .inten1 = {
        .a_full_en      = 1,    // enable fifo
    },
    .fifocfg1 = {
        .fifo_a_full    = (128 - (MAX86141_NUM_OF_SAMPLES / 2)),
    },
    .fifocfg2 = {
        .flush_fifo     = 1,    // required
        .fifo_stat_clr  = 1,    // set to 1 to clear interrup when reading fifo (no need to read status reg)
        .a_full_type    = 0,    // this is the default value
        .fifo_ro        = 1,    // drop old samples when fifo full
    },
    .sysctrl = {
        .single_ppg     = 0,    // both channels are used
    },
    .ppgcfg1 = {
        .ppg2_adc_rge   = 3,
        .ppg1_adc_rge   = 3,    // 0b00  4.097uA (full scale)
                                // 0b01  8.192uA
                                // 0b10 16.384uA
                                // 0b11 32.768uA

        .ppg_tint       = 0,    // pulse width = tint + tsetlng + 0.5uS = 129.8uS
                                // if tsetlng = 01 (6uS, default) then pw = 123.8uS
                                // as in the value provided in pseudo code
                                //
                                // 0b00 integration time is  14.8uS
                                // 0b01 integration time is  29.4uS
                                // 0b10 integration time is  58.7uS <-
                                // 0b11 integration time is 117.3uS
    },
    .ppgcfg2 = {
        .ppg_sr         = 0x12, // 0x00   25 sps
                                // 0x01   50 sps
                                // 0x02   84 sps
                                // 0x03  100 sps
                                // 0x04  200 sps
                                // 0x05  400 sps
                                // 0x06   25 sps (2 pulses per sample)
                                // 0x07   50 sps (2 pulses per sample)
                                // 0x08   84 sps (2 pulses per sample)
                                // 0x09  100 sps (2 pulses per sample)
                                // 0x0a    8 sps
                                // 0x0b   16 sps
                                // 0x0c   32 sps
                                // 0x0d   64 sps
                                // 0x0e  128 sps
                                // 0x0f  256 sps
                                // 0x10  512 sps
                                // 0x11 1024 sps
                                // 0x12 2048 sps
                                // 0x13 4096 sps

        .smp_ave        = 0,    // 0b000 (0) 1
                                // 0b001 (1) 2
                                // 0b010 (2) 4
                                // 0b011 (3) 8
                                // 0b100 (4) 16
                                // 0b101 (5) 32
                                // 0b110 (6) 64
                                // 0b111 (7) 128
    },
    .ppgcfg3 = {
        .led_setlng     = 0,    // 0b00  4.0uS
                                // 0b01  6.0uS
                                // 0b10  8.0uS
                                // 0b11 12.0us
    },
    .pdbias = {
        .pdbias2        = 1,
        .pdbias1        = 1,    // 0-64pF, smallest
    },

    // all leds (ir1, red1, ir2) are set to certain range
    .ledrge1 = {
        .led36_rge      = 1,
        .led25_rge      = 1,    // 0b00  31mA
        .led14_rge      = 1,    // 0b01  62mA
                                // 0b10  93mA
                                // 0b11 124mA
    },

    // all leds (ir1, red1, ir2) are set to certain power amplifier ratio.
    .led1pa             = 0x30, // lsb =  0.12 when rge is 00 (0b00)
                                //        0.24 when rge is 01 (0b01)
                                //        0.36 when rge is 02 (0b10)
                                //        0.48 when rge is 03 (0b11)

    .led2pa             = 0x30,
    .led3pa             = 0x30,

    /*
     * only LED Sequence Register 1 (0x20) needs to be set
     *  .ledc135 (LEDC1[3:0]) set to 5 (0101)
     *  .ledc246 (LEDC2[3:0]) set to 2 (0010)
     */
    .ledseq1 = {
        .ledc135        = 5,    // 0101 LED1 + LED3
        .ledc246        = 2,    // 0010 LED2
    },
};

#else

const static max86141_cfg_t spo_maxcfg = {
    .inten1 = {
        .a_full_en      = 1,    // enable fifo
    },
    .fifocfg1 = {
        .fifo_a_full    = (128 - FIFO_READ_SAMPLES),
    },
    .fifocfg2 = {
        .flush_fifo     = 1,    // required
        .fifo_stat_clr  = 1,    // not sure TODO
        .a_full_type    = 1,    // not sure TODO
        .fifo_ro        = 1,    // drop old samples when fifo full
    },
    .sysctrl = {
        .single_ppg     = 1,    // only one channle is used
    },
    .ppgcfg1 = {
//      .ppg2_adc_rge   = 3,
        .ppg1_adc_rge   = 3,    // 0b00  4.097uA (full scale)
                                // 0b01  8.192uA
                                // 0b10 16.384uA
                                // 0b11 32.768uA <-

        .ppg_tint       = 3,    // pulse width = tint + tsetlng + 0.5uS = 129.8uS
                                // if tsetlng = 01 (6uS, default) then pw = 123.8uS
                                // as in the value provided in pseudo code
                                //
                                // 0b00 integration time is  14.8uS
                                // 0b01 integration time is  29.4uS
                                // 0b10 integration time is  58.7uS
                                // 0b11 integration time is 117.3uS
    },
    .ppgcfg2 = {
        .ppg_sr         = 0x05, // 0x00   25sps
                                // 0x01   50 sps
                                // 0x02   84 sps
                                // 0x03  100 sps
                                // 0x04  200 sps
                                // 0x05  400 sps
                                // 0x06   25 sps (2 pulses per sample)
                                // 0x07   50 sps (2 pulses per sample)
                                // 0x08   84 sps (2 pulses per sample)
                                // 0x09  100 sps (2 pulses per sample)
                                // 0x0a    8 sps
                                // 0x0b   16 sps
                                // 0x0c   32 sps
                                // 0x0d   64 sps
                                // 0x0e  128 sps
                                // 0x0f  256 sps
                                // 0x10  512 sps
                                // 0x11 1024 sps
                                // 0x12 2048 sps
                                // 0x13 4096 sps

        .smp_ave        = 3,    // 0b000 (0) 1
                                // 0b001 (1) 2
                                // 0b010 (2) 4
                                // 0b011 (3) 8
    },
    .ppgcfg3 = {
        .led_setlng     = 3,    // 0b00  4.0uS
                                // 0b01  6.0uS
                                // 0b10  8.0uS
                                // 0b11 12.0us
    },
    .pdbias = {
//      .pdbias2        = 1,
        .pdbias1        = 1,    // 0-64pF, smallest
    },
    .ledrge1 = {
        .led25_rge      = 1,    // 0b00  31mA
        .led14_rge      = 1,    // 0b01  62mA
                                // 0b10  93mA
                                // 0b11 124mA
    },
    .led1pa             = 0x24, // lsb =  0.12 when rge is 00 (0b00)
                                //        0.24 when rge is 01 (0b01)
                                //        0.36 when rge is 02 (0b10)
                                //        0.48 when rge is 03 (0b11)

    .led2pa             = 0x32,
    .ledseq1 = {
        .ledc135        = 1,      // 0001 LED1
        .ledc246        = 2,      // 0010 LED2
    },
};

const static max86141_cfg_t abp_maxcfg = {
    .inten1 = {
        .a_full_en      = 1,    // enable fifo
    },
    .fifocfg1 = {
        .fifo_a_full    = (128 - FIFO_READ_SAMPLES),
    },
    .fifocfg2 = {
        .flush_fifo     = 1,    // required
        .fifo_stat_clr  = 1,    // not sure TODO
        .a_full_type    = 1,    // not sure TODO
        .fifo_ro        = 1,    // drop old samples when fifo full
    },
    .sysctrl = {
        .single_ppg     = 0,    // both channels are used
    },
    .ppgcfg1 = {
        .ppg2_adc_rge   = 3,
        .ppg1_adc_rge   = 3,    // 0b00  4.097uA (full scale)
                                // 0b01  8.192uA
                                // 0b10 16.384uA
                                // 0b11 32.768uA

        .ppg_tint       = 0,    // pulse width = tint + tsetlng + 0.5uS = 129.8uS
                                // if tsetlng = 01 (6uS, default) then pw = 123.8uS
                                // as in the value provided in pseudo code
                                //
                                // 0b00 integration time is  14.8uS
                                // 0b01 integration time is  29.4uS
                                // 0b10 integration time is  58.7uS <-
                                // 0b11 integration time is 117.3uS
    },
    .ppgcfg2 = {
        // .ppg_sr         = 0x13  // max
        .ppg_sr         = 0x01, // 0x00   25 sps
                                // 0x01   50 sps
                                // 0x02   84 sps
                                // 0x03  100 sps
                                // 0x04  200 sps
                                // 0x05  400 sps
                                // 0x06   25 sps (2 pulses per sample)
                                // 0x07   50 sps (2 pulses per sample)
                                // 0x08   84 sps (2 pulses per sample)
                                // 0x09  100 sps (2 pulses per sample)
                                // 0x0a    8 sps
                                // 0x0b   16 sps
                                // 0x0c   32 sps
                                // 0x0d   64 sps
                                // 0x0e  128 sps
                                // 0x0f  256 sps
                                // 0x10  512 sps
                                // 0x11 1024 sps
                                // 0x12 2048 sps
                                // 0x13 4096 sps

        .smp_ave        = 0,    // 0b000 (0) 1
                                // 0b001 (1) 2
                                // 0b010 (2) 4
                                // 0b011 (3) 8
                                // 0b100 (4) 16
                                // 0b101 (5) 32
                                // 0b110 (6) 64
                                // 0b111 (7) 128
    },
    .ppgcfg3 = {
        .led_setlng     = 3,    // 0b00  4.0uS
                                // 0b01  6.0uS
                                // 0b10  8.0uS
                                // 0b11 12.0us
    },
    .pdbias = {
        .pdbias2        = 1,
        .pdbias1        = 1,    // 0-64pF, smallest
    },
    .ledrge1 = {
//      .led36_rge      = 1,
        .led25_rge      = 1,    // 0b00  31mA
        .led14_rge      = 1,    // 0b01  62mA
                                // 0b10  93mA
                                // 0b11 124mA
    },
    .led1pa             = 0x30, // lsb =  0.12 when rge is 00 (0b00)
                                //        0.24 when rge is 01 (0b01)
                                //        0.36 when rge is 02 (0b10)
                                //        0.48 when rge is 03 (0b11)

    .led2pa             = 0x30,
//  .led3pa             = 0x40,
    .ledseq1 = {
        .ledc135        = 4,    // 0001 LED1
//      .ledc246        = 2,    // 0010 LED2
                                // 0100 LED1 + LED2
    },
//  .ledseq2 = {
//      .ledc135        = 3,    // 0011 LED3
//  },
};

#endif

#ifdef COMBO_MODE

max86141_ctx_t combo_ctx = {
    .p_maxcfg = &combo_maxcfg,
};

#else

static max86141_ctx_t spo_ctx = {
    .p_maxcfg = &spo_maxcfg,
};

static max86141_ctx_t abp_ctx = {
    .p_maxcfg = &abp_maxcfg,
};

#endif

#ifdef COMBO_MODE

static max86141_packet_helper_t m_combo_packet_helper = {
    .instance_id = 0x80,
    .ppg1_led = 0,
    .ppg2_led = 0,
    .ppf_prox = 0,
    .use_rougu_spo = 0,
};

#else

static max86141_packet_helper_t m_spo_packet_helper = {
    .instance_id = 0,
    .ppg1_led = 0x03,   // PPG1_LED1, PPG1_LED2
    .ppg2_led = 0,
    .ppf_prox = 0,
    .use_rougu_spo = 1,
};

static max86141_packet_helper_t m_abp_packet_helper = {
    .instance_id = 1,
    .ppg1_led = 0x07,
    .ppg2_led = 0x07,
    .ppf_prox = 0,
    .use_rougu_spo = 0,
};

#endif

#ifdef COMBO_MODE

static sens_packet_t *p_current_combo_packet;

#else

static sens_packet_t *p_current_spo_packet;
static sens_packet_t *p_current_abp_packet;

#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void write_reg(max86141_ctx_t * ctx, uint8_t addr, uint8_t data_in);

static uint8_t read_reg(max86141_ctx_t * ctx, uint8_t addr);
static void read_registers(max86141_ctx_t * ctx, uint8_t startAddr, int num, uint8_t * outbuf);
static void print_registers(max86141_ctx_t * ctx);
static bool max86141_probe(max86141_ctx_t * ctx);
static void max86141_config(max86141_ctx_t * ctx);
static void max86141_run(max86141_ctx_t * ctx);
static void read_fifo(max86141_ctx_t * ctx);
// static void dynamic(uint32_t ir, uint32_t red);

#ifdef COMBO_MODE
static sens_packet_t * next_combo_packet(void);
#else
static sens_packet_t * next_spo_packet(void);
static sens_packet_t * next_abp_packet(void);
#endif

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

static void read_registers(max86141_ctx_t * ctx, uint8_t startAddr, int num, uint8_t * outbuf)
{
    for (int i = 0; i < num; i++)
    {
        int addr = startAddr + i;
        uint8_t val = read_reg(ctx, addr);

#ifdef COMBO_MODE
#else
        if (ctx == &spo_ctx)
        {
            // NRF_LOG_INFO("spo reg %02x: %02x", addr, val);
        }
        else if (ctx == &abp_ctx)
        {
            // NRF_LOG_INFO("abp reg %02x: %02x", addr, val);
        }
#endif
        if (outbuf)
        {
            outbuf[i] = val;
        }
    }
}

static bool max86141_probe(max86141_ctx_t *ctx)
{
    uint8_t val = read_reg(ctx, 0xff);
    NRF_LOG_INFO("max86141 probe 0xff returns: %02x", val);

    if (val == 0x25) {
#ifdef COMBO_MODE
        if (ctx == &combo_ctx)
        {
            NRF_LOG_INFO("combo max86141 probed");
        }
#else
        if (ctx == &spo_ctx)
        {
            NRF_LOG_INFO("spo max86141 probed");
        }

        if (ctx == &abp_ctx)
        {
            NRF_LOG_INFO("abp max86141 probed");
        }
#endif
    }

    return (val == 0x25);
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
static void max86141_config(max86141_ctx_t * ctx)
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
    write_reg(ctx, REG_LED3_PA,       CFGVAL(led3pa)  );
    write_reg(ctx, REG_LED4_PA,       CFGVAL(led4pa)  );
    write_reg(ctx, REG_LED5_PA,       CFGVAL(led5pa)  );
    write_reg(ctx, REG_LED6_PA,       CFGVAL(led6pa)  );
    write_reg(ctx, REG_LED_PILOT_PA,  CFGVAL(pilotpa) );

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

#ifdef COMBO_MODE

static void combo_ctx_init()
{
    combo_ctx.xfer.p_tx_data = combo_ctx.txbuf;
    combo_ctx.xfer.tx_length = 3;
    combo_ctx.xfer.p_rx_data = combo_ctx.rxbuf;
    combo_ctx.xfer.rx_length = 3;

    // abp_ctx.spicfg = NRF_DRV_SPI_DEFAULT_CONFIG;
    combo_ctx.spicfg.ss_pin        = MAX86141_COMBO_CS_PIN;
    combo_ctx.spicfg.sck_pin       = MAX86141_COMBO_CK_PIN;
    combo_ctx.spicfg.mosi_pin      = MAX86141_COMBO_DI_PIN;
    combo_ctx.spicfg.miso_pin      = MAX86141_COMBO_DO_PIN;
    combo_ctx.spicfg.orc           = 0xff;
    combo_ctx.spicfg.mode          = NRF_DRV_SPI_MODE_0;
    combo_ctx.spicfg.irq_priority  = APP_IRQ_PRIORITY_LOWEST;
    combo_ctx.spicfg.frequency     = NRF_DRV_SPI_FREQ_4M;
    combo_ctx.spicfg.bit_order     = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
}

#else

static void spo_ctx_init()
{
    spo_ctx.xfer.p_tx_data = spo_ctx.txbuf;
    spo_ctx.xfer.tx_length = 3;
    spo_ctx.xfer.p_rx_data = spo_ctx.rxbuf;
    spo_ctx.xfer.rx_length = 3;

    // spo_ctx.spicfg = NRF_DRV_SPI_DEFAULT_CONFIG;
    spo_ctx.spicfg.ss_pin        = MAX86141_SPO_CS_PIN;
    spo_ctx.spicfg.sck_pin       = MAX86141_SPO_CK_PIN;
    spo_ctx.spicfg.mosi_pin      = MAX86141_SPO_DI_PIN;
    spo_ctx.spicfg.miso_pin      = MAX86141_SPO_DO_PIN;
    spo_ctx.spicfg.orc           = 0xff;
    spo_ctx.spicfg.mode          = NRF_DRV_SPI_MODE_0;
    spo_ctx.spicfg.irq_priority  = APP_IRQ_PRIORITY_LOWEST;
    spo_ctx.spicfg.frequency     = NRF_DRV_SPI_FREQ_4M;
    spo_ctx.spicfg.bit_order     = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
}

static void abp_ctx_init()
{
    abp_ctx.xfer.p_tx_data = abp_ctx.txbuf;
    abp_ctx.xfer.tx_length = 3;
    abp_ctx.xfer.p_rx_data = abp_ctx.rxbuf;
    abp_ctx.xfer.rx_length = 3;

    // abp_ctx.spicfg = NRF_DRV_SPI_DEFAULT_CONFIG;
    abp_ctx.spicfg.ss_pin        = MAX86141_ABP_CS_PIN;
    abp_ctx.spicfg.sck_pin       = MAX86141_ABP_CK_PIN;
    abp_ctx.spicfg.mosi_pin      = MAX86141_ABP_DI_PIN;
    abp_ctx.spicfg.miso_pin      = MAX86141_ABP_DO_PIN;
    abp_ctx.spicfg.orc           = 0xff;
    abp_ctx.spicfg.mode          = NRF_DRV_SPI_MODE_0;
    abp_ctx.spicfg.irq_priority  = APP_IRQ_PRIORITY_LOWEST;
    abp_ctx.spicfg.frequency     = NRF_DRV_SPI_FREQ_4M;
    abp_ctx.spicfg.bit_order     = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
}

#endif

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
#if 0
static void dynamic(uint32_t ir, uint32_t red)
{
    (void)dynamic;

    static uint8_t led1_ir  = 0;
    static uint8_t led2_red = 0;

    if (led1_ir == 0) {
        led1_ir = read_reg(&spo_ctx, REG_LED1_PA);
        // TODO
    }

    if (led2_red == 0) {
        led2_red = read_reg(&spo_ctx, REG_LED2_PA);
        // TODO
    }

    uint8_t old_led1_ir  = led1_ir;
    uint8_t old_led2_red = led2_red;

    if (red > 0x078000)
    {
        if (led2_red > MIN_RED_PA)
        {
            led2_red = (led2_red - PA_DOWN_STEP) > MIN_RED_PA ? (led2_red - PA_DOWN_STEP) : MIN_RED_PA;
            write_reg(&spo_ctx, REG_LED2_PA, led2_red);
        }
    }
    else if (red < 0x030000)
    {
        if (led2_red < MAX_RED_PA)
        {
            led2_red = (led2_red + PA_UP_STEP) < MAX_RED_PA ? (led2_red + PA_UP_STEP) : MAX_RED_PA;
            write_reg(&spo_ctx, REG_LED2_PA, led2_red);
        }
    }

    if (ir > 0x078000)
    {
        // NRF_LOG_INFO("ir %d is greater than 0x060000, led1ir %d", ir, led1_ir);
        if (led1_ir > MIN_IR_PA)
        {
            led1_ir = (led1_ir - PA_DOWN_STEP) > MIN_IR_PA ? (led1_ir - PA_DOWN_STEP) : MIN_IR_PA;
            write_reg(&spo_ctx, REG_LED1_PA, led1_ir);
        }
    }
    else if (ir < 0x030000)
    {
        if (led1_ir < MAX_IR_PA)
        {
            led1_ir = (led1_ir + PA_UP_STEP) < MAX_IR_PA ? (led1_ir + PA_UP_STEP) : MAX_IR_PA;
            write_reg(&spo_ctx, REG_LED1_PA, led1_ir);
        }
    }

    char red_sign[2] = {0};
    char ir_sign[2] = {0};

    red_sign[0] = (led2_red == old_led2_red) ? ' ' : (led2_red < old_led2_red) ? '-' : '+';
    ir_sign [0] = (led1_ir  == old_led1_ir ) ? ' ' : (led1_ir  < old_led1_ir ) ? '-' : '+';

    NRF_LOG_INFO("red: 0x%06x %s, ir: 0x%06x %s", red, red_sign, ir, ir_sign);

}
#endif

static bool is_invalid(uint8_t msb)
{
    return ((msb >> 3) == 0x1e);
}

#if defined MIMIC_ROUGU && MIMIC_ROUGU == 1

static void max86141_task(void * pvParameters)
{
    TickType_t xFrequency = 20;
    uint8_t count = 0;

    spo_ctx_init();
    max86141_config(&spo_ctx);
    max86141_run(&spo_ctx);

    NRF_LOG_INFO("spo max86141 (rougu version) started");

    for (;;)
    {
        vTaskDelay(xFrequency);

        read_fifo_count:
        count = read_reg(&spo_ctx, REG_FIFO_DATA_COUNT);

        if (count < 2) {
            NRF_LOG_INFO("[%d] data count: %d, wait a few ticks", xTaskGetTickCount(),count);
            vTaskDelay(4);
            goto read_fifo_count;
        }
        else
        {
            // NRF_LOG_INFO("[%d] data count: %d", xTaskGetTickCount(),count);
        }

        read_fifo_rougu(&spo_ctx);

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
}

#else

static nrf_spi_mngr_transfer_t read_fifo_xfers[] =
{
    {
        .p_tx_data = spi_fifo_tx,
        .tx_length = 2,
        .p_rx_data = NULL,
        .rx_length = RDBUF_SIZE
    }
};

static void read_fifo_end_callback(ret_code_t result, void * p_user_data)
{
    // NRF_LOG_INFO("read fifo end");
    xQueueSendFromISR(rdbuf_pending, &read_fifo_xfers[0].p_rx_data, NULL); 
}

static nrf_spi_mngr_transaction_t read_fifo_trans = {
    .begin_callback = NULL,
    .end_callback = read_fifo_end_callback,
    .p_user_data = NULL,
    .p_transfers = read_fifo_xfers,
    .number_of_transfers = sizeof(read_fifo_xfers) / sizeof(read_fifo_xfers[0]),
    .p_required_spi_cfg = &combo_ctx.spicfg
};

static uint8_t int_status_tx[2] = { REG_INT_STAT_1, REG_OP_READ };
static uint8_t int_status_rx[3] = { 0 };

static nrf_spi_mngr_transfer_t read_int_status_xfers[] =
{
    {
        .p_tx_data = int_status_tx,
        .tx_length = 2,
        .p_rx_data = int_status_rx,
        .rx_length = 3
    }
};

static void read_int_status_end_callback(ret_code_t esult, void * p_user_data)
{
    if (int_status_rx[2] & 0x80)
    {
        nrf_spi_mngr_schedule(&m_max86141_spi_mngr, &read_fifo_trans);
    }
    else 
    {
        // TODO do something
    }
}

static nrf_spi_mngr_transaction_t read_int_status_trans = {
    .begin_callback = NULL,
    .end_callback = read_int_status_end_callback,
    .p_user_data = NULL,
    .p_transfers = read_int_status_xfers,
    .number_of_transfers = 1,
    .p_required_spi_cfg = &combo_ctx.spicfg
};

// function type (nrfx_gpiote_evt_handler_t) defined in nrfx_gpiote.h, line 209
static void max86141_int_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    static int count = 0;
    NRF_LOG_INFO("int handler %d", count++);
    
    // TODO this is an error condition
    if (pdTRUE == xQueueReceiveFromISR(rdbuf_idle, &read_fifo_xfers[0].p_rx_data, NULL))
    {
        nrf_spi_mngr_schedule(&m_max86141_spi_mngr, &read_int_status_trans);        
    }
    else
    {
        NRF_LOG_INFO("rdbuf_idle empty, data lost");
    }
}

static void max86141_enable_int_pin(void)
{
    ret_code_t ret;
    nrfx_gpiote_in_config_t int_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    ret = nrfx_gpiote_in_init(MAX86141_COMBO_INT_PIN, &int_cfg, max86141_int_handler);
    APP_ERROR_CHECK(ret);
    
    nrfx_gpiote_in_event_enable(MAX86141_COMBO_INT_PIN, true);
    NRF_LOG_INFO("max86141 int pin %d initialized and enabled", MAX86141_COMBO_INT_PIN);
}

static void spi_config(void)
{
    uint32_t err_code;

    nrf_drv_spi_config_t const config = {
       .sck_pin            = MAX86141_COMBO_CK_PIN,
       .mosi_pin           = MAX86141_COMBO_DI_PIN,
       .miso_pin           = MAX86141_COMBO_DO_PIN,
       .ss_pin             = MAX86141_COMBO_CS_PIN,
       .irq_priority       = APP_IRQ_PRIORITY_LOWEST,
       .orc                = 0xff,
       .frequency          = NRF_DRV_SPI_FREQ_4M,

       // https://devzone.nordicsemi.com/f/nordic-q-a/39942/nrf52840---ads1292/155794
       // also datasheet
       .mode               = NRF_DRV_SPI_MODE_0,
       .bit_order          = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };

    err_code = nrf_spi_mngr_init(&m_max86141_spi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}

static void max86141_task(void * pvParameters)
{
    // very fast, change this to interrupt driven?
    TickType_t xFrequency = 1;

#ifdef COMBO_MODE
    
    rdbuf_idle = xQueueCreate(NUM_OF_RDBUF, sizeof(void *));
    ASSERT(rdbuf_idle != NULL);
    rdbuf_pending = xQueueCreate(NUM_OF_RDBUF, sizeof(void *));
    ASSERT(rdbuf_pending != NULL);
    
    for (int i = 0; i < NUM_OF_RDBUF; i++)
    {
        uint8_t *p = &rdbuf[i][0];
        xQueueSend(rdbuf_idle, &p, portMAX_DELAY);
    }
    
    // spi_config();
    
    uint8_t combo_count = 0;
    int combo_pkt_count = 0;
#else
    uint8_t spo_count = 0;
    uint8_t abp_count = 0;
    int spo_pkt_count = 0;
    int abp_pkt_count = 0;
#endif

	bloodoxygenInit();
	set_spo2_cofe(-7.1984,33.5650,60.5454,-46.3775,130.3776,0.9044);
	BLOOD_OXYGEN_INFO_STRU light_data = {0};
	BLOOD_OXYGEN_RESULT_STRU blood_result = {0};

#ifdef COMBO_MODE
    combo_ctx_init();
#else
    spo_ctx_init();
    abp_ctx_init();
#endif

#ifdef COMBO_MODE
    p_current_combo_packet = max86141_probe(&combo_ctx) ? next_combo_packet() : NULL;
    if (p_current_combo_packet)
    {
        // max86141_init_gpio();
        
        max86141_config(&combo_ctx);
        read_registers(&combo_ctx, 0x10, 7, combo_0x10_0x16_regs);
        read_registers(&combo_ctx, 0x20, 12, combo_0x20_0x2B_regs);
        
        
        max86141_enable_int_pin();
        max86141_run(&combo_ctx);

        NRF_LOG_INFO("combo max86141 started");
    }
#else
    p_current_spo_packet = max86141_probe(&spo_ctx) ? next_spo_packet() : NULL;
    if (p_current_spo_packet)
    {
        max86141_config(&spo_ctx);

        read_registers(&spo_ctx, 0x10, 7, spo_0x10_0x16_regs);
        read_registers(&spo_ctx, 0x20, 12, spo_0x20_0x2B_regs);
        max86141_run(&spo_ctx);

        NRF_LOG_INFO("spo max86141 started");
    }

    p_current_abp_packet = max86141_probe(&abp_ctx) ? next_abp_packet() : NULL;
    // p_current_abp_packet = NULL;
    if (p_current_abp_packet)
    {
        max86141_config(&abp_ctx);
        read_registers(&abp_ctx, 0x10, 7, abp_0x10_0x16_regs);
        read_registers(&abp_ctx, 0x20, 12, abp_0x20_0x2B_regs);
        max86141_run(&abp_ctx);

        NRF_LOG_INFO("abp max86141 started");
    }
#endif

    if (!p_current_combo_packet)
    {
        vTaskDelay(portMAX_DELAY);
    }
    
    // vTaskDelay(portMAX_DELAY);
    
    uint8_t *rdbuf;
    
    for (;;)
    {
        uint8_t *p = (uint8_t *)&m_combo_packet_helper.p_sample->value;
        int len = MAX86141_NUM_OF_SAMPLES / 2 * 3;
        
        // first half
        xQueueReceive(rdbuf_pending, &rdbuf, portMAX_DELAY);
        memcpy(p, &rdbuf[2], len);
        xQueueSend(rdbuf_idle, &rdbuf, portMAX_DELAY);
        
        // second half
        xQueueReceive(rdbuf_pending, &rdbuf, portMAX_DELAY);
        memcpy(&p[len], &rdbuf[2], len);
        xQueueSend(rdbuf_idle, &rdbuf, portMAX_DELAY);
        
        // TODO use multiple helper
        memcpy(&m_combo_packet_helper.p_ppgcfg->value, combo_0x10_0x16_regs, 7);
        memcpy(&m_combo_packet_helper.p_ledcfg->value, combo_0x20_0x2B_regs, 12);

        simple_crc((uint8_t *)&p_current_combo_packet->type, &m_combo_packet_helper.p_crc[0], &m_combo_packet_helper.p_crc[1]);
        cdc_acm_send_packet((uint8_t *)p_current_combo_packet, m_combo_packet_helper.packet_size);
        p_current_combo_packet = next_combo_packet();            
    }

    for (int rf = 0;; rf++)
    {
        vTaskDelay(xFrequency);
#ifdef COMBO_MODE

        // combo_count = read_reg(&combo_ctx, REG_FIFO_DATA_COUNT);
        if (combo_count < MAX86141_NUM_OF_SAMPLES) continue;
        // NRF_LOG_INFO("[%d], int status %d", combo_count, read_reg(&combo_ctx, REG_INT_STAT_1));

        // uint8_t int_status = read_reg(&combo_ctx, REG_INT_STAT_1);
        // if (!(int_status & 0x80)) continue;
        // NRF_LOG_INFO("r f %d", rf);
        
        read_fifo(&combo_ctx);

        memcpy(&m_combo_packet_helper.p_sample->value, buf, MAX86141_NUM_OF_SAMPLES * 3);
        memcpy(&m_combo_packet_helper.p_ppgcfg->value, combo_0x10_0x16_regs, 7);
        memcpy(&m_combo_packet_helper.p_ledcfg->value, combo_0x20_0x2B_regs, 12);

        for (int i = 0; i < MAX86141_NUM_OF_SAMPLES / 2; i++)
        {
        }

        simple_crc((uint8_t *)&p_current_combo_packet->type, &m_combo_packet_helper.p_crc[0], &m_combo_packet_helper.p_crc[1]);
        cdc_acm_send_packet((uint8_t *)p_current_combo_packet, m_combo_packet_helper.packet_size);
        p_current_combo_packet = next_combo_packet();

//        combo_pkt_count++;
//        if ((combo_pkt_count % 1000) == 0)
//        {
//            NRF_LOG_INFO("combo: %d packets sent", combo_pkt_count);
//        }
#else
        if (p_current_spo_packet)
        {
            spo_count = read_reg(&spo_ctx, REG_FIFO_DATA_COUNT);

            // NRF_LOG_INFO("spo_count: %d", spo_count);

            if (spo_count >= MAX86141_NUM_OF_SAMPLES)
            {
                read_fifo(&spo_ctx);

                // fill samples (type and length correct?)
                memcpy(&m_spo_packet_helper.p_sample->value, buf, MAX86141_NUM_OF_SAMPLES * 3);
                memcpy(&m_spo_packet_helper.p_ppgcfg->value, spo_0x10_0x16_regs, 7);
                memcpy(&m_spo_packet_helper.p_ledcfg->value, spo_0x20_0x2B_regs, 12);

                for (int i = 0; i < MAX86141_NUM_OF_SAMPLES / 2; i++)
                {
                    light_data.ir = (((unsigned int)(buf[i * 6 + 0] & 0x07)) << 16) + (((unsigned int)buf[i * 6 + 1]) << 8) + (unsigned int)buf[i * 6 + 2];
                    light_data.rd = (((unsigned int)(buf[i * 6 + 3] & 0x07)) << 16) + (((unsigned int)buf[i * 6 + 4]) << 8) + (unsigned int)buf[i * 6 + 5];

                    getBOResult(light_data, &blood_result);
                    max86141_rougu_data_t *p_data_base = (max86141_rougu_data_t *)&m_spo_packet_helper.p_rougu_spo->value;
                    max86141_rougu_data_t *p_data = &p_data_base[i];
                    p_data->ir = light_data.ir;
                    p_data->rd = light_data.rd;
                    p_data->irdc = light_data.irdc;
                    p_data->rddc = light_data.rddc;
                    p_data->irFilt = blood_result.filterIr;
                    p_data->rdFilt = blood_result.filterRd;
                    p_data->spo = blood_result.saO2;
                    p_data->hr = blood_result.heartRate;
                }

                simple_crc((uint8_t *)&p_current_spo_packet->type, &m_spo_packet_helper.p_crc[0], &m_spo_packet_helper.p_crc[1]);
                cdc_acm_send_packet((uint8_t *)p_current_spo_packet, m_spo_packet_helper.packet_size);
                p_current_spo_packet = next_spo_packet();

                spo_pkt_count++;
                if ((spo_pkt_count % 100) == 0)
                {
                    NRF_LOG_INFO("spo: %d packets sent", spo_pkt_count);
                }
            }
        }

        if (p_current_abp_packet)
        {
            abp_count = read_reg(&abp_ctx, REG_FIFO_DATA_COUNT);
            if (abp_count >= MAX86141_NUM_OF_SAMPLES)
            {
                read_fifo(&abp_ctx);

                // fill samples (type and length correct?)
                memcpy(&m_abp_packet_helper.p_sample->value, buf, MAX86141_NUM_OF_SAMPLES * 3);
                memcpy(&m_abp_packet_helper.p_ppgcfg->value, abp_0x10_0x16_regs, 7);
                memcpy(&m_abp_packet_helper.p_ledcfg->value, abp_0x20_0x2B_regs, 12);

                simple_crc((uint8_t *)&p_current_abp_packet->type, &m_abp_packet_helper.p_crc[0], &m_abp_packet_helper.p_crc[1]);
                cdc_acm_send_packet((uint8_t *)p_current_abp_packet, m_abp_packet_helper.packet_size);
                p_current_abp_packet = next_abp_packet();

                abp_pkt_count++;
                if (abp_pkt_count % 100 == 0)
                {
                    NRF_LOG_INFO("abp: %d packets sent", abp_pkt_count);
                }
            }

#endif
    }
}

#endif // rougu task or normal task

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

#ifdef COMBO_MODE

static sens_packet_t * next_combo_packet(void)
{
    static bool initialized = false;
    static sens_packet_t * p_packet_pool[SENS_PACKET_POOL_SIZE] = {0};
    static int next_modulo = 0;

    if (!initialized)
    {
        // init m_packet helper
        sens_init_max86141_packet(&m_combo_packet_helper, NULL);

        // alloc mem
        for (int i = 0; i < SENS_PACKET_POOL_SIZE; i++)
        {
            p_packet_pool[i] = pvPortMalloc(m_combo_packet_helper.packet_size);
            APP_ERROR_CHECK_BOOL(p_packet_pool[i] != NULL);
        }

        initialized = true;

        NRF_LOG_INFO("combo packets init, len: %d, size: %d, %d samples",
            m_combo_packet_helper.payload_len,
            m_combo_packet_helper.packet_size,
            MAX86141_NUM_OF_SAMPLES);
    }

    // init next packet
    sens_init_max86141_packet(&m_combo_packet_helper, p_packet_pool[next_modulo]);
    sens_packet_t * next = p_packet_pool[next_modulo];
    next_modulo = (next_modulo + 1) % SENS_PACKET_POOL_SIZE;
    return next;
}

#else

static sens_packet_t * next_spo_packet(void)
{
    static bool initialized = false;
    static sens_packet_t * p_packet_pool[SENS_PACKET_POOL_SIZE] = {0};
    static int next_modulo = 0;

    if (!initialized)
    {
        // init m_packet helper
        sens_init_max86141_packet(&m_spo_packet_helper, NULL);

        // alloc mem
        for (int i = 0; i < SENS_PACKET_POOL_SIZE; i++)
        {
            p_packet_pool[i] = pvPortMalloc(m_spo_packet_helper.packet_size);
            APP_ERROR_CHECK_BOOL(p_packet_pool[i] != NULL);
        }

        initialized = true;

        NRF_LOG_INFO("spo packets init, len: %d, size: %d, %d samples",
            m_spo_packet_helper.payload_len,
            m_spo_packet_helper.packet_size,
            MAX86141_NUM_OF_SAMPLES);
    }

    // init next packet
    sens_init_max86141_packet(&m_spo_packet_helper, p_packet_pool[next_modulo]);
    sens_packet_t * next = p_packet_pool[next_modulo];
    next_modulo = (next_modulo + 1) % SENS_PACKET_POOL_SIZE;
    return next;
}

static sens_packet_t * next_abp_packet(void)
{
    static bool initialized = false;
    static sens_packet_t * p_packet_pool[SENS_PACKET_POOL_SIZE] = {0};
    static int next_modulo = 0;

    if (!initialized)
    {
        // init m_packet helper
        sens_init_max86141_packet(&m_abp_packet_helper, NULL);

        // alloc mem
        for (int i = 0; i < SENS_PACKET_POOL_SIZE; i++)
        {
            p_packet_pool[i] = pvPortMalloc(m_abp_packet_helper.packet_size);
            APP_ERROR_CHECK_BOOL(p_packet_pool[i] != NULL);
        }

        initialized = true;

        NRF_LOG_INFO("abp packets init, len: %d, size: %d, %d samples",
            m_abp_packet_helper.payload_len,
            m_abp_packet_helper.packet_size,
            MAX86141_NUM_OF_SAMPLES);
    }

    // init next packet
    sens_init_max86141_packet(&m_abp_packet_helper, p_packet_pool[next_modulo]);
    sens_packet_t * next = p_packet_pool[next_modulo];
    next_modulo = (next_modulo + 1) % SENS_PACKET_POOL_SIZE;
    return next;
}

#endif
