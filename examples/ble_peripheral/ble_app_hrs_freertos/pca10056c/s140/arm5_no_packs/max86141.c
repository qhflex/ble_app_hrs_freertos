/*******************************************************************************
 * INCLUDES
 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "SEGGER_RTT.h"

#include "nrfx_gpiote.h"
#include "nrf_drv_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_spi_mngr.h"

#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

#include "crc32.h"

#include "usbcdc.h"
#include "max86141.h"
#include "sens-proto.h"

#include "ble_nus_tx.h"
#include "oled.h"

/***********************************************************************
 * includes for abp algo
 */

// #include "check.h"
#include "dynamic_array.h"
#include "feature.h"
#include "pressure.h"
// #include "log.h"
// #include "macro.h"
// #include "type.h"
// #include "util.hh"

/**********************************************************************
 * MACROS
 */
int watchout = 0;

/**********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */ 
// obsolete
typedef struct __attribute__((packed)) max_ble_pac
{
    uint16_t len;           //          dataview offset
    uint8_t type;           //          0
    uint8_t seq;            //  4       1
    uint16_t heartRate;     //          2
    uint16_t saO2;          //  4       4
    uint32_t rsvd;          //  4       6
    uint8_t ir1[10 * 4];    // 40       10
    uint8_t ir2[10 * 4];    // 40       50
    uint8_t rd1[10 * 4];    // 40       90
                            // 120 + 12 = 132
} max_ble_pac_t;

STATIC_ASSERT(sizeof(max_ble_pac_t) == 132);

typedef struct __attribute__((packed)) max_ble_spo_pac
{
    uint16_t len;
    uint8_t type;           // 3
    uint8_t seq;            // not used
    uint32_t saO2;          // 50 values avg
    uint32_t heartRate;     // 50 values avg
    uint32_t ir1[10];       // 5 values avg
    uint32_t rd1[10];       // 5 values avg
} max_ble_spo_pac_t;        // 90 bytes in total (packet size), 1 packet per second

STATIC_ASSERT(sizeof(max_ble_spo_pac_t) == 92);

/**
 * for abp, sampling rate is 2048 and each packet contains 256 samples, which means 
 * 8 packets per second. 1 of them have feature / bp data.
 * for ble, we have 2 samples per packet.
 */
typedef struct __attribute__((packed)) max_ble_abp_pac
{
    uint16_t    len;
    uint8_t     type;           // 4
    uint8_t     seq;            // not used
    int         sbp;            // -1 for invalid
    int         dbp;            // -1 for invalid
    uint32_t    ir1[2];         // 128-avg
    uint32_t    ir2[2];         // 128-avg
} max_ble_abp_pac_t;

STATIC_ASSERT(sizeof(max_ble_abp_pac_t) == 28);

typedef struct __attribute__((packed)) max_ble_abp_coeff
{
    uint16_t    len;
    uint8_t     type;
    uint8_t     seq;
    ieee754_t   sbp[2];
    ieee754_t   dbp[2];
} max_ble_abp_coeff_t;

STATIC_ASSERT(sizeof(max_ble_abp_coeff_t) == 20);

#define PREF_PAYLOAD_SIZE   16

typedef struct __attribute__((packed)) pref {
    ieee754_t sbp[2]; // high
    ieee754_t dbp[2]; // low
    uint32_t crc32;
} pref_t;

STATIC_ASSERT(sizeof(pref_t) == PREF_PAYLOAD_SIZE + 4);

static pref_t m_pref;

/**********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static const nrf_drv_timer_t m_max86141_timer = NRF_DRV_TIMER_INSTANCE(MAX86141_TIMER_INDEX);

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
static uint32_t nrf5_flash_end_addr_get(void);

#define PREF_PAGE_START 0xff000
#define PREF_PAGE_END   0xfffff

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = PREF_PAGE_START,      // 0x3e000,
    .end_addr   = PREF_PAGE_END,        // 0x3ffff,
};


/*********************************************************************
 * macro, typedef, statics for get/set abp coefficient
 */
bool m_get_abp_coeff = false;
static QueueHandle_t q_set_abp_coeff = NULL;
static void handle_abp_coeff_req(void);

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void max86141_timer_callback(nrf_timer_event_t event_type, void *p_context);
static void max86141_timer_init(void);
static void max86141_timer_start(uint32_t millisec);
static void max86141_timer_stop(void);

static void max86141_spi_config(void);

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get(void)
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

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

#define TSK_MAX86141_STACK_SIZE             (256 * 10)       // 9KBytes
#define TSK_MAX86141_PRIORITY               2

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

#define MAX86141_CS_PIN                     17
#define MAX86141_CK_PIN                     14
#define MAX86141_DI_PIN                     15
#define MAX86141_DO_PIN                     13

// The same
//#define MAX86141_SPO_CS_PIN                 17
//#define MAX86141_SPO_CK_PIN                 14
//#define MAX86141_SPO_DI_PIN                 15
//#define MAX86141_SPO_DO_PIN                 13

//#define MAX86141_ABP_CS_PIN                 17
//#define MAX86141_ABP_CK_PIN                 14
//#define MAX86141_ABP_DI_PIN                 15
//#define MAX86141_ABP_DO_PIN                 13

#endif // COMBO_MODE

#define MAX_PENDING_TRANSACTIONS            5

/* this value only determine when the interrupt is fired */
#define FIFO_READ_SAMPLES                   MAX86141_NUM_OF_SAMPLE_ITEMS         // 60 * 3 + 2 should not exceed 255 (uint8_t of spi rx buf size)
#define SPI_FIFO_RX_SIZE                    255
// #define SPI_FIFO_RX_SIZE_ROUGU              (2 * 3 + 2)

#define SENS_PACKET_POOL_SIZE               4

#define NUM_OF_PKTBUF                       2
#define PKTBUF_SIZE                         1600 // 1596 for abp

#define NUM_OF_RAWBUF                       4
#define MAX_FIFO_READ_ITEM_COUNT            84

/*********************************************************************
 * TYPEDEFS
 */
typedef struct __attribute__((packed)) max86141_sample_item
{
    uint8_t byte[3];
    // unsigned int tag : 5;
    // unsigned int val : 19;
} max86141_sample_item_t;

STATIC_ASSERT(sizeof(max86141_sample_item_t) == 3);

// max spi buffer 255 bytes, with first two bytes invalid.
// => max items 84, since 84 * 3 = 252
typedef struct __attribute__((packed)) raw_sample_data
{
    uint8_t item_count;
    uint8_t rx[2];
    max86141_sample_item_t items[MAX_FIFO_READ_ITEM_COUNT];
} raw_sample_data_t;

STATIC_ASSERT(sizeof(raw_sample_data_t) == 255);

//typedef struct __attribute__((packed)) max86141_spo_sample
//{
//    unsigned int tag1 : 5;
//    unsigned int val_high : 3;
//    unsigned int val_low: 16;
//    unsigned int tag2 : 5;
//    unsigned int val2_high: 3;
//    unsigned int val2_low : 16;
//} max86141_spo_sample_t;

//STATIC_ASSERT(sizeof(max86141_spo_sample_t) == 8);

/**********************************************************************
 * GLOBAL VARIABLES
 */

// 0 for idle
// 1 for spo
// 2 for abp
int max86141_function = 0;

/**********************************************************************
 * LOCAL VARIABLES
 */
static TaskHandle_t m_max86141_thread = NULL;


// static QueueHandle_t q_pkts_idle = NULL;
// static QueueHandle_t q_pkts_pend = NULL;
static uint8_t pktbuf[NUM_OF_PKTBUF][PKTBUF_SIZE];
static uint8_t cfg_pktbuf[MAX86141_ABP_CFGPKT_SIZE];

static QueueHandle_t q_smpl_idle = NULL;
static QueueHandle_t q_smpl_pend = NULL;
static raw_sample_data_t raw_sample_buf[NUM_OF_RAWBUF];

NRF_SPI_MNGR_DEF(m_max86141_spi_mngr, MAX_PENDING_TRANSACTIONS, SPI_INSTANCE_ID);

//const static uint8_t spi_fifo_tx[2] = { REG_FIFO_DATA, REG_OP_READ };
//static uint8_t spi_fifo_rx[SPI_FIFO_RX_SIZE] = {0};
//static uint8_t * buf = &spi_fifo_rx[2];

#if defined MIMIC_ROUGU && MIMIC_ROUGU == 1
static uint8_t rougu[15] = {0};
#endif

#ifdef COMBO_MODE

static uint8_t combo_0x10_0x16_regs[7] = {0};
static uint8_t combo_0x20_0x2B_regs[12] = {0};

#else



//static uint8_t abp_0x10_0x16_regs[7] = {0}; // for 0x10-0x16
//static uint8_t abp_0x20_0x2B_regs[12] = {0};

#endif

//const static nrf_spi_mngr_transfer_t max86141_fifo_xfers[] =
//{
//    NRF_SPI_MNGR_TRANSFER(spi_fifo_tx, 2, spi_fifo_rx, SPI_FIFO_RX_SIZE),
//};

//const static nrf_spi_mngr_transfer_t max86141_fifo_xfers_rougu[] =
//{
//    NRF_SPI_MNGR_TRANSFER(spi_fifo_tx, 2, spi_fifo_rx, 8), // 6 + 2
//};

#ifdef COMBO_MODE

const static max86141_cfg_t combo_maxcfg = {
    .inten1 = {
        .a_full_en      = 1,    // enable fifo
    },
    .fifocfg1 = {
        .fifo_a_full    = (128 - (MAX86141_NUM_OF_SAMPLE_ITEMS / 2)),
    },
    .fifocfg2 = {
        .flush_fifo     = 1,    // required
        .fifo_stat_clr  = 1,    // set to 1 to clear interrup when reading fifo (no need to read status reg)
        .a_full_type    = 1,    // 0 - AFULL_RPT (REPEAT, default), 1 - AFULL_ONCE
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
        .led_setlng     = 3,    // 0b00  4.0uS
                                // 0b01  6.0uS
                                // 0b10  8.0uS
                                // 0b11 12.0us <-
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
        .fifo_stat_clr  = 1,    // set to 1 to clear interrup when reading fifo (no need to read status reg)
        .a_full_type    = 1,    // 0 - AFULL_RPT (REPEAT, default), 1 - AFULL_ONCE
        .fifo_ro        = 1,    // drop old samples when fifo full
    },
    .sysctrl = {
        .single_ppg     = 1,    // only one channle is used
    },
    .ppgcfg1 = {
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
        .ledc135        = 1,      // 0001 LED1, ir
        .ledc246        = 2,      // 0010 LED2, red
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
        .fifo_stat_clr  = 1,    // see above
        .a_full_type    = 1,    // see above
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

        .ppg_tint       = 2,    // pulse width = tint + tsetlng + 0.5uS = 129.8uS
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
                                // 0x12 2048 sps <-
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
        .led36_rge      = 1,
//      .led25_rge      = 1,    // 0b00  31mA
        .led14_rge      = 1,    // 0b01  62mA
                                // 0b10  93mA
                                // 0b11 124mA
    },
    .led1pa             = 0x30, // lsb =  0.12 when rge is 00 (0b00)
                                //        0.24 when rge is 01 (0b01)
                                //        0.36 when rge is 02 (0b10)
                                //        0.48 when rge is 03 (0b11)

//  .led2pa             = 0x30,
    .led3pa             = 0x30,

    .ledseq1 = {
        .ledc135        = 5,    // 0101 LED1 + LED3
    },
};

#endif

static max86141_ctx_t spo_ctx = { .p_maxcfg = &spo_maxcfg, };
static max86141_ctx_t abp_ctx = { .p_maxcfg = &abp_maxcfg, };
// static max86141_packet_helper_t m_spo_packet_helper = { .instance_id = 0 };
// static max86141_packet_helper_t m_abp_packet_helper = { .instance_id = 1 };


#ifdef COMBO_MODE

// static sens_packet_t *p_current_combo_packet;

#else

// static sens_packet_t *p_current_spo_packet;
// static sens_packet_t *p_current_abp_packet;

#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void write_reg(max86141_ctx_t * ctx, uint8_t addr, uint8_t data_in);

static uint8_t read_reg(max86141_ctx_t * ctx, uint8_t addr);
static void read_registers(max86141_ctx_t * ctx, uint8_t startAddr, int num, uint8_t * outbuf);
static void print_registers(max86141_ctx_t * ctx);
// static bool max86141_probe(max86141_ctx_t * ctx);
static void max86141_config(max86141_ctx_t * ctx);
static void max86141_start(max86141_ctx_t * ctx);
static void max86141_stop(max86141_ctx_t * ctx);
// static void read_fifo(max86141_ctx_t * ctx);

// static void reset_pkts_queue(void);
static void reset_smpl_queue(void);


#ifdef COMBO_MODE
// this sums up to 240 (or 180)
uint32_t btx_ir1_buf[10];
uint32_t btx_ir2_buf[10];
uint32_t btx_rd1_buf[10];
uint32_t btx_item_num = 0;
#endif

#ifdef COMBO_MODE
static sens_packet_t * next_combo_packet(void);
#else
// static sens_packet_t * next_spo_packet(void);
// static sens_packet_t * next_abp_packet(void);
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

//static void read_fifo(max86141_ctx_t * ctx)
//{
//    ret_code_t err_code;
//    err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, NULL, max86141_fifo_xfers, 1, NULL);
//    APP_ERROR_CHECK(err_code);
//}

//static void read_fifo_rougu(max86141_ctx_t * ctx)
//{
//    ret_code_t err_code;
//    err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, &ctx->spicfg, max86141_fifo_xfers_rougu, 1, NULL);
//    APP_ERROR_CHECK(err_code);
//}

static void write_reg(max86141_ctx_t * ctx, uint8_t addr, uint8_t data_in)
{
    ret_code_t err_code;
    uint8_t txbuf[3] = {0};
    uint8_t rxbuf[3] = {0};
    nrf_spi_mngr_transfer_t xfer;

    txbuf[0] = addr;
    txbuf[1] = REG_OP_WRITE;
    txbuf[2] = data_in;

    xfer.p_tx_data = txbuf;
    xfer.tx_length = 3;
    xfer.p_rx_data = rxbuf;
    xfer.rx_length = 3;

    err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, NULL, &xfer, 1, NULL);
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
    uint8_t txbuf[3] = {0};
    uint8_t rxbuf[3] = {0};
    nrf_spi_mngr_transfer_t xfer;

    txbuf[0] = addr;
    txbuf[1] = REG_OP_READ;
    txbuf[2] = 0;

    xfer.p_tx_data = txbuf;
    xfer.tx_length = 3;
    xfer.p_rx_data = rxbuf;
    xfer.rx_length = 3;

    err_code = nrf_spi_mngr_perform(&m_max86141_spi_mngr, NULL, &xfer, 1, NULL);
    APP_ERROR_CHECK(err_code);

    // NRF_LOG_INFO("read reg %d %d", addr, rxbuf[2]);
    // SEGGER_RTT_printf(0, "rreg %d %d\r\n", addr, rxbuf[2]);

    return rxbuf[2];
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

//static bool max86141_probe(max86141_ctx_t *ctx)
//{
//    uint8_t val = read_reg(ctx, 0xff);
//    NRF_LOG_INFO("max86141 probe 0xff returns: %02x", val);

//    if (val == 0x25) {
//#ifdef COMBO_MODE
//        if (ctx == &combo_ctx)
//        {
//            NRF_LOG_INFO("combo max86141 probed");
//        }
//#else
//        if (ctx == &spo_ctx)
//        {
//            NRF_LOG_INFO("spo max86141 probed");
//        }

//        if (ctx == &abp_ctx)
//        {
//            NRF_LOG_INFO("abp max86141 probed");
//        }
//#endif
//    }

//    return (val == 0x25);
//}

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

static void max86141_start(max86141_ctx_t * ctx)
{
    uint8_t sysctrl = read_reg(ctx, REG_SYSCTRL);
    sysctrl &= ~(0x02);    // SHDN @ bit 1, clear this bit
    write_reg(ctx, REG_SYSCTRL, sysctrl);
}

static void max86141_stop(max86141_ctx_t * ctx)
{
    write_reg(ctx, REG_SYSCTRL, 0x01);              // 0b00000001);    // Soft Reset
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

// static void spo_ctx_init()
// {
//    spo_ctx.xfer.p_tx_data = spo_ctx.txbuf;
//    spo_ctx.xfer.tx_length = 3;
//    spo_ctx.xfer.p_rx_data = spo_ctx.rxbuf;
//    spo_ctx.xfer.rx_length = 3;
// }

// static void abp_ctx_init()
// {
//    abp_ctx.xfer.p_tx_data = abp_ctx.txbuf;
//    abp_ctx.xfer.tx_length = 3;
//    abp_ctx.xfer.p_rx_data = abp_ctx.rxbuf;
//    abp_ctx.xfer.rx_length = 3;

    // abp_ctx.spicfg = NRF_DRV_SPI_DEFAULT_CONFIG;
//    abp_ctx.spicfg.ss_pin        = MAX86141_ABP_CS_PIN;
//    abp_ctx.spicfg.sck_pin       = MAX86141_ABP_CK_PIN;
//    abp_ctx.spicfg.mosi_pin      = MAX86141_ABP_DI_PIN;
//    abp_ctx.spicfg.miso_pin      = MAX86141_ABP_DO_PIN;
//    abp_ctx.spicfg.orc           = 0xff;
//    abp_ctx.spicfg.mode          = NRF_DRV_SPI_MODE_0;
//    abp_ctx.spicfg.irq_priority  = APP_IRQ_PRIORITY_LOWEST;
//    abp_ctx.spicfg.frequency     = NRF_DRV_SPI_FREQ_4M;
//    abp_ctx.spicfg.bit_order     = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
// }

//static bool is_invalid(uint8_t msb)
//{
//    return ((msb >> 3) == 0x1e);
//}

//static nrf_spi_mngr_transfer_t read_fifo_xfers[] =
//{
//    {
//        .p_tx_data = spi_fifo_tx,
//        .tx_length = 2,
//        .p_rx_data = NULL,
//        .rx_length = RDBUF_SIZE
//    }
//};

//static void read_fifo_end_callback(ret_code_t result, void * p_user_data)
//{
//    // NRF_LOG_INFO("read fifo end");
//    xQueueSendFromISR(rdbuf_pending, &read_fifo_xfers[0].p_rx_data, NULL);
//}

//static nrf_spi_mngr_transaction_t read_fifo_trans = {
//    .begin_callback = NULL,
//    .end_callback = read_fifo_end_callback,
//    .p_user_data = NULL,
//    .p_transfers = read_fifo_xfers,
//    .number_of_transfers = sizeof(read_fifo_xfers) / sizeof(read_fifo_xfers[0]),
//    // .p_required_spi_cfg = &combo_ctx.spicfg
//};

// not used
// static uint8_t int_status_tx[2] = { REG_INT_STAT_1, REG_OP_READ };
// static uint8_t int_status_rx[3] = { 0 };

// not used
//static nrf_spi_mngr_transfer_t read_int_status_xfers[] =
//{
//    {
//        .p_tx_data = int_status_tx,
//        .tx_length = 2,
//        .p_rx_data = int_status_rx,
//        .rx_length = 3
//    }
//};

// not used
//static void read_int_status_end_callback(ret_code_t esult, void * p_user_data)
//{
//    if (int_status_rx[2] & 0x80)
//    {
//        nrf_spi_mngr_schedule(&m_max86141_spi_mngr, &read_fifo_trans);
//    }
//    else
//    {
//        // TODO do something
//    }
//}

// not used
//static nrf_spi_mngr_transaction_t read_int_status_trans = {
//    .begin_callback = NULL,
//    .end_callback = read_int_status_end_callback,
//    .p_user_data = NULL,
//    .p_transfers = read_int_status_xfers,
//    .number_of_transfers = 1,
//    // .p_required_spi_cfg = &combo_ctx.spicfg
//};

// function type (nrfx_gpiote_evt_handler_t) defined in nrfx_gpiote.h, line 209
//static void max86141_int_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
//{
//    static int count = 1;
//    count++;

//    static int sum = 0;
//    sum += uxQueueMessagesWaitingFromISR(rdbuf_pending);

//    if (count % 100 == 0)
//    {
//        NRF_LOG_INFO("max isr %d, watermark: %d", count, sum);
//        sum = 0;
//    }

//    // TODO this is an error condition
//    if (pdTRUE == xQueueReceiveFromISR(rdbuf_idle, &read_fifo_xfers[0].p_rx_data, NULL))
//    {
//        // nrf_spi_mngr_schedule(&m_max86141_spi_mngr, &read_int_status_trans);
//        nrf_spi_mngr_schedule(&m_max86141_spi_mngr, &read_fifo_trans);
//    }
//    else
//    {
//        NRF_LOG_INFO("rdbuf_idle empty, data lost");
//    }
//}

#if 0
static void max86141_enable_int_pin(void)
{
    ret_code_t ret;
    nrfx_gpiote_in_config_t int_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    ret = nrfx_gpiote_in_init(MAX86141_COMBO_INT_PIN, &int_cfg, max86141_int_handler);
    APP_ERROR_CHECK(ret);

    nrfx_gpiote_in_event_enable(MAX86141_COMBO_INT_PIN, true);
    NRF_LOG_INFO("max86141 int pin %d initialized and enabled", MAX86141_COMBO_INT_PIN);
}
#endif

//static uint32_t read_sample_item(uint8_t *p, int sample_index, int item_index, int item_per_sample)
//{
//    uint32_t val = 0;
//    int item_size = 3;
//    int abs_item_index = sample_index * item_per_sample + item_index;

//    val += ((unsigned int)(p[item_size * abs_item_index + 0] & 0x07)) << 16;
//    val += ((unsigned int)(p[item_size * abs_item_index + 1])) << 8;
//    val += ((unsigned int)(p[item_size * abs_item_index + 2]));

//    return val;
//}

/*
 * This callback enqueue result buffer to pending queue.
 *
 */
static void read_fifo_data_callback(ret_code_t result, void *p_user_data)
{
    raw_sample_data_t *p_raw = (raw_sample_data_t*)p_user_data;
    if (result == NRF_SUCCESS)
    {
        xQueueSend(q_smpl_pend, &p_raw, 0);
    }
    else
    {
        // TODO is this acceptable?
        xQueueSend(q_smpl_idle, &p_raw, 0);
    }

    // SEGGER_RTT_printf(0, "fd cb, %d items read\r\n", p_raw->item_count);
}

/*
 * This callback accepts fifo data (item) count returned from spi read, and
 * schedule a spi read on fifo data, reading max possible even number of items.
 * The pointer to raw sample buffer is passed to next callback
 */
static void read_fifo_data_count_callback(ret_code_t result, void *p_user_data)
{
    uint8_t *p_item_count = (uint8_t *)p_user_data;

    APP_ERROR_CHECK(result);
    APP_ERROR_CHECK_BOOL(p_item_count != NULL);

    // SEGGER_RTT_printf(0, "fdc cb %d, q idle %d, count: %d\r\n", result, uxQueueMessagesWaitingFromISR(q_smpl_idle), *p_item_count);

    // alloc buffer
    raw_sample_data_t *p_raw = NULL;
    if (pdTRUE != xQueueReceiveFromISR(q_smpl_idle, &p_raw, NULL))
    {
        return; // !!! TODO
    }

    // SEGGER_RTT_printf(0, "p_raw %p\r\n", p_raw);

    APP_ERROR_CHECK_BOOL(p_raw != NULL);

    // SEGGER_RTT_printf(0, "p_raw not null\r\n");

    p_raw->item_count = *p_item_count / 2 * 2;
    if (p_raw->item_count > MAX_FIFO_READ_ITEM_COUNT)
    {
        p_raw->item_count = MAX_FIFO_READ_ITEM_COUNT;
    }

    // TODO could this be const?
    static uint8_t txbuf[3] = { REG_FIFO_DATA, REG_OP_READ, 0xff };
    static nrf_spi_mngr_transfer_t read_fifo_data_xfers[] =
    {
        {
            .p_tx_data = txbuf,
            .tx_length = 3,
            .p_rx_data = NULL,  // to be set
            .rx_length = 0,     // to be set
        }
    };

    static nrf_spi_mngr_transaction_t read_fifo_data_trans = {
        .begin_callback = NULL,
        .end_callback = read_fifo_data_callback,
        .p_user_data = NULL,    // to be set
        .p_transfers = read_fifo_data_xfers,
        .number_of_transfers = sizeof(read_fifo_data_xfers) / sizeof(read_fifo_data_xfers[0]),
    };

    read_fifo_data_xfers[0].p_rx_data = p_raw->rx;
    read_fifo_data_xfers[0].rx_length = 2 + 3 * p_raw->item_count;
    read_fifo_data_trans.p_user_data = p_raw;   // pointer to raw sample buffer

    nrf_spi_mngr_schedule(&m_max86141_spi_mngr, &read_fifo_data_trans);
}

//static void read_int_status_end_callback(ret_code_t esult, void * p_user_data)
//{
//    if (int_status_rx[2] & 0x80)
//    {
//        nrf_spi_mngr_schedule(&m_max86141_spi_mngr, &read_fifo_trans);
//    }
//    else
//    {
//        // TODO do something
//    }
//}

/*
 * This function schedule a spi read on REG_FIFO_DATA_COUNT
 * and pass the pointer to the result to the next callback.
 * The value of the result is the number of sample items (in the unit of triple bytes)
 * in fifo, according to MAX86141 datasheet.
 */
static void max86141_timer_callback(nrf_timer_event_t event_type, void *p_context)
{
    static uint8_t txbuf[3] = { REG_FIFO_DATA_COUNT, REG_OP_READ, 0xff };
    static uint8_t rxbuf[3];

    static nrf_spi_mngr_transfer_t read_fifo_data_count_xfers[] =
    {
        {
            .p_tx_data = txbuf,
            .tx_length = 3,
            .p_rx_data = rxbuf,
            .rx_length = 3
        }
    };

    static nrf_spi_mngr_transaction_t read_fifo_data_count_trans = {
        .begin_callback = NULL,
        .end_callback = read_fifo_data_count_callback,
        .p_user_data = &rxbuf[2], // pass the pointer to single byte result to next callback
        .p_transfers = read_fifo_data_count_xfers,
        .number_of_transfers = sizeof(read_fifo_data_count_xfers) / sizeof(read_fifo_data_count_xfers[0]),
    };

    // NRF_LOG_INFO("timer callback");
    // SEGGER_RTT_printf(0, "timer callback\r\n");

    nrf_spi_mngr_schedule(&m_max86141_spi_mngr, &read_fifo_data_count_trans);
}

static void max86141_timer_init()
{
    ret_code_t err_code;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32; // 16bit max is 65.535ms which is not enough

    err_code = nrf_drv_timer_init(&m_max86141_timer, &timer_cfg, max86141_timer_callback);
    APP_ERROR_CHECK(err_code);
}

//static void max86141_timer_uninit()
//{
//    nrf_drv_timer_uninit(&m_max86141_timer);
//}

static void max86141_timer_start(uint32_t cc_value_us)
{
    // uint32_t cc_value_us = 480; // 1000000 / 2048 = 488.28125
    nrf_drv_timer_extended_compare(&m_max86141_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   cc_value_us,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
    nrf_drv_timer_enable(&m_max86141_timer);
}

static void max86141_timer_stop()
{
    // nrf_drv_timer_compare_int_disable(&m_max86141_timer, NRF_TIMER_CC_CHANNEL0);
    nrf_drv_timer_disable(&m_max86141_timer);
}

static sens_packet_t* prepare_spo_packet(max86141_packet_helper_t *p_helper, sens_packet_t* p_curr)
{
    memset(p_helper, 0, sizeof(max86141_packet_helper_t));
    p_helper->has_spo_sample = true;
    p_helper->has_spo_rougu = true;
    p_helper->instance_id = 0;

    // TODO factor out a new function
    sens_packet_t* p_next = NULL;

    if (p_curr == (sens_packet_t *)&pktbuf[0][0])
    {
        p_next = (sens_packet_t *)&pktbuf[1][0];
    }
    else if (p_curr == (sens_packet_t *)&pktbuf[1][0])
    {
        p_next = (sens_packet_t *)&pktbuf[0][0];
    }
    else
    {
        p_next = (sens_packet_t *)&pktbuf[0][0];
    }

    if (!sens_format_max86141_packet(p_helper, p_next, PKTBUF_SIZE))
    {
        SEGGER_RTT_printf(0, "pktbuf undersized for spo");
        return NULL;
    }

    // spo pkt 782,
    // payload len 768 (9 + 150 + 600 = 759, + 3 * 3 = 768)
    // brief len 9
    // sample len 150 (25 samples and 50 items, by 3 bytes)
    // rougu len 600 (25 samples by 24 bytes)
    // SEGGER_RTT_printf(0, "spo pkt size %d\r\n", p_helper->packet_size);
    // SEGGER_RTT_printf(0, "spo payload len %d\r\n", p_helper->payload_len);
    // SEGGER_RTT_printf(0, "spo brief len %d\r\n", p_helper->p_brief->length);
    // SEGGER_RTT_printf(0, "spo sample len %d\r\n", p_helper->p_spo_sample->length);
    // SEGGER_RTT_printf(0, "spo rougu len %d\r\n", p_helper->p_spo_rougu->length);

    return p_next;
}

static sens_packet_t* prepare_abp_packet(max86141_packet_helper_t *p_helper, sens_packet_t* p_curr)
{
    memset(p_helper, 0, sizeof(max86141_packet_helper_t));
    p_helper->has_abp_sample = true;
    p_helper->has_abp_feature = true;
    p_helper->instance_id = 1;

    sens_packet_t * p_next = NULL;

    if (p_curr == (sens_packet_t *)&pktbuf[0][0])
    {
        p_next = (sens_packet_t *)&pktbuf[1][0];
    }
    else if (p_curr == (sens_packet_t *)&pktbuf[1][0])
    {
        p_next = (sens_packet_t *)&pktbuf[0][0];
    }
    else
    {
        p_next = (sens_packet_t *)&pktbuf[0][0];
    }

    if (!sens_format_max86141_packet(p_helper, p_next, PKTBUF_SIZE))
    {
        SEGGER_RTT_printf(0, "pktbuf undersized for abp");
        return NULL;
    }

    return p_next;
}

static void max86141_run_spo(void)
{
    // 120 packets per minute
    int max_pkts = 3 * 60 * 50 / MAX86141_NUM_OF_SPO_SAMPLES;
    
    sens_packet_t *p_pkt = NULL;
    sens_packet_t *p_cfgpkt = NULL;

    max86141_packet_helper_t helper;

    // int cfgpkt_size = 0;
    bool cfgpkt_sent = false;
    int sample_in_pkt = 0;
    
    int spo_pkt_count = 0;
    
    int saO2_sum = 0;
    int hr_sum = 0;
    int ir_sum[10] = {0};
    int rd_sum[10] = {0};

    // reset_pkts_queue();
    reset_smpl_queue();

    bloodoxygenInit();
	set_spo2_cofe(-7.1984,33.5650,60.5454,-46.3775,130.3776,0.9044);
	BLOOD_OXYGEN_INFO_STRU light_data = {0};
	BLOOD_OXYGEN_RESULT_STRU blood_result = {0};

    max86141_config(&spo_ctx);

    // prepare config packet
    p_cfgpkt = (sens_packet_t*)cfg_pktbuf;
    memset(&helper, 0, sizeof(helper));
    helper.has_ppgcfg = true;
    helper.has_ledcfg = true;
    helper.instance_id = 0; // cfgpkt only
    
    APP_ERROR_CHECK_BOOL(true == sens_format_max86141_packet(&helper, p_cfgpkt, MAX86141_SPO_CFGPKT_SIZE));
    APP_ERROR_CHECK_BOOL(MAX86141_SPO_CFGPKT_SIZE == helper.packet_size);
    read_registers(&spo_ctx, 0x10, 7, helper.p_ppgcfg->value);
    read_registers(&spo_ctx, 0x20, 12, helper.p_ledcfg->value);
    simple_crc((uint8_t *)&p_cfgpkt->type, &helper.p_crc[0], &helper.p_crc[1]);
    // cfgpkt_size = helper.packet_size;   // preserve a copy

    // prepare packet
    p_pkt = prepare_spo_packet(&helper, NULL);
    sample_in_pkt = 0;

    max86141_start(&spo_ctx);
    max86141_timer_start(450 * 1000);

    // SEGGER_RTT_printf(0, "timer started\r\n");

    for (;;)
    {
        if (!cdc_acm_port_open())
        {
            cfgpkt_sent = false;
        }

        if (!cfgpkt_sent && cdc_acm_port_open())
        {
            cdc_acm_send_packet((uint8_t *)p_cfgpkt, MAX86141_SPO_CFGPKT_SIZE);
            cfgpkt_sent = true;

            SEGGER_RTT_printf(0, "spo cfg pkt (%d)\r\n", MAX86141_SPO_CFGPKT_SIZE);
        }

        raw_sample_data_t *p_raw;
        xQueueReceive(q_smpl_pend, &p_raw, portMAX_DELAY);

        // SEGGER_RTT_printf(0, "spo smpl %d\r\n", p_raw->item_count);

        for (int i = 0; i < p_raw->item_count / 2; i++)
        {
            // copy a sample (two items), unrolling
            uint8_t *src = &p_raw->items[i * 2].byte[0];
            uint8_t *dst = &helper.p_spo_sample->value[sample_in_pkt * 6];
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
            dst[3] = src[3];
            dst[4] = src[4];
            dst[5] = src[5];

            // calc a result
            light_data.ir = (((unsigned int)(src[0] & 0x07)) << 16) + (((unsigned int)src[1]) << 8) + (unsigned int)src[2];
            ir_sum[sample_in_pkt / 5] += light_data.ir;
            light_data.rd = (((unsigned int)(src[3] & 0x07)) << 16) + (((unsigned int)src[4]) << 8) + (unsigned int)src[5];
            rd_sum[sample_in_pkt / 5] += light_data.rd;
            getBOResult(light_data, &blood_result);

            // fill rougu data
            max86141_rougu_data_t *p_rougu = (max86141_rougu_data_t *)&helper.p_spo_rougu->value[sample_in_pkt * MAX86141_SPO_ROUGU_DATA_SIZE];
            p_rougu->irdc = light_data.irdc;
            p_rougu->rddc = light_data.rddc;
            p_rougu->irFilt = blood_result.filterIr;
            p_rougu->rdFilt = blood_result.filterRd;
            p_rougu->spo = blood_result.saO2;
            p_rougu->hr = blood_result.heartRate;

            // increment cursor
            saO2_sum += blood_result.saO2;
            hr_sum += blood_result.heartRate;
            sample_in_pkt++;

            if (sample_in_pkt == MAX86141_NUM_OF_SPO_SAMPLES)
            {
                int saO2_avg = saO2_sum / MAX86141_NUM_OF_SPO_SAMPLES;
                int hr_avg = hr_sum / MAX86141_NUM_OF_SPO_SAMPLES;
                
                oled_update_spo(saO2_avg);
                
                if (cdc_acm_port_open() && cfgpkt_sent)
                {
                    simple_crc((uint8_t *)&p_pkt->type, &helper.p_crc[0], &helper.p_crc[1]);
                    cdc_acm_send_packet((uint8_t *)p_pkt, helper.packet_size);
                }
                
                if (ble_nus_tx_running())
                {
                    ble_nus_tx_buf_t *buf = ble_nus_tx_alloc();
                    if (buf)
                    {
                        max_ble_spo_pac_t *pac = (max_ble_spo_pac_t *)buf;
                        pac->len = sizeof(max_ble_spo_pac_t) - sizeof(uint16_t);
                        pac->type = 3;
                        pac->seq = 0;
                        pac->saO2 = saO2_avg; // blood_result.saO2;
                        pac->heartRate = hr_avg; // blood_result.heartRate;
                        
                        for (int i = 0; i < 10; i++)
                        {
                            pac->ir1[i] = ir_sum[i] / 5;
                            pac->rd1[i] = rd_sum[i] / 5;
                        }
                        
                        ble_nus_tx_send(buf);
                    }
                }

                spo_pkt_count++;
                if (spo_pkt_count % 2 == 0)
                {
                    SEGGER_RTT_printf(0, "spo %d pkts (%d)\r\n", spo_pkt_count, helper.packet_size); 
                }
                
                if (spo_pkt_count == max_pkts)
                {
                    // there are dual loops, single break insufficient
                    // break;
                    SEGGER_RTT_printf(0, "spo max pkts %d\r\n", spo_pkt_count);
                    goto teardown;
                }

                saO2_sum = 0;
                hr_sum = 0;
                sample_in_pkt = 0;
                for (int i = 0; i < 10; i++) 
                {
                    ir_sum[i] = 0;
                    rd_sum[i] = 0;
                }
                p_pkt = prepare_spo_packet(&helper, p_pkt);
                // SEGGER_RTT_printf(0, "next spo pkt, %p, size %d\r\n", p_pkt, helper.packet_size);
            }
        }

        xQueueSend(q_smpl_idle, &p_raw, portMAX_DELAY);
        
        handle_abp_coeff_req();
    }

    teardown:

    // spi read may occur after this operation, put buffers into queue, so a delay required.
    max86141_timer_stop();
    max86141_stop(&spo_ctx);

    vTaskDelay(1);

    SEGGER_RTT_printf(0, "spo stopped\r\n");

#if 0
        int spo_count = read_reg(&spo_ctx, REG_FIFO_DATA_COUNT);
        // NRF_LOG_INFO("spo_count: %d", spo_count);

        if (spo_count >= MAX86141_NUM_OF_SAMPLE_ITEMS)
        {
            read_fifo(&spo_ctx);

            // fill samples (type and length correct?)
            memcpy(&m_spo_packet_helper.p_spo_sample->value, buf, MAX86141_NUM_OF_SAMPLE_ITEMS * 3);
            // memcpy(&m_spo_packet_helper.p_ppgcfg->value, spo_0x10_0x16_regs, 7);
            // memcpy(&m_spo_packet_helper.p_ledcfg->value, spo_0x20_0x2B_regs, 12);

            for (int i = 0; i < MAX86141_NUM_OF_SAMPLE_ITEMS / 2; i++)
            {
                light_data.ir = (((unsigned int)(buf[i * 6 + 0] & 0x07)) << 16) + (((unsigned int)buf[i * 6 + 1]) << 8) + (unsigned int)buf[i * 6 + 2];
                light_data.rd = (((unsigned int)(buf[i * 6 + 3] & 0x07)) << 16) + (((unsigned int)buf[i * 6 + 4]) << 8) + (unsigned int)buf[i * 6 + 5];

                getBOResult(light_data, &blood_result);

                max86141_rougu_data_t *p_data_base = (max86141_rougu_data_t *)&m_spo_packet_helper.p_spo_rougu->value;
                max86141_rougu_data_t *p_data = &p_data_base[i];
                // p_data->ir = light_data.ir;
                // p_data->rd = light_data.rd;
                p_data->irdc = light_data.irdc;
                p_data->rddc = light_data.rddc;
                p_data->irFilt = blood_result.filterIr;
                p_data->rdFilt = blood_result.filterRd;
                p_data->spo = blood_result.saO2;
                p_data->hr = blood_result.heartRate;
            }

            // TODO send ble data

            // send usb data
            if (cdc_acm_port_open())
            {
                simple_crc((uint8_t *)&p_current_spo_packet->type, &m_spo_packet_helper.p_crc[0], &m_spo_packet_helper.p_crc[1]);
                cdc_acm_send_packet((uint8_t *)p_current_spo_packet, m_spo_packet_helper.packet_size);
            }

            // p_current_spo_packet = next_spo_packet();

            spo_pkt_count++;
            if ((spo_pkt_count % 500) == 0)
            {
                NRF_LOG_INFO("spo: %d packets sent", spo_pkt_count);
            }
        }

        if (spo_pkt_count >= max_pkts) break;

        // vTaskDelay(period);
#endif
}

static void max86141_run_abp(void)
{
    // 16 packets per second for 128 samples per packet, 16 * 60 = 960
    int max_pkts = 3 * 60 * 2048 / MAX86141_NUM_OF_ABP_SAMPLES;
    
    sens_packet_t *p_pkt = NULL;
    sens_packet_t *p_cfgpkt = NULL;

    max86141_packet_helper_t helper;

    bool cfgpkt_sent = false;
    int sample_in_pkt = 0;
    int abp_pkt_count = 0;

    // reset_pkts_queue();
    reset_smpl_queue();

    CFeatureExtractor fe;
    CFeatureExtractorInit(&fe, 
                          2048, // fs, 
                          500,  // max_peaks, 
                          0.05, // pair_thresh,
                          5,    // warmup_sec, 
                          1);   // output_freq); this is the frequency of feature output.
//  for (I32 i = 0; i < MIN(VSIZE(sig), 2048 * 10); i++) {
//    CFeature feature;
//    CFeatureExtractorApply(&fe, sig[i], sig2[i], &feature);
//    if (feature.valid == FALSE) { continue; }
//    features.push_back(feature);
//    if (verbose) {
//      printf("index = %d, ptt = %.6f, idc = %.6f, imax = %.6f, imin = %.6f\n",
//             feature.index, feature.ptt, feature.idc, feature.imax,
//             feature.imin);
//    }
//  }
//  CFeatureExtractorDestroy(&fe);    

    max86141_config(&abp_ctx);

    SEGGER_RTT_printf(0, "abp cfg ok\r\n");

    // prepare config packet
    p_cfgpkt = (sens_packet_t*)cfg_pktbuf;
    memset(&helper, 0, sizeof(helper));
    helper.has_ppgcfg = true;
    helper.has_ledcfg = true;
    // helper.has_abp_coeff = true;
    helper.instance_id = 1; // cfgpkt only
    
    APP_ERROR_CHECK_BOOL(true == sens_format_max86141_packet(&helper, p_cfgpkt, MAX86141_ABP_CFGPKT_SIZE));
    APP_ERROR_CHECK_BOOL(MAX86141_ABP_CFGPKT_SIZE == helper.packet_size);
    read_registers(&spo_ctx, 0x10, 7, helper.p_ppgcfg->value);
    read_registers(&spo_ctx, 0x20, 12, helper.p_ledcfg->value);
//    helper.p_abp_coeff->sbp[0].u = m_pref.sbp[0].u;
//    helper.p_abp_coeff->sbp[1].u = m_pref.sbp[1].u;
//    helper.p_abp_coeff->sbp[2].u = m_pref.sbp[2].u;
//    helper.p_abp_coeff->sbp[3].u = m_pref.sbp[3].u;
//    helper.p_abp_coeff->dbp[0].u = m_pref.dbp[0].u;
//    helper.p_abp_coeff->dbp[1].u = m_pref.dbp[1].u;
//    helper.p_abp_coeff->dbp[2].u = m_pref.dbp[2].u;
//    helper.p_abp_coeff->dbp[3].u = m_pref.dbp[3].u;    
    simple_crc((uint8_t *)&p_cfgpkt->type, &helper.p_crc[0], &helper.p_crc[1]);

    // prepare packet
    p_pkt = prepare_abp_packet(&helper, NULL);
    sample_in_pkt = 0;
    
    uint32_t ir1sum[2] = {0};
    uint32_t ir2sum[2] = {0};

    max86141_start(&abp_ctx);

    // choose spi read every 32 samples (64 items)
    // which is 2048 / 32 = 64 interrupts per second
    // this translates to 15625uS
    // choose spi read every 51.2 samples (102.4 items)
    // which is 2048 / 51.2 = 40 interrupts per second
    // this translates to 25ms <-- NO! this cannot be achieved for spi read limitation (max 255 bytes)
    // choose spi read every 41 samples, (82 items)
    // which is 2048 / 41 ~= 50 interrupts per second
    // this translates to 20ms
    max86141_timer_start(20 * 1000);

    // SEGGER_RTT_printf(0, "timer started\r\n");

    int loop;
    for (loop = 0;; loop++)
    {   
        if (!cdc_acm_port_open())
        {
            cfgpkt_sent = false;
        }

        if (!cfgpkt_sent && cdc_acm_port_open())
        {
            cdc_acm_send_packet((uint8_t *)p_cfgpkt, MAX86141_ABP_CFGPKT_SIZE);
            cfgpkt_sent = true;

            SEGGER_RTT_printf(0, "abp cfg pkt (%d)\r\n", MAX86141_ABP_CFGPKT_SIZE);
        }

        raw_sample_data_t *p_raw;
        xQueueReceive(q_smpl_pend, &p_raw, portMAX_DELAY);

        for (int i = 0; i < p_raw->item_count / 2; i++)
        {
            // copy a sample (two items), unrolling
            uint8_t *src = &p_raw->items[i * 2].byte[0];
            uint8_t *dst = &helper.p_abp_sample->value[sample_in_pkt * 6];
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
            dst[3] = src[3];
            dst[4] = src[4];
            dst[5] = src[5];

            // calc a result TODO
            uint32_t ir1int = (((unsigned int)(src[0] & 0x07)) << 16) + (((unsigned int)src[1]) << 8) + (unsigned int)src[2];
            uint32_t ir2int = (((unsigned int)(src[3] & 0x07)) << 16) + (((unsigned int)src[4]) << 8) + (unsigned int)src[5];            
            
            ir1sum[sample_in_pkt / 128] += ir1int;
            ir2sum[sample_in_pkt / 128] += ir2int;
            
            double ir1 = (double)ir1int;
            double ir2 = (double)ir2int;
            
            CFeature feature;
            // SEGGER_RTT_printf(0, "b %d\r\n", sample_in_pkt);
            CFeatureExtractorApply(&fe, ir1, ir2, &feature);

            // fill feature data TODO
            if (feature.valid)
            {
                F64 coeffs[2];
                
                // SEGGER_RTT_printf(0, "feat valid\r\n");
                helper.p_abp_feature->index = sample_in_pkt;
                helper.p_abp_feature->ptt.f = (float)feature.ptt;
                // helper.p_abp_feature->idc.f = (float)feature.idc;
                // helper.p_abp_feature->imin.f = (float)feature.imin;
                // helper.p_abp_feature->imax.f = (float)feature.imax;
                
                coeffs[0] = m_pref.sbp[0].f;
                coeffs[1] = m_pref.sbp[1].f;
                // coeffs[2] = m_pref.sbp[2].f;
                // coeffs[3] = m_pref.sbp[3].f;
                helper.p_abp_feature->sbp.f = (float)CPressureCompute(feature, coeffs);
                
                coeffs[0] = m_pref.dbp[0].f;
                coeffs[1] = m_pref.dbp[1].f;
                // coeffs[2] = m_pref.dbp[2].f;
                // coeffs[3] = m_pref.dbp[3].f;
                helper.p_abp_feature->dbp.f = (float)CPressureCompute(feature, coeffs);
                
                oled_update_abp(helper.p_abp_feature->sbp.f, helper.p_abp_feature->dbp.f);                
            }   

            // increment cursor
            sample_in_pkt++;

            if (sample_in_pkt == MAX86141_NUM_OF_ABP_SAMPLES)
            {  
                if (cdc_acm_port_open() && cfgpkt_sent)
                {
                    // seal variadic length packet TODO

                    simple_crc((uint8_t *)&p_pkt->type, &helper.p_crc[0], &helper.p_crc[1]);
                    cdc_acm_send_packet((uint8_t *)p_pkt, helper.packet_size);

                    // SEGGER_RTT_printf(0, "max abp pkt size %d sent\r\n", helper.packet_size);
                }
                
                if (ble_nus_tx_running())
                {
                    ble_nus_tx_buf_t *buf = ble_nus_tx_alloc();
                    if (buf)
                    {
                        max_ble_abp_pac_t *pac = (max_ble_abp_pac_t *)buf;
                        pac->len = sizeof(max_ble_abp_pac_t) - sizeof(uint16_t);
                        pac->type = 4;
                        // pac->seq = 0;
                        if (helper.p_abp_feature->index != -1)
                        {
                            pac->seq = 1;
                            pac->sbp = helper.p_abp_feature->sbp.u;
                            pac->dbp = helper.p_abp_feature->dbp.u;
                        }
                        else
                        {
                            pac->seq = 0;
                            pac->sbp = -1;
                            pac->dbp = -1;
                        }
                        
                        pac->ir1[0] = ir1sum[0] / 128;
                        pac->ir1[1] = ir1sum[1] / 128;
                        pac->ir2[0] = ir2sum[0] / 128;
                        pac->ir2[1] = ir2sum[1] / 128;
                        
                        ble_nus_tx_send(buf);
                    }
                }

                abp_pkt_count++;
                
                if (abp_pkt_count % 16 == 0)
                {
                    SEGGER_RTT_printf(0, "abp %d pkts (%d)\r\n", abp_pkt_count, helper.packet_size);
                }                             
                
                if (abp_pkt_count == max_pkts)
                {
                    SEGGER_RTT_printf(0, "abp %d pkts (max)\r\n", abp_pkt_count);
                    goto teardown;
                }

                sample_in_pkt = 0;
                ir1sum[0] = 0;
                ir1sum[1] = 0;
                ir2sum[0] = 0;
                ir2sum[1] = 0;
                
                p_pkt = prepare_abp_packet(&helper, p_pkt);
                // SEGGER_RTT_printf(0, "next abp pkt, %p, size %d\r\n", p_pkt, helper.packet_size);
            }
        }           

        xQueueSend(q_smpl_idle, &p_raw, portMAX_DELAY);
        
        handle_abp_coeff_req();
        
//        if (loop > 190)
//        {
//            SEGGER_RTT_printf(0, "loop %d\r\n", loop);
//        }
    }

    teardown:
    
    // SEGGER_RTT_printf(0, "loop %d\r\n", loop);

    // spi read may occur after this operation, put buffers into queue, so a delay required.
    max86141_timer_stop();
    max86141_stop(&abp_ctx);
    
    CFeatureExtractorDestroy(&fe);    

    vTaskDelay(1);

    SEGGER_RTT_printf(0, "abp stopped\r\n");

#if 0

    for (;;)
    {
        if (abp_pkt_count >= max_pkts) break;
        vTaskDelay(period);

        // if (max86141_function != 2) break;

        int abp_count = read_reg(&abp_ctx, REG_FIFO_DATA_COUNT);
        // NRF_LOG_INFO("spo_count: %d", spo_count);

        abp_count = read_reg(&abp_ctx, REG_FIFO_DATA_COUNT);
        if (abp_count >= MAX86141_NUM_OF_SAMPLE_ITEMS)
        {
            read_fifo(&abp_ctx);

            // fill samples (type and length correct?)
            // memcpy(&m_abp_packet_helper.p_sample->value, buf, MAX86141_NUM_OF_SAMPLE_ITEMS * 3);
            // memcpy(&m_abp_packet_helper.p_ppgcfg->value, abp_0x10_0x16_regs, 7);
            // memcpy(&m_abp_packet_helper.p_ledcfg->value, abp_0x20_0x2B_regs, 12);


            // TODO send ble data

            // send usb data
            if (cdc_acm_port_open())
            {
                simple_crc((uint8_t *)&p_current_abp_packet->type, &m_abp_packet_helper.p_crc[0], &m_abp_packet_helper.p_crc[1]);
                cdc_acm_send_packet((uint8_t *)p_current_abp_packet, m_abp_packet_helper.packet_size);
            }

            // p_current_abp_packet = next_abp_packet();

            abp_pkt_count++;
            if (abp_pkt_count % 100 == 0)
            {
                NRF_LOG_INFO("abp: %d packets sent", abp_pkt_count);
            }
        }
    }

    max86141_stop(&abp_ctx);

    NRF_LOG_INFO("max86141 abp stopped");

#endif
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        vTaskDelay(1);
    }
}

#if 0
// 2144df1c as constant? yes
const pref_t default_pref = {
    .sbp = {1,1,1,1},
    .dbp = {1,1,1,1},
    .crc32 = 0x4cfc497e
    // .crc32 = 0x7e49fc4c
};

const pref_t default_pref2 = {
    .sbp = {2,1,1,1},
    .dbp = {1,1,1,1},
    .crc32 = 0x85e341c1
};
#endif

static void max86141_task(void * pvParameters)
{
    // suppress compiler complaints
    (void)nrf5_flash_end_addr_get();
    
    nrf_fstorage_api_t * p_fs_api =  &nrf_fstorage_sd;
    ret_code_t rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);
    
    /*
     * erase unit:      4096 bytes
     * program unit:    4 bytes
     * flash end:       0x00100000 <- this is 1MB flash
     */
    // SEGGER_RTT_printf(0, "erase unit: \t%d bytes\r\n",      fstorage.p_flash_info->erase_unit);
    // SEGGER_RTT_printf(0, "program unit: \t%d bytes\r\n",    fstorage.p_flash_info->program_unit);
    
    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    // SEGGER_RTT_printf(0, "flash end addr: \t0x%08x\r\n", nrf5_flash_end_addr_get());
    
    rc = nrf_fstorage_read(&fstorage, PREF_PAGE_START, &m_pref, sizeof(m_pref));
    APP_ERROR_CHECK(rc);
    
    if (m_pref.crc32 != crc32_compute((const uint8_t *)&m_pref, PREF_PAYLOAD_SIZE, NULL))
    {
        SEGGER_RTT_printf(0, "bad coeff crc\r\n");
        for (int i = 0; i < 2; i++) 
        {
            m_pref.sbp[i].f = 1.0;
            m_pref.dbp[i].f = 1.0;
        }
    }
    else
    {
        // TODO printf pref
    }
    
    q_set_abp_coeff = xQueueCreate(2, 4 * sizeof(ieee754_t));
    
    q_smpl_idle = xQueueCreate(NUM_OF_RAWBUF, 4);
    q_smpl_pend = xQueueCreate(NUM_OF_RAWBUF, 4);
    
    max86141_spi_config();
    max86141_timer_init();

    for (;;)
    {
        max86141_run_spo();
        vTaskDelay(64);
        max86141_run_abp();
        vTaskDelay(64);
        SEGGER_RTT_printf(0, " - freertos min heap %d\r\n", xPortGetMinimumEverFreeHeapSize());
    }
}

#if 0

static void max86141_task(void * pvParameters)
{
    ret_code_t err_code;
    // very fast, change this to interrupt driven?
    TickType_t xFrequency = 1;

    uint8_t spo_count = 0;
    uint8_t abp_count = 0;
    int spo_pkt_count = 0;
    int abp_pkt_count = 0;

	bloodoxygenInit();
	set_spo2_cofe(-7.1984,33.5650,60.5454,-46.3775,130.3776,0.9044);
	BLOOD_OXYGEN_INFO_STRU light_data = {0};
	BLOOD_OXYGEN_RESULT_STRU blood_result = {0};

    spo_ctx_init();
    abp_ctx_init();

#ifdef COMBO_MODE
    p_current_combo_packet = max86141_probe(&combo_ctx) ? next_combo_packet() : NULL;
    if (p_current_combo_packet)
    {
        // max86141_init_gpio();

        max86141_config(&combo_ctx);
        read_registers(&combo_ctx, 0x10, 7, combo_0x10_0x16_regs);
        read_registers(&combo_ctx, 0x20, 12, combo_0x20_0x2B_regs);

        max86141_enable_int_pin();
        max86141_start(&combo_ctx);

        NRF_LOG_INFO("combo max86141 started");
    }
#else
    p_current_spo_packet = max86141_probe(&spo_ctx) ? next_spo_packet() : NULL;
    if (p_current_spo_packet)
    {
        max86141_config(&spo_ctx);
        read_registers(&spo_ctx, 0x10, 7, spo_0x10_0x16_regs);
        read_registers(&spo_ctx, 0x20, 12, spo_0x20_0x2B_regs);
        max86141_start(&spo_ctx);

        NRF_LOG_INFO("spo max86141 started");
    }

    p_current_abp_packet = max86141_probe(&abp_ctx) ? next_abp_packet() : NULL;
    // p_current_abp_packet = NULL;
    if (p_current_abp_packet)
    {
        max86141_config(&abp_ctx);
        read_registers(&abp_ctx, 0x10, 7, abp_0x10_0x16_regs);
        read_registers(&abp_ctx, 0x20, 12, abp_0x20_0x2B_regs);
        max86141_start(&abp_ctx);

        NRF_LOG_INFO("abp max86141 started");
    }
#endif

#ifdef COMBO
    if (!p_current_combo_packet)
    {
        vTaskDelay(portMAX_DELAY);
    }

    for (int i = 0; i < SENS_PACKET_POOL_SIZE; i++)
    {
        memcpy(&m_combo_packet_helper.p_ppgcfg->value, combo_0x10_0x16_regs, 7);
        memcpy(&m_combo_packet_helper.p_ledcfg->value, combo_0x20_0x2B_regs, 12);
        p_current_combo_packet = next_combo_packet();
    }

//    CFeatureExtractor fe;
//    CFeatureExtractorInit(&fe, MAXBP_SAMPLING_RATE, MAXBP_MAX_PEAKS, MAXBP_PAIR_THRESH, MAXBP_WARMUP_SEC, MAXBP_OUTPUT_FREQ);
//    for (I32 i = 0; i < MIN(VSIZE(sig), 2048 * 10); i++)
//    {
//        CFeature feature;
//        CFeatureExtractorApply(&fe, sig[i], sig2[i], &feature);
//        if (feature.valid == FALSE) { continue; }
//        features.push_back(feature);
//    }
//    CFeatureExtractorDestroy(&fe);

    for (;;)
    {
        uint8_t *buf;
        uint8_t *p = (uint8_t *)&m_combo_packet_helper.p_sample->value;
        uint8_t *q;

        int len = MAX86141_NUM_OF_SAMPLE_ITEMS / 2 * 3;

        int ir1_sum_avg = 0;
        int ir2_sum_avg = 0;
        int rd1_sum_avg = 0;

        // first half
        xQueueReceive(rdbuf_pending, &buf, portMAX_DELAY);
        memcpy(p, &buf[2], len);
        xQueueSend(rdbuf_idle, &buf, portMAX_DELAY);

        // second half
        xQueueReceive(rdbuf_pending, &buf, portMAX_DELAY);
        memcpy(&p[len], &buf[2], len);
        xQueueSend(rdbuf_idle, &buf, portMAX_DELAY);

        for (int i = 0; i < MAX86141_NUM_OF_SAMPLES; i++)
        {
            ir1_sum_avg += read_sample_item(p, i, 0, MAX86141_ITEMS_PER_SAMPLE);
            ir2_sum_avg += read_sample_item(p, i, 1, MAX86141_ITEMS_PER_SAMPLE);
            rd1_sum_avg += read_sample_item(p, i, 2, MAX86141_ITEMS_PER_SAMPLE);
        }

        ir1_sum_avg /= MAX86141_NUM_OF_SAMPLES;
        ir2_sum_avg /= MAX86141_NUM_OF_SAMPLES;
        rd1_sum_avg /= MAX86141_NUM_OF_SAMPLES;

        light_data.ir = ir1_sum_avg;
        light_data.rd = rd1_sum_avg;

        // after this function, blood_reslt.saO2 and blood_result.heartRate is available
        getBOResult(light_data, &blood_result);

        if (ble_nus_tx_running())
        {
            btx_ir1_buf[btx_item_num] = ir1_sum_avg;
            btx_ir2_buf[btx_item_num] = ir2_sum_avg;
            btx_rd1_buf[btx_item_num] = rd1_sum_avg;
            btx_item_num++;

            if (btx_item_num == 10)
            {
                ble_nus_tx_buf_t *buf = ble_nus_tx_alloc();
                if (buf)
                {
                    max_ble_pac_t *pac = (max_ble_pac_t *)buf;
                    pac->len = sizeof(max_ble_pac_t) - sizeof(uint16_t);
                    pac->type = 3;
                    pac->seq = 0;
                    pac->heartRate = blood_result.heartRate;
                    pac->saO2 = blood_result.saO2;
                    pac->rsvd = 0;

                    memcpy(pac->ir1, btx_ir1_buf, sizeof(uint32_t) * 10);
                    memcpy(pac->ir2, btx_ir2_buf, sizeof(uint32_t) * 10);
                    memcpy(pac->rd1, btx_rd1_buf, sizeof(uint32_t) * 10);

                    ble_nus_tx_send(buf);
                }
                else
                {
                    NRF_LOG_INFO("max failed to alloc tx buf");
                }

                btx_item_num = 0;
            }
        }
        else
        {
            btx_item_num = 0;
        }

        if (cdc_acm_port_open())
        {
            simple_crc((uint8_t *)&p_current_combo_packet->type, &m_combo_packet_helper.p_crc[0], &m_combo_packet_helper.p_crc[1]);
            cdc_acm_send_packet((uint8_t *)p_current_combo_packet, m_combo_packet_helper.packet_size);
        }

        p_current_combo_packet = next_combo_packet();
    }
#endif

    for (int rf = 0;; rf++)
    {
        vTaskDelay(xFrequency);

        if (p_current_spo_packet)
        {
            spo_count = read_reg(&spo_ctx, REG_FIFO_DATA_COUNT);

            // NRF_LOG_INFO("spo_count: %d", spo_count);

            if (spo_count >= MAX86141_NUM_OF_SAMPLE_ITEMS)
            {
                read_fifo(&spo_ctx);

                // fill samples (type and length correct?)
                memcpy(&m_spo_packet_helper.p_sample->value, buf, MAX86141_NUM_OF_SAMPLE_ITEMS * 3);
                memcpy(&m_spo_packet_helper.p_ppgcfg->value, spo_0x10_0x16_regs, 7);
                memcpy(&m_spo_packet_helper.p_ledcfg->value, spo_0x20_0x2B_regs, 12);

                for (int i = 0; i < MAX86141_NUM_OF_SAMPLE_ITEMS / 2; i++)
                {
                    light_data.ir = (((unsigned int)(buf[i * 6 + 0] & 0x07)) << 16) + (((unsigned int)buf[i * 6 + 1]) << 8) + (unsigned int)buf[i * 6 + 2];
                    light_data.rd = (((unsigned int)(buf[i * 6 + 3] & 0x07)) << 16) + (((unsigned int)buf[i * 6 + 4]) << 8) + (unsigned int)buf[i * 6 + 5];

                    getBOResult(light_data, &blood_result);
                    max86141_rougu_data_t *p_data_base = (max86141_rougu_data_t *)&m_spo_packet_helper.p_spo_rougu->value;
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
            if (abp_count >= MAX86141_NUM_OF_SAMPLE_ITEMS)
            {
                read_fifo(&abp_ctx);

                // fill samples (type and length correct?)
                memcpy(&m_abp_packet_helper.p_sample->value, buf, MAX86141_NUM_OF_SAMPLE_ITEMS * 3);
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
        }
    }
}

#endif

void app_max86141_freertos_init(void)
{
    BaseType_t xReturned;

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

#if 0
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
            MAX86141_NUM_OF_SAMPLE_ITEMS);
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
            MAX86141_NUM_OF_SAMPLE_ITEMS);
    }

    // init next packet
    sens_init_max86141_packet(&m_abp_packet_helper, p_packet_pool[next_modulo]);
    sens_packet_t * next = p_packet_pool[next_modulo];
    next_modulo = (next_modulo + 1) % SENS_PACKET_POOL_SIZE;
    return next;
}
#endif

static void max86141_spi_config(void)
{
    uint32_t err_code;

    nrf_drv_spi_config_t const config = {
       .ss_pin             = MAX86141_CS_PIN,
       .sck_pin            = MAX86141_CK_PIN,
       .mosi_pin           = MAX86141_DI_PIN,
       .miso_pin           = MAX86141_DO_PIN,
       .orc                = 0xff,
       .mode               = NRF_DRV_SPI_MODE_0,
       .irq_priority       = APP_IRQ_PRIORITY_LOWEST,
       .frequency          = NRF_DRV_SPI_FREQ_4M,
       .bit_order          = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };

    err_code = nrf_spi_mngr_init(&m_max86141_spi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}

static void reset_smpl_queue(void)
{
    xQueueReset(q_smpl_idle);
    xQueueReset(q_smpl_pend);

    for (int i = 0; i < NUM_OF_RAWBUF; i++)
    {
        raw_sample_data_t *p_raw = &raw_sample_buf[i];
        xQueueSend(q_smpl_idle, &p_raw, 0);
    }
}

void get_abp_coeff()
{
    m_get_abp_coeff = true;
}

/*
 * ptr - pointer to 32bytes (8 * 4 (ieee754_t))
 */
void set_abp_coeff(uint8_t * ptr)
{
    xQueueSend(q_set_abp_coeff, ptr, 0);
}

void set_abp_coeff_from_isr(uint8_t * ptr)
{
    xQueueSendFromISR(q_set_abp_coeff, ptr, 0);
}

static void handle_abp_coeff_req(void)
{
    // 14 + 12 + 35 = 61 as of this writing
#define COEFF_PKT_SIZE (14 + MAX86141_BRIEF_TLV_SIZE + MAX86141_ABP_COEFF_TLV_SIZE)
    
    static max86141_packet_helper_t helper;
    static uint8_t buf[COEFF_PKT_SIZE];
    static pref_t pref __attribute__((aligned(32)));
    
    ret_code_t rc;
    
    // process set request
    if (pdTRUE == xQueueReceive(q_set_abp_coeff, &pref, 0))
    {
        pref.crc32 = crc32_compute((const uint8_t *)&pref, PREF_PAYLOAD_SIZE, NULL);
        rc = nrf_fstorage_erase(&fstorage, PREF_PAGE_START, 1, NULL);
        if (rc == NRF_SUCCESS)
        {
            SEGGER_RTT_printf(0, "flash erased\r\n");
            // must be 32 aligned
            rc = nrf_fstorage_write(&fstorage, PREF_PAGE_START, &pref, sizeof(pref), NULL);
            if (rc == NRF_SUCCESS)
            {
                SEGGER_RTT_printf(0, "flash written\r\n");
                memcpy(&m_pref, &pref, sizeof(pref));
            }
            else
            {
                SEGGER_RTT_printf(0, "flash write err (%d)\r\n", rc);
            }
        }
        else
        {
            SEGGER_RTT_printf(0, "flash erase err (%d)\r\n", rc);
        }
    }
    
    // process get request
    if (m_get_abp_coeff)
    {
        m_get_abp_coeff = false;
        
        SEGGER_RTT_printf(0, "handle get abp coeff\r\n");
        
        if (ble_nus_tx_running())
        {
            ble_nus_tx_buf_t* buf = ble_nus_tx_alloc();
            if (buf)
            {
                max_ble_abp_coeff_t *pac = (max_ble_abp_coeff_t *)buf;
                pac->len = sizeof(max_ble_abp_coeff_t) - sizeof(uint16_t);
                pac->type = 255;
                pac->seq = 0;
                pac->sbp[0].u = m_pref.sbp[0].u;
                pac->sbp[1].u = m_pref.sbp[1].u;
                pac->dbp[0].u = m_pref.dbp[0].u;
                pac->dbp[1].u = m_pref.dbp[1].u;
                
                ble_nus_tx_send(buf);
                
                SEGGER_RTT_printf(0, "ble abp coeff sent\r\n");
            }
        }
        
        if (cdc_acm_port_open())
        {
            memset(&helper, 0, sizeof(helper));
            helper.has_abp_coeff = true;
            helper.instance_id = 1;
            
            sens_packet_t * p_pkt = (sens_packet_t *)buf;
            if (sens_format_max86141_packet(&helper, p_pkt, COEFF_PKT_SIZE))
            {
                helper.p_abp_coeff->sbp[0] = m_pref.sbp[0];
                helper.p_abp_coeff->sbp[1] = m_pref.sbp[1];
                // helper.p_abp_coeff->sbp[2] = m_pref.sbp[2];
                // helper.p_abp_coeff->sbp[3] = m_pref.sbp[3];
                helper.p_abp_coeff->dbp[0] = m_pref.dbp[0];
                helper.p_abp_coeff->dbp[1] = m_pref.dbp[1];
                // helper.p_abp_coeff->dbp[2] = m_pref.dbp[2];
                // helper.p_abp_coeff->dbp[3] = m_pref.dbp[3];                
                
                simple_crc((uint8_t *)&p_pkt->type, &helper.p_crc[0], &helper.p_crc[1]);
                cdc_acm_send_packet((uint8_t *)p_pkt, helper.packet_size);
            }
            else
            {
                SEGGER_RTT_printf(0, "insufficient coeff pkt size\r\n");
            }
        }
        else
        {
            SEGGER_RTT_printf(0, "cdc not open?\r\n");
        }
    }
}
