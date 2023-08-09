#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nrfx_gpiote.h"

#include "nrf_log.h"
#include "nrf_assert.h"

#include "nrf_spi_mngr.h"

#include "sens-proto.h"
#include "ads1292r.h"
#include "usbcdc.h"

#include "ECG_2D.h"
#include "BRHPFilter.h"
#include "qrsdetect.h"


#define SENS_PACKET_POOL_SIZE       4

// TODO move these definition to header file
// TODO don't use cs pin
#define ADS1292R_CS_PIN             (32 + 13) // 1.13 ADS-CS-P1.13
#define ADS1292R_CK_PIN             26        // 0.26 ADS-CLK-P0.26
#define ADS1292R_DI_PIN             1         // 0.01 ADS-DI-P0.01 (ads1292r pin 19, this is nrf MOSI)
#define ADS1292R_DO_PIN             27        // 0.27 ADS-DO-P0.27 (ads1292r pin 21, this is nrf MISO)
#define ADS1292R_DRDY_PIN           (32 + 14) // 1.14

// if you define these
// #define ADS1292R_RESET_PIN
// #define ADS1292R_START_PIN

#define SPI_INSTANCE_ID             1

#define MAX_PENDING_TRANSACTIONS    5

// #define SPI0_ENABLED             1
// #define SPI0_USE_EASY_DMA 0      0           // TODO try dma
NRF_SPI_MNGR_DEF(m_nrf_spi_mngr, MAX_PENDING_TRANSACTIONS, SPI_INSTANCE_ID);

// forward declaration
static void ads1292r_cs_low(void);
static void ads1292r_cs_high(void);

#if 0
static unsigned int parse_24bit_unsigned_be(const uint8_t octet[3])
{
  return (((uint32_t)octet[0]) << 16)
       + (((uint32_t)octet[1]) <<  8)
       + (((uint32_t)octet[2]) <<  0);
}

static          int parse_24bit___signed_be(const uint8_t octet[3])
{
  uint32_t u = (((uint32_t)octet[0]) << 24)
             + (((uint32_t)octet[1]) << 16)
             + (((uint32_t)octet[2]) <<  8);
  return ((int32_t)u) >> 8;
}
#endif

// a rdatac sample, 9 bytes
typedef struct __attribute__((packed)) rdatac_record
{
  uint8_t octet[9];
} rdatac_record_t;

// a string buffer for printing rdatac sample in hex mode
typedef struct rdatac_record_hex
{
    char hex[18];
    char cr;
    char lf;
    char end;
} rdatac_record_hex_t;

typedef struct rdatac_parsed
{
  unsigned int status;
  int chan1;
  int chan2;
} rdatac_parsed_t;

/**
static rdatac_parsed_t parse_rdatac_record(const rdatac_record_t * p_rec)
{
  rdatac_parsed_t parsed;
  parsed.status = parse_24bit_unsigned_be(&p_rec->octet[0]);
  parsed.chan1  = parse_24bit___signed_be(&p_rec->octet[3]);
  parsed.chan2  = parse_24bit___signed_be(&p_rec->octet[6]);
  return parsed;
}
*/

static ads129x_packet_helper_t m_packet_helper = {0};
static sens_packet_t *p_current_packet;

static sens_packet_t *next_packet(void);

static rdatac_record_t rdatac_records[8];
QueueHandle_t p_records_idle = NULL;
QueueHandle_t p_records_pending = NULL;

static void init_rdatac_records(void);

/*****************************************************
 Porting Begin ADS1x9x.c
 *****************************************************/

unsigned char txbuf[10];
unsigned char rxbuf[12];

unsigned char ADC_Read_data[16];
unsigned char ADS129x_SPI_cmd_Flag = 0;
unsigned char ADS129x_SPI_data_Flag = 0;
unsigned char SPI_Send_count = 0;
unsigned char SPI_Tx_Count = 0;
unsigned char SPI_Tx_buf[10];
unsigned char SPI_Rx_Data_Flag = 0;
unsigned char SPI_Rx_buf[12];
unsigned char SPI_Rx_Count = 0;
unsigned char SPI_Rx_exp_Count = 0 ;
unsigned char ECG_Data_rdy;
long ADS1x9x_ECG_Data_buf[6];

extern struct ADS1x9x_state ECG_Recoder_state;
extern unsigned char ECGRecorder_data_Buf[256], Recorder_head,Recorder_tail;
//extern struct ECGPage ECGPageBuf2_data;
//extern unsigned char ECGRecorder_ACQdata_Buf[128];
extern unsigned char Store_data_rdy;
//unsigned char bankFlag = 0;
//unsigned char ECG_recorderReadyFlag = 0;
//unsigned short sampleCount = 0;

#define DELAY_COUNT 2

/* ADS1x9x Register values*/
// the initial value has no significance.
// they are going to be updated during power on init.
unsigned char ADS1x9xRegVal[16] = {
  //Device ID read Ony
  0x00,
  //CONFIG1
  0x02,
  //CONFIG2
  0xE0,
  //LOFF
  0xF0,
  //CH1SET (PGA gain = 6)
  0x00,
  //CH2SET (PGA gain = 6)
  0x00,
  //RLD_SENS (default)
  0x2C,
  //LOFF_SENS (default)
  0x0F,
  //LOFF_STAT
  0x00,
  //RESP1
  0xEA,
  //RESP2
  0x03,
  //GPIO
  0x0C
};

unsigned char gREG_ST[] = {
    0x00,   /* 0x00:    id */
    0x01,   /* 0x01:    CONFIG1 */
    0xE0,   /* 0x02:    CONFIG2 */
    0x10,   /* 0x03:    LOFF */
    0x81,   /* 0x04:    CH1SET */
    0x07,   /* 0x05:    CH2SET */
    0x20,   /* 0x06:    RLD_SENS */
    0x00,   /* 0x07:    LOFF_SENS */
    0x00,   /* 0x08:    LOFF_STAT */
    0x02,   /* 0x09:    RESP1 */
    0x01,   /* 0x0a:    RESP2 */
    0x0C    /* 0x0b:    GPIO */
};

// there is also a ADS1x9x_Default_Register_Settings in original code
unsigned char ADS1x9xR_Default_Register_Settings[15] = {
  0x00,     // Device ID read Ony

  0x02,     // CONFIG1,
            // single shot bit = 0 (continuous mode)
            // dr2/1/0 = 010 (500sps)

  0xE0,     // CONFIG2, (in kalam32, this is 0xA0, with PDB_LOFF_COMP off
            // bit7 = 1 (fixed)
            // PDB_LOFF_COMP = 1 (lead-off comparators enabled)
            // PDB_REFBUF = 1 (reference buffer is enabled)
            // VREF_4V = 0 (2.42-V reference)
            // CLK_EN = 0 (oscillator clock output disabled)
            // bit2 = 0 (fixed)
            // INT_TEST = 0 (test signal off)
            // TEST_FREQ = 0 (dc, not 1Hz square wave)

  0xF0,     // LOFF (in kalam32, this is 0x10)
            // COMP_TH2/1/0 = 111 (comparator positive 70%, negative 30%)
            // bit4 = 1 (fixed)
            // ILEAD_OFF1/0 = 0 (6nA ledd-off current)
            // bit1 = 0 (fixed)
            // FLEAD_OFF = 0 (dc lead-off detect)


  0x00,     // CH1SET (PGA gain = 6)
            // PD1 = 0 (Normal operation, not power-down)
            // GAIN1_2/1/0 = 000 (gain = 6)
            // MUX1_3/2/1/0 = 0000 (normal electrode input, IN1P -> PGA1_INP, IN1N -> PGA1_INN)

  0x00,     // CH2SET (PGA gain = 6)
            // same with CH1SET

  0x2C,     // RLD_SENS (default)
            // CHOP1/0 = 00 (PGA chop frequence = fMOD / 16
            // PDB_RLD = 1 (RLD buffer is enabled)
            // RLD_LOFF_SENSE = 0 (RLD lead-off sense is disabled)
            // RLD2N = 1 (RLD connected to IN2N) !!!! not the default value in datasheet
            // RLD2P = 1 (RLD connected to IN2P) !!!! not the default value in datasheet

  0x0F,     // LOFF_SENS (default)
            // bit7/6 = 00 (fixed)
            // FLIP2/1 = 00 (current direction selection disabled)
            // LOFF2N/2P/1N/1P = 1111 (all enabled)

  0x00,     // LOFF_STAT
            // bit7 = 0 (fixed)
            // CLK_DIV = 0 (fMOD = fCLK / 4, the latter is 512KHz, so the former is 128KHz)
            // bit5 = 0 (fixed)
            // RLD_STATE/IN2N_OFF/IN2P_OFF/IN1N_OFF/IN1P_OFF
            //  (all read-only, 15 means RLD connected, all others disconnected)

  0xEA,     // RESP1
            // RESP_DEMOD_EN1 = 1 (resp demodulation circuitry turned on)
            // RESP_MOD_EN = 1 (resp modulation curcuitry turned on)
            // RESP_PH3/2/1/0 = 1010 (112.5 degree)
            // bit1 = 1 (fixed)
            // RESP_CTRL = 0 (internal resp with internal clock)

  0x03,     // RESP2
            // CALIB_ON = 0 (calibration off)
            // bit6/5/4/3 = 0 (fixed)
            // RESP_FREQ = 0 (32kHz)
            // RLDREF_INT = 1 (RLDREF 1.65V)
            // bit0 = 1 (fixed)

  0x0C      // GPIO
            // bit7/6/5/4 = 0 (fixed)
            // GPIOC2/1 = 11 (both gpio is input)
            // GPIOD2/1 = 00 (read input or write output)
};

unsigned char kalam32_defconfig[15] = {
  0x00,         // 0x00
  0x00,         // 0x01 CONFIG1, 125sps
  0xa0,         // 0x02 CONFIG2, lead-off comp off, 0b10100000
  0x10,         // 0x03 LOFF 0b00010000
  0x40,         // 0x04 CH1SET 0b01000000
  0x60,         // 0x05 CH2SET 0b01100000
  0x2c,         // 0x06 RLD_SENS 0b00101100
  0x00,         // 0x07 LOFF_SENS
  0x00,         // 0x08 LOFF_STAT
  0xf2,         // 0x09 RESP1 0b11110010
  0x03,   // 0x0a RESP2 0b00000011
  0x0c          // 0x0b GPIO
};

// find the source
unsigned char arduino_defconfig[15] = {
    0x00,         // id
    0x01,         // config1, 250 sps
    0xa0,         // config2, lead-off comp off, reference buffer enabled, 2.42V reference
    0x10,         // +95%/-5%, 6nA, DC lead off detect
    0x40,         // gain=4, normal input
    0x60,         // gain=12, normal input
    0x2c,         // rld buffer enabled, rld lead-off sense disabled,
                    // rld not connected to any input
    0x00,         // lead-off detect disabled for all inputs
    0x00,         // lead-off stat,
    0xf2,         // resp1, resp modulation/demodulation on, phase 135 degree,
                // internal respiration with internal clock
    0x03,         // resp2, calib off, resp_freq = 32kHz, RLDREF_INT = 1 (internal reference)
    0x0c,         // gpio, two gpio pins are input
};

/** REG_STRUCT_DEF gREG_ST in rougu code */
unsigned char rougu_defconfig[15] = {
    0x00,   /* 0x00:    id          */
    0x01,   /* 0x01:    CONFIG1     */
    0xE0,   /* 0x02:    CONFIG2     */ /* PDB_LOFF_COMP set */
    0x10,   /* 0x03:    LOFF        */
    0x81,   /* 0x04:    CH1SET      */ /* Power Down, Input shorted (measuring offset) */
    0x07,   /* 0x05:    CH2SET      */ /* gain = 6, RLD_DRM chan2 negative connected to RLDIN */
    0x20,   /* 0x06:    RLD_SENS    */ /* RLD buffer enabled */
    0x00,   /* 0x07:    LOFF_SENS   */
    0x00,   /* 0x08:    LOFF_STAT   */
    0x02,   /* 0x09:    RESP1       */ /* resp demodulation off, modulation off, phase 0 degree */
    0x01,   /* 0x0a:    RESP2       */ /* RLDREF externally fed */
    0x0C    /* 0x0b:    GPIO        */
};

unsigned char adjusted_config[15] = {
    0x00,       // id
    0x01,       // config1, 250 sps
    0xe0,       // config2, lead-off comp on, reference buffer enabled, 2.42V reference
    0x10,       // +95%/-5%, 6nA, DC lead off detect
    0x40,       // gain=4, normal input
    0x60,       // gain=12, normal input
    0x20,       // rld buffer enabled, rld lead-off sense disabled,
                // rld not connected to any input
    0x0f,       // lead-off detect disabled for all inputs
    0x00,       // lead-off stat,
    0x02,       // resp1, resp modulation/demodulation on, phase 135 degree,
                // internal respiration with internal clock
    0x03,       // resp2, calib off, resp_freq = 32kHz, RLDREF_INT = 1 (internal reference)
    0x0c,       // gpio, two gpio pins are input
};

// this is not used in custom board, clksel pulled high, using internal clock
void ADS1x9x_Clock_Select(unsigned char clock_in);

// it seems that ads-rst is not used in custom board.
void ADS1x9x_Reset(void)
{
#if 0
  unsigned short i;
  P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;         // Set High
  /* Provide suficient dealy*/
  for(i= 0;   i < 5000; i++);                         // Wait 1 mSec
  P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_RESET;        // Set to low
  for(i= 0;   i < 5000; i++);                         // Wait 1 mSec
  P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;         // Set High
  for(i= 0;   i < 35000; i++);
#endif

#if defined(ADS1292R_RESET_PIN)
  nrfx_gpiote_out_set(ADS1292R_RESET_PIN);
  vTaskDelay(32);
  nrfx_gpiote_out_clear(ADS1292R_RESET_PIN);
  vTaskDelay(32);
  nrfx_gpiote_out_set(ADS1292R_RESET_PIN);
  vTaskDelay(128);
#endif
}

// do nothing in custom board, start pin pulled down by resistor
// not tested
void ADS1x9x_Disable_Start(void)
{
#if 0
  unsigned short i;
  P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_START;        // Set to LOW
  for(i=0; i<35000; i++);                             // Small Delay to settle
#endif

#if defined(ADS1292R_START_PIN)
  nrfx_gpiote_out_clear(ADS1292R_START_PIN);
  vTaskDelay(32); // 31.25ms
#endif
}

// do nothing in custom board, start pin pulled down by resistor
// not tested
void ADS1x9x_Enable_Start(void)
{
#if 0
  unsigned short i;
  P8OUT |= (enum PORT8_ADC_CONTROL)ADC_START;         // Set to High
  for(i=0; i<50000; i++);                             // Small Delay to settle
#endif

#if defined(ADS1292R_START_PIN)
  nrfx_gpiote_out_set(ADS1292R_START_PIN);
  vTaskDelay(32);
#endif
}

// this may not be used for spi config with cs
void Set_ADS1x9x_Chip_Enable(void)
{
#if 0
  /* ADS1x9x CS is Active low*/
  P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_CS;           // Set to LOW
#endif
  ads1292r_cs_low();
}

// this may not be usd for spi config with cs
void Clear_ADS1x9x_Chip_Enable (void)
{
#if 0
unsigned char CsDelay;

for ( CsDelay = 0;  CsDelay < 100 ;CsDelay++);
/* ADS1x9x CS is Active low*/
P8OUT |= (enum PORT8_ADC_CONTROL)ADC_CS;              // Set to High
#endif
  ads1292r_cs_high();
}

void Init_ADS1x9x_DRDY_Interrupt (void)
{
#if 0
P1DIR &= ~0x02;
P1REN |= BIT1;                                // Enable P1.1 internal resistance
P1OUT |= BIT1;                                // Set P1.1 as pull-Up resistance
P1IES |= BIT1;                                // P1.1 Lo/Hi edge
P1IFG &= ~BIT1;                               // P1.1 IFG cleared
P1IE &= ~BIT1;                                // P1.1 interrupt disabled
#endif
}
/**********************************************************************************************************
* Enable_ADS1x9x_DRDY_Interrupt                                                                                                                                   *
**********************************************************************************************************/
void Enable_ADS1x9x_DRDY_Interrupt (void)
{
#if 0
P1IFG &= ~BIT1;                               // P1.1 IFG cleared
P1IE |= BIT1;                                 // P1.1 interrupt enabled
#endif
}
/**********************************************************************************************************
* Disable_ADS1x9x_DRDY_Interrupt                                                                                                                                          *
**********************************************************************************************************/
void Disable_ADS1x9x_DRDY_Interrupt (void)
{
#if 0
P1IFG &= ~BIT1;                               // P1.1 IFG cleared
P1IE &= ~BIT1;                                // P1.1 interrupt disabled
#endif
}

void Set_GPIO(void)
{
#if 0
P2SEL = 0x00;
P2DIR |= 0x8F;
P2OUT |= (enum PORT2_ADC_CONTROL)POW_CE;
P8DIR |= 0x07;
P8OUT &= 0xF8;
P8OUT |= (enum PORT8_ADC_CONTROL)ADC_CS;      // Set RESET, START to Low and CS to High
P8OUT |= (enum PORT8_ADC_CONTROL)ADC_RESET;   // Set RESET, START to Low and CS to High
P2OUT = 0x03;
//dataCnt = 0;
#endif
}

void Set_UCB0_SPI(void)
{
#if 0
P3SEL |= BIT2+BIT1+BIT0;                      // Set SPI peripheral bits
P3DIR |= BIT0+BIT2;                           // Clock and DOUT as output
P3DIR &= ~BIT1;                               // Din as input
UCB0CTL1 |= UCSWRST;                          // Enable SW reset
UCB0CTL0 |= UCMSB+UCMST+UCSYNC;               // [b0]   1 -  Synchronous mode
                                              // [b2-1] 00-  3-pin SPI
                                              // [b3]   1 -  Master mode
                                              // [b4]   0 - 8-bit data
                                              // [b5]   1 - MSB first
                                              // [b6]   0 - Clock polarity low.
                                              // [b7]   1 - Clock phase - Data is captured on the first UCLK edge and chan  ged on the following edge.

UCB0CTL1 |= UCSSEL__ACLK;                     // ACLK
UCB0BR0 = 24;                                 // 1 MHz
UCB0BR1 = 0;                                  //
UCB0CTL1 &= ~UCSWRST;                         // Clear SW reset, resume operation
#endif
}

void Set_DMA_SPI(void)
{
#if 0
DMACTL0 = DMA0TSEL_12;                                        // USCI_B0 Transmit Ready Trigger
//DMA0SA = (void (*)())&UCB0RXBUF;                            // Source block address
//DMA0DA = (void (*)())ADC_Read_data;                         // Destination single address
DMA0SZ = 16;                                                  // Block size
DMA0CTL = DMADT_4 + DMADSTINCR_3 + DMADSTBYTE + DMASRCBYTE;
                                                              // Rpt, inc src, byte-byte
DMA0CTL |= DMAEN;                                             // Enable DMA for consecutive Xfers
#endif
}

/*
 * Notice that in TI's official code, cs pin is pulled low then high, before
 * doing some real data transfer. Is this a trick to ensure that the chip
 * has fully cleared the previous transfer?
 */
void ADS1x9x_SPI_Command_Data(unsigned char Data)
{
#if 0
unsigned char delayVar;
Set_ADS1x9x_Chip_Enable();
for (delayVar = 0; delayVar < 50; delayVar++);
Clear_ADS1x9x_Chip_Enable();
Set_ADS1x9x_Chip_Enable();

UCB0TXBUF = Data;                                     // Send the data sitting at the pointer DATA to the TX Buffer
while ( (UCB0STAT & UCBUSY) );

delayVar = UCB0RXBUF;

for (delayVar = 0; delayVar < 150; delayVar++);
#endif

  ads1292r_cs_low();
  vTaskDelay(2);
  ads1292r_cs_high();
  vTaskDelay(2);
  ads1292r_cs_low();
  vTaskDelay(2);

  txbuf[0] = Data;
  nrf_spi_mngr_transfer_t xfers[] =
  {
    NRF_SPI_MNGR_TRANSFER(txbuf, 1, NULL, 0)
  };

  APP_ERROR_CHECK(nrf_spi_mngr_perform(&m_nrf_spi_mngr, NULL, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL));

  // vTaskDelay(2);
  // ads1292r_cs_high();
}

void Init_ADS1x9x_Resource(void)
{
#if 0
Set_GPIO();                                           // Initializes ADS1x9x's input control lines
Set_UCB0_SPI();                                       // Initialize SPI regs.
//Set_DMA_SPI();                                      // Initialize DMA regs for SPI.
#endif
}

void Wake_Up_ADS1x9x(void)
{
#if 0
  ADS1x9x_SPI_Command_Data (WAKEUP);                  // Send 0x02 to the ADS1x9x
#endif
  ADS1x9x_SPI_Command_Data(ADS1292R_CMD_WAKEUP);      // Send 0x02 to the ADS1x9x
  NRF_LOG_INFO("[ads1292r] wakeup cmd sent");
}

void Put_ADS1x9x_In_Sleep(void)
{
#if 0
  ADS1x9x_SPI_Command_Data (STANDBY);                 // Send 0x04 to the ADS1x9x
#endif
  ADS1x9x_SPI_Command_Data(ADS1292R_CMD_STANDBY);     // Send 0x04 to the ADS1x9x
  NRF_LOG_INFO("[ads1292r] standby cmd sent");
}

void Soft_Reset_ADS1x9x(void)
{
#if 0
  ADS1x9x_SPI_Command_Data (RESET);                   // Send 0x06 to the ADS1x9x
#endif
  ADS1x9x_SPI_Command_Data(ADS1292R_CMD_RESET);       // Send 0x06 to the ADS1x9x
  NRF_LOG_INFO("[ads1292r] reset cmd sent");
}

void Soft_Start_ReStart_ADS1x9x(void)
{
#if 0
  ADS1x9x_SPI_Command_Data (START);                   // Send 0x08 to the ADS1x9x
  Clear_ADS1x9x_Chip_Enable ();
#endif
  ADS1x9x_SPI_Command_Data(ADS1292R_CMD_START);       // Send 0x08 to the ADS1x9x
  vTaskDelay(2);
  ads1292r_cs_high();
  NRF_LOG_INFO("[ads1292r] start cmd sent and clear cs");
}

void Hard_Start_ReStart_ADS1x9x(void)
{
#if 0
  P8OUT |= (enum PORT8_ADC_CONTROL)ADC_START;         // Set Start pin to High
#endif
  ADS1x9x_Enable_Start();
}

void Soft_Start_ADS1x9x(void)
{
#if 0
  ADS1x9x_SPI_Command_Data (START);                   // Send 0x0A to the ADS1x9x
#endif
  ADS1x9x_SPI_Command_Data(ADS1292R_CMD_START);       // Send 0x08 to the ADS1x9x
  NRF_LOG_INFO("[ads1292r] start cmd sent");
}

void Soft_Stop_ADS1x9x (void)
{
#if 0
  ADS1x9x_SPI_Command_Data (STOP);                   // Send 0x0A to the ADS1x9x
#endif
  ADS1x9x_SPI_Command_Data(ADS1292R_CMD_STOP);       // Send 0x0A to the ADS1x9x
  NRF_LOG_INFO("[ads1292r] stop cmd sent");
}

// This is the same with ADS1x9x_Disable_Start, delay more,
// but we don't need to delay more.
void Hard_Stop_ADS1x9x (void)
{
#if 0
  unsigned short i, j;
  P8OUT &= ~(enum PORT8_ADC_CONTROL)ADC_START;      // Set Start pin to Low
  for (j = 0; j < DELAY_COUNT; j++)
  {
    for ( i=0; i < 35000; i++);
  }
#endif
  ADS1x9x_Disable_Start();
}

void Stop_Read_Data_Continuous (void)
{
#if 0
  ADS1x9x_SPI_Command_Data(SDATAC);                 // Send 0x11 to the ADS1x9x
#endif
  ADS1x9x_SPI_Command_Data(ADS1292R_CMD_SDATAC);    // Send 0x11 to the ADS1x9x
  NRF_LOG_INFO("[ads1292r] sdatac (stop read data continuously) cmd sent");
}

void Start_Read_Data_Continuous (void)
{
#if 0
  ADS1x9x_SPI_Command_Data (RDATAC);                // Send 0x10 to the ADS1x9x
#endif
  ADS1x9x_SPI_Command_Data(ADS1292R_CMD_RDATAC);    // Send 0x10 to the ADS1x9x
  NRF_LOG_INFO("[ads1292r] rdatac (read data continuouesly) cmd sent");
}

void Start_Data_Conv_Command (void)
{
#if 0
  ADS1x9x_SPI_Command_Data (START);                 // Send 0x08 to the ADS1x9x
#endif
  Soft_Start_ADS1x9x();
}

void Stop_Data_Conv_Command (void)
{
  Soft_Stop_ADS1x9x();
}

void Init_ADS1x9x (void)
{
#if 0
  ADS1x9x_Reset();
  ADS1x9x_Disable_Start();
  ADS1x9x_Enable_Start();
#endif
}

void enable_ADS1x9x_Conversion (void)
{
#if 0
  Start_Read_Data_Continuous ();              //RDATAC command
  Hard_Start_ReStart_ADS1x9x();
#endif
}

void ADS1x9x_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{
#if 0
  short i;
  switch (READ_WRITE_ADDRESS)
  {
    case 1:
            DATA = DATA & 0x87;
    break;
    case 2:
            DATA = DATA & 0xFB;
            DATA |= 0x80;

    break;
    case 3:
            DATA = DATA & 0xFD;
            DATA |= 0x10;

    break;
    case 7:
            DATA = DATA & 0x3F;
    break;
    case 8:
            DATA = DATA & 0x5F;
    break;
    case 9:
            DATA |= 0x02;
    break;
    case 10:
            DATA = DATA & 0x87;
            DATA |= 0x01;
    break;
    case 11:
            DATA = DATA & 0x0F;
    break;

    default:

    break;
  }
  SPI_Tx_buf[0] = READ_WRITE_ADDRESS | WREG;
  SPI_Tx_buf[1] = 0;                      // Write Single byte
  SPI_Tx_buf[2] = DATA;                   // Write Single byte
  Set_ADS1x9x_Chip_Enable();

  for ( i =0; i < 50;i++);

  UCB0TXBUF = SPI_Tx_buf[0];              // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );          // USCI_B0 TX buffer ready?
  i = UCB0RXBUF;                          // Read Rx buf

  UCB0TXBUF = SPI_Tx_buf[1];              // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );          // USCI_B0 TX buffer ready?
  i = UCB0RXBUF;
  UCB0TXBUF = SPI_Tx_buf[2];              // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );          // USCI_B0 TX buffer ready?
  i = UCB0RXBUF;
#endif

  // short i;
  switch (READ_WRITE_ADDRESS)
  {
    case 1:
            DATA = DATA & 0x87;
    break;
    case 2:
            DATA = DATA & 0xFB;
            DATA |= 0x80;

    break;
    case 3:
            DATA = DATA & 0xFD;
            DATA |= 0x10;

    break;
    case 7:
            DATA = DATA & 0x3F;
    break;
    case 8:
            DATA = DATA & 0x5F;
    break;
    case 9:
            DATA |= 0x02;
    break;
    case 10:
            DATA = DATA & 0x87;
            DATA |= 0x01;
    break;
    case 11:
            DATA = DATA & 0x0F;
    break;

    default:

    break;
  }
  txbuf[0] = READ_WRITE_ADDRESS | ADS1292R_CMD_WREG;
  txbuf[1] = 0;                           // Write Single byte
  txbuf[2] = DATA;                        // Write Single byte

  Set_ADS1x9x_Chip_Enable();

  vTaskDelay(1);

  nrf_spi_mngr_transfer_t xfers[] =
  {
    NRF_SPI_MNGR_TRANSFER(txbuf, 3, NULL, 0)
  };

  APP_ERROR_CHECK(nrf_spi_mngr_perform(&m_nrf_spi_mngr, NULL, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL));
  NRF_LOG_INFO("[ads1292r] write %d to reg %d", txbuf[2], READ_WRITE_ADDRESS);

  Clear_ADS1x9x_Chip_Enable();            // Disable chip select
}

unsigned char ADS1x9x_Reg_Read(unsigned char Reg_address)
{
#if 0
  unsigned char retVal;
  SPI_Tx_buf[0] = Reg_address | RREG;
  SPI_Tx_buf[1] = 0;                          // Read number of bytes - 1

  Set_ADS1x9x_Chip_Enable();                  // Set chip select to low

  UCB0TXBUF = SPI_Tx_buf[0];                  // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );              // USCI_B0 TX buffer ready?
  UCB0TXBUF = SPI_Tx_buf[1];                  // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );              // USCI_B0 TX buffer ready?
  retVal = UCB0RXBUF;                         // Read RX buff
  UCB0TXBUF = 0x00;                           // Send the first data to the TX Buffer
  while ( (UCB0STAT & UCBUSY) );              // USCI_B0 TX buffer ready?
  retVal = UCB0RXBUF;                         // Read RX buff

  Clear_ADS1x9x_Chip_Enable();                // Disable chip select
  return  retVal;
#endif

  // unsigned char retVal;

  Set_ADS1x9x_Chip_Enable();                  // Set chip select to low

  txbuf[0] = Reg_address | ADS1292R_CMD_RREG;
  txbuf[1] = 0;                               // read number is 0 + 1;

//  rxbuf[0] = 0xa5;
//  rxbuf[1] = 0xa6;
//  rxbuf[2] = 0xa7;

  nrf_spi_mngr_transfer_t xfers[] =
  {
    NRF_SPI_MNGR_TRANSFER(txbuf, 2, rxbuf, 3)
  };

//  nrf_spi_mngr_transaction_t transaction_1 =
//  {
//    .begin_callback = NULL,
//    .end_callback = NULL,
//    .p_user_data = (void *)0,
//    .p_transfers = xfers,
//    .number_of_transfers = sizeof(xfers) / sizeof(xfers[0]),
//    .p_required_spi_cfg = NULL
//  };

  APP_ERROR_CHECK(nrf_spi_mngr_perform(&m_nrf_spi_mngr, NULL, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL));
  NRF_LOG_INFO("[ads1292r] read reg: %d, ret %d", Reg_address, rxbuf[2]);
  Clear_ADS1x9x_Chip_Enable();                // Disable chip select
  return rxbuf[2];
}

void ADS1x9x_Default_Reg_Init(void)
{
#if 0
  unsigned char Reg_Init_i;
  Set_ADS1x9x_Chip_Enable();
  for ( Reg_Init_i =0; Reg_Init_i <100;Reg_Init_i++);
  Clear_ADS1x9x_Chip_Enable();

  if ((ADS1x9xRegVal[0] & 0X20) == 0x20)
  {
    for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
    {
      ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9xR_Default_Register_Settings[Reg_Init_i]);
    }
  }
  else
  {
    for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
    {
      ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9x_Default_Register_Settings[Reg_Init_i]);
    }
  }
#endif

  unsigned char Reg_Init_i;

  Set_ADS1x9x_Chip_Enable();
  vTaskDelay(1);
  Clear_ADS1x9x_Chip_Enable();

  if ((ADS1x9xRegVal[0] & 0X20) == 0x20)
  {
    for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
    {
      // ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9xR_Default_Register_Settings[Reg_Init_i]);
      ADS1x9x_Reg_Write(Reg_Init_i, adjusted_config[Reg_Init_i]);
    }
  }
}

void ADS1x9x_Read_All_Regs(unsigned char ADS1x9xeg_buf[])
{
    unsigned char Regs_i;
    Set_ADS1x9x_Chip_Enable();
    // for ( Regs_i =0; Regs_i <200;Regs_i++);
    vTaskDelay(1);
    Clear_ADS1x9x_Chip_Enable();

    for ( Regs_i = 0; Regs_i < 12; Regs_i++)
    {
        ADS1x9xeg_buf[Regs_i] = ADS1x9x_Reg_Read(Regs_i);
    }
}

/*
 * For custom board the following functions have no effect.
 * The reset pin is mandatory. Start pin may or may not be used in future.
 */
void ADS1x9x_PowerOn_Init(void)
{
#if 0
  volatile unsigned short Init_i, j;
  Init_ADS1x9x_Resource();
  ADS1x9x_Reset();
  for (j = 0; j < DELAY_COUNT; j++)
  {
    for ( Init_i =0; Init_i < 20000; Init_i++);
    for ( Init_i =0; Init_i < 20000; Init_i++);
    for ( Init_i =0; Init_i < 20000; Init_i++);
  }
  Init_ADS1x9x_DRDY_Interrupt();
  ADS1x9x_Clock_Select(1);             // Set internal clock
  for ( Init_i =0; Init_i < 20000; Init_i++);
  for ( Init_i =0; Init_i < 20000; Init_i++);
  for ( Init_i =0; Init_i < 20000; Init_i++);
  ADS1x9x_Disable_Start();
  ADS1x9x_Enable_Start();

  Hard_Stop_ADS1x9x();
  Start_Data_Conv_Command();
  Soft_Stop_ADS1x9x();

  for (j = 0; j < DELAY_COUNT; j++)
  {
    for ( Init_i =0; Init_i < 20000; Init_i++);
  }
  Stop_Read_Data_Continuous();                                 // SDATAC command
  for (j = 0; j < DELAY_COUNT; j++)
  {
    for ( Init_i =0; Init_i < 35000; Init_i++);
  }
  for (j = 0; j < DELAY_COUNT; j++)
  {
    for ( Init_i =0; Init_i < 35000; Init_i++);
  }
  ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
  ADS1x9x_Default_Reg_Init();
  ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
#endif

  ADS1x9x_Reset();
  // clock select omitted
  ADS1x9x_Disable_Start();
  ADS1x9x_Enable_Start();
  Hard_Stop_ADS1x9x();
  Start_Data_Conv_Command();
  Stop_Data_Conv_Command();
  vTaskDelay(16);
  Stop_Read_Data_Continuous();
  vTaskDelay(32);

  ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
  ADS1x9x_Default_Reg_Init();
  ADS1x9x_Read_All_Regs(ADS1x9xRegVal);

  // memcpy(&packet_buffer.regval[0], ADS1x9xRegVal, 12);
}

void ADS1292x_Parse_data_packet(void)
{
#if 0
  unsigned char ECG_Chan_num;
  switch (ECG_Recoder_state.state)
  {
  case DATA_STREAMING_STATE:
    {
      for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
      {
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[3*ECG_Chan_num];
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+1];
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+2];
      }
    }
    break;
  case ACQUIRE_DATA_STATE:
  case ECG_RECORDING_STATE:
    {
      unsigned char *ptr;

      ptr = &ECGRecorder_data_Buf[Recorder_head << 3];  // Point to Circular buffer at head*8;
      *ptr++ = SPI_Rx_buf[0];                           // Store status
      *ptr++ = SPI_Rx_buf[1];                           // Store status
      //SPI_Rx_buf[2] is always 0x00 so it is discarded

      *ptr++ = SPI_Rx_buf[3];                           // CH0[23-16] = MSB ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[4];                           // CH0[15-8] = MID ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[5];                           // CH0[7-0] = LSB ( 24 Bit device)

      *ptr++ = SPI_Rx_buf[6];                           // CH1[23-16] = MSB ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[7];                           // CH1[15-8] = MID ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[8];                           // CH1[7-0] = LSB ( 24 Bit device)

      Recorder_head ++;                                 // Increment Circuler buffer pointer
      if (Recorder_head == 32)                          // Check for circuler buffer depth.
        Recorder_head = 0;                              // Rest once it reach to MAX
    }
    break;
  default:
    break;
  }
#endif
}

#if 0
__interrupt void USCI_B0_ISR(void)
{
  switch(__even_in_range(UCB0IV,4))
  {
    case 0:break;                               // Vector 0 - no interrupt
    case 2:                                     // Vector 2 - RXIFG
      //while (!(UCB0IFG&UCTXIFG));             // USCI_B0 TX buffer ready?
      SPI_Rx_buf[SPI_Rx_Count] = UCB0RXBUF;
      SPI_Rx_Count++;
      if ( SPI_Rx_Count == SPI_Rx_exp_Count)
      {
        UCB0IE &= ~UCRXIE;                      // Disable USCI_B0 RX interrupt
        ADS1x9x_Parse_data_packet();
      }
      else
      {
        UCB0TXBUF = 0;                          // To get Next byte.
      }
      break;
    case 4:
      break;                                    // Vector 4 - TXIFG
    default:
      break;
  }
}

__interrupt void Port_1(void)
{
  if ( P1IFG &= BIT1)
  {
    P1IFG &= ~BIT1;                 // Clear P1.1 IFG i.e Data RDY interrupt status
    SPI_Rx_Count = UCB0RXBUF;       // Dummy Read
    SPI_Rx_Count=0;

    UCB0TXBUF = 0;
    UCB0IE |= UCRXIE;               // Enable USCI_B0 RX interrupt
  }
}
#endif

void Set_Device_out_bytes(void)
{
#if 0
  switch( ADS1x9xRegVal[0] & 0x03)
  {
    case ADS1191_16BIT:
      SPI_Rx_exp_Count=4;             // 2 byte status + 2 bytes CH0 data
      break;
    case ADS1192_16BIT:
      SPI_Rx_exp_Count=6;             // 2 byte status + 2 bytes ch1 data + 2 bytes CH0 data
      break;
    case ADS1291_24BIT:
      SPI_Rx_exp_Count=6;             // 3 byte status + 3 bytes CH0 data
      break;
    case ADS1292_24BIT:
      SPI_Rx_exp_Count=9;             // 3 byte status + 3 bytes ch1 data + 3 bytes CH0 data
      break;
  }
#endif
}

/*****************************************************
 Porting End
 *****************************************************/


/*
 * FreeRTOS Task Handler
 */
static TaskHandle_t m_ads1292r_thread = NULL;
// static SemaphoreHandle_t sem = NULL;

#if 0
typedef struct
{
    uint8_t sck_pin;      ///< SCK pin number.
    uint8_t mosi_pin;     ///< MOSI pin number (optional).
                          /**< Set to @ref NRF_DRV_SPI_PIN_NOT_USED
                           *   if this signal is not needed. */
    uint8_t miso_pin;     ///< MISO pin number (optional).
                          /**< Set to @ref NRF_DRV_SPI_PIN_NOT_USED
                           *   if this signal is not needed. */
    uint8_t ss_pin;       ///< Slave Select pin number (optional).
                          /**< Set to @ref NRF_DRV_SPI_PIN_NOT_USED
                           *   if this signal is not needed. The driver
                           *   supports only active low for this signal.
                           *   If the signal should be active high,
                           *   it must be controlled externally. */
    uint8_t irq_priority; ///< Interrupt priority.
    uint8_t orc;          ///< Over-run character.
                          /**< This character is used when all bytes from the TX buffer are sent,
                               but the transfer continues due to RX. */
    nrf_drv_spi_frequency_t frequency; ///< SPI frequency.
    nrf_drv_spi_mode_t      mode;      ///< SPI mode.
    nrf_drv_spi_bit_order_t bit_order; ///< SPI bit order.
} nrf_drv_spi_config_t;
#endif

// TWI (with transaction manager) initialization.
static void spi_config(void)
{
    uint32_t err_code;

    nrf_drv_spi_config_t const config = {
       .sck_pin            = ADS1292R_CK_PIN,
       .mosi_pin           = ADS1292R_DI_PIN,
       .miso_pin           = ADS1292R_DO_PIN,
       .ss_pin             = NRF_DRV_SPI_PIN_NOT_USED,
       .irq_priority       = APP_IRQ_PRIORITY_LOWEST,
       .orc                = 0x00,
       .frequency          = NRF_DRV_SPI_FREQ_500K,

       // https://devzone.nordicsemi.com/f/nordic-q-a/39942/nrf52840---ads1292/155794
       // also datasheet
       .mode               = NRF_DRV_SPI_MODE_1,
       .bit_order          = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };

    err_code = nrf_spi_mngr_init(&m_nrf_spi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}

// data ready interrupt handler
// static int test_count = 0;

static nrf_spi_mngr_transfer_t rdatac_xfers[] =
{
  // NRF_SPI_MNGR_TRANSFER(NULL, 0, rxbuf, 9)
  {
    .p_tx_data = (uint8_t const *)NULL,
    .tx_length = (uint8_t)        0,
    .p_rx_data = (uint8_t *)      rxbuf,
    .rx_length = (uint8_t)        9,
  }
};

static void spi_rdatac_end_callback(ret_code_t result, void * p_user_data)
{
  uint32_t err_count = 0;
  if (rdatac_xfers[0].p_rx_data == rxbuf)
  {
        err_count++;
        if (err_count % 1000 == 0)
        {
            NRF_LOG_INFO("[ads1292r] rxbuf used as rdatac buffer, bad thing happends!");
        }
  }
  else
  {
    void * p = rdatac_xfers[0].p_rx_data;
    if (pdTRUE == xQueueSendFromISR(p_records_pending, &p, NULL))
    {
      // NRF_LOG_INFO("put %p into pending", p);
    }
    else
    {
      // NRF_LOG_INFO("failed to put %p into pending", p);
    }
  }

  // result always 0?
//  if (test_count % 500 == 0)
//  {
//    NRF_LOG_INFO("rdatac test count %d", test_count);
//  }
//
//  test_count++;

//  if (test_count == 2000)
//  {
//    for(;;){}
//  }
}

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

static nrf_spi_mngr_transaction_t rdatac_trans = {
  .begin_callback = NULL,
  .end_callback = spi_rdatac_end_callback,
  .p_user_data = NULL,
  .p_transfers = rdatac_xfers,
  .number_of_transfers = sizeof(rdatac_xfers) / sizeof(rdatac_xfers[0]),
  .p_required_spi_cfg = NULL
};

static void ads1292r_drdy_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

  void * p = NULL;
  // int avail = uxQueueSpacesAvailable(p_records_idle);
  // NRF_LOG_INFO("avail %d", avail);

  if (pdTRUE == xQueueReceiveFromISR(p_records_idle, &p, NULL))
  {
    // NRF_LOG_INFO("get %p in isr", p);
    rdatac_xfers[0].p_rx_data = p;
  }
  else
  {
    // NRF_LOG_INFO("2");
    rdatac_xfers[0].p_rx_data = rxbuf;
  }

  nrf_spi_mngr_schedule(&m_nrf_spi_mngr, &rdatac_trans);

//  if (test_count % 500 == 0)
//  {
//    NRF_LOG_INFO("test count %d", test_count);
//  }
//
//  test_count++;
}

static void ads1292r_cs_low(void)
{
  nrfx_gpiote_out_clear(ADS1292R_CS_PIN);
}

static void ads1292r_cs_high(void)
{
  nrfx_gpiote_out_set(ADS1292R_CS_PIN);
}

//static void ads1292r_cs_pulse(void)
//{
//  nrfx_gpiote_out_set(ADS1292R_CS_PIN);
//  vTaskDelay(1);
//  nrfx_gpiote_out_clear(ADS1292R_CS_PIN);
//  vTaskDelay(1);
//  nrfx_gpiote_out_set(ADS1292R_CS_PIN);
//  vTaskDelay(1);
//}

static void ads1292r_init_gpio(void)
{
#if defined (ADS1292R_RESET_PIN)
  // initial high
  nrfx_gpiote_out_config_t reset_cfg = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(1);
  APP_ERROR_CHECK(nrfx_gpiote_out_init(ADS1292R_RESET_PIN, &reset_cfg));
#endif

#if defined (ADS1292R_START_PIN)
  // initial low
  nrfx_gpiote_out_config_t start_cfg = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(0);
  APP_ERROR_CHECK(nrfx_gpiote_out_init(ADS1292R_START_PIN, &start_cfg));
#endif

  // initial high
  nrfx_gpiote_out_config_t cs_cfg = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(1);
  APP_ERROR_CHECK(nrfx_gpiote_out_init(ADS1292R_CS_PIN, &cs_cfg));

  // set drdy to input, high accu (IN_EVENT, instead of PORT_EVENT)
  nrfx_gpiote_in_config_t drdy_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  APP_ERROR_CHECK(nrfx_gpiote_in_init(ADS1292R_DRDY_PIN, &drdy_cfg, ads1292r_drdy_handler));
}

#if 0

// see table 10, ideal output versus input signal on page 29 in datasheet
// test 7fffffh, 000001h, 000000h, ffffffh, 800000h
const static rdatac_record_t trec_test_data[5]  = {
    {0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0x7f, 0xff, 0xff}, // max
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01}, // positive 1
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // zero
    {0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, // negative 1
    {0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00}  // min
};

const static char * trec_test_name[5] = {
  "max positive",
  "  positive 1",
  "        zero",
  "  negative 1",
  "min negative"
};

#endif

static void ads1292r_task(void * pvParameters)
{
#if 0
    NRF_LOG_INFO("sizeof In_Signal1 is %d", sizeof(In_Signal1)); // 4
    NRF_LOG_INFO("sizeof In_Signal2 is %d", sizeof(In_Signal2));
    NRF_LOG_INFO("sizeof Out_Signal1 is %d", sizeof(Out_Signal1));
    NRF_LOG_INFO("sizeof Out_Signal2 is %d", sizeof(Out_Signal2));
    NRF_LOG_INFO("sizeof Out_Signal1 is %d", sizeof(Out_Signal1));
    NRF_LOG_INFO("sizeof BRHPFilter_U.Input is %d", sizeof(BRHPFilter_U.Input)); // 8
    NRF_LOG_INFO("sizeof BRHPFilter_Y.Output is %d", sizeof(BRHPFilter_Y.Output));

    NRF_LOG_INFO("--- formatted print test begin ---");
    for (int j = 0; j < 5; j++)
    {
        rdatac_parsed_t parsed = parse_rdatac_record(&trec_test_data[j]);
        NRF_LOG_RAW_INFO("  %s : %d, %d\n",
                     trec_test_name[j],
                     parsed.chan1,
                     parsed.chan2);
    }
    NRF_LOG_INFO("---- formatted print test end ----");
#endif

    NRF_LOG_INFO("ads129x_brief_tlv_t size: %d", sizeof(ads129x_brief_tlv_t));

    p_current_packet = next_packet();
    init_rdatac_records();

    ECG_2D_initialize();
    BRHPFilter_initialize();
    resetQRSDetect(200);

    ads1292r_init_gpio();
    spi_config();

    ADS1x9x_PowerOn_Init();

    // enable drdy interrupt
    nrfx_gpiote_in_event_enable(ADS1292R_DRDY_PIN, true);

    // ads1292_Start_Read_Data_Continuous(spi);
    // Start_Read_Data_Continuous();
    ads1292r_cs_high();
    vTaskDelay(1);
    ads1292r_cs_low();
    vTaskDelay(1);

    txbuf[0] = ADS1292R_CMD_START;
    nrf_spi_mngr_transfer_t xfer1[] =
    {
        NRF_SPI_MNGR_TRANSFER(txbuf, 1, NULL, 0)
    };

    APP_ERROR_CHECK(nrf_spi_mngr_perform(&m_nrf_spi_mngr, NULL, xfer1, sizeof(xfer1) / sizeof(xfer1[0]), NULL));

    vTaskDelay(1);
    ads1292r_cs_high();
    vTaskDelay(1);
    ads1292r_cs_low();
    vTaskDelay(1);

    txbuf[0] = ADS1292R_CMD_RDATAC;
    nrf_spi_mngr_transfer_t xfer2[] =
    {
        NRF_SPI_MNGR_TRANSFER(txbuf, 1, NULL, 0)
    };

    APP_ERROR_CHECK(nrf_spi_mngr_perform(&m_nrf_spi_mngr, NULL, xfer2, sizeof(xfer2) / sizeof(xfer2[0]), NULL));

  // ads1292_Enable_Start();
  // Soft_Start_ReStart_ADS1x9x();
  // Soft_Start_ADS1x9x();

  // start continuous mode reading
  // Start_Read_Data_Continuous();
  // Start_Data_Conv_Command();

  // Stop_Read_Data_Continuous();
  // vTaskDelay(10);
  // NRF_LOG_INFO("ads1292 initial read reg %d: %d", 0, ADS1x9x_Reg_Read(0));

    // calculate hr
    int rog_qrs_count = 0;
    int rog_winPeak;
    int rog_filterData;
    int rog_sbPeak;
    int rog_hr;

    int rog_last_hr = 255;

    for (int i = 0;;i++)
    {
        // Soft_Reset_ADS1x9x();
        // vTaskDelay(20);
        // NRF_LOG_INFO("ads1292 initial read reg %d: %d", 0, ADS1x9x_Reg_Read(0));
        // vTaskDelay(1024);

        /**
            uint16_t                payload_len;
            uint16_t                packet_size;
            uint16_t                sample_count;       // increment

            ads129x_brief_tlv_t     *p_global;          // RR, HR (if rog ecg/resp configured)
            tlv_t                   *p_reg;             // before send
            tlv_t                   *p_sample;          // fill one by one (in the unit of rdatac)
            tlv_t                   *p_rog_ecg;         // if configured
            tlv_t                   *p_rog_resp;        // if configured and sample_count mod 10
            tlv_t                   *p_rog_internal;    // not used yet
            uint8_t                 *p_crc;             // before send
        */


        // wait for rdatac
        rdatac_record_t* p_src = NULL;
        xQueueReceive(p_records_pending, &p_src, portMAX_DELAY);

        // copy to p_sample
        int sample_start = sizeof(rdatac_record_t) * m_packet_helper.num_of_samples;
        rdatac_record_t* p_dst = (rdatac_record_t *)&m_packet_helper.p_sample->value[sample_start];
        *p_dst = *p_src;

        // rdatac_parsed_t parsed = parse_rdatac_record(p_rec);
        // NRF_LOG_RAW_INFO("%d, %d\n", parsed.chan1, parsed.chan2);

        if (ADS129X_USE_ROG_LIB)
        {
            int result, result2;
            uint8_t *p_octet;

            // fill rougu resp
            if((m_packet_helper.num_of_samples % 10) == 0)
            {
                int resp_start = 14 * (m_packet_helper.num_of_samples / 10);

                result = p_src->octet[3] << 24 | p_src->octet[4] << 16 | p_src->octet[5] << 8;
                In_Signal1 = (float)(result>>8);

                ECG_2D_step_1();

                // convert to uint32_t and save
                uint32_t out1 = (uint32_t)Out_Signal1;
                memcpy(&m_packet_helper.p_rog_resp->value[resp_start], &out1, sizeof(out1));

                BRHPFilter_U.Input = Out_Signal1;
                BRHPFilter_step();

                uint32_t resp = (uint32_t)BRHPFilter_Y.Output;
                memcpy(&m_packet_helper.p_rog_resp->value[resp_start + 4], &resp, sizeof(resp));
            }

            // fill rougu ecg
            int ecg_start = m_packet_helper.num_of_samples * 2; // sizeof(uint16_t)

            // result2 = (pSend_ADS[i].CHn[1].DH<<24) | (pSend_ADS[i].CHn[1].DM<<16) | (pSend_ADS[i].CHn[1].DL<<8);
            result2 = p_src->octet[6] << 24 | p_src->octet[7] << 16 | p_src->octet[8] << 8;
            In_Signal2 = (float)(result2>>14);

            ECG_2D_step_2();
            // pGET_BLE_BAG->ECG.CH2[i] = (uint16_t)Out_Signal2;
            uint16_t ecg = (uint16_t)Out_Signal2;

            p_octet = (uint8_t *)&ecg;
            for (int j = 0; j < 2; j++)
            {
              m_packet_helper.p_rog_ecg->value[ecg_start + j] = p_octet[j];
            }

            // fill hr
            int delay = QRSDetect(Out_Signal2, rog_qrs_count, &rog_winPeak, &rog_filterData, &rog_sbPeak, &rog_hr);
            if (delay > 0) {
                // NRF_LOG_INFO("rog qrs detected, hr %d, delay %d, qrs count %d", rog_hr, delay, rog_qrs_count);
                rog_qrs_count++;
            }

            m_packet_helper.p_brief->heart_rate = rog_hr;
            if (rog_hr != rog_last_hr)
            {
                rog_last_hr = rog_hr;
            }

            // NRF_LOG_INFO("heart rate (zr algo): %d", rog_hr);
        }

        m_packet_helper.num_of_samples += 1;

        // sample tlv is ready
        if (m_packet_helper.num_of_samples == ADS129X_NUM_OF_SAMPLES)
        {
            if (cdc_acm_port_open())
            {
                // fill reg tlv
                memcpy(m_packet_helper.p_reg->value, ADS1x9xRegVal, 12);
                // fill crc
                simple_crc((uint8_t *)&p_current_packet->type, &m_packet_helper.p_crc[0], &m_packet_helper.p_crc[1]);

                cdc_acm_send_packet((uint8_t *)p_current_packet, m_packet_helper.packet_size);
                p_current_packet = next_packet();
            }

            // reset sample count, packet not changed
            m_packet_helper.num_of_samples = 0;
            rog_qrs_count = 0;

            // NRF_LOG_INFO("heart rate (zr algo): %d", rog_last_hr);
        }

        xQueueSend(p_records_idle, &p_src, portMAX_DELAY);

        if (i % 5000 == 0)
        {
            NRF_LOG_INFO("%d samples", i);
        }
    }
}

//        if (cdc_acm_port_open())
//        {
//            rdatac_record_hex_t* p_pkt = (rdatac_record_hex_t*)pvPortMalloc(sizeof(rdatac_record_hex_t));
//            if (p_pkt)
//            {
//                static const char hexes[] = "0123456789ABCDEF";
//
//                for (int i = 0; i < 9; i++)
//                {
//                    p_pkt->hex[i * 2 + 0] = hexes[(p_rec->octet[i] >>   4)];
//                    p_pkt->hex[i * 2 + 1] = hexes[(p_rec->octet[i] & 0x0F)];
//                }
//
//                p_pkt->cr = '\r';
//                p_pkt->lf = '\n';
//                p_pkt->end = '\0';
//
//                cdc_acm_send_packet((uint8_t *)p_pkt, sizeof(rdatac_record_hex_t));
//                count++;
//                if (count % 1000 == 0)
//                {
//                    NRF_LOG_INFO("%d packets sent", count);
//                }
//            }

//            packet_buffer.record[sample_count++] = *p_rec;
//            if (sample_count == 25)
//            {
//                simple_crc((uint8_t *)&packet_buffer.packet_type, &packet_buffer.ck_a, &packet_buffer.ck_b);
//                uint8_t * p_pkt = (uint8_t *)pvPortMalloc(sizeof(packet_buffer));
//                if (p_pkt)
//                {
//                    memcpy(p_pkt, &packet_buffer, sizeof(packet_buffer));
//                    cdc_acm_send_packet(p_pkt, sizeof(packet_buffer));
//                }
//                sample_count = 0;
//            }


//        }
//        else
//        {
//            sample_count = 0;
//        }

//        if((i%10) == 0)
//        {
//            // result = (pSend_ADS[i].CHn[0].DH<<24) | (pSend_ADS[i].CHn[0].DM<<16) | (pSend_ADS[i].CHn[0].DL<<8);
//            result = p_rec->octet[3] << 24 | p_rec->octet[4] << 16 | p_rec->octet[5] << 8;
//            In_Signal1 = (float)(result>>8);
//            ECG_2D_step_1();
//            BRHPFilter_U.Input = Out_Signal1;
//            BRHPFilter_step();
//            // pGET_BLE_BAG->BR.CH1[j++] = (uint16_t)BRHPFilter_Y.Output;
//            br = (uint16_t)BRHPFilter_Y.Output;
//        }
//        // result2 = (pSend_ADS[i].CHn[1].DH<<24) | (pSend_ADS[i].CHn[1].DM<<16) | (pSend_ADS[i].CHn[1].DL<<8);
//        result2 = p_rec->octet[6] << 24 | p_rec->octet[7] << 16 | p_rec->octet[8] << 8;
//        In_Signal2 = (float)(result2>>14);

//        ECG_2D_step_2();
//        // pGET_BLE_BAG->ECG.CH2[i] = (uint16_t)Out_Signal2;
//        ecg = (uint16_t)Out_Signal2;


void app_ads1292r_freertos_init(void)
{
    BaseType_t xReturned = xTaskCreate(ads1292r_task,
                                       "ads1292r",
                                       TSK_ADS1292R_STACK_SIZE,
                                       NULL,
                                       TSK_ADS1292R_PRIORITY,
                                       &m_ads1292r_thread);
    if (xReturned != pdPASS)
    {
        NRF_LOG_ERROR("[ads1292r] task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    else
    {
        NRF_LOG_INFO("[ads1292r] task created.");
    }
}


static sens_packet_t * next_packet(void)
{
    static bool initialized = false;
    static sens_packet_t * p_packet_pool[SENS_PACKET_POOL_SIZE] = {0};
    static int next_modulo = 0;

    if (!initialized)
    {
        // init m_packet helper
        sens_init_ads129x_packet(&m_packet_helper, NULL);

        // alloc mem
        for (int i = 0; i < SENS_PACKET_POOL_SIZE; i++)
        {
            p_packet_pool[i] = pvPortMalloc(m_packet_helper.packet_size);
            APP_ERROR_CHECK_BOOL(p_packet_pool[i] != NULL);
        }

        initialized = true;

        NRF_LOG_INFO("ads129x packets initialized, payload length: %d, packet size: %d, %d samples per packet",
            m_packet_helper.payload_len,
            m_packet_helper.packet_size,
            ADS129X_NUM_OF_SAMPLES);
    }

    // init next packet
    sens_init_ads129x_packet(&m_packet_helper, p_packet_pool[next_modulo]);
    sens_packet_t * next = p_packet_pool[next_modulo];
    next_modulo = (next_modulo + 1) % SENS_PACKET_POOL_SIZE;
    return next;
}

static void init_rdatac_records(void)
{
    p_records_idle = xQueueCreate(8, sizeof(void*));
    ASSERT(p_records_idle != NULL);
    p_records_pending = xQueueCreate(8, sizeof(void*));
    ASSERT(p_records_pending != NULL);

    for (int i = 0; i < 8; i++)
    {
        rdatac_record_t * p_rec = &rdatac_records[i];
        APP_ERROR_CHECK_BOOL(pdTRUE == xQueueSend(p_records_idle, &p_rec, 0));
    }
}
