#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nrf_log.h"

#include "nrf_spi_mngr.h"

#include "ads1292r.h"


#define ADS1292R_CS_PIN             (32 + 13) // 1.13
#define ADS1292R_CK_PIN             26        // 0.26
#define ADS1292R_DI_PIN             1         // 0.01
#define ADS1292R_DO_PIN             27        // 0.27
#define ADS1292R_DR_PIN             (32 + 14) // 1.14

#define SPI_INSTANCE_ID             1

#define MAX_PENDING_TRANSACTIONS    5

// #define SPI0_ENABLED             1
// #define SPI0_USE_EASY_DMA 0      0 // originally 1)
//                                       dunno whether this is necessary (or even wrong),
//                                       but the example project does so.
NRF_SPI_MNGR_DEF(m_nrf_spi_mngr, MAX_PENDING_TRANSACTIONS, SPI_INSTANCE_ID);


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
       .mosi_pin           = ADS1292R_DO_PIN,
       .miso_pin           = ADS1292R_DI_PIN,
       .ss_pin             = ADS1292R_CS_PIN,
       .irq_priority       = APP_IRQ_PRIORITY_LOWEST,
       .orc                = 0,
       .frequency          = NRF_DRV_SPI_FREQ_250K,
      
       // https://devzone.nordicsemi.com/f/nordic-q-a/39942/nrf52840---ads1292/155794
       // also datasheet
       .mode               = NRF_DRV_SPI_MODE_1,
       .bit_order          = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };

    err_code = nrf_spi_mngr_init(&m_nrf_spi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}

static void ads1292r_task(void * pvParameters)
{
  spi_config();
  for (;;)
  {
    vTaskDelay(portMAX_DELAY);
  }
}

void app_ads1292r_freertos_init(void)
{
  BaseType_t xReturned = xTaskCreate(ads1292r_task,
                                     "qma6110p",
                                     TSK_ADS1292R_STACK_SIZE,
                                     NULL,
                                     TSK_ADS1292R_PRIORITY,
                                     &m_ads1292r_thread);
  if (xReturned != pdPASS)
  {
    NRF_LOG_ERROR("qma6110p task not created.");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  else
  {
    NRF_LOG_INFO("qma6110p task created.");
  }
}
