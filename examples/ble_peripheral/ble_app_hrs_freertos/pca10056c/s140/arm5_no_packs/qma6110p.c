#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nrf_log.h"

#include "nrf_twi_mngr.h"

#include "qma6110p.h"

#define QMA6110P_SCL_PIN            7
#define QMA6110P_SDA_PIN            6

#define TWI_INSTANCE_ID             0

#define MAX_PENDING_TRANSACTIONS    5

// must define TWI0_ENABLED
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

// NRF_DRV_TWI_INSTANCE_0

/*
 * FreeRTOS Task Handler
 */
static TaskHandle_t m_qma6110p_thread = NULL;
// static SemaphoreHandle_t sem = NULL;

// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = QMA6110P_SCL_PIN,
       .sda                = QMA6110P_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}

// Set Active mode.
// static uint8_t init_buf[] = { QMA6110P_REG_CHIP_ID, 0 };
static uint8_t read_buf[16] = {0xff};

nrf_twi_mngr_transfer_t qma_init_transfers[] =
{
    // NRF_TWI_MNGR_READ(QMA6110P_ADDR, init_buf, sizeof(init_buf), 0)
  QMA6110P_READ(QMA6110P_REG_CHIP_ID, read_buf, 1)
};

void qma6110p_task(void * pvParameters)
{ 
  ret_code_t err_code;
  twi_config();
  
  for (int i = 0; i < 64; i++)
  {
    uint8_t buf = 0xee;
    nrf_twi_mngr_transfer_t xfers [] =
    {
      QMA6110P_READ(i, &buf, 1)
    };
    
    nrf_drv_twi_config_t const config = {
       .scl                = QMA6110P_SCL_PIN,
       .sda                = QMA6110P_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };
        
    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, 
                           &config, 
                           xfers, 
                           sizeof(xfers) / sizeof(xfers[0]), 
                           NULL);
    APP_ERROR_CHECK(err_code);
                           
    vTaskDelay(16);                           
                           
    NRF_LOG_INFO("reg %d: %d", i, buf & 0xff);
  }

  for (;;)
  {
    vTaskDelay(portMAX_DELAY);
  }
}

void app_qma6110p_freertos_init(void)
{
  BaseType_t xReturned = xTaskCreate(qma6110p_task,
                                     "qma6110p",
                                     TSK_QMA6110P_STACK_SIZE,
                                     NULL,
                                     TSK_QMA6110P_PRIORITY,
                                     &m_qma6110p_thread);
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

