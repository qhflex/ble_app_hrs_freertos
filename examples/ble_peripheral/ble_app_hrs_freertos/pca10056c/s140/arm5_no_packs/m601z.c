#include <stdint.h>
#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "nrf_log.h"
#include "m601z.h"

static TaskHandle_t m_m601z_thread = NULL;

void m601z_task(void * pvParameters)
{
  NRF_LOG_INFO("m601z task starts");
  for (;;)
  {
    // In portmacro_cmsis.h, portTICK_PERIOD_MS is defined as (1000 / 1024)
    // which evaluates to zero !!! This is a serious design flaw.
    // Without modifying either potTICK_PERIOD_MS or configTICK_RATE_HZ,
    // we cannot use the familiar expression
    // ( duration_in_milliseconds / portTICK_PERIOD_MS ) anymore.
    // We must calculate the ticksToDelay on our own. Here, 1024 means 1 second.
    vTaskDelay(1024);
    NRF_LOG_INFO("m601z task ticks");
  }
}

void app_m601z_freertos_init(void){
	BaseType_t xReturned = xTaskCreate(m601z_task, 
                                     "m601z",
                                     TSK_M601Z_STACK_SIZE,
                                     NULL,
                                     TSK_M601Z_PRIORITY,
                                     &m_m601z_thread);
	if (xReturned != pdPASS)
	{
		NRF_LOG_ERROR("m601z task not created.");
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
	else 
	{
		NRF_LOG_INFO("m601z task created.");
	}
}
