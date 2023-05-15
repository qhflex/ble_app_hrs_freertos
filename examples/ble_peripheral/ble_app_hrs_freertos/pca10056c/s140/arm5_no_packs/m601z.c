#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "nrf_log.h"
#include "m601z.h"

static TaskHandle_t m_m601z_thread;

void m601z_task(void * pvParameters)
{
	for (;;)
	{
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
		NRF_LOG_INFO("m601z task started");
	}
}
