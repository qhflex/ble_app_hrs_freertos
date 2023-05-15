#include <stdint.h>
#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "nrfx_timer.h"
#include "nrf_log.h"
#include "m601z.h"

static TaskHandle_t m_m601z_thread = NULL;

/** @brief Timer driver instance default configuration. */
//  #define NRFX_TIMER_DEFAULT_CONFIG                                                    \
//  {                                                                                    \
//      .frequency          = (nrf_timer_frequency_t)NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY,\
//      .mode               = (nrf_timer_mode_t)NRFX_TIMER_DEFAULT_CONFIG_MODE,          \
//      .bit_width          = (nrf_timer_bit_width_t)NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH,\
//      .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,                    \
//      .p_context          = NULL                                                       \
//  }
//  where
//  NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY is 16MHz
//  NRFX_TIMER_DEFAULT_CONFIG_MODE is Timer
//  NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH is 16 bit
//    (for 16MHz clock, 65536 ticks is 4.096ms, 65536^2 is ~268 seconds)
//  NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY is 6 (0-7, 0 is highest)

const nrfx_timer_t timer1 = NRFX_TIMER_INSTANCE(1);
static nrfx_timer_config_t timer1_config = NRFX_TIMER_DEFAULT_CONFIG;

static uint32_t test_counter = 0;

static void timer1_callback(nrf_timer_event_t event_type, void* p_context)
{
  switch(event_type)
  {
    case NRF_TIMER_EVENT_COMPARE0:
      test_counter++;
      break;
    default:
      break;
  }
}

void m601z_task(void * pvParameters)
{
  NRF_LOG_INFO("m601z task starts");

  uint32_t elapsed = 0;
  uint32_t time_ticks = 0;

  timer1_config.frequency = NRF_TIMER_FREQ_1MHz;
  timer1_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

  nrfx_err_t err = nrfx_timer_init(&timer1, &timer1_config, timer1_callback);
  APP_ERROR_CHECK(err); // this will eventually trigger a reset

  time_ticks = nrfx_timer_us_to_ticks(&timer1, 100 * 1000);

  nrfx_timer_extended_compare(
         &timer1, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  nrfx_timer_enable(&timer1);

  for (;;)
  {
    // In portmacro_cmsis.h, portTICK_PERIOD_MS is defined as (1000 / 1024)
    // which evaluates to zero !!! This is a serious design flaw.
    // Without modifying either potTICK_PERIOD_MS or configTICK_RATE_HZ,
    // we cannot use the familiar expression
    // ( duration_in_milliseconds / portTICK_PERIOD_MS ) anymore.
    // We must calculate the ticksToDelay on our own. Here, 1024 means 1 second.
    vTaskDelay(1024);
    elapsed++;
    NRF_LOG_INFO("m601z: elapsed %d seconds, counted %d times", elapsed, test_counter);
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
