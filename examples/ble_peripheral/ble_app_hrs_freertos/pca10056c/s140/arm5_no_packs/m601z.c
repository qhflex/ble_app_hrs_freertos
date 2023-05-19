#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nrfx_timer.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "m601z.h"

#define   MDQ                                       0           // P0.00
#define   MINT                                      (32 + 15)   // P1.15

typedef enum err_onewire
{
  err_onewire_none = 0,
  err_onewire_not_detected,
  err_onewire_timeout,
} err_onewire_t;

typedef enum continuation {
  CONT_RESET = 0,
  CONT_RESET_DONE,
  CONT_DETECT,
  CONT_DETECT_DONE,
  CONT_WRITE_LOW,
  CONT_WRITE_HIGH,
  CONT_WRITE_REC,
  CONT_READ_BIT,
} continuation_t;

typedef struct onewire_transaction_ctx
{
  continuation_t cont;
  err_onewire_t err;

  uint8_t odata[16];
  uint8_t odata_len;      // at least 1, for command.

  uint8_t idata[16];
  uint8_t idata_len;

  uint8_t bit_cnt;
  uint8_t ignore_undetected;
} onewire_transaction_ctx_t;

static onewire_transaction_ctx_t ctx;

/*
 * FreeRTOS Task Handler
 */
static TaskHandle_t m_m601z_thread = NULL;
static SemaphoreHandle_t sem = NULL;

/*
 * According to document, nRF52840 has 5 timers, Timer0 to Timer4.
 * Timer0/1/2 has 4 CC registers and Timer 3/4 has 6 CC registers.
 *
 * Timer0 is used by softdevice;
 * Timer1 is used in this module, exclusively.
 */
const nrfx_timer_t timer1 = NRFX_TIMER_INSTANCE(1);

/*
 * Timer1 configuration
 */
static const nrfx_timer_config_t timer1_config = {
  .frequency = NRF_TIMER_FREQ_1MHz,             // default config is 16MHz
  .mode = NRF_TIMER_MODE_TIMER,
  .bit_width = NRF_TIMER_BIT_WIDTH_32,          // default config is 16bit.
  .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
  .p_context = &ctx,
};

//static void config_mdq_od_output()
//{
//  // initially high (open drain)
//  nrf_gpio_pin_set(MDQ);
//  nrf_gpio_cfg(MDQ,
//               NRF_GPIO_PIN_DIR_OUTPUT,
//               NRF_GPIO_PIN_INPUT_DISCONNECT,
//               NRF_GPIO_PIN_NOPULL,
//               NRF_GPIO_PIN_S0D1,       // standard 0 disconnect 1, open drain
//               NRF_GPIO_PIN_NOSENSE);   // sense only affect sleep wakeup
//}

//static void config_mdq_od_input()
//{
//  nrf_gpio_cfg(MDQ,
//               NRF_GPIO_PIN_DIR_INPUT,
//               NRF_GPIO_PIN_INPUT_CONNECT,
//               NRF_GPIO_PIN_NOPULL,
//               NRF_GPIO_PIN_S0D1,
//               NRF_GPIO_PIN_NOSENSE);
//}

static void mdq_hitolo_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if (pin == MDQ && action == NRF_GPIOTE_POLARITY_HITOLO)
  {
    nrfx_timer_resume(&timer1);
  }
}

/*
 * In gpio driver, open drain mode is possible (note NRF_GPIO_PIN_S0D1 in commented code)
 * But gpiote driver does not provide this option.
 */
static void mdq_out_init()
{
  nrfx_err_t err;

  nrfx_gpiote_out_config_t cfg = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(1);
  err = nrfx_gpiote_out_init(MDQ, &cfg);
  APP_ERROR_CHECK(err);

  nrf_gpio_cfg(MDQ,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_H0D1,       // high 0 disconnect 1, open drain
               NRF_GPIO_PIN_NOSENSE);   // only affect sleep wakeup
}

static void mdq_out_uninit()
{
  nrfx_gpiote_out_uninit(MDQ);
}

static void mdq_in_init()
{
  nrfx_err_t err;

  nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  err = nrfx_gpiote_in_init(MDQ, &config, mdq_hitolo_handler);
  APP_ERROR_CHECK(err);
}

static void mdq_in_uninit()
{
  nrfx_gpiote_in_uninit(MDQ);
}

//static void config_mdq_in_event()
//{
//  // true for IN_EVENT, skip gpio setup, dunno whether there are any side effects
//  // TODO
//  nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
//
//  // TODO error checking
//  nrfx_gpiote_in_init(MDQ, &config, mdq_hitolo_handler);
//}

// static nrfx_gpiote_out_config_t mdq_out_low_cfg = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);

static void onewire_handle(onewire_transaction_ctx_t* p_ctx)
{
  switch(p_ctx->cont)
  {
    case CONT_RESET:
      goto cont_reset;
    case CONT_RESET_DONE:
      goto cont_reset_done;
    case CONT_DETECT:
      goto cont_detect;
    case CONT_DETECT_DONE:
      goto cont_detect_done;
    case CONT_WRITE_LOW:
      goto cont_write_low;
    case CONT_WRITE_HIGH:
      goto cont_write_high;
    case CONT_WRITE_REC:
      goto cont_write_rec;
    case CONT_READ_BIT:
      goto cont_read_bit;
  }

  cont_reset:
  {
    // deprecated config_mdq_od_output();
    mdq_out_init();
    nrf_gpio_pin_write(MDQ, 0);
    nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 600, true);
    p_ctx->cont = CONT_RESET_DONE;
    return;
  }

  cont_reset_done:
  {
    mdq_out_uninit();
    mdq_in_init();
    nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 645, true);
    p_ctx->cont = CONT_DETECT;
    return;
  }

  cont_detect:
  {
    if (nrfx_gpiote_in_is_set(MDQ))
    {
      mdq_in_uninit();
      nrfx_timer_pause(&timer1);
      p_ctx->err = err_onewire_not_detected;
      xSemaphoreGiveFromISR(sem, NULL);
    }
    else
    {
      // we shall not end input state here for the device may pull down
      // the dq line for a while
      nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 1200, true);
      p_ctx->cont = CONT_DETECT_DONE;
    }
    return;
  }

  cont_detect_done:
  {
    mdq_in_uninit();
    mdq_out_init();
    p_ctx->bit_cnt = 0;
    // no return here
  }

  /*
   * slot 90, trec 10
   * 1: low 5us, high (90 - 5)us
   * 0: low 90us, high 0us
   */
  cont_write_low:
  {
    nrf_gpio_pin_write(MDQ, 0);

    uint8_t i = p_ctx->bit_cnt;
    uint8_t bit = p_ctx->odata[i / 8] & ((uint8_t)1 << (i % 8));
    uint32_t delay = bit ? 5 : 90;
    nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, delay, true);
    nrfx_timer_clear(&timer1);

    p_ctx->cont = CONT_WRITE_HIGH;
    return;
  }

  cont_write_high:
  {
    nrf_gpio_pin_write(MDQ, 1);

    nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 90, true);

    p_ctx->cont = CONT_WRITE_REC;
    return;
  }

  cont_write_rec:
  {
    p_ctx->bit_cnt++;

    if (p_ctx->bit_cnt < p_ctx->odata_len * 8)
    {
      // loop
      nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 100, true);
      p_ctx->cont = CONT_WRITE_LOW;
      return;
    }

    if (p_ctx->idata_len == 0)
    {
      // no data to read, finished
      mdq_out_uninit();
      nrfx_timer_pause(&timer1);
      p_ctx->err = err_onewire_none;
      xSemaphoreGiveFromISR(sem, NULL);
      return;
    }
    else
    {
      mdq_out_uninit();
      mdq_in_init();

      // pause and clear timer
      nrfx_timer_pause(&timer1);
      nrfx_timer_clear(&timer1);
      nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 12, true);

      // enable input interrupt
      nrfx_gpiote_in_event_enable(MDQ, true);

      // reset loop variable
      p_ctx->bit_cnt = 0;
      p_ctx->cont = CONT_READ_BIT;
      return;
    }
  }

  cont_read_bit:
  {
    // pause and clear timer
    nrfx_timer_pause(&timer1);
    nrfx_timer_clear(&timer1);

    uint32_t idx = p_ctx->bit_cnt / 8;
    uint32_t mask = (1U << (p_ctx->bit_cnt % 8));

    // uint32_t val = nrf_gpio_pin_read(MDQ);
    if (nrfx_gpiote_in_is_set(MDQ))
    {
      p_ctx->idata[idx] |= mask;
    }
    else
    {
      p_ctx->idata[idx] &= ~mask;
    }

    p_ctx->bit_cnt++;
    if (p_ctx->bit_cnt < p_ctx->idata_len * 8)
    {
      return;
    }
    else
    {
      // no data to read, finished
      nrfx_gpiote_in_event_disable(MDQ); // possibly unnecessary
      mdq_in_uninit();

      p_ctx->err = err_onewire_none;
      xSemaphoreGiveFromISR(sem, NULL);
      return;
    }
  }
}

/*
 * CC0 is used as toggling channel
 */
static void timer1_callback(nrf_timer_event_t event_type, void* p_context)
{
  switch(event_type)
  {
    case NRF_TIMER_EVENT_COMPARE0:
      onewire_handle((onewire_transaction_ctx_t*)p_context);
      break;
    case NRF_TIMER_EVENT_COMPARE1:  // pull up
      nrfx_gpiote_out_set(MDQ);
      break;
    case NRF_TIMER_EVENT_COMPARE2:  // sampling
      break;
    case NRF_TIMER_EVENT_COMPARE3:  // period end, clear
      xSemaphoreGiveFromISR(sem, NULL);
      break;
    default:
      break;
  }
}

void m601z_task(void * pvParameters)
{
  NRF_LOG_INFO("m601z task starts");

  sem = xSemaphoreCreateBinary(); // TODO check error
//  nrf_gpio_cfg_input(MDQ, NRF_GPIO_PIN_NOPULL);
//  nrf_gpio_cfg_output(MDQ);

  uint32_t elapsed = 0;

  // timer1_config.frequency = NRF_TIMER_FREQ_1MHz;
  // timer1_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

  nrfx_err_t err = nrfx_timer_init(&timer1, &timer1_config, timer1_callback);
  APP_ERROR_CHECK(err); // this will eventually trigger a reset

//  nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 0, true);
//  nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL1, 600, true);
//  nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL2, 645, true);
//  nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL3, 1200, true);

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
    NRF_LOG_INFO("m601z: elapsed %d seconds", elapsed);

    // just start timer1, oneshot.
    memset(&ctx, 0, sizeof(ctx));
    ctx.odata[0] = 0xCC;  // skip rom
    ctx.odata[1] = 0x44;  // convert T
    ctx.odata_len = 2;
    nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 1, true);
    nrfx_timer_enable(&timer1);

    xSemaphoreTake(sem, portMAX_DELAY);
    nrfx_timer_disable(&timer1);  // TASK_SHUTDOWN is deprecated actually :)

    // convert time 10.5ms, 5.5ms, 4ms, configurable
    // each tick is 1/1024 second, 11 ticks is 10.75ms
    vTaskDelay(11);

    memset(&ctx, 0, sizeof(ctx));
    ctx.odata[0] = 0xCC;  // skip rom
    ctx.odata[1] = 0xBE;  // read scratchpad
    ctx.odata_len = 2;
    ctx.idata_len = 9;    // including crc

    nrfx_timer_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 1, true);
    nrfx_timer_enable(&timer1);

    xSemaphoreTake(sem, portMAX_DELAY);
    nrfx_timer_disable(&timer1);  // TASK_SHUTDOWN is deprecated actually :)
  }
}

void app_m601z_freertos_init(void)
{
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
