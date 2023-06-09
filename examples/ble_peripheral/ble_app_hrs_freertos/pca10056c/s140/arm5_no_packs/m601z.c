#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nrfx_timer.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_assert.h"

#include "m601z.h"

#ifdef USE_DK_BOARD
#define MDQ                                         (0 + 27)    // P0.27 (SCL)
#define MDR                                         (0 + 26)    // toggle this pin when sampling
                                                                // short SB32 to enable pull-up
                                                                
#else
#define   MDQ                                       0           // P0.00
#define   MINT                                      (32 + 15)   // P1.15, not used
#endif

typedef enum err_onewire
{
  ERR_OW_NONE = 0,
  ERR_OW_UNDETECTED
} err_onewire_t;

typedef enum continuation {
  CONT_INIT = 0,
  CONT_RESET,
  CONT_RESET_DONE,
  CONT_DETECT,
  CONT_DETECT_DONE,
  CONT_WRITE_LOW,
  CONT_WRITE_HIGH1, // early
  CONT_WRITE_HIGH0, // late
  CONT_WRITE_REC,
  CONT_READ_LOW,
  CONT_READ_HIGH,
  CONT_READ_SAMPLE,
  CONT_READ_REC
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
  // uint8_t ignore_undetected;
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
  .interrupt_priority = 4, // NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
  .p_context = &ctx,
};

uint8_t crc8(uint8_t *data, size_t len)
{
  uint8_t crc = 0xff;
  size_t i, j;
  for (i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (j = 0; j < 8; j++)
    {
      if ((crc & 0x80) != 0)
      {
        crc = (uint8_t)((crc << 1) ^ 0x31);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

unsigned char CRC8(unsigned char *serial, unsigned char length)
{
  unsigned char result = 0x00;
  unsigned char pDataBuf;
  unsigned char i;

  while(length--)
  {
    pDataBuf = *serial++;
    for(i=0; i<8; i++)
    {
      if((result^(pDataBuf))&0x01)
      {
        result ^= 0x18;
        result >>= 1;
        result |= 0x80;
      }
      else {
        result >>= 1;
      }
      pDataBuf >>= 1;
    }
  }
  return result;
}

/*
 * In gpio driver, open drain mode is possible (note NRF_GPIO_PIN_S0D1 in commented code)
 * But gpiote driver does not provide this option.
 */
static void mdq_out()
{
  nrf_gpio_cfg(MDQ,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0D1,       // standard 0 disconnect 1, open drain
               NRF_GPIO_PIN_NOSENSE);   // only affect sleep wakeup
}

static void mdq_in()
{
  nrf_gpio_cfg(MDQ,
               NRF_GPIO_PIN_DIR_INPUT,
               NRF_GPIO_PIN_INPUT_CONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0D1,
               NRF_GPIO_PIN_NOSENSE);
}

static uint32_t mdq_read()
{
  uint32_t level;

  mdq_in();
  level = nrf_gpio_pin_read(MDQ);
  mdq_out();

  return level;
}

static void next_stop(continuation_t cont, uint32_t delay)
{
  nrf_timer_cc_write(timer1.p_reg, NRF_TIMER_CC_CHANNEL0, delay);
  nrfx_timer_clear(&timer1);
  nrfx_timer_resume(&timer1);

  ctx.cont = cont;
}

static void onewire_handle(onewire_transaction_ctx_t* p_ctx)
{
  switch(p_ctx->cont)
  {
    case CONT_INIT:
      xSemaphoreGiveFromISR(sem, NULL);
      return;
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
    case CONT_WRITE_HIGH1:
      goto cont_write_high1;
    case CONT_WRITE_HIGH0:
      goto cont_write_high0;
    case CONT_WRITE_REC:
      goto cont_write_rec;
    case CONT_READ_LOW:
      goto cont_read_low;
    case CONT_READ_HIGH:
      goto cont_read_high;
    case CONT_READ_SAMPLE:
      goto cont_read_sample;
    case CONT_READ_REC:
      goto cont_read_rec;
  }

  cont_reset:
  {
    nrf_gpio_pin_write(MDQ, 0);
    next_stop(CONT_RESET_DONE, 960);
    return;
  }

  cont_reset_done:
  {
    nrf_gpio_pin_write(MDQ, 1);
    next_stop(CONT_DETECT, 70);
    return;
  }

  cont_detect:
  {
    if (mdq_read())
    {
      p_ctx->err = ERR_OW_UNDETECTED;
      xSemaphoreGiveFromISR(sem, NULL);
    }
    else
    {
      next_stop(CONT_DETECT_DONE, (960 - 70));
    }
    return;
  }

  cont_detect_done:
  {
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

    uint8_t bit_cnt = p_ctx->bit_cnt;
    uint8_t idx = bit_cnt / 8;
    uint8_t mask = (uint8_t)1 << (bit_cnt % 8);
    uint8_t bit = p_ctx->odata[idx] & mask;

    if (bit)
    {
      // next_stop(CONT_WRITE_HIGH1, 2); // measure ~12
      next_stop(CONT_WRITE_HIGH1, 1); // measure ~12
    }
    else
    {
      next_stop(CONT_WRITE_HIGH0, 80); // measured ~90
    }
    return;
  }

  cont_write_high1:
  {
    nrf_gpio_pin_write(MDQ, 1);
    next_stop(CONT_WRITE_REC, 49); // measured around 94
    return;
  }

  cont_write_high0:
  {
    nrf_gpio_pin_write(MDQ, 1);
    // no return
  }

  cont_write_rec:
  {
    p_ctx->bit_cnt++;
    if (p_ctx->bit_cnt < (p_ctx->odata_len * 8))
    {
      // loop
      next_stop(CONT_WRITE_LOW, 10);
      return;
    }

    if (p_ctx->idata_len == 0)
    {
      p_ctx->err = ERR_OW_NONE;
      xSemaphoreGiveFromISR(sem, NULL);
      return;
    }
    else
    {
      // reset loop variable
      p_ctx->bit_cnt = 0;
      next_stop(CONT_READ_LOW, 10);
      return;
    }
  }

  cont_read_low:
  {
    nrf_gpio_pin_write(MDQ, 0);
    next_stop(CONT_READ_HIGH, 1); // around 8us
    return;
  }

  cont_read_high:
  {
    nrf_gpio_pin_write(MDQ, 1);
    next_stop(CONT_READ_SAMPLE, 2); // 
    return;
  }

  cont_read_sample:
  {
    uint32_t idx = p_ctx->bit_cnt / 8;
    uint32_t mask = (1U << (p_ctx->bit_cnt % 8));
    
//#ifdef USE_DK_BOARD    
//    nrf_gpio_pin_toggle(MDR);
//#endif
    
    if (mdq_read())
    {
      
//#ifdef USE_DK_BOARD    
//      nrf_gpio_pin_toggle(MDR);
//#endif
      
      p_ctx->idata[idx] |= mask;
    }
    else
    {
      
//#ifdef USE_DK_BOARD    
//    nrf_gpio_pin_toggle(MDR);
//#endif
      
      p_ctx->idata[idx] &= ~mask; // this is not required actually TODO
    }

    next_stop(CONT_READ_REC, (40)); // ~96us in total
    return;
  }

  cont_read_rec:
  {
    p_ctx->bit_cnt++;

    if (p_ctx->bit_cnt < p_ctx->idata_len * 8)
    {
      next_stop(CONT_READ_LOW, 10);
      return;
    }
    else
    {
      p_ctx->err = ERR_OW_NONE;
      xSemaphoreGiveFromISR(sem, NULL);
      return;
    }
  }
}

/*
 * Only channel 0 is used
 */
static void timer1_callback(nrf_timer_event_t event_type, void* p_context)
{
  switch(event_type)
  {
    case NRF_TIMER_EVENT_COMPARE0:
      onewire_handle((onewire_transaction_ctx_t*)p_context);
      break;
    default:
      break;
  }
}

/*
 * onewire convert T command
 */
uint32_t onewire_convert()
{
  memset(&ctx, 0, sizeof(ctx));
  ctx.odata[0] = 0xCC;  // skip rom   11001100b
  ctx.odata[1] = 0x44;  // convert T  01000100b
  ctx.odata_len = 2;

  next_stop(CONT_RESET, 1);
  xSemaphoreTake(sem, portMAX_DELAY);

  return ctx.err;
}

uint32_t onewire_read_scratchpad()
{
  memset(&ctx, 0, sizeof(ctx));
  ctx.odata[0] = 0xCC;  // skip rom
  ctx.odata[1] = 0xBE;  // read scratchpad
  ctx.odata_len = 2;
  ctx.idata_len = 9;    // including crc

  next_stop(CONT_RESET, 1);
  xSemaphoreTake(sem, portMAX_DELAY);

  return ctx.err;
}

static void print_temp(int num)
{
  const char *code = "0123456789ABCDEF";
  char buf[12] = {0};
  char hex[28] = {0};
  
  int16_t *st = (int16_t*)ctx.idata;
  float temp = (float)(*st)/256 + 40;
  snprintf(buf, 10, "%.4f", temp);
  
  int j = 0;
  for (int i = 0; i < ctx.idata_len; i++)
  {
    hex[j++] = ' ';
    hex[j++] = code[ctx.idata[i] >> 4];
    hex[j++] = code[ctx.idata[i] & 0x0f];
  }
  
  char *crc = (CRC8(&ctx.idata[0], ctx.idata_len - 1) == ctx.idata[ctx.idata_len - 1]) ? "crc ok" : "crc bad";
  
  NRF_LOG_RAW_INFO("[m601z] %s (%d) (%s, %s)\n", buf, num, &hex[1], crc);
}

/*
 * freertos task for m601z
 */
void m601z_task(void * pvParameters)
{
  NRF_LOG_INFO("m601z task starts");

  // init semaphore
  sem = xSemaphoreCreateBinary();

  // init timer
  nrfx_err_t err = nrfx_timer_init(&timer1, &timer1_config, timer1_callback);
  APP_ERROR_CHECK(err); // this will eventually trigger a reset

  // delay 1us to put the timer in STOP state
  nrfx_timer_extended_compare(&timer1, NRF_TIMER_CC_CHANNEL0, 1, NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);
  nrfx_timer_enable(&timer1);

  xSemaphoreTake(sem, portMAX_DELAY);

  // avoid negative pulse
  nrf_gpio_pin_set(MDQ);
  mdq_out();

//#ifdef USE_DK_BOARD  
//  nrf_gpio_cfg(MDR,
//             NRF_GPIO_PIN_DIR_OUTPUT,
//             NRF_GPIO_PIN_INPUT_DISCONNECT,
//             NRF_GPIO_PIN_NOPULL,
//             NRF_GPIO_PIN_S0D1,       // standard 0 disconnect 1, open drain
//             NRF_GPIO_PIN_NOSENSE);   // only affect sleep wakeup
//#endif             
  
  NRF_LOG_RAW_INFO("[m601z] test pin level %d\n", mdq_read());

  for (uint32_t i = 1;;i++)
  {
    // In portmacro_cmsis.h, portTICK_PERIOD_MS is defined as (1000 / 1024)
    // which evaluates to zero !!! This is a serious design flaw.
    // Without modifying either potTICK_PERIOD_MS or configTICK_RATE_HZ,
    // we cannot use the familiar expression
    // ( duration_in_milliseconds / portTICK_PERIOD_MS ) anymore.
    // We must calculate the ticksToDelay on our own. Here, 1024 means 1 second.
    vTaskDelay(5120);

    if (onewire_convert())
    {
      NRF_LOG_RAW_INFO("[m601z] sensor not detected (%d)\n", i);
      continue;
    }

    // convert time 10.5ms, 5.5ms, 4ms, configurable
    // each tick is 1/1024 second, 11 ticks is 10.75ms
    vTaskDelay(12);

    if (onewire_read_scratchpad())
    {
      NRF_LOG_RAW_INFO("[m601z] sensor not responding (%d)\n", i);
      continue;
    }
    
    print_temp(i);

//    uint8_t crc = CRC8(&ctx.idata[0], ctx.idata_len - 1);
//    int16_t *st = (int16_t*)ctx.idata;
//    float temp = (float)(*st)/256 + 40;
//    
//    char *sign = temp < 0 ? "-" : "+";
//    float abs = (temp < 0) ? -temp : temp;
//    int int_digits = abs;
//    float frac = abs - int_digits;
//    int frac_digits = trunc(frac * 10000);

//    if (crc == ctx.idata[8])
//    {
//      // NRF_LOG_RAW_INFO("[m601z] temp: %s%d.%d (%d, crc ok, %s%s)\n", sign, int_digits, frac_digits, i);
//    }
//    else
//    {
//      const char *code = "0123456789ABCDEF";
//      char hex[28]; // 9 spaces + 9 * 2 chars + 1 null terminator = 28
//      memset(hex, 0, 27);
//      
//      int j = 0;
//      for (int i=0; i < ctx.idata_len; i++)
//      {
//        hex[j++] = ' ';
//        
//        int high = ctx.idata[i] >> 4;
//        hex[j++] = code[high];
//        int low = ctx.idata[i] & 0x0f;
//        hex[j++] = code[low];
//      }
//      
//      NRF_LOG_RAW_INFO("[m601z] temp: %s%d.%d (%d, %s %s)\n", sign, int_digits, frac_digits, i, code, hex);
//      // NRF_LOG_RAW_INFO("[m601z] temp: " NRF_LOG_FLOAT_MARKER " (%d), bad crc \n", NRF_LOG_FLOAT(temp), i);
//    }
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
