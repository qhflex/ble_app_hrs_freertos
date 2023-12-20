#define U8X8_WITHOUT_SET_CONTRAST
#define U8G2_WITHOUT_UNICODE

#include "u8x8.h"
#include "u8g2.h"

#include "FreeRTOS.h"
#include "task.h"

#include "SEGGER_RTT.h"
#include "nrfx_gpiote.h"

#include "oled.h"

#define TSK_OLED_STACK_SIZE         256
#define TSK_OLED_PRIORITY           1

#define RESET_PIN                   11          // P0.11
#define SCL_PIN                     12          // P0.12
#define SDA_PIN                     (32 + 9)    // P1.09

static void draw_u8g2(u8g2_t *u8g2);

static TaskHandle_t m_oled_thread = NULL;

static u8g2_t m_u8g2;

// from u8g2_d_memory.c
uint8_t *u8g2_m_16_8_1(uint8_t *page_cnt)
{
  #ifdef U8G2_USE_DYNAMIC_ALLOC
  *page_cnt = 1;
  return 0;
  #else
  static uint8_t buf[128];
  *page_cnt = 1;
  return buf;
  #endif
}

static void pinout(nrfx_gpiote_pin_t pin, uint8_t arg_int)
{
    if (arg_int) 
    {
        nrfx_gpiote_out_set(pin);
    }
    else
    {
        nrfx_gpiote_out_clear(pin);    
    }
}


/**
    https://github.com/olikraus/u8g2/wiki/Porting-to-new-MCU-platform
    
    #define U8X8_MSG_GPIO_DC			U8X8_MSG_GPIO(U8X8_PIN_DC)
    #define U8X8_MSG_GPIO_RESET 		U8X8_MSG_GPIO(U8X8_PIN_RESET)
    #define U8X8_MSG_GPIO_I2C_CLOCK	    U8X8_MSG_GPIO(U8X8_PIN_I2C_CLOCK)
    #define U8X8_MSG_GPIO_I2C_DATA		U8X8_MSG_GPIO(U8X8_PIN_I2C_DATA)
*/
static uint8_t u8x8_nrfx_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
                                        U8X8_UNUSED uint8_t msg, 
                                        U8X8_UNUSED uint8_t arg_int,
                                        U8X8_UNUSED void *arg_ptr)
{
    switch (msg)
    {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            vTaskDelay(1); //HAL_Delay(1);
            break;
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(arg_int); // HAL_Delay(arg_int);
            break;
        case U8X8_MSG_GPIO_DC:
            // nothin to do, dc pin connected to ground permanently
            break;
        case U8X8_MSG_GPIO_RESET:
            pinout(RESET_PIN, arg_int);
            break;
        case U8X8_MSG_GPIO_I2C_CLOCK:
            pinout(SCL_PIN, arg_int);
            break;
        case U8X8_MSG_GPIO_I2C_DATA:
            pinout(SDA_PIN, arg_int);
            break;
        default:
            break;
    }
    return 1;
}

void u8g2_Setup_ssd1306_i2c_128x64_noname_1(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb)
{
    uint8_t tile_buf_height;
    uint8_t *buf;
    u8g2_SetupDisplay(u8g2, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_fast_i2c, byte_cb, gpio_and_delay_cb);
    buf = u8g2_m_16_8_1(&tile_buf_height);
    u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}

#if 0
void u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb)
{
  uint8_t tile_buf_height;
  uint8_t *buf;
  u8g2_SetupDisplay(u8g2, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_fast_i2c, byte_cb, gpio_and_delay_cb);
  buf = u8g2_m_16_8_f(&tile_buf_height);
  u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}
#endif

static void oled_task(void * pvParameters)
{
    nrfx_gpiote_out_config_t pin_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true); // true for init_high
    nrfx_gpiote_out_init(RESET_PIN, &pin_config);
    nrfx_gpiote_out_init(SCL_PIN, &pin_config);
    nrfx_gpiote_out_init(SDA_PIN, &pin_config);
    
    u8g2_Setup_ssd1306_i2c_128x64_noname_1(&m_u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_nrfx_gpio_and_delay);
    u8g2_InitDisplay(&m_u8g2);
    u8g2_SetPowerSave(&m_u8g2, 0);
    u8g2_ClearBuffer(&m_u8g2);
    for (;;)
    {
       u8g2_FirstPage(&m_u8g2);
       do
       {
            draw_u8g2(&m_u8g2);
       } while (u8g2_NextPage(&m_u8g2));
        
       vTaskDelay(128);
       // SEGGER_RTT_printf(0, "oled\r\n");
    }
}
    
static void draw_u8g2(u8g2_t *u8g2)
{
    u8g2_SetFontMode(u8g2, 1); /*??????*/
    u8g2_SetFontDirection(u8g2, 0); /*??????*/
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf); /*????*/
    u8g2_DrawStr(u8g2, 0, 20, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);
  
    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
}

void app_oled_freertos_init(void)
{
    BaseType_t xReturned;

    xReturned = xTaskCreate(oled_task,
                            "oled",
                            TSK_OLED_STACK_SIZE,
                            NULL,
                            TSK_OLED_PRIORITY,
                            &m_oled_thread);

    if (xReturned != pdPASS)
    {
        SEGGER_RTT_printf(0, "no mem for oled task.\r\n");
    }
    else
    {
        SEGGER_RTT_printf(0, "oled ready.\r\n");
    }
}
