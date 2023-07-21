#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "nrf_log.h"

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

// #include "boards.h"
// #include "bsp.h"
// #include "bsp_cli.h"
// #include "nrf_cli.h"
// #include "nrf_cli_uart.h"

#include "nrfx_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "usbcdc.h"

#define TSK_USBCDC_STACK_SIZE   (64)
#define TSK_USBCDC_PRIORITY      1

const nrfx_timer_t timer2 = NRFX_TIMER_INSTANCE(2);

static const nrfx_timer_config_t timer2_config = {
    .frequency = NRF_TIMER_FREQ_250kHz,
    .mode = NRF_TIMER_MODE_TIMER,
    .bit_width = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = 3,    // NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY is 6, and smaller
                                // value means higher priority
    .p_context = NULL // &ctx,
};

static void timer2_callback(nrf_timer_event_t event_type, void * p_context);

static TaskHandle_t m_usbcdc_thread;

#define QUEUE_LENGTH            100
#define QUEUE_ITEM_SIZE         6

static  QueueHandle_t m_queue = NULL;
static  bool usbd_running = false;

#if NRF_CLI_ENABLED
/**
 * @brief CLI interface over UART
 */
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            4);
#endif

/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief USBD CDC ACM example
 *
 */

#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

#define BTN_CDC_DATA_SEND       0
#define BTN_CDC_NOTIFY_SEND     1

#define BTN_CDC_DATA_KEY_RELEASE        (bsp_event_t)(BSP_EVENT_KEY_LAST + 1)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            NRF_LOG_INFO("cdc acm port open");
            xQueueReset(m_queue);
            nrfx_timer_enable(&timer2);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            nrfx_timer_disable(&timer2);
            NRF_LOG_INFO("cdc acm port close");
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            // bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            m_rx_buffer,
                                            READ_SIZE);
            } while (ret == NRF_SUCCESS);

            // bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            // bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            // bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            NRF_LOG_INFO("usbd started");
            break;
        case APP_USBD_EVT_STOPPED:
            NRF_LOG_INFO("usbd stopped");
            app_usbd_disable();
            // bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("usb power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("usb power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("usb power ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

void usb_new_event_isr_handler(app_usbd_internal_evt_t const * const p_event, bool queued)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    UNUSED_PARAMETER(p_event);
    UNUSED_PARAMETER(queued);
    ASSERT(m_usbcdc_thread != NULL);
    /* Release the semaphore */
    vTaskNotifyGiveFromISR(m_usbcdc_thread, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//static void bsp_event_callback(bsp_event_t ev)
//{
//    ret_code_t ret;
//    switch ((unsigned int)ev)
//    {
//        case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_DATA_SEND):
//        {
//            m_send_flag = 1;
//            break;
//        }
//
//        case BTN_CDC_DATA_KEY_RELEASE :
//        {
//            m_send_flag = 0;
//            break;
//        }

//        case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_NOTIFY_SEND):
//        {
//            ret = app_usbd_cdc_acm_serial_state_notify(&m_app_cdc_acm,
//                                                       APP_USBD_CDC_ACM_SERIAL_STATE_BREAK,
//                                                       false);
//            UNUSED_VARIABLE(ret);
//            break;
//        }

//        default:
//            return; // no implementation needed
//    }
//}

//static void init_bsp(void)
//{
//    ret_code_t ret;
//    ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
//    APP_ERROR_CHECK(ret);
//
//    UNUSED_RETURN_VALUE(bsp_event_to_button_action_assign(BTN_CDC_DATA_SEND,
//                                                          BSP_BUTTON_ACTION_RELEASE,
//                                                          BTN_CDC_DATA_KEY_RELEASE));
//
//    /* Configure LEDs */
//    bsp_board_init(BSP_INIT_LEDS);
//}

#if NRF_CLI_ENABLED
static void init_cli(void)
{
    ret_code_t ret;
    ret = bsp_cli_init(bsp_event_callback);
    APP_ERROR_CHECK(ret);
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}
#endif

static uint8_t rougu[15] = { 0 };

//          uint16_t sigma = 0;
//          for (int i = 0; i < 14; i++)
//          {
//              sigma += (uint16_t)rougu[i];
//          }
//          rougu[14] = ((~sigma)+1) & 0xff;
// size_t size = sprintf(m_tx_buffer, "USB CDC Timer fired: %u\r\n", frame_counter);
static void timer2_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0: {
            spo2_sample_t sample;
            static int counter = 0;
            static int speed = 0;
            
            if (pdTRUE != xQueueReceiveFromISR(m_queue, &sample, NULL))
            {
                return;
            }
                        
            rougu[0] = 0x18;
            rougu[1] = 0xff;                
            rougu[2] = sample.byte[3];
            rougu[3] = sample.byte[4];
            rougu[4] = sample.byte[5];
            rougu[5] = sample.byte[0];
            rougu[6] = sample.byte[1];
            rougu[7] = sample.byte[2];
            rougu[8] = rougu[2];
            rougu[9] = rougu[3];
            rougu[10] = rougu[4];
            rougu[11] = rougu[5];
            rougu[12] = rougu[6];
            rougu[13] = rougu[7];
            
            uint16_t sum = 0;
            for (int j = 0; j < 14; j++)
            {
                sum += (uint16_t)rougu[j];
            }
            
            if (sum < 256)
            {
                rougu[14] = sum;
            }
            else;
            {
                rougu[14] = ((~sum)+1) & 0xff;
            }

            // rougu[14] = (sum < 255) ? (uint8_t)sum : (uint8_t)(((~sum) + 1) & 0xff);
            
            ret_code_t err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, rougu, sizeof(rougu));
            if (err_code == NRF_SUCCESS)
            {
                counter++;
                if (counter % 1000 == 0)
                {
                    NRF_LOG_INFO("%d packets sent", counter);
                };                
            }
            else
            {
               NRF_LOG_INFO("cdc_acm_write failed, error %d", err_code);
            }
            
            int waiting = uxQueueMessagesWaitingFromISR(m_queue);
            int expect = 0;
            if (waiting > 60)
            {
                expect = 3;
            }
            else if (waiting > 48)
            {
                expect = 2;
            }
            else if (waiting > 36)
            {
                expect = 1;
            }
            
            if (expect != speed)
            {
                uint32_t cc = 5000 - 1; // accurate
                if (expect == 1)
                {
                    cc = 4900 - 1;      // 2%
                }
                else if (expect == 2)
                {
                    cc = 4500 - 1;      // 10%
                }
                else if (expect == 3)
                {
                    cc = 2500 - 1;      // 40%
                }

                nrfx_timer_extended_compare(&timer2, NRF_TIMER_CC_CHANNEL0, cc,
                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
                

                // NRF_LOG_INFO("waiting %d, %s", waiting, (expect == 0) ? "-" : (expect == 1) ? "+2%" : (expect == 2) ? "+11%" : "+100%");
                speed = expect;
            }
        }   break;
        default:
            break;   
    }
}

bool cdc_acm_running()
{
    return nrfx_timer_is_enabled(&timer2);
}

void enqueue(spo2_sample_t * p_smpl)
{
    if (pdTRUE != xQueueSend(m_queue, p_smpl, 0))
    {
        NRF_LOG_INFO("samples dropped due to queue full");
    }
}

static void usbcdc_task(void * pvParameters)
{
    ret_code_t err_code;

#ifdef MIMIC_ROUGU
    m_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    APP_ERROR_CHECK_BOOL(m_queue != NULL);

    nrfx_err_t err = nrfx_timer_init(&timer2, &timer2_config, timer2_callback);
    APP_ERROR_CHECK(err);

    nrfx_timer_extended_compare(&timer2, NRF_TIMER_CC_CHANNEL0, (4900),
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
#endif

    static const app_usbd_config_t usbd_config = {
        .ev_isr_handler = usb_new_event_isr_handler,
        .ev_state_proc = usbd_user_ev_handler
    };

    err_code = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("usbcdc task started.");

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    err_code = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(err_code);

    if (USBD_POWER_DETECTION)
    {
        err_code = app_usbd_power_events_enable();
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }

    // vTaskDelay(portMAX_DELAY);
    UNUSED_RETURN_VALUE(xTaskNotifyGive(xTaskGetCurrentTaskHandle()));

    for (;;)
    {
        UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));
        static int counter = 0;
        // NRF_LOG_INFO("usbd %d", counter++);
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }

        // NRF_LOG_INFO("app_usbd_event_queue_process done");
        // vTaskDelay(128);

//        if(m_send_flag)
//        {
//            static int  frame_counter;

//            size_t size = sprintf(m_tx_buffer, "Hello USB CDC FA demo: %u\r\n", frame_counter);

//            err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
//            if (err_code == NRF_SUCCESS)
//            {
//                ++frame_counter;
//            }
//        }

#if NRF_CLI_ENABLED
        nrf_cli_process(&m_cli_uart);
#endif

        // UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        /* Sleep CPU only if there was no interrupt since last loop processing */
        // __WFE();
    }
}



void app_usbcdc_freertos_init(void)
{
    BaseType_t xReturned = xTaskCreate(usbcdc_task,
                                       "usbcdc",
                                       TSK_USBCDC_STACK_SIZE,
                                       NULL,
                                       TSK_USBCDC_PRIORITY,
                                       &m_usbcdc_thread);
    if (xReturned != pdPASS)
    {
        NRF_LOG_ERROR("usbcdc task not created");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    else
    {
        NRF_LOG_INFO("usbcdc task created");
    }

}

