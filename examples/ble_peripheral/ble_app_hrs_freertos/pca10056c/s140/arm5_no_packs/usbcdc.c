#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "SEGGER_RTT.h"

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

#include "nrfx_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "max86141.h"
#include "usbcdc.h"
#include "sens-proto.h"

#define TSK_USBCDC_STACK_SIZE   (256)   // 1024
#define TSK_USBCDC_PRIORITY      3

#define NOT_INSIDE_ISR          (( SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk ) == 0 )
#define INSIDE_ISR              (!(NOT_INSIDE_ISR))

typedef __packed struct pending_item {
    uint8_t * p_pkt;
    uint32_t size;
} pending_item_t;

// static QueueHandle_t m_pending_queue = NULL;
// static uint8_t * p_pkt_sending = NULL;

static TaskHandle_t m_usbcdc_thread;

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

static bool m_cdc_acm_port_open = false;

extern void get_abp_coeff(void);
extern void set_abp_coeff(uint8_t * ptr);

static void handle_command(uint16_t type, uint16_t length, uint8_t * payload)
{
    char buf[16];
    // SEGGER_RTT_printf(0, "inside irs %d\r\n", INSIDE_ISR); // output 0, not inside isr
    SEGGER_RTT_printf(0, "cdc rx, type %d len %d\r\n", type, length);
//    for (int i = 0; i < 32; i++)
//    {
//        SEGGER_RTT_printf(0, "%d\r\n", payload[i]);
//    }
    
    // this function switch by type, it does not understand instanceId, 
    // if this is required, we can chaining all handlers, of which only one really works.
    
    // NRF_LOG_INFO("recv cmd, type: %d, len: %d", type, length);
    if (type == 2)  // get abp-coeff
    {
        SEGGER_RTT_printf(0, "get-abp-coeff\r\n");
        get_abp_coeff();
    }
    else if (type == 3 && length == 16) // set abp-coeff
    {
        SEGGER_RTT_printf(0, "set-abp-coeff\r\n");
        
        ieee754_t coeffs[4];
        for (int i = 0; i < 4; i++)
        {
            coeffs[i].u = (uint32_t)payload[i * 4] + (((uint32_t)payload[i * 4 + 1]) << 8) + (((uint32_t)payload[i * 4 + 2]) << 16) + (((uint32_t)payload[i * 4 + 3]) << 24);
            sprintf(buf, "%9.6f", coeffs[i].f);
            SEGGER_RTT_printf(0, "%s\r\n", buf);
        }
        set_abp_coeff((uint8_t *)coeffs);
    }
}

// see peripheral/usbd_cdc_acm/main.c:138

// static char m_rx_buffer[50];
// static int m_rx_bytes_in_buffer = 0;
// static char m_incomming_

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    static uint8_t rxchar;
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            // NRF_LOG_INFO("cdc acm port open (inside isr %d)", (INSIDE_ISR) ? 1 : 0);
            m_cdc_acm_port_open = true;

            /* bootstrap rx */
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, &rxchar, 1);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE: {
            m_cdc_acm_port_open = false;

//            if (p_pkt_sending)
//            {
//                // NRF_LOG_INFO("clearing sending item when port close");
//                // vPortFree(p_pkt_sending);
//                p_pkt_sending = NULL;
//                SEGGER_RTT_printf(0, "cdc sended\r\n");
//            }

//            uint32_t pending = uxQueueMessagesWaiting(m_pending_queue);
//            if (pending)
//            {
//                // NRF_LOG_INFO("clearing %d pending item when port close", pending);
//                // pending_item_t item;
//                // while (pdTRUE == xQueueReceive(m_pending_queue, &item, NULL)) {
//                    // vPortFree(item.p_pkt);
//                // }
//                xQueueReset(m_pending_queue);
//            }

            // NRF_LOG_INFO("cdc acm port close (pending %d)", uxQueueMessagesWaiting(m_pending_queue));
            SEGGER_RTT_printf(0, "cdc acm port close\r\n");
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE: {   // assume this is isr context

//            int pending = uxQueueMessagesWaiting(m_pending_queue);
//            
//            if (p_pkt_sending)
//            {
//                p_pkt_sending = NULL;
//                SEGGER_RTT_printf(0, "cdc sent (%d)\r\n", pending);
//            }
//            
//            pending_item_t item;
////            BaseType_t result;

//            try_next_item:
////            if (NOT_INSIDE_ISR)
////            {
////                result = xQueueReceive(m_pending_queue, &item, NULL);
////            }
////            else
////            {
////                result = xQueueReceiveFromISR(m_pending_queue, &item, NULL);
////            }

//            if (pdTRUE == xQueueReceiveFromISR(m_pending_queue, &item, NULL))
//            {
//                uint8_t * p_pkt = item.p_pkt;
//                uint32_t size = item.size;
//                ret_code_t err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, p_pkt, size);
//                if (err_code == NRF_SUCCESS)
//                {
//                    p_pkt_sending = p_pkt;
//                    SEGGER_RTT_printf(0, "cdc sendding\r\n");
//                }
//                else
//                {
//                    // vPortFree(p_pkt);
//                    SEGGER_RTT_printf(0, "cdc write error %x04x\r\n", err_code);
//                    goto try_next_item;
//                }
//            } else if (pending) {
//                SEGGER_RTT_printf(0, "cdc qrcv failed (%d)\r\n", pending);
//            }
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            
            static uint8_t payload[512];
            static uint16_t type;
            static uint16_t length;
            static uint8_t cka, ckb;    // CRC: ckb += (cka += rxchar);

            static int pos = 0;         //  0..6 expecting first..seventh byte of preamble (0x55)
                                        //  7    expecting last byte of preamble (0xd5)
                                        //  8..9 expecting first..second byte of type (0x01)
                                        // 10..11 expecting first..second byte of length
                                        // 12..(12 + length - 1) expecting data blindly
                                        // 12 + length cka
                                        // 12 + length + 1 ckb
                                        // 8 (preamble) + 2 (type) + 2 (pay len) + data + 2 (crc) = data len + 14
                                        // when data len is 32, total len is 46

            // NRF_LOG_INFO("Bytes buffered in cdc_acm: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);

                if (pos < 7)
                {
                    pos = (rxchar == 0x55) ? pos + 1 : 0;
                }
                else if (pos == 7)
                {
                    pos = (rxchar == 0xd5) ? pos + 1 : 0;
                }
                else if (pos == 8)
                {
                    cka = 0;
                    ckb = 0;
                    ckb += (cka += rxchar);
                    type = rxchar;
                    pos++;
                }
                else if (pos == 9)
                {
                    ckb += (cka += rxchar);
                    type += rxchar * 256;
                    pos++;
                }
                else if (pos == 10)
                {
                    ckb += (cka += rxchar);
                    length = rxchar;
                    pos++;
                }
                else if (pos == 11)
                {
                    ckb += (cka += rxchar);
                    length += rxchar * 256;
                    pos = length > 512 ? 0 : pos + 1;
                }
                else if (pos < length + 12)
                {
                    ckb += (cka += rxchar);
                    payload[pos - 12] = rxchar;
                    pos++;
                }
                else if (pos == length + 12)
                {
                    if (rxchar != cka)
                    {
                        SEGGER_RTT_printf(0, "cdc rx bad crc (a)\r\n");
                        pos = 0;
                    }
                    else
                    {
                        pos++;
                    }
                }
                else
                {
                    if (rxchar != ckb)
                    {
                        SEGGER_RTT_printf(0, "cdc rx bad crc (b)\r\n");
                    }
                    else
                    {
                        handle_command(type, length, payload);
                    }
                    pos = 0;
                }

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, &rxchar, 1);
            } while (ret == NRF_SUCCESS);
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    // ret_code_t err;
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
            SEGGER_RTT_printf(0, "usb start\r\n");
            break;
        case APP_USBD_EVT_STOPPED:
            NRF_LOG_INFO("usbd stopped");
            SEGGER_RTT_printf(0, "usb stop\r\n");
            app_usbd_disable();
            // bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("usb power detected");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
                NRF_LOG_INFO("usbd enabled");
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

bool cdc_acm_port_open()
{
    // return nrfx_timer_is_enabled(&timer2);
    return m_cdc_acm_port_open;
}

static void usbcdc_task(void * pvParameters)
{
    vTaskDelay(1000);
    
    ret_code_t err_code;

//    m_pending_queue = xQueueCreate(8, sizeof(pending_item_t));
//    APP_ERROR_CHECK_BOOL(m_pending_queue != NULL);
    
    // vTaskDelay(100);

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

    UNUSED_RETURN_VALUE(xTaskNotifyGive(xTaskGetCurrentTaskHandle()));

    for (;;)
    {
        UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));

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

void cdc_acm_send_packet(uint8_t * p_pkt, uint32_t size)
{
    if (!m_cdc_acm_port_open)
    {
        // vPortFree(p_pkt);
        return;
    }

#if 0    
    if (p_pkt_sending)
    {
//        BaseType_t result;
        pending_item_t item = { .p_pkt = p_pkt, size = size };

//        if (NOT_INSIDE_ISR)
//        {
//            result = xQueueSend(m_pending_queue, &item, NULL);
//        }
//        else
//        {
//            result = xQueueSendFromISR(m_pending_queue, &item, NULL);
//        }

        if (pdTRUE == xQueueSend(m_pending_queue, &item, NULL))
        {
            SEGGER_RTT_printf(0, "cdc q %d", uxQueueMessagesWaiting(m_pending_queue));
        } else {
            // NRF_LOG_INFO("cdc acm drop packet due to queue full, %d", (NOT_INSIDE_ISR));
            SEGGER_RTT_printf(0, "cdc qfull, %d\r\n", uxQueueMessagesWaiting(m_pending_queue));
            // vPortFree(p_pkt);            
        }
        return;
    }
#endif
//    if (p_pkt_sending)
//    {
//        SEGGER_RTT_printf(0, "cdc delay delay delay\r\n");
//        vTaskDelay(1);
//    }
    for (int i = 0;;i++)
    {
        ret_code_t err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, p_pkt, size);
        if (err_code == NRF_SUCCESS)
        {
            // SEGGER_RTT_printf(0, "cdc send done\r\n");
            if (i) 
            {
                SEGGER_RTT_printf(0, "cdc delay %dms\r\n", i);
            }
            return;
        }
        else if (err_code == NRF_ERROR_BUSY)
        {
            vTaskDelay(1);
            continue;
        }
        else
        {
            SEGGER_RTT_printf(0, "cdc acm write error %x04x", err_code);
            return;
        }
    }

#if 0    
    ret_code_t err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, p_pkt, size);
    if (err_code == NRF_SUCCESS)
    {
        p_pkt_sending = p_pkt;
        SEGGER_RTT_printf(0, "cdc sending\r\n");
    }
    else
    {
        NRF_LOG_INFO("cdc acm write error %x04x%", err_code);
        // vPortFree(p_pkt);
        SEGGER_RTT_printf(0, "cdc acm write error %x04x", err_code);
    }
#endif    
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

