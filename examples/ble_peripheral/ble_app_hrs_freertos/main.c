#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"

#include "SEGGER_RTT.h"

#include "app_error.h"

#include "app_util_platform.h"
#include "nrf_strerror.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif

#include "ble_conn_params.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "nrfx_systick.h"
#include "nrfx_gpiote.h"
#include "nrfx_uart.h"

#include "app_usbd_serial_num.h"

#include "ble_nus.h"
#include "ble_nus_tx.h"

#include "sens-proto.h"

// #include "m601z.h"
#include "qma6110p.h"
#include "ads1292r.h"
#include "max86141.h"
#include "owuart.h"
#include "oled.h"

#include "usbcdc.h"
// #include "mem_manager.h"

// #include "ble_spp.h"

 /**********************************************************************
 * MACROS
 */

/**********************************************************************
 * CONSTANTS
 */

#define DEVICE_NAME                         "ifet-wearable"                         /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "ifet-tsinghua"                         /**< Manufacturer. Will be passed to Device Information Service. */
#define NUS_SERVICE_UUID_TYPE               BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                    18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_ADV_FAST_INTERVAL               300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_FAST_DURATION               6000                                    /**< The advertising duration (60 seconds) in units of 10 milliseconds. */
#define APP_ADV_SLOW_INTERVAL               3000                                    // 1.875s
#define APP_ADV_SLOW_DURATION               0


#define SENSOR_CONTACT_DETECTED_INTERVAL    5000                                    /**< Sensor Contact Detected toggle interval (ms). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(15, UNIT_1_25_MS)         /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(40, UNIT_1_25_MS)         /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

#define POWER_ON_PIN                        29 // P0.29
#define OLED_RST_PIN                        11

/*********************************************************************
 * TYPEDEFS
 */

/**********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES for ble_nux_tx.h
 */

bool m_ble_nus_tx_running = false;
ble_nus_tx_buf_t* m_ble_nus_tx_sending = NULL;

#define NUM_OF_BLE_BUF              32

ble_nus_tx_buf_t ble_nus_tx_buffer[NUM_OF_BLE_BUF];

QueueHandle_t ble_nus_tx_idle;
QueueHandle_t ble_nus_tx_pending;

static void ble_nus_tx_init(void);

/*********************************************************************
 * LOCAL VARIABLES
 */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t m_conn_handle            = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
// static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    // {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

#if 0
static ble_uuid_t m_adv_uuids[] =                                                   /**< Universally unique service identifiers. */
{
    /* {BLE_UUID_HEART_RATE_SERVICE, BLE_UUID_TYPE_BLE}, */
    /* {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}, */
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    // {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};
#endif


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void advertising_start(void * p_erase_bonds);

/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*********************************************************************
 * PROFILE FUNCTIONS AND CALLBACKS
 */

/**********************************************************************
 * PUBLIC FUNCTIONS
 */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
#if PEER_MANAGER_ENABLED == 1
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    bool delete_bonds = false;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(&delete_bonds);
            break;

        default:
            break;
    }
}
#endif


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
extern void get_abp_coeff(void);
extern void set_abp_coeff(uint8_t * ptr);

static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    char buf[16];

    // TODO
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint8_t type = p_evt->params.rx_data.p_data[0];
        if (type == 2)
        {
            get_abp_coeff();
        }
        else if (type == 3 && p_evt->params.rx_data.length == 17)
        {
            ieee754_t coeffs[4];
            const uint8_t *payload = &p_evt->params.rx_data.p_data[1];
            for (int i = 0; i < 4; i++)
            {
                coeffs[i].u = (uint32_t)payload[i * 4] + (((uint32_t)payload[i * 4 + 1]) << 8) + (((uint32_t)payload[i * 4 + 2]) << 16) + (((uint32_t)payload[i * 4 + 3]) << 24);
                sprintf(buf, "%9.6f", coeffs[i].f);
                SEGGER_RTT_printf(0, "%s\r\n", buf);
            }
            set_abp_coeff((uint8_t *)coeffs);
        }
    }
    else if (p_evt->type == BLE_NUS_EVT_TX_RDY)
    {

// #define NOT_INSIDE_ISR          (( SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk ) == 0 )
// #define INSIDE_ISR              (!(NOT_INSIDE_ISR))
//         NRF_LOG_INFO("BLE_NUS_EVT_TX_RDY, %s", INSIDE_ISR ? "in_isr" : "not_in_isr");
        xQueueSend(ble_nus_tx_idle, &m_ble_nus_tx_sending, 0);
        m_ble_nus_tx_sending = NULL;

        if (pdTRUE == xQueueReceive(ble_nus_tx_pending, &m_ble_nus_tx_sending, 0))
        {
            ble_nus_data_send(&m_nus, &m_ble_nus_tx_sending->type, &m_ble_nus_tx_sending->len, m_conn_handle);
        }
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        m_ble_nus_tx_running = true;
        SEGGER_RTT_printf(0, "BLE_NUS_EVT_COMM_STARTED\r\n");
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
    {
        m_ble_nus_tx_running = false; // TODO should this be updated when disconnect directly?
        SEGGER_RTT_printf(0, "BLE_NUS_EVT_COMM_STOPPED\r\n");
    }
    else
    {}
}

//uint32_t ble_nus_comm_send(uint8_t *p_data, uint16_t *p_length)
//{
//    int ret_code = 0;
//
//    if (ble_nus_comm_started)
//    {
//        if (m_ble_nus_sending)
//        {
//            ble_nus_outgoing_t outgoing = { .p = p_data, .size = *p_length };
//            if (errQUEUE_FULL == xQueueSend(outgoing_queue, &outgoing, 0))
//            {
//                NRF_LOG_INFO("ble outgoing queue full");
//            }
//        }
//        else
//        {
//            ret_code = ble_nus_data_send(&m_nus, p_data, p_length, m_conn_handle);
//            if (ret_code == NRF_SUCCESS)
//            {
//                NRF_LOG_INFO("nus send succeeded");
//                m_ble_nus_sending = true;
//            }
//            else
//            {
//                NRF_LOG_INFO("nus send failed, %d", ret_code);
//            }
//        }
//    }
//
//    return ret_code;
//}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t          err_code;
    ble_nus_init_t      nus_init;
    nrf_ble_qwr_init_t  qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // biosens_init();
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID; // set nordic template
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    // TODO see ble_app_uart example as reference
    (void)sleep_mode_enter;
    ret_code_t err_code;

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
//    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            SEGGER_RTT_printf(0, "[BLE] Fast advertising.");
            // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            // APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            SEGGER_RTT_printf(0, "[BLE] Slow advertising.");
            // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            // APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            SEGGER_RTT_printf(0, "[BLE] Idle advertising. no connectable advertising ongoing.");
            // sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    // TODO see ble_app_uart as example

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            SEGGER_RTT_printf(0, "[BLE] Connected\r\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            // m_spp.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            SEGGER_RTT_printf(0, "[BLE] Disconnected\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
// bool m_ble_nus_tx_running = false;
// ble_nus_tx_buf_t* m_ble_nus_tx_sending = NULL;
            m_ble_nus_tx_running = false;
            // ble_nus_comm_started = false;
            if (m_ble_nus_tx_sending)
            {
                xQueueSend(ble_nus_tx_idle, &m_ble_nus_tx_sending, 0);
                m_ble_nus_tx_sending = NULL;

                ble_nus_tx_buf_t * buf;
                while (pdTRUE == xQueueReceive(ble_nus_tx_pending, &buf, 0))
                {
                    xQueueSend(ble_nus_tx_idle, &buf, 0);
                }
            }
            // spp_sample_notification_disabled();
            // m_spp.conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            SEGGER_RTT_printf(0, "[BLE] PHY update request.\r\n");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            SEGGER_RTT_printf(0, "[BLE] GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            SEGGER_RTT_printf(0, "[BLE] GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization. */

static void peer_manager_init(void)
{
#if PEER_MANAGER_ENABLED == 1
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
#if PEER_MANAGER_ENABLED == 1
    ret_code_t err_code;
    // NRF_LOG_INFO("Erase bonds!");
    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type                  = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance         = true;
    init.advdata.flags                      = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt    = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids     = m_adv_uuids;

    init.config.ble_adv_fast_enabled        = true;
    init.config.ble_adv_fast_interval       = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout        = APP_ADV_FAST_DURATION;

    init.config.ble_adv_slow_enabled        = true;
    init.config.ble_adv_slow_interval       = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout        = APP_ADV_SLOW_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

/*
 * https://devzone.nordicsemi.com/f/nordic-q-a/95398/nrf52840-correct-freertos-logging-using-nrf_log-module
 */
#if NRF_LOG_ENABLED && NRF_LOG_DEFERRED
 void log_pending_hook( void )
 {

    BaseType_t YieldRequired = pdFAIL;
    if ( __get_IPSR() != 0 )
    {
        YieldRequired = xTaskResumeFromISR( m_logger_thread );
        portYIELD_FROM_ISR( YieldRequired );
    }
    else
    {
        UNUSED_RETURN_VALUE(vTaskResume(m_logger_thread));
    }
 }
#endif

 /**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    // TODO what does this mean?
//    nrf_drv_clock_lfclk_request(NULL); // is this related to initial xPortPendSVHandler crash?
//    while (!nrf_drv_clock_lfclk_is_running())
//    {
//        // just waiting
//    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    bool erase_bonds;

//    err_code = nrf_mem_init();
//    APP_ERROR_CHECK(err_code);

    ble_nus_tx_init();

    // Initialize modules.
    // log_init();

    app_usbd_serial_num_generate();

    err_code = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(err_code);

    // nrfx_systick_init();

    clock_init();

    // err_code = app_timer_init();
    // APP_ERROR_CHECK(err_code);

    nrfx_gpiote_init();

    nrfx_gpiote_out_config_t power_on_pin_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    nrfx_gpiote_out_init(POWER_ON_PIN, &power_on_pin_config);
    nrfx_gpiote_out_config_t oled_rst_pin_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    nrfx_gpiote_out_init(OLED_RST_PIN, &oled_rst_pin_config);

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Configure and initialize the BLE stack.
    ble_stack_init();

    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    // NRF_SDH_BLE_GATT_MAX_MTU_SIZE is definedin sdk_config.h
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(advertising_start, &erase_bonds);

    // app_qma6110p_freertos_init();
    app_oled_freertos_init();
    owuart_freertos_init();
    app_ads1292r_freertos_init();
    app_max86141_freertos_init();
    app_usbcdc_freertos_init();

    SEGGER_RTT_printf(0, "Program started\r\n.");

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

static void ble_nus_tx_init()
{
    // outgoing_queue =xQueueCreate(16, sizeof(ble_nus_outgoing_t));
    ble_nus_tx_idle = xQueueCreate(NUM_OF_BLE_BUF, sizeof(ble_nus_tx_buf_t *));
    ble_nus_tx_pending = xQueueCreate(NUM_OF_BLE_BUF, sizeof(ble_nus_tx_buf_t *));

    for (int i = 0; i < NUM_OF_BLE_BUF; i++)
    {
        ble_nus_tx_buf_t *buf = &ble_nus_tx_buffer[i];
        xQueueSend(ble_nus_tx_idle, &buf, 0);
    }

//    NRF_LOG_INFO("ble_nus_tx_idle waiting %d", uxQueueMessagesWaiting(ble_nus_tx_idle));
//    ble_nus_tx_buf_t* buf = NULL;
//    uint8_t x = xQueueReceive(ble_nus_tx_idle, &buf, 0);
//    NRF_LOG_INFO("dequeue %d, %p", x, buf);
}

bool ble_nus_tx_running()
{
    return m_ble_nus_tx_running;
}

void ble_nus_tx_send(ble_nus_tx_buf_t *buf)
{
    APP_ERROR_CHECK_BOOL(m_ble_nus_tx_running == true);

    if (m_ble_nus_tx_sending)
    {
        // put into pending queue
        xQueueSend(ble_nus_tx_pending, &buf, 0);
    }
    else
    {
        m_ble_nus_tx_sending = buf;
        int ret = ble_nus_data_send(&m_nus, &buf->type, &buf->len, m_conn_handle);
//        if (ret == NRF_SUCCESS)
//        {
//            NRF_LOG_INFO("sent");
//        }
//        else
//        {
//            NRF_LOG_INFO("sent failed");
//        }
    }
}

ble_nus_tx_buf_t* ble_nus_tx_alloc(void)
{
    ble_nus_tx_buf_t* buf = NULL;
    if (pdTRUE == xQueueReceive(ble_nus_tx_idle, &buf, 0))
    {
        APP_ERROR_CHECK_BOOL(buf != NULL);
        return buf;
    }
    else
    {
        SEGGER_RTT_printf(0, "ble_nus_tx_alloc, xQueueReceive failed\r\n");
        return NULL;
    }
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
{
    volatile int i = 0;
    for (;;)
    {
        i++;
    }
}

/*lint -save -e14 */
/**
 * Function is implemented as weak so that it can be overwritten by custom application error handler
 * when needed.
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    __disable_irq();

#ifndef DEBUG
    SEGGER_RTT_printf(0, "Fatal error\r\n");
#else
    switch (id)
    {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
        case NRF_FAULT_ID_SD_ASSERT:
            SEGGER_RTT_printf(0, "SOFTDEVICE: ASSERTION FAILED\r\n");
            break;
        case NRF_FAULT_ID_APP_MEMACC:
            SEGGER_RTT_printf(0, "SOFTDEVICE: INVALID MEMORY ACCESS\r\n");
            break;
#endif
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            assert_info_t * p_info = (assert_info_t *)info;
            SEGGER_RTT_printf(0, "ASSERTION FAILED at %s:%u\r\n",
                              p_info->p_file_name,
                              p_info->line_num);
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t * p_info = (error_info_t *)info;
            SEGGER_RTT_printf(0, "ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x\r\n",
                              p_info->err_code,
                              "[noname]", // nrf_strerror_get(p_info->err_code),
                              p_info->p_file_name,
                              p_info->line_num,
                              pc);
            SEGGER_RTT_printf(0, "End of error report\r\n");
            break;
        }
        default:
            SEGGER_RTT_printf(0, "UNKNOWN FAULT at 0x%08X\r\n", pc);
            break;
    }
#endif

    NRF_BREAKPOINT_COND;
    // On assert, the system can only recover with a reset.

#ifndef DEBUG
    SEGGER_RTT_printf(0, "System reset\r\n");
    NVIC_SystemReset();
#else
    app_error_save_and_stop(id, pc, info);
#endif // DEBUG
}
