/**
 * Copyright (c) 2012 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * https://devzone.nordicsemi.com/guides/short-range-guides/b/bluetooth-low-energy/posts/ble-services-a-beginners-tutorial
 * https://github.com/bjornspockeli/custom_ble_service_example
 */


/** @file
 *
 * @defgroup ble_bas EEG Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief EEG Service module.
 *
 * @details This module implements the EEG Service with the Battery Level characteristic.
 *          During initialization it adds the EEG Service and Battery Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the EEG Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_biosens_battery_level_update() function.
 *          If an event handler is supplied by the application, the EEG Service will
 *          generate EEG Service events to the application.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_biosens_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_BIOSENS_BLE_OBSERVER_PRIO,
 *                                   ble_biosens_on_ble_evt, &instance);
 *          @endcode
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */
#ifndef BLE_BIOSENS_H__
#define BLE_BIOSENS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

// base uuid 9be60000-929b-4a95-bf8d-c012badc07f7
#define BLE_BIOSENS_SERVICE_UUID_BASE \
  {0xf7, 0x07, 0xdc, 0xba, 0x12, 0xc0, 0x8d, 0xbf, 0x95, 0x4a, 0x9b, 0x92, 0x00, 0x00, 0xe6, 0x9b}

#define BLE_BIOSENS_SERVICE_UUID            0x0000
#define BLE_BIOSENS_CHAR_SAMPLE_UUID        0x0001
#define BLE_BIOSENS_CHAR_SPS_UUID           0x0002
#define BLE_BIOSENS_CHAR_GAIN_UUID          0x0003
#define BLE_BIOSENS_CHAR_STIM_UUID          0x0004


/**@brief Macro for defining a ble_bas instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_BIOSENS_DEF(_name)                          \
    static ble_biosens_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                 \
                         BLE_BIOSENS_BLE_OBSERVER_PRIO, \
                         ble_biosens_on_ble_evt,        \
                         &_name)

/**@brief EEG Service event type. */
typedef enum
{
    BLE_BIOSENS_EVT_NOTIFICATION_ENABLED, /**< Battery value notification enabled event. */
    BLE_BIOSENS_EVT_NOTIFICATION_DISABLED /**< Battery value notification disabled event. */
} ble_biosens_evt_type_t;

/**@brief EEG Service event. */
typedef struct
{
    ble_biosens_evt_type_t evt_type;    /**< Type of event. */
    uint16_t           conn_handle; /**< Connection handle. */
} ble_biosens_evt_t;

// Forward declaration of the ble_biosens_t type.
typedef struct ble_biosens_s ble_biosens_t;

/**@brief EEG Service event handler type. */
typedef void (* ble_biosens_evt_handler_t) (ble_biosens_t * p_eeg, ble_biosens_evt_t * p_evt);
typedef void (* ble_biosens_sample_notification_enable_disable_t) ();
typedef void (* ble_biosens_sps_write_handler_t) (uint16_t conn_handle, ble_biosens_t * p_eeg, uint8_t new_sps);
typedef void (* ble_biosens_gain_write_handler_t) (uint16_t conn_handle, ble_biosens_t * p_eeg, uint8_t new_gain);
typedef void (* ble_biosens_stim_write_handler_t) (uint16_t conn_handle, ble_biosens_t * p_eeg, uint8_t new_stim);

/**@brief EEG Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_biosens_evt_handler_t   evt_handler;                /**< Event handler to be called for handling events in the EEG Service. */
//  bool                    support_notification;       /**< TRUE if notification of Battery Level measurement is supported. */

//  only for HID in original ble_bas
//  ble_srv_report_ref_t *  p_report_ref;               /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */

//  uint8_t                 initial_batt_level;         /**< Initial battery level */
//  security_req_t          bl_rd_sec;                  /**< Security requirement for reading the BL characteristic value. */
//  security_req_t          bl_cccd_wr_sec;             /**< Security requirement for writing the BL characteristic CCCD. */
//  security_req_t          bl_report_rd_sec;           /**< Security requirement for reading the BL characteristic descriptor. */

    uint8_t                 initial_sps;                // 0x00 for 100sps, 01 for 200sps (default)
    uint8_t                 initial_gain;               // 0x00 to 0x07 TODO
    uint8_t                 initial_stim;               // 0x00

    ble_biosens_sample_notification_enable_disable_t        sample_notification_enabled;
    ble_biosens_sample_notification_enable_disable_t        sample_notification_disabled;

    ble_biosens_sps_write_handler_t                         sps_write_handler;
    ble_biosens_gain_write_handler_t                        gain_write_handler;
    ble_biosens_stim_write_handler_t                        stim_write_handler;
} ble_biosens_init_t;

/**@brief EEG Service structure. This contains various status information for the service. */
struct ble_biosens_s
{
    ble_biosens_evt_handler_t     evt_handler;              // Event handler to be called for handling events in the EEG Service.
    uint16_t                  service_handle;           // Handle of EEG Service (as provided by the BLE stack).
    ble_gatts_char_handles_t  sps_handles;              // Handles related to the SPS characteristic.
    ble_gatts_char_handles_t  gain_handles;             // Handles related to the Gain characteristic.
    ble_gatts_char_handles_t  samples_handles;          // Handles related to the Samples (Data) characteristic.
    ble_gatts_char_handles_t  stim_handles;             // Handles related to the Stim characteristic.
    uint8_t                   samples_last;             // Last Battery Level measurement passed to the EEG Service.

    uint8_t                   uuid_type;
    uint8_t                   sps_shadow;
    uint8_t                   gain_shadow;

    uint16_t                  conn_handle;
    bool                      sample_notifying;

    ble_biosens_sample_notification_enable_disable_t        sample_notification_enabled;
    ble_biosens_sample_notification_enable_disable_t        sample_notification_disabled;
    ble_biosens_sps_write_handler_t                         sps_write_handler;
    ble_biosens_gain_write_handler_t                        gain_write_handler;
    ble_biosens_stim_write_handler_t                        stim_write_handler;
};

uint8_t biosens_get_sps(void);
uint8_t biosens_get_gain(void);

void biosens_set_stim(uint8_t new_value);
uint8_t biosens_get_stim(void);


/**@brief Function for initializing the EEG Service.
 *
 * @param[out]  p_eeg       EEG Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_biosens_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
ret_code_t ble_biosens_init(ble_biosens_t * p_eeg, const ble_biosens_init_t * p_biosens_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the EEG Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_biosens_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   EEG Service structure.
 */
void ble_biosens_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for updating the battery level.
 *
 * @details The application calls this function after having performed a battery measurement.
 *          The battery level characteristic will only be sent to the clients which have
 *          enabled notifications. \ref BLE_CONN_HANDLE_ALL can be used as a connection handle
 *          to send notifications to all connected devices.
 *
 * @param[in]   p_eeg          EEG Service structure.
 * @param[in]   battery_level  New battery measurement value (in percent of full capacity).
 * @param[in]   conn_handle    Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t ble_biosens_battery_level_update(ble_biosens_t * p_eeg,
                                        uint8_t     battery_level,
                                        uint16_t    conn_handle);


/**@brief Function for sending the last battery level when bonded client reconnects.
 *
 * @details The application calls this function, in the case of a reconnection of
 *          a bonded client if the value of the battery has changed since
 *          its disconnection.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       this function must be called upon reconnection if the battery level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_eeg          EEG Service structure.
 * @param[in]   conn_handle    Connection handle.
 *
 * @return      NRF_SUCCESS on success,
 *              NRF_ERROR_INVALID_STATE when notification is not supported,
 *              otherwise an error code returned by @ref sd_ble_gatts_hvx.
 */
ret_code_t ble_biosens_battery_lvl_on_reconnection_update(ble_biosens_t * p_eeg,
                                                      uint16_t    conn_handle);


#ifdef __cplusplus
}
#endif

#endif // BLE_BIOSENS_H__

/** @} */
