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
 * @defgroup ble_bas SPP Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief SPP Service module.
 *
 * @details This module implements the SPP Service with the Battery Level characteristic.
 *          During initialization it adds the SPP Service and Battery Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the SPP Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_eeg_battery_level_update() function.
 *          If an event handler is supplied by the application, the SPP Service will
 *          generate SPP Service events to the application.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_eeg_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_SPP_BLE_OBSERVER_PRIO,
 *                                   ble_eeg_on_ble_evt, &instance);
 *          @endcode
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */
#ifndef BLE_SPP_H__
#define BLE_SPP_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

// base uuid 9be60000-929b-4a95-bf8d-c012badc07f7
// base uuid 64290000-af93-49fe-9aff-9786c0229a18
#define BLE_SPP_SERVICE_UUID_BASE \
  {0x18, 0x9a, 0x22, 0xc0, 0x86, 0x97, 0xff, 0x9a, 0xfe, 0x49, 0x93, 0xaf, 0x00, 0x00, 0x29, 0x64}
  
#define BLE_SPP_SERVICE_UUID            0x0000
#define BLE_SPP_CHAR_DAT_UUID           0x0001  
#define BLE_SPP_CHAR_CMD_UUID           0x0002


/**@brief Macro for defining a ble_bas instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_SPP_DEF(_name)                          \
    static ble_spp_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,             \
                         BLE_SPP_BLE_OBSERVER_PRIO, \
                         ble_spp_on_ble_evt,        \
                         &_name)

/**@brief SPP Service event type. */
typedef enum
{
    BLE_SPP_EVT_NOTIFICATION_ENABLED, /**< Battery value notification enabled event. */
    BLE_SPP_EVT_NOTIFICATION_DISABLED /**< Battery value notification disabled event. */
} ble_spp_evt_type_t;

/**@brief SPP Service event. */
typedef struct
{
    ble_spp_evt_type_t evt_type;    /**< Type of event. */
    uint16_t           conn_handle; /**< Connection handle. */
} ble_spp_evt_t;

// Forward declaration of the ble_spp_t type.
typedef struct ble_spp_s ble_spp_t;

/**@brief SPP Service event handler type. */
typedef void (* ble_spp_evt_handler_t) (ble_spp_t * p_spp, ble_spp_evt_t * p_evt);
typedef void (* ble_spp_sample_notification_enable_disable_t) ();
typedef void (* ble_spp_command_handler_t) (uint16_t conn_handle, ble_spp_t * p_spp, uint8_t new_sps);

/**@brief SPP Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_spp_evt_handler_t  evt_handler;                 /**< Event handler to be called for handling events in the SPP Service. */
  
    uint8_t                 initial_sps;                // 0x00 for 100sps, 01 for 200sps (default)
  
    ble_spp_sample_notification_enable_disable_t sample_notification_enabled;
    ble_spp_sample_notification_enable_disable_t sample_notification_disabled;
    ble_spp_command_handler_t   command_handler;
} ble_spp_init_t;

/**@brief SPP Service structure. This contains various status information for the service. */
struct ble_spp_s
{
    ble_spp_evt_handler_t     evt_handler;               // Event handler to be called for handling events in the SPP Service.
    uint16_t                  service_handle;            // Handle of SPP Service (as provided by the BLE stack).
    ble_gatts_char_handles_t  sps_handles;               // Handles related to the SPS characteristic.
    ble_gatts_char_handles_t  samples_handles;           // Handles related to the Samples (Data) characteristic.
    uint8_t                   samples_last;              // Last Battery Level measurement passed to the SPP Service.
    uint8_t                   uuid_type;
    uint8_t                   sps_shadow;

    uint16_t                  conn_handle;
    bool                      sample_notifying;
  
    ble_spp_sample_notification_enable_disable_t sample_notification_enabled;
    ble_spp_sample_notification_enable_disable_t sample_notification_disabled;  
    ble_spp_command_handler_t   command_handler;
};


/**@brief Function for initializing the SPP Service.
 *
 * @param[out]  p_spp       SPP Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_spp_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
ret_code_t ble_spp_init(ble_spp_t * p_spp, const ble_spp_init_t * p_spp_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the SPP Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_spp_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   SPP Service structure.
 */
void ble_spp_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


#ifdef __cplusplus
}
#endif

#endif // BLE_SPP_H__

/** @} */
