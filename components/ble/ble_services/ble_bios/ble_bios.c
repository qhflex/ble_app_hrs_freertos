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
/* Attention!
 * To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 * qualification listings, this section of source code must not be modified.
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_BIOSENS)
#include "ble_bios.h"
#include <string.h>
#include "ble_srv_common.h"
#include "ble_conn_state.h"
#include "app_error.h"

#define NRF_LOG_MODULE_NAME ble_ecg_eeg
#if BLE_EEG_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_EEG_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_EEG_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_EEG_CONFIG_DEBUG_COLOR
#else // BLE_BAS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       NRF_LOG_DEFAULT_LEVEL
#endif // BLE_BAS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#if 0
/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_lbs_t * p_lbs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (   (p_evt_write->handle == p_lbs->led_char_handles.value_handle)
        && (p_evt_write->len == 1)
        && (p_lbs->led_write_handler != NULL))
    {
        p_lbs->led_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_lbs, p_evt_write->data[0]);
    }
}

#endif

uint8_t biosens_get_gain()
{
    return 0; // not implemented yet
}

void biosens_set_gain(uint8_t gain)
{
}

uint8_t biosens_get_sps() {
    return 0; // not implemented yet
}

void biosens_set_sps(uint8_t sps) {
}



static void on_write(ble_biosens_t * p_eeg, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // notification cccd
    if (    (p_evt_write->handle == p_eeg->samples_handles.cccd_handle)
        &&  (p_evt_write->len == 2))  // debug confirmed this is 2, but why?
    {
        if (p_eeg->evt_handler == NULL)
        {
          return;
        }

        if (  ble_srv_is_notification_enabled(p_evt_write->data))
        {
          if (p_eeg->sample_notification_enabled != NULL)
          {
            p_eeg->sample_notification_enabled();
          }
        }
        else
        {
          if (p_eeg->sample_notification_disabled != NULL)
          {
            p_eeg->sample_notification_disabled();
          }
        }
    }

    // sps
    if (    (p_evt_write->handle == p_eeg->sps_handles.value_handle)
        &&  (p_evt_write->len == 1)
        &&  (p_eeg->sps_write_handler != NULL))
    {
        p_eeg->sps_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_eeg, p_evt_write->data[0]);
    }

    // gain
    if (    (p_evt_write->handle == p_eeg->gain_handles.value_handle)
        &&  (p_evt_write->len == 1)
        &&  (p_eeg->gain_write_handler != NULL))
    {
        p_eeg->gain_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_eeg, p_evt_write->data[0]);
    }

    // stim
    if (    (p_evt_write->handle == p_eeg->stim_handles.value_handle)
        &&  (p_evt_write->len == 1)
        &&  (p_eeg->stim_write_handler != NULL))
    {
        p_eeg->stim_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_eeg, p_evt_write->data[0]);
    }

//    if (    (p_evt_write->handle == p_bas->battery_level_handles.cccd_handle)
//        &&  (p_evt_write->len == 2))
//    {
//        if (p_bas->evt_handler == NULL)
//        {
//            return;
//        }

//        ble_bas_evt_t evt;

//        if (ble_srv_is_notification_enabled(p_evt_write->data))
//        {
//            evt.evt_type = BLE_BAS_EVT_NOTIFICATION_ENABLED;
//        }
//        else
//        {
//            evt.evt_type = BLE_BAS_EVT_NOTIFICATION_DISABLED;
//        }
//        evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;

//        // CCCD written, call application event handler.
//        p_bas->evt_handler(p_bas, &evt);
//    }
}

void ble_biosens_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_biosens_t * p_eeg = (ble_biosens_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_eeg, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

#if 0
typedef struct
{
    uint16_t               max_size;                      /**< Maximum size of the user descriptor*/
    uint16_t               size;                          /**< Size of the user descriptor*/
    uint8_t                *p_char_user_desc;             /**< User descriptor content, pointer to a UTF-8 encoded string (non-NULL terminated)*/
    bool                   is_var_len;                    /**< Indicates if the user descriptor has variable length.*/
    ble_gatt_char_props_t  char_props;                    /**< user descriptor properties.*/
    bool                   is_defered_read;               /**< Indicate if deferred read operations are supported.*/
    bool                   is_defered_write;              /**< Indicate if deferred write operations are supported.*/
    security_req_t         read_access;                   /**< Security requirement for reading the user descriptor.*/
    security_req_t         write_access;                  /**< Security requirement for writing the user descriptor.*/
    bool                   is_value_user;                 /**< Indicate if the content of the characteristic is to be stored in the application (user) or in the stack.*/
}ble_add_char_user_desc_t;
#endif

static ret_code_t sample_char_add(ble_biosens_t * p_eeg, const ble_biosens_init_t * p_biosens_init)
{
    ret_code_t                        err_code;
    ble_add_char_user_desc_t          user_desc;
    ble_add_char_params_t             add_char_params;
    uint8_t                           user_desc_str[] = "Sample";

    memset(&user_desc, 0, sizeof(user_desc));
    user_desc.is_var_len              = false;
    user_desc.char_props.read         = 1;
    user_desc.size                    = (sizeof(user_desc_str) - 1);
    user_desc.max_size                = (sizeof(user_desc_str) - 1);
    user_desc.p_char_user_desc        = user_desc_str;
    user_desc.read_access             = SEC_OPEN;
    // user_desc.write_access            = SEC_OPEN;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_BIOSENS_CHAR_SAMPLE_UUID;
    add_char_params.uuid_type         = p_eeg->uuid_type;
    add_char_params.max_len           = NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3; //
    add_char_params.init_len          = 0;    // this is OK
    add_char_params.p_init_value      = NULL;
    add_char_params.is_var_len        = true; //
    // add_char_params.is_value_user     = BLE_GATTS_VLOC_INVALID; // notification only
    // add_char_params.char_props.read   = 1;
    // add_char_params.char_props.write  = 1;
    add_char_params.char_props.notify = 1;
    // add_char_params.read_access       = 1;
    // add_char_params.write_access      = 1;
    add_char_params.cccd_write_access = SEC_OPEN;
    add_char_params.p_user_descr      = &user_desc;

    err_code = characteristic_add(p_eeg->service_handle,
                                  &add_char_params,
                                  &(p_eeg->samples_handles));

    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

static ret_code_t sps_char_add(ble_biosens_t * p_eeg, const ble_biosens_init_t * p_biosens_init)
{
    ret_code_t                  err_code;
    ble_add_char_user_desc_t    user_desc;
    ble_add_char_params_t       add_char_params;
    uint8_t                     user_desc_str[] = "SPS";
    uint8_t                     initial_sps = p_biosens_init->initial_sps;

    memset(&user_desc, 0, sizeof(user_desc));
    user_desc.is_var_len              = false;
    user_desc.char_props.read         = 1;
    user_desc.size                    = (sizeof(user_desc_str) - 1);
    user_desc.max_size                = (sizeof(user_desc_str) - 1);
    user_desc.p_char_user_desc        = user_desc_str;
    user_desc.read_access             = SEC_OPEN;
    // user_desc.write_access            = SEC_OPEN;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_BIOSENS_CHAR_SPS_UUID;
    add_char_params.uuid_type         = p_eeg->uuid_type;
    add_char_params.max_len           = sizeof(uint8_t);
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.p_init_value      = &initial_sps;
    add_char_params.is_var_len        = false;
    add_char_params.is_value_user     = false;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;

    // if user_desc.write_access is set to SEC_OPEN then this must be set
    // if user_desc.write_access is set to SEC_NO_ACCESS then this must NOT be set
    // https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.s132.api.v7.3.0/group___b_l_e___g_a_t_t_s___f_u_n_c_t_i_o_n_s.html
    // add_char_params.char_ext_props.wr_aux = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.p_user_descr      = &user_desc;

    err_code = characteristic_add(p_eeg->service_handle,
                                  &add_char_params,
                                  &p_eeg->sps_handles);
    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

static ret_code_t gain_char_add(ble_biosens_t * p_eeg, const ble_biosens_init_t * p_biosens_init)
{
    ret_code_t                  err_code;
    ble_add_char_user_desc_t    user_desc;
    ble_add_char_params_t       add_char_params;
    uint8_t                     user_desc_str[] = "Gain";
    uint8_t                     initial_gain = p_biosens_init->initial_gain;

    memset(&user_desc, 0, sizeof(user_desc));
    user_desc.is_var_len              = false;
    user_desc.char_props.read         = 1;
    user_desc.size                    = (sizeof(user_desc_str) - 1);
    user_desc.max_size                = (sizeof(user_desc_str) - 1);
    user_desc.p_char_user_desc        = user_desc_str;
    user_desc.read_access             = SEC_OPEN;
    // user_desc.write_access            = SEC_OPEN;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_BIOSENS_CHAR_GAIN_UUID;
    add_char_params.uuid_type         = p_eeg->uuid_type;
    add_char_params.max_len           = sizeof(uint8_t);
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.p_init_value      = (uint8_t*)&initial_gain;
    add_char_params.is_var_len        = false;
    // add_char_params.is_value_user     = BLE_GATTS_VLOC_STACK;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.p_user_descr      = &user_desc;

    err_code = characteristic_add(p_eeg->service_handle,
                                  &add_char_params,
                                  &p_eeg->gain_handles);
    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

static uint8_t stim_value;

void biosens_set_stim(uint8_t new_value)
{
    stim_value = new_value;
}

uint8_t biosens_get_stim()
{
    return stim_value;
}

static ret_code_t stim_char_add(ble_biosens_t * p_eeg, const ble_biosens_init_t * p_biosens_init)
{
    ret_code_t                          err_code;
    ble_add_char_user_desc_t            user_desc;
    ble_add_char_params_t               add_char_params;
    uint8_t                             user_desc_str[] = "STIM";

    stim_value = p_biosens_init->initial_stim;

    memset(&user_desc, 0, sizeof(user_desc));
    user_desc.is_var_len                = false;
    user_desc.char_props.read           = 1;
    user_desc.size                      = (sizeof(user_desc_str) - 1);
    user_desc.max_size                  = (sizeof(user_desc_str) - 1);
    user_desc.p_char_user_desc          = user_desc_str;
    user_desc.read_access               = SEC_OPEN;
    // user_desc.write_access            = SEC_OPEN;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_BIOSENS_CHAR_STIM_UUID;
    add_char_params.uuid_type           = p_eeg->uuid_type;
    add_char_params.max_len             = sizeof(uint8_t);
    add_char_params.init_len            = sizeof(uint8_t);
    add_char_params.p_init_value        = &stim_value;
    add_char_params.is_var_len          = false;
    add_char_params.is_value_user       = true;
    add_char_params.char_props.read     = 1;
    add_char_params.char_props.write    = 1;

    // if user_desc.write_access is set to SEC_OPEN then this must be set
    // if user_desc.write_access is set to SEC_NO_ACCESS then this must NOT be set
    // https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.s132.api.v7.3.0/group___b_l_e___g_a_t_t_s___f_u_n_c_t_i_o_n_s.html
    // add_char_params.char_ext_props.wr_aux = 1;

    add_char_params.read_access         = SEC_OPEN;
    add_char_params.write_access        = SEC_OPEN;
    add_char_params.p_user_descr        = &user_desc;

    err_code = characteristic_add(p_eeg->service_handle,
                                  &add_char_params,
                                  &p_eeg->stim_handles);
    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

ret_code_t ble_biosens_init(ble_biosens_t * p_eeg, const ble_biosens_init_t * p_biosens_init)
{
  if (p_eeg == NULL || p_biosens_init == NULL)
  {
    return NRF_ERROR_NULL;
  }

  ret_code_t err_code;
  ble_uuid_t ble_uuid;

  // Initialize service structure
  // p_cus->conn_handle                 = BLE_CONN_HANDLE_INVALID;

  p_eeg->evt_handler                    = p_biosens_init->evt_handler;
  p_eeg->sps_write_handler              = p_biosens_init->sps_write_handler;
  p_eeg->gain_write_handler             = p_biosens_init->gain_write_handler;
  p_eeg->stim_write_handler             = p_biosens_init->stim_write_handler;
  p_eeg->sample_notification_enabled    = p_biosens_init->sample_notification_enabled;
  p_eeg->sample_notification_disabled   = p_biosens_init->sample_notification_disabled;
  p_eeg->conn_handle                    = BLE_CONN_HANDLE_INVALID;
  p_eeg->sample_notifying               = false;

  p_eeg->sps_shadow                     = p_biosens_init->initial_sps;
  p_eeg->gain_shadow                    = p_biosens_init->initial_gain;

  ble_uuid128_t base_uuid = {BLE_BIOSENS_SERVICE_UUID_BASE};

  // this is a service call (SVCALL) defined and documented in ble.h
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_eeg->uuid_type); // BLE_UUID_TYPE_VENDOR_BEGIN  expected
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_eeg->uuid_type;
  ble_uuid.uuid = BLE_BIOSENS_SERVICE_UUID;

  // Add the Custom Service
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_eeg->service_handle);
  if (err_code != NRF_SUCCESS)
  {
      return err_code;
  }

  err_code = sample_char_add(p_eeg, p_biosens_init);
  if (err_code != NRF_SUCCESS)
  {
      return err_code;
  }

  err_code = sps_char_add(p_eeg, p_biosens_init);
  if (err_code != NRF_SUCCESS)
  {
      return err_code;
  }

  err_code = gain_char_add(p_eeg, p_biosens_init);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  return stim_char_add(p_eeg, p_biosens_init);
}

uint32_t ble_biosens_sample_value_update(ble_biosens_t * p_eeg, uint8_t * p_value)
{
  return NRF_SUCCESS;
}


#endif // NRF_MODULE_ENABLED(BLE_EEG)
