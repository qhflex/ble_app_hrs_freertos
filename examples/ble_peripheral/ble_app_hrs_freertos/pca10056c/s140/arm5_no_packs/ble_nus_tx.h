#ifndef BLE_NUS_TX_H
#define BLE_NUS_TX_H

// #ifdef BLE_NUS_ENABLED
// extern bool ble_nus_comm_started;                   // event name in NUS BLE_NUS_EVT_COMM_STARTED
#include <stdbool.h>
#include <stdint.h>

typedef struct ble_nus_tx_buf
{
    uint16_t len;
    uint8_t type;
    uint8_t seq;
    uint8_t data[242];
} ble_nus_tx_buf_t;

// true if running
bool ble_nus_tx_running(void);

// send a buffer
void ble_nus_tx_send(ble_nus_tx_buf_t *buf);

// return a buffer, or null
ble_nus_tx_buf_t* ble_nus_tx_alloc(void);
void ble_nus_tx_free(ble_nus_tx_buf_t* buf);

#endif
