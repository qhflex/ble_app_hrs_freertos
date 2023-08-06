#ifndef APP_USBCDC_H
#define APP_USBCDC_H

#include <stdbool.h>
#include <stdint.h>

#if defined MIMIC_ROUGU && MIMIC_ROUGU == 1
typedef __packed struct spo2_sample {
    uint8_t byte[6];
} spo2_sample_t;

void rougu_enqueue(spo2_sample_t * p_smpl);

#else 

void cdc_acm_send_packet(uint8_t * p_packet, uint32_t size);
    
#endif 

bool cdc_acm_port_open(void);
void app_usbcdc_freertos_init(void);

#endif
