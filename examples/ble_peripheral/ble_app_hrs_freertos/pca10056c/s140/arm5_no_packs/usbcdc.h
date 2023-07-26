#ifndef APP_USBCDC_H
#define APP_USBCDC_H

#include <stdbool.h>
#include <stdint.h>

typedef __packed struct spo2_sample {
    uint8_t byte[6];
} spo2_sample_t;

bool cdc_acm_running(void);
void rougu_enqueue(spo2_sample_t * p_smpl);

void app_usbcdc_freertos_init(void);

#endif
