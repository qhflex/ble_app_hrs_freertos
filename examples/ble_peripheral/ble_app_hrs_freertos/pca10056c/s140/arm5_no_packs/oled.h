#ifndef __OLED_H_
#define __OLED_H_

#include <stdint.h>

void app_oled_freertos_init(void);

void oled_update_ecg(uint8_t hr);
void oled_update_spo(int spo);
void oled_update_abp(float sbp, float dbp);
void oled_update_tmp(float tmp);

#endif
