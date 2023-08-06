#ifndef __SENS_PROTO_H__
#define __SENS_PROTO_H__

#include <stdbool.h>
#include <stdint.h>
#include "app_util.h"

#define SENS_DATA_PACKET_TYPE           0x0101

#define SENS_PREAMBLE_SIZE              8
#define SENS_BRIEF_TLV_TYPE             0xff

#define ADS129X_BRIEF_TLV_LEN           7
#define ADS129X_BRIEF_TLV_SIZE          (3 + ADS129X_BRIEF_TLV_LEN)
#define ADS129X_SAMPLE_NUM              50
#define ADS129X_SENSOR_ID               2

#define ADS129X_REG_TLV_TYPE            0x00
#define ADS129X_REG_TLV_LEN             12
#define ADS129X_REG_TLV_SIZE            (3 + ADS129X_REG_TLV_LEN)

#define ADS129X_SAMPLE_TLV_TYPE         0x10
#define ADS129X_SAMPLE_TLV_LEN          (ADS129X_SAMPLE_NUM * 9)
#define ADS129X_HAS_ROG_ECG             1
#define ADS129x_ROG_ECG_TLV_TYPE        0x80
#define ADS129X_ROG_ECG_TLV_LEN         (ADS129X_SAMPLE_NUM * 2)
#define ADS129X_HAS_ROG_RESP            1
#define ADS129X_ROG_RESP_TLV_TYPE       0x81
#define ADS129X_ROG_RESP_TLV_LEN        ((ADS129X_SAMPLE_NUM / 10) * 8) // dual uint32_t
#define ADS129X_HAS_ROG_INTERNAL        0

#define ADS129X_USE_ROG_LIB             (!!ADS129X_HAS_ROG_ECG || !!ADS129X_HAS_ROG_RESP || !!ADS129X_HAS_ROG_INTERNAL)

typedef __packed struct tlv
{
    uint8_t type;
    uint16_t length;
    uint8_t value[1];
} tlv_t;

typedef __packed struct sens_packet
{
    uint8_t preamble[8];
    uint16_t type;
    uint16_t length;
    uint8_t payload_crc[1];
} sens_packet_t;

typedef __packed struct ads129x_global_tlv
{
    uint8_t type;
    uint16_t length;
    uint16_t sensor_id;
    uint8_t instance_id;
    uint8_t version;
    uint8_t sample_num;
    uint8_t heart_rate;
    uint8_t resp_rate;
} ads129x_brief_tlv_t;

STATIC_ASSERT(sizeof(ads129x_brief_tlv_t) == ADS129X_BRIEF_TLV_SIZE);

typedef __packed struct ads129x_reg_tlv
{
    uint8_t type;
    uint16_t length;
    uint8_t value[ADS129X_REG_TLV_LEN];
} ads129x_reg_tlv_t;

STATIC_ASSERT(sizeof(ads129x_reg_tlv_t) == ADS129X_REG_TLV_SIZE);

// this is best used as static
typedef struct ads129x_packet_helper
{
    uint16_t payload_len;
    uint16_t packet_size;
    uint16_t sample_count;
   
    // tlvs
    ads129x_brief_tlv_t     *p_brief;
    ads129x_reg_tlv_t       *p_reg;
    tlv_t                   *p_sample;
    tlv_t                   *p_rog_ecg;
    tlv_t                   *p_rog_resp;
    tlv_t                   *p_rog_internal;
    uint8_t                 *p_crc;
} ads129x_packet_helper_t;

void sens_init_ads129x_packet(ads129x_packet_helper_t * p_helper, sens_packet_t * p_pkt);

#endif
