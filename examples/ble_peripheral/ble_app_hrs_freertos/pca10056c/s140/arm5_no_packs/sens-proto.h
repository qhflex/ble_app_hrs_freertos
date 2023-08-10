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
#define ADS129X_NUM_OF_SAMPLES          50
#define ADS129X_SENSOR_ID               2

#define ADS129X_REG_TLV_TYPE            0x00
#define ADS129X_REG_TLV_LEN             12
#define ADS129X_REG_TLV_SIZE            (3 + ADS129X_REG_TLV_LEN)

#define ADS129X_SAMPLE_TLV_TYPE         0x10
#define ADS129X_SAMPLE_TLV_LEN          (ADS129X_NUM_OF_SAMPLES * 9)
#define ADS129X_HAS_ROG_ECG             1
#define ADS129x_ROG_ECG_TLV_TYPE        0x80
#define ADS129X_ROG_ECG_TLV_LEN         (ADS129X_NUM_OF_SAMPLES * 2)
#define ADS129X_HAS_ROG_RESP            1
#define ADS129X_ROG_RESP_TLV_TYPE       0x81
#define ADS129X_ROG_RESP_TLV_LEN        ((ADS129X_NUM_OF_SAMPLES / 10) * 8) // dual uint32_t
#define ADS129X_HAS_ROG_INTERNAL        0

#define ADS129X_USE_ROG_LIB             (!!ADS129X_HAS_ROG_ECG || !!ADS129X_HAS_ROG_RESP || !!ADS129X_HAS_ROG_INTERNAL)

#define MAX86141_BRIEF_TLV_LEN          7
#define MAX86141_BRIEF_TLV_SIZE         (3 + MAX86141_BRIEF_TLV_LEN)
#define MAX86141_NUM_OF_SAMPLES         60      // It is possible but not necessarily to change this value.
                                                // For maximum number of spi tx bytes is 255. 60 * 3 plus 2 leading characters is 182,
                                                // which is large enough.
#define MAX86141_SENSOR_ID              1

#define MAX86141_SAMPLE_TLV_TYPE        0x08
#define MAX86141_SAMPLE_TLV_LEN         (MAX86141_NUM_OF_SAMPLES * 3)
#define MAX86141_SAMPLE_TLV_SIZE        (3 + MAX86141_SAMPLE_TLV_LEN)

#define MAX86141_PPGCFG_TLV_TYPE        0x10
#define MAX86141_PPGCFG_TLV_LEN         7
#define MAX86141_PPGCFG_TLV_SIZE        (3 + MAX86141_PPGCFG_TLV_LEN)

#define MAX86141_LEDCFG_TLV_TYPE        0x20
#define MAX86141_LEDCFG_TLV_LEN         12
#define MAX86141_LEDCFG_TLV_SIZE        (3 + MAX86141_LEDCFG_TLV_LEN)

typedef struct __attribute__((packed)) tlv
{
    uint8_t type;
    uint16_t length;
    uint8_t value[1];
} tlv_t;

typedef struct __attribute__((packed)) sens_packet
{
    uint8_t preamble[8];
    uint16_t type;
    uint16_t length;
    uint8_t payload_crc[1];
} sens_packet_t;

typedef struct __attribute__((packed)) ads129x_brief_tlv
{
    uint8_t type;
    uint16_t length;
    uint16_t sensor_id;
    uint8_t instance_id;
    uint8_t version;
    uint8_t num_of_samples;
    uint8_t heart_rate;
    uint8_t resp_rate;
} ads129x_brief_tlv_t;

STATIC_ASSERT(sizeof(ads129x_brief_tlv_t) == ADS129X_BRIEF_TLV_SIZE);

typedef struct __attribute__((packed)) ads129x_reg_tlv
{
    uint8_t type;
    uint16_t length;
    uint8_t value[ADS129X_REG_TLV_LEN];
} ads129x_reg_tlv_t;

STATIC_ASSERT(sizeof(ads129x_reg_tlv_t) == ADS129X_REG_TLV_SIZE);

// this is best used as static
typedef struct ads129x_packet_helper
{
    uint16_t                payload_len;
    uint16_t                packet_size;
    uint16_t                num_of_samples;

    // tlvs
    ads129x_brief_tlv_t     *p_brief;
    ads129x_reg_tlv_t       *p_reg;
    tlv_t                   *p_sample;
    tlv_t                   *p_rog_ecg;
    tlv_t                   *p_rog_resp;
    tlv_t                   *p_rog_internal;
    uint8_t                 *p_crc;
} ads129x_packet_helper_t;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint16_t length;
    uint16_t sensor_id;
    uint8_t version;
    uint8_t instance_id;
    uint8_t single_ppg;
    uint8_t low_power;
    uint8_t num_of_samples;
} max86141_brief_tlv_t;

STATIC_ASSERT(sizeof(max86141_brief_tlv_t) == MAX86141_BRIEF_TLV_SIZE);

typedef struct __attribute__((packed)) max86141_sample_tlv {
    uint8_t type;
    uint16_t length;
    uint8_t value[MAX86141_SAMPLE_TLV_LEN];
} max86141_sample_tlv_t;

STATIC_ASSERT(sizeof(max86141_sample_tlv_t) == MAX86141_SAMPLE_TLV_SIZE);

typedef struct __attribute__((packed)) max86141_ppgcfg_tlv {
    uint8_t type;
    uint16_t length;
    uint8_t value[MAX86141_PPGCFG_TLV_LEN];
} max86141_ppgcfg_tlv_t;

STATIC_ASSERT(sizeof(max86141_ppgcfg_tlv_t) == MAX86141_PPGCFG_TLV_SIZE);

typedef struct __attribute__((packed)) max86141_ledcfg_tlv {
    uint8_t type;
    uint16_t length;
    uint8_t value[MAX86141_LEDCFG_TLV_LEN];
} max86141_ledcfg_tlv_t;

STATIC_ASSERT(sizeof(max86141_ledcfg_tlv_t) == MAX86141_LEDCFG_TLV_SIZE);

typedef struct max86141_packet_helper
{
    uint16_t                payload_len;
    uint16_t                packet_size;
    uint16_t                num_of_samples;
    uint8_t                 instance_id;
    bool                    single_ppg;
    bool                    low_power;

    max86141_brief_tlv_t    *p_brief;
    max86141_sample_tlv_t   *p_sample;
    max86141_ppgcfg_tlv_t   *p_ppgcfg;  // with picket fence
    max86141_ledcfg_tlv_t   *p_ledcfg;  //

    uint8_t                 *p_crc;

} max86141_packet_helper_t;

void sens_init_ads129x_packet(ads129x_packet_helper_t * p_helper, sens_packet_t * p_pkt);
void sens_init_max86141_packet(max86141_packet_helper_t * p_helper, sens_packet_t * p_pkt);
void simple_crc(uint8_t * buf, uint8_t * cka, uint8_t * ckb);

#endif