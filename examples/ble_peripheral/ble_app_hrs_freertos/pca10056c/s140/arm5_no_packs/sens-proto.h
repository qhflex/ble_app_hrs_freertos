#ifndef __SENS_PROTO_H__
#define __SENS_PROTO_H__

#include <stdbool.h>
#include <stdint.h>
#include "app_util.h"
#include "feature.h"

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

#define MAX86141_BRIEF_TLV_LEN          9
#define MAX86141_BRIEF_TLV_SIZE         (3 + MAX86141_BRIEF_TLV_LEN)

// for combo, 41, 4
#define MAX86141_NUM_OF_SAMPLES         30
#define MAX86141_ITEMS_PER_SAMPLE       2
#define MAX86141_NUM_OF_SAMPLE_ITEMS    (30 * 2)    // 60
                                                    // It is possible but not necessarily to change this value.
                                                    // For maximum number of spi tx bytes is 255. 60 * 3 plus 2 leading characters is 182,
                                                    // which is large enough.

#define MAX86141_NUM_OF_SPO_SAMPLES         (25 * 2)
#define MAX86141_ITERMS_PER_SPO_SAMPLE      2
#define MAX86141_NUM_OF_SPO_SAMPLE_ITEMS    (MAX86141_NUM_OF_SPO_SAMPLES * MAX86141_ITERMS_PER_SPO_SAMPLE)

#define MAX86141_NUM_OF_ABP_SAMPLES         (128 * 2) // 32
#define MAX86141_ITERMS_PER_ABP_SAMPLE      2
#define MAX86141_NUM_OF_ABP_SAMPLE_ITEMS    (MAX86141_NUM_OF_ABP_SAMPLES * MAX86141_ITERMS_PER_ABP_SAMPLE)

#define MAX86141_SENSOR_ID                  1

#define MAX86141_SAMPLE_TLV_TYPE            0x08

#define MAX86141_SPO_SAMPLE_TLV_LEN         (MAX86141_NUM_OF_SPO_SAMPLE_ITEMS * 3)
#define MAX86141_SPO_SAMPLE_TLV_SIZE        (3 + MAX86141_SPO_SAMPLE_TLV_LEN)

#define MAX86141_ABP_SAMPLE_TLV_LEN         (MAX86141_NUM_OF_ABP_SAMPLE_ITEMS * 3)
#define MAX86141_ABP_SAMPLE_TLV_SIZE        (3 + MAX86141_ABP_SAMPLE_TLV_LEN)

#define MAX86141_ABP_COEFF_TLV_TYPE         0xE7
#define MAX86141_ABP_COEFF_TLV_LEN          (16)
#define MAX86141_ABP_COEFF_TLV_SIZE         (3 + MAX86141_ABP_COEFF_TLV_LEN)

#define MAX86141_ABP_FEATURE_TLV_TYPE       0xE8
#define MAX86141_ABP_FEATURE_TLV_LEN        (sizeof(int) * (1 + 1 + 2)) // index, 1 float field in feature, plus 2 bps
#define MAX86141_ABP_FEATURE_TLV_SIZE       (3 + MAX86141_ABP_FEATURE_TLV_LEN)

#define MAX86141_PPGCFG_TLV_TYPE            0x10
#define MAX86141_PPGCFG_TLV_LEN             7
#define MAX86141_PPGCFG_TLV_SIZE            (3 + MAX86141_PPGCFG_TLV_LEN)

#define MAX86141_LEDCFG_TLV_TYPE            0x20
#define MAX86141_LEDCFG_TLV_LEN             12
#define MAX86141_LEDCFG_TLV_SIZE            (3 + MAX86141_LEDCFG_TLV_LEN)

#define MAX86141_SPO_ROUGU_DATA_SIZE        24 // 32
#define MAX86141_SPO_ROUGU_TLV_TYPE         0xf5
#define MAX86141_SPO_ROUGU_TLV_LEN          (MAX86141_NUM_OF_SPO_SAMPLES * MAX86141_SPO_ROUGU_DATA_SIZE)
#define MAX86141_SPO_ROUGU_TLV_SIZE         (3 + MAX86141_SPO_ROUGU_TLV_LEN)

// config packet includes, brief, ppg, and led tlvs.
#define MAX86141_SPO_CFGPKT_SIZE            (14 + MAX86141_BRIEF_TLV_SIZE + MAX86141_PPGCFG_TLV_SIZE + MAX86141_LEDCFG_TLV_SIZE)
#define MAX86141_ABP_CFGPKT_SIZE            (14 + MAX86141_BRIEF_TLV_SIZE + MAX86141_PPGCFG_TLV_SIZE + MAX86141_LEDCFG_TLV_SIZE)

#define OW_M601Z_SENSOR_ID              4

#define OW_M601Z_BRIEF_TLV_LEN          5
#define OW_M601Z_BRIEF_TLV_SIZE         (3 + OW_M601Z_BRIEF_TLV_LEN)

#define OW_M601Z_TEMPLIST_TLV_TYPE      0x00
#define OW_M601Z_TEMPLIST_TLV_LEN       80
#define OW_M601Z_TEMPLIST_TLV_SIZE      (3 + OW_M601Z_TEMPLIST_TLV_LEN)


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

/*******************************************************************************
 *
 * max86141
 *
 ******************************************************************************/
typedef union __attribute__((packed)) {
    uint32_t u;
    float f;
} ieee754_t;

/**
 * brief tlv
 */
typedef struct __attribute__((packed)) {
    uint8_t type;
    uint16_t length;
    uint16_t sensor_id;
    uint8_t instance_id;
    uint8_t version;
    uint8_t ppg1_led;
    uint8_t ppg2_led;
    uint8_t ppf_prox;
    uint8_t low_power;
    uint8_t num_of_samples;
} max86141_brief_tlv_t;

STATIC_ASSERT(sizeof(max86141_brief_tlv_t) == MAX86141_BRIEF_TLV_SIZE);

/**
 * rougu tlv (for spo)
 */
typedef struct __attribute__((packed)) max86141_rougu_data
{
    // int ir;
    // int rd;
    int irdc;
    int rddc;
    int irFilt;
    int rdFilt;
    int spo;
    int hr;
} max86141_rougu_data_t;

STATIC_ASSERT(sizeof(max86141_rougu_data_t) == MAX86141_SPO_ROUGU_DATA_SIZE);

/**
 * 
 */
typedef struct __attribute__((packed)) max86141_spo_sample_tlv {
    uint8_t type;
    uint16_t length;
    uint8_t value[MAX86141_SPO_SAMPLE_TLV_LEN];
} max86141_spo_sample_tlv_t;

STATIC_ASSERT(sizeof(max86141_spo_sample_tlv_t) == MAX86141_SPO_SAMPLE_TLV_SIZE);

typedef struct __attribute__((packed)) max86141_abp_sample_tlv {
    uint8_t type;
    uint16_t length;
    uint8_t value[MAX86141_ABP_SAMPLE_TLV_LEN];
} max86141_abp_sample_tlv_t;

STATIC_ASSERT(sizeof(max86141_abp_sample_tlv_t) == MAX86141_ABP_SAMPLE_TLV_SIZE);

typedef struct __attribute__((packed)) max86141_abp_feature_tlv {
    uint8_t type;
    uint16_t length;
    int index;
    ieee754_t ptt;
    // ieee754_t idc;
    // ieee754_t imax;
    // ieee754_t imin;    
    ieee754_t sbp;
    ieee754_t dbp;
} max86141_abp_feature_tlv_t;

STATIC_ASSERT(sizeof(max86141_abp_feature_tlv_t) == MAX86141_ABP_FEATURE_TLV_SIZE);

typedef struct __attribute__((packed)) mx86141_abp_coeff_tlv {
    uint8_t type;
    uint16_t length;
    ieee754_t sbp[2];
    ieee754_t dbp[2];
} max86141_abp_coeff_tlv_t;

STATIC_ASSERT(sizeof(max86141_abp_coeff_tlv_t) == MAX86141_ABP_COEFF_TLV_SIZE);

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

typedef struct __attribute__((packed)) max86141_rougu_spo_tlv {
    uint8_t type;
    uint16_t length;
    uint8_t value[MAX86141_SPO_ROUGU_TLV_LEN];
} max86141_spo_rougu_tlv_t;

STATIC_ASSERT(sizeof(max86141_spo_rougu_tlv_t) == MAX86141_SPO_ROUGU_TLV_SIZE);

STATIC_ASSERT(sizeof(CFeature) == 16);

typedef struct max86141_packet_helper
{
    bool                        has_ppgcfg;
    bool                        has_ledcfg;
    bool                        has_spo_sample;
    bool                        has_spo_rougu;
    bool                        has_abp_sample;
    bool                        has_abp_feature;
    bool                        has_abp_coeff;
    
    uint16_t                    payload_len;
    uint16_t                    packet_size;
    uint8_t                     version;
    uint8_t                     instance_id;

    max86141_brief_tlv_t        *p_brief;
    max86141_ppgcfg_tlv_t       *p_ppgcfg;  // with picket fence
    max86141_ledcfg_tlv_t       *p_ledcfg;  //
    max86141_spo_sample_tlv_t   *p_spo_sample;
    max86141_spo_rougu_tlv_t    *p_spo_rougu; // p_spo_rougu
    max86141_abp_sample_tlv_t   *p_abp_sample;
    max86141_abp_feature_tlv_t  *p_abp_feature;
    max86141_abp_coeff_tlv_t    *p_abp_coeff;   // 

    uint8_t                     *p_crc;
} max86141_packet_helper_t;

typedef struct __attribute__((packed)) ow_m601z_brief_tlv
{
    uint8_t type;
    uint16_t length;
    uint16_t sensor_id;
    uint8_t instance_id;
    uint8_t version;
    uint8_t num_of_devices;    
} ow_m601z_brief_tlv_t;

STATIC_ASSERT(sizeof(ow_m601z_brief_tlv_t) == OW_M601Z_BRIEF_TLV_SIZE);

typedef struct __attribute__((packed)) ow_m601z_id_temp
{
    uint8_t id[8];
    uint8_t temp[2];
} ow_m601z_id_temp_t;

STATIC_ASSERT(sizeof(ow_m601z_id_temp_t) == 10);

typedef struct __attribute__((packed)) ow_m601z_templist_tlv
{
    uint8_t type;
    uint16_t length;
    ow_m601z_id_temp_t id_temp[8];
} ow_m601z_templist_tlv_t;

STATIC_ASSERT(sizeof(ow_m601z_templist_tlv_t) == OW_M601Z_TEMPLIST_TLV_SIZE);

typedef struct ow_m601z_packet_helper
{
    uint16_t                payload_len;        // set by init 
    uint16_t                packet_size;        // set by init
    uint8_t                 instance_id;        // set by user before init
    uint8_t                 num_of_devices;     // set by user before init
    uint8_t                 (*p_serial)[16][8];
    
    ow_m601z_brief_tlv_t    *p_brief;
    ow_m601z_templist_tlv_t *p_templist;
    uint8_t                 *p_crc;
} ow_m601z_packet_helper_t;

void sens_init_ads129x_packet(ads129x_packet_helper_t * p_helper, sens_packet_t * p_pkt);
void sens_init_max86141_packet(max86141_packet_helper_t * p_helper, sens_packet_t * p_pkt);
bool sens_format_max86141_packet(max86141_packet_helper_t *p_helper, sens_packet_t *p_pkt, size_t max_len);
void sens_init_ow_m601z_packet(ow_m601z_packet_helper_t * p_helper, sens_packet_t * p_pkt);
void simple_crc(uint8_t * buf, uint8_t * cka, uint8_t * ckb);

#endif
