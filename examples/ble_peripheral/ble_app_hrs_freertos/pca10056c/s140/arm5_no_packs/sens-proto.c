#include <string.h>

#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "sens-proto.h"

const uint8_t preamble[SENS_PREAMBLE_SIZE] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xD5};

/*
 * This is a simple and fast crc originally from ublox gps protocol
 * u-blox8-M8_RecerverDescrProtSpec_UBX-12003221.pdf
 */
void simple_crc(uint8_t * buf, uint8_t * cka, uint8_t * ckb)
{
    int num = cka - buf;
    *cka = 0;
    *ckb = 0;
    for (int i = 0; i < num; i++)
    {
        *cka = *cka + buf[i];
        *ckb = *ckb + *cka;
    }
}

void sens_init_ads129x_packet(ads129x_packet_helper_t * p_helper, sens_packet_t * p_pkt)
{
    uint16_t payload_len = 0;

    if (p_pkt)
    {
        memset(p_pkt, 0, p_helper->packet_size);
        memcpy(&p_pkt->preamble, preamble, SENS_PREAMBLE_SIZE);
        p_pkt->type = SENS_DATA_PACKET_TYPE;
        p_pkt->length = p_helper->payload_len;
    }

    // brief tlv
    if (p_pkt)
    {
        p_helper->p_brief = (ads129x_brief_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_brief->type = SENS_BRIEF_TLV_TYPE;
        p_helper->p_brief->length = ADS129X_BRIEF_TLV_LEN;
        p_helper->p_brief->sensor_id = ADS129X_SENSOR_ID;
        p_helper->p_brief->version = 0;
        p_helper->p_brief->instance_id = 0;
        p_helper->p_brief->num_of_samples = ADS129X_NUM_OF_SAMPLES;
        p_helper->p_brief->heart_rate = 255;
        p_helper->p_brief->resp_rate = 255;
    }
    payload_len += sizeof(ads129x_brief_tlv_t);

    // regs tlv
    if (p_pkt)
    {
        p_helper->p_reg = (ads129x_reg_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_reg->type = ADS129X_REG_TLV_TYPE;
        p_helper->p_reg->length = ADS129X_REG_TLV_LEN;
    }
    payload_len += 3 + ADS129X_REG_TLV_LEN;

    // sample tlv
    if (p_pkt)
    {
        p_helper->p_sample = (tlv_t*)&p_pkt->payload_crc[payload_len];
        p_helper->p_sample->type = ADS129X_SAMPLE_TLV_TYPE;
        p_helper->p_sample->length = ADS129X_SAMPLE_TLV_LEN;
    }
    payload_len += 3 + ADS129X_SAMPLE_TLV_LEN;

    if (ADS129X_HAS_ROG_ECG)
    {
        if(p_pkt) {
            p_helper->p_rog_ecg = (tlv_t*)&p_pkt->payload_crc[payload_len];
            p_helper->p_rog_ecg->type = ADS129x_ROG_ECG_TLV_TYPE;
            p_helper->p_rog_ecg->length = ADS129X_ROG_ECG_TLV_LEN;
        }
        payload_len += 3 + ADS129X_ROG_ECG_TLV_LEN;
    }

    if (ADS129X_HAS_ROG_RESP)
    {
        if (p_pkt) {
            p_helper->p_rog_resp = (tlv_t*)&p_pkt->payload_crc[payload_len];
            p_helper->p_rog_resp->type = ADS129X_ROG_RESP_TLV_TYPE;
            p_helper->p_rog_resp->length = ADS129X_ROG_RESP_TLV_LEN;
        }
        payload_len += 3 + ADS129X_ROG_RESP_TLV_LEN;
    }

    // p_helper->p_rog_internal = NULL;

    if (p_pkt)
    {
        p_helper->p_crc = &p_pkt->payload_crc[payload_len];
        p_helper->num_of_samples = 0;
    }
    else
    {
        p_helper->payload_len = payload_len;
        p_helper->packet_size = payload_len + 14;
    }
}

#if 0
void sens_init_max86141_packet(max86141_packet_helper_t * p_helper, sens_packet_t * p_pkt)
{
    uint16_t payload_len = 0;

    if (p_pkt)
    {
        memset(p_pkt, 0, p_helper->packet_size);
        memcpy(&p_pkt->preamble, preamble, SENS_PREAMBLE_SIZE);
        p_pkt->type = SENS_DATA_PACKET_TYPE;
        p_pkt->length = p_helper->payload_len;
    }

    if (p_pkt)
    {
        p_helper->p_brief = (max86141_brief_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_brief->type = SENS_BRIEF_TLV_TYPE;
        p_helper->p_brief->length = MAX86141_BRIEF_TLV_LEN;
        p_helper->p_brief->sensor_id = MAX86141_SENSOR_ID;
        p_helper->p_brief->version = 0;
        p_helper->p_brief->instance_id = p_helper->instance_id;
//        p_helper->p_brief->ppg1_led = p_helper->ppg1_led;
//        p_helper->p_brief->ppg2_led = p_helper->ppg2_led;
//        p_helper->p_brief->ppf_prox = p_helper->ppf_prox;
//        p_helper->p_brief->low_power = 0;
//        p_helper->p_brief->num_of_samples = MAX86141_NUM_OF_SAMPLES; // p_helper->num_of_samples
    }
    payload_len += sizeof(max86141_brief_tlv_t);

    if (p_pkt)
    {
        p_helper->p_sample = (max86141_sample_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_sample->type = MAX86141_SAMPLE_TLV_TYPE;
        p_helper->p_sample->length = MAX86141_SAMPLE_TLV_LEN;
    }
    payload_len += sizeof(max86141_sample_tlv_t);

    if (p_pkt)
    {
        p_helper->p_ppgcfg = (max86141_ppgcfg_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_ppgcfg->type = MAX86141_PPGCFG_TLV_TYPE;
        p_helper->p_ppgcfg->length = MAX86141_PPGCFG_TLV_LEN;
    }
    payload_len += sizeof(max86141_ppgcfg_tlv_t);

    if (p_pkt)
    {
        p_helper->p_ledcfg = (max86141_ledcfg_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_ledcfg->type = MAX86141_LEDCFG_TLV_TYPE;
        p_helper->p_ledcfg->length = MAX86141_LEDCFG_TLV_LEN;
    }
    payload_len += sizeof(max86141_ledcfg_tlv_t);

    if (p_helper->has_spo_rougu)
    {
        if (p_pkt)
        {
            p_helper->p_spo_rougu = (max86141_spo_rougu_tlv_t *)&p_pkt->payload_crc[payload_len];
            p_helper->p_spo_rougu->type = MAX86141_SPO_ROUGU_TLV_TYPE;
            p_helper->p_spo_rougu->length = MAX86141_SPO_ROUGU_TLV_LEN;
        }
        payload_len += sizeof(max86141_spo_rougu_tlv_t);
    }

    if (p_pkt)
    {
        p_helper->p_crc = &p_pkt->payload_crc[payload_len];
    }
    else
    {
        p_helper->payload_len = payload_len;
        p_helper->packet_size = payload_len + 14;
    }
}
#endif

bool sens_format_max86141_packet(max86141_packet_helper_t *p_helper, 
                                 sens_packet_t *p_pkt,
                                 size_t max_len)
{
    uint16_t payload_len = 0;
    uint16_t packet_size = 0;
    
    payload_len += sizeof(max86141_brief_tlv_t);
    payload_len += p_helper->has_ppgcfg ? sizeof(max86141_ppgcfg_tlv_t) : 0;
    payload_len += p_helper->has_ledcfg ? sizeof(max86141_ledcfg_tlv_t) : 0;
    payload_len += p_helper->has_abp_coeff ? sizeof(max86141_abp_coeff_tlv_t) : 0;
    payload_len += p_helper->has_spo_sample ? sizeof(max86141_spo_sample_tlv_t) : 0;
    payload_len += p_helper->has_spo_rougu ? sizeof(max86141_spo_rougu_tlv_t) : 0;
    payload_len += p_helper->has_abp_sample ? sizeof(max86141_abp_sample_tlv_t) : 0;
    payload_len += p_helper->has_abp_feature ? sizeof(max86141_abp_feature_tlv_t) : 0;
    
    packet_size = payload_len + 14;
    if (packet_size > max_len) {
        SEGGER_RTT_printf(0, "undersize, required %d, provided %d", packet_size, max_len);
        return false;
    }
    
    p_helper->payload_len = payload_len;
    p_helper->packet_size = packet_size;
    
    // reset cursor
    payload_len = 0;

    // senspacket
    memset(p_pkt, 0, p_helper->packet_size);
    memcpy(&p_pkt->preamble, preamble, SENS_PREAMBLE_SIZE);
    p_pkt->type = SENS_DATA_PACKET_TYPE;
    p_pkt->length = p_helper->payload_len;

    // brief
    p_helper->p_brief = (max86141_brief_tlv_t *)&p_pkt->payload_crc[payload_len];
    p_helper->p_brief->type = SENS_BRIEF_TLV_TYPE;
    p_helper->p_brief->length = MAX86141_BRIEF_TLV_LEN;
    p_helper->p_brief->sensor_id = MAX86141_SENSOR_ID;
    p_helper->p_brief->version = p_helper->version;
    p_helper->p_brief->instance_id = p_helper->instance_id;
    payload_len += sizeof(max86141_brief_tlv_t);
    
    if (p_helper->has_ppgcfg)
    {
        p_helper->p_ppgcfg = (max86141_ppgcfg_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_ppgcfg->type = MAX86141_PPGCFG_TLV_TYPE;
        p_helper->p_ppgcfg->length = MAX86141_PPGCFG_TLV_LEN;
        payload_len += sizeof(max86141_ppgcfg_tlv_t);
    }
    else
    {
        p_helper->p_ppgcfg = NULL;
    }
    
    
    if (p_helper->has_ledcfg)
    {
        p_helper->p_ledcfg = (max86141_ledcfg_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_ledcfg->type = MAX86141_LEDCFG_TLV_TYPE;
        p_helper->p_ledcfg->length = MAX86141_LEDCFG_TLV_LEN;
        payload_len += sizeof(max86141_ledcfg_tlv_t);    
    }
    else
    {
        p_helper->p_ledcfg = NULL;
    }
    
    if (p_helper->has_abp_coeff)
    {
        p_helper->p_abp_coeff = (max86141_abp_coeff_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_abp_coeff->type = MAX86141_ABP_COEFF_TLV_TYPE;
        p_helper->p_abp_coeff->length = MAX86141_ABP_COEFF_TLV_LEN;
        payload_len += sizeof(max86141_abp_coeff_tlv_t);
    }
    
    if (p_helper->has_spo_sample)
    {
        p_helper->p_spo_sample = (max86141_spo_sample_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_spo_sample->type = MAX86141_SAMPLE_TLV_TYPE;
        p_helper->p_spo_sample->length = MAX86141_SPO_SAMPLE_TLV_LEN;
        payload_len += sizeof(max86141_spo_sample_tlv_t);
    }
    else
    {
        p_helper->p_spo_sample = NULL;
    }
    
    if (p_helper->has_spo_rougu)
    {
        p_helper->p_spo_rougu = (max86141_spo_rougu_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_spo_rougu->type = MAX86141_SPO_ROUGU_TLV_TYPE;
        p_helper->p_spo_rougu->length = MAX86141_SPO_ROUGU_TLV_LEN;
        payload_len += sizeof(max86141_spo_rougu_tlv_t);
    }
    else
    {
        p_helper->p_spo_rougu = NULL;
    }

    if (p_helper->has_abp_sample)
    {
        p_helper->p_abp_sample = (max86141_abp_sample_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_abp_sample->type = MAX86141_SAMPLE_TLV_TYPE;
        p_helper->p_abp_sample->length = MAX86141_ABP_SAMPLE_TLV_LEN;
        payload_len += sizeof(max86141_abp_sample_tlv_t);
    }
    else
    {
        p_helper->p_abp_sample = NULL;
    }
    
    if (p_helper->has_abp_feature)
    {
        p_helper->p_abp_feature = (max86141_abp_feature_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_abp_feature->type = MAX86141_ABP_FEATURE_TLV_TYPE;
        p_helper->p_abp_feature->length = MAX86141_ABP_FEATURE_TLV_LEN;
        p_helper->p_abp_feature->index = -1;
        payload_len += sizeof(max86141_abp_feature_tlv_t);
    }
    else
    {
        p_helper->p_abp_feature = NULL;
    }
    
    p_helper->p_crc = &p_pkt->payload_crc[payload_len];
    
    return true;
}

void sens_init_ow_m601z_packet(ow_m601z_packet_helper_t * p_helper, sens_packet_t * p_pkt)
{
    uint16_t payload_len = 0;

    if (p_pkt)
    {
        memset(p_pkt, 0, p_helper->packet_size);
        memcpy(&p_pkt->preamble, preamble, SENS_PREAMBLE_SIZE);
        p_pkt->type = SENS_DATA_PACKET_TYPE;
        p_pkt->length = p_helper->payload_len;
    }

    if (p_pkt)
    {
        p_helper->p_brief = (ow_m601z_brief_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_brief->type = SENS_BRIEF_TLV_TYPE;
        p_helper->p_brief->length = OW_M601Z_BRIEF_TLV_LEN;
        p_helper->p_brief->sensor_id = OW_M601Z_SENSOR_ID;
        p_helper->p_brief->version = 0;
        p_helper->p_brief->instance_id = p_helper->instance_id;
        p_helper->p_brief->num_of_devices = p_helper->num_of_devices;
    }
    payload_len += sizeof(ow_m601z_brief_tlv_t);

    if (p_pkt)
    {
        p_helper->p_templist = (ow_m601z_templist_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_templist->type = OW_M601Z_TEMPLIST_TLV_TYPE;
        p_helper->p_templist->length = OW_M601Z_TEMPLIST_TLV_LEN;
        
        for (int i = 0; i < p_helper->num_of_devices; i++)
        {
            memcpy(p_helper->p_templist->id_temp[i].id, &(*p_helper->p_serial)[i][0], 8);
        }
    }
    payload_len += sizeof(ow_m601z_templist_tlv_t);

    if (p_pkt)
    {
        p_helper->p_crc = &p_pkt->payload_crc[payload_len];
    }
    else
    {
        p_helper->payload_len = payload_len;
        p_helper->packet_size = payload_len + 14;
    }    
}
