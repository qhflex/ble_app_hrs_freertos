#include <string.h>

#include "FreeRTOS.h"
#include "sens-proto.h"

const uint8_t preamble[SENS_PREAMBLE_SIZE] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xD5};
    
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
    
    // global tlv
    if (p_pkt)
    {
        p_helper->p_brief = (ads129x_brief_tlv_t *)&p_pkt->payload_crc[payload_len];
        p_helper->p_brief->type = SENS_BRIEF_TLV_TYPE;
        p_helper->p_brief->length = ADS129X_BRIEF_TLV_LEN;
        p_helper->p_brief->sensor_id = ADS129X_SENSOR_ID;
        p_helper->p_brief->instance_id = 0;
        p_helper->p_brief->version = 0;
        p_helper->p_brief->sample_num = 50;
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
        p_helper->sample_count = 0;
    }
    else
    {
        p_helper->payload_len = payload_len;
        p_helper->packet_size = payload_len + 14;
    }
}
