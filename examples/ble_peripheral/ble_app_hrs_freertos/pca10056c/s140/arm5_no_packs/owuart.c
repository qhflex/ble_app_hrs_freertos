/**
 * Copyright (c) 2018 - 2021, Nordic Semiconductor ASA
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
/** @file
 * @defgroup libuarte_example_main main.c
 * @{
 * @ingroup libuarte_example
 * @brief Libuarte Example Application main file.
 *
 * This file contains the source code for a sample application using libuarte.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <bsp.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_systick.h"
#include "nrfx_uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include "owuart.h"
#include "sens-proto.h"
#include "usbcdc.h"

#include "ble_nus_tx.h"

#define HIGH				    1
#define LOW					    0

#define OWWRITEBIT1             0xFF
#define OWWRITEBIT0             0x00

#define OWREADBIT1			    0xFF
#define OWREADBIT0	            0xFE

#define OWRESET                 0xF0
#define OWSEARCHROM			    0xF0

#define OWROMTOTALBITS 		    64

#define SENS_PACKET_POOL_SIZE   2

static TaskHandle_t m_owuart_thread;

static uint8_t  OW_DeviceROMCodes[OWROMTOTALBITS * 16];
static uint8_t  m_device_serial[16][8];
static int      m_device_count = 0;
static uint8_t  temp[9];

// static nrfx_uarte_t m_owuart = NRFX_UARTE_INSTANCE(0);
static nrfx_uart_t m_owuart = {                                                             
    .p_reg        = NRF_UART0,             
    .drv_inst_idx = NRFX_UART0_INST_IDX, 
    .skip_pin_cfg = 1,
};

//static nrfx_uart_config_t slow_config = {
//    .pseltxd    = OWUART_TX_PIN_NUMBER,
//    .pselrxd    = OWUART_RX_PIN_NUMBER,
//    .pselcts    = NRF_UART_PSEL_DISCONNECTED,
//    .pselrts    = NRF_UART_PSEL_DISCONNECTED,
//    .p_context  = NULL,
//    .hwfc       = NRF_UART_HWFC_DISABLED,
//    .parity     = NRF_UART_PARITY_EXCLUDED,
//    .baudrate   = (nrf_uart_baudrate_t)NRF_UART_BAUDRATE_9600,
//    .interrupt_priority = NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY,
//};

static nrfx_uart_config_t fast_config = {
    .pseltxd    = OWUART_TX_PIN_NUMBER,
    .pselrxd    = OWUART_TX_PIN_NUMBER,
    .pselcts    = NRF_UART_PSEL_DISCONNECTED,
    .pselrts    = NRF_UART_PSEL_DISCONNECTED,
    .p_context  = NULL,
    .hwfc       = NRF_UART_HWFC_DISABLED,
    .parity     = NRF_UART_PARITY_EXCLUDED,
    .baudrate   = (nrf_uart_baudrate_t)NRF_UART_BAUDRATE_115200,
    .interrupt_priority = NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY,
};

ow_m601z_packet_helper_t m_m601z_packet_helper = {0};
static sens_packet_t * p_current_m601z_packet = NULL;

static char * hexstr(uint8_t serial[], int num);
static uint8_t bits2byte(uint8_t * bits);
static void bits2bytes(uint8_t * bits, uint8_t * bytes);
static uint8_t CRC8(uint8_t *buf, int length);

static int onewire_readbit(void);
static int onewire_writebit(uint8_t bit);
static int onewire_writebyte(uint8_t byte);

static int onewire_reset(void);
static int onewire_search(unsigned char *ROMCodes);
static int onewire_m601z_readtemp(uint8_t buf[9]);

static sens_packet_t * next_m601z_packet(void);

static char * hexstr(uint8_t serial[], int num)
{
    static const char hexchar[] = "0123456789ABCDEF";
    static char strbuf[64];
    for (int i = 0; i < num; i++)
    {
        strbuf[i * 3 + 0] = hexchar[(serial[i] >> 4) & 0x0F];
        strbuf[i * 3 + 1] = hexchar[(serial[i] >> 0) & 0x0F];
        strbuf[i * 3 + 2] = (i < num - 1) ? ' ' : '\0';
    }
    return strbuf;
}

static int onewire_readbit(void)
{
    nrfx_err_t err;
    uint8_t wd = OWWRITEBIT1, rd;

    err = nrfx_uart_tx(&m_owuart, &wd, 1);
    if (err) return -1;


    err = nrfx_uart_rx(&m_owuart, &rd, 1);
    if (err) return -1;

    return (rd == OWWRITEBIT1) ? 1 : 0;
}

static int onewire_writebit(uint8_t bit)
{
    nrfx_err_t err;
    uint8_t rd, wd = 0x00;

    if (bit > 1) return -1;
    if (bit == 1) wd = 0xff;

    err = nrfx_uart_tx(&m_owuart, &wd, 1);
    if (err) return -1;

    err = nrfx_uart_rx(&m_owuart, &rd, 1);
    if (err) return -1;

    return (wd == rd) ? 0 : -1;
}

static int onewire_writebyte(uint8_t byte)
{
    int ret;
    for(uint8_t i = 0; i < 8; i++)
    {
        ret = onewire_writebit((byte >> i) & 0x01);
        if (ret < 0) return ret;
    }//for

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 Function:	  onewire_reset 								     														   //
//   Description: Sends the Reset Pulse and detect slave presence. Baud Rate should be set to 9600. Then 0xF0 will be sent,    //
//				  enough to reset the slaves. The if the Rx reads 0xF0, it means there are no slaves present.   			   //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int onewire_reset(void)
{
    uint8_t wd = OWRESET, rd;

    // rx disable/enabled cannot be removed
    nrfx_uart_rx_disable(&m_owuart);                    // Set Baud Rate to 9600
    NRF_UART0->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud9600 << UART_BAUDRATE_BAUDRATE_Pos);
    nrfx_uart_rx_enable(&m_owuart);

	nrfx_uart_tx(&m_owuart, &wd, 1);                    // Send F0
    nrfx_uart_rx(&m_owuart, &rd, 1);                    // Read returned data

    nrfx_uart_rx_disable(&m_owuart);                    // Set UART to 115200Kbps
    NRF_UART0->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);
    nrfx_uart_rx_enable(&m_owuart);
    
    return (rd != wd) ? HIGH : LOW;
}// OW_Reset

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 Function: onewire_search                            	     																   //
//   Description: Determine no. of Slaves 				 	 																   //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int onewire_search(unsigned char *ROMCodes)
{
    int ret, bit;
	unsigned char OW_FirstBit		 = 0;
	unsigned char OW_SecondBit		 = 0;
	unsigned char OW_ErrorFlag		 = 0;
 	int           OW_BitIndex		 = 0;
	unsigned char OW_NewDeviation	 = 0;
	unsigned char OW_LastDeviation	 = 0;
    int           OW_NumberOfDevices = 0;
	int           OW_Loop			 = 0;

	unsigned char OW_BitPattern[64];										    // ROM code Container 64*3 Wide

  	do{
        ret = onewire_reset();
        if (ret < 0) return ret;
        if (ret == 0)
        {
            NRF_LOG_INFO("onewire_reset returns 0, break");
            break;
        }

		// OW_SendROMCommand(OWSEARCHROM);											// Send ROM Function : Search ROM (0xF0)
        ret = onewire_writebyte(OWSEARCHROM);
        if (ret < 0) return ret;

		OW_NewDeviation	= 0;													// Set New Deviation to 0

		for( OW_BitIndex  = 1 ;
			 OW_BitIndex <= OWROMTOTALBITS;
			 OW_BitIndex++ )
		{
			// OW_FirstBit  = (unsigned char)OW_GetDataBit();						// Get the (AND) First Bit Sent by the Slaves
			// OW_SecondBit = (unsigned char)OW_GetDataBit(); 						// Get the Sencond Bit (Complement of the First)
            bit = onewire_readbit();
            if (bit < 0) return bit;
            OW_FirstBit = bit;

            bit = onewire_readbit();
            if (bit < 0) return bit;
            OW_SecondBit = bit;

			if(OW_FirstBit == HIGH && OW_SecondBit == HIGH)						// If Both Bits are HIGH
			{																	// Either no Slaves Found or Error on the Bus Line
				OW_NewDeviation = 0;											// Set New Deviation to 0
				OW_ErrorFlag	 = 1;											// Set Error Flag
				break;
			}
			else if(OW_FirstBit != OW_SecondBit)								// If First Bit and Second Bit Not Equal.
			{																	// Either one of them is HIGH
				OW_BitPattern[OW_BitIndex - 1] = OW_FirstBit;					// Set BitPattern[BitIndex] to the First Bit
				ret = onewire_writebit(OW_FirstBit);									// Then Send it to the Slaves
                if (ret < 0) return ret;
			}
			else if(!(OW_FirstBit && OW_SecondBit))								// If Both Bits are LOW
			{																	// Evaluate
				if( OW_BitIndex == OW_LastDeviation )							// If BitIndex (For Loop Index where the Discrepancy
				{																// happens is equal to the Last Deviation value
					OW_BitPattern[OW_BitIndex - 1] = HIGH;						// Set BitPattern[BitIndex] to HIGH
					ret = onewire_writebit(HIGH);										// Then Send it to the Slaves
                    if (ret < 0) return ret;
				}
				else if(OW_BitIndex > OW_LastDeviation)							// If BitIndex is greater then the Last Deviation
				{
					OW_BitPattern[OW_BitIndex - 1] = LOW;						// Set BitPattern[Bit_Index] to LOW
					OW_NewDeviation = OW_BitIndex;								// Set New Deviation to BitIndex
					ret = onewire_writebit(LOW);										// Send BitPattern[BitIndex] to the Slaves
                    if (ret < 0) return ret;
				}
				else if(OW_BitPattern[OW_BitIndex - 1] == LOW)					// If BitPattern[BitIndex] (previous search) is LOW
				{
					OW_NewDeviation = OW_BitIndex;								// Set New Deviation to value of BitIndex
					ret = onewire_writebit(OW_BitPattern[OW_BitIndex - 1]);		// Send BitPattern[BitIndex]
                    if (ret < 0) return ret;
				}
				else
				{
					ret = onewire_writebit(OW_BitPattern[OW_BitIndex - 1]);	    // Send BitPattern[BitIndex]
                    if (ret < 0) return ret;
				}
			}
			else
			{
				OW_ErrorFlag = 1;												// Set Error Flag
				break;
			}
		}//for
        
		if(OW_ErrorFlag!= 1)
		{
			OW_NumberOfDevices++;												// Increment Number of Devices if no ERROR
		}
		else
		{
			OW_NumberOfDevices = 0;												// else set it to ZERO
		}
		OW_LastDeviation = OW_NewDeviation;										// Set Last Deviation to New Deviation
        
        // NRF_LOG_INFO("numOfDevices: %d", OW_NumberOfDevices);

		for(OW_Loop = ((OW_NumberOfDevices - 1) * OWROMTOTALBITS);				// Post process ROM Code and Save.
			OW_Loop < (OWROMTOTALBITS * OW_NumberOfDevices);
			OW_Loop++)
		{
			ROMCodes[OW_Loop] = OW_BitPattern[OW_Loop - (OWROMTOTALBITS * (OW_NumberOfDevices - 1) )];
		}
	} while(OW_LastDeviation);													// If Last Deviation is not ZERO Loop.

	return(OW_NumberOfDevices);													// Return Number of Slaves Found.
}// OWSearch

static uint8_t bits2byte(uint8_t * bits)
{
    uint8_t byte;
    for (int i = 0; i< 8; i++)
    {
        if (bits[i])
        {
            byte |= 1 << i;
        }
        else
        {
            byte &= ~(1 << i);
        }
    }
    return byte;
}

static void bits2bytes(uint8_t * bits, uint8_t * bytes)
{
    for (int i = 0; i < 8; i++)
    {
        bytes[i] = bits2byte(&bits[i * 8]);
    }
}

static int onewire_m601z_readtemp(uint8_t buf[9])
{
    int ret;
    for (int j = 0; j < 72; j++)
    {
        ret = onewire_readbit();
        // NRF_LOG_INFO("readbit: %d", ret);

        if (ret < 0) return -1;
        if (ret == 1)
        {
            buf[j / 8] |= 1 << (j % 8);
        }
        else
        {
            buf[j / 8] &= ~(1 << (j % 8));
        }
    }
    return 0;
}

static uint8_t CRC8(uint8_t *buf, int length)
{
  uint8_t result = 0x00;
  uint8_t byte;

  while(length--)
  {
    byte = *buf++;
    for(int i = 0; i < 8; i++)
    {
      if((result^byte) & 0x01)
      {
        result ^= 0x18;
        result >>= 1;
        result |= 0x80;
      }
      else {
        result >>= 1;
      }
      byte >>= 1;
    }
  }
  return result;
}

#define OW_CHECK(x)             if ((ret = x) < 0) { goto fail; }                  

void owuart_task(void * pvParameters)
{
    int ret;
    
    onewire_probe();
    
//    for (int i = 0;; i++)
//    {
//        vTaskDelay(1024);
//    }
    
    m_m601z_packet_helper.num_of_devices = m_device_count;
    m_m601z_packet_helper.p_serial = &m_device_serial;
    
    p_current_m601z_packet = next_m601z_packet();
    
    for (int n = 0;;n++)
    {
        vTaskDelay(2000);
        
        for (int j = 0; j < m_device_count; j++)
        {
            OW_CHECK(onewire_reset());
            if (ret == 0) goto fail;
            OW_CHECK(onewire_writebyte(0x55));
            OW_CHECK(onewire_writebyte(m_device_serial[j][0]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][1]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][2]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][3]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][4]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][5]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][6]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][7]));        
            OW_CHECK(onewire_writebyte(0x44));  // convert T
            
            // nrfx_systick_delay_ms(10);
            vTaskDelay(12);
            
            OW_CHECK(onewire_reset());
            if (ret == 0) goto fail;

            OW_CHECK(onewire_writebyte(0x55));
            OW_CHECK(onewire_writebyte(m_device_serial[j][0]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][1]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][2]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][3]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][4]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][5]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][6]));
            OW_CHECK(onewire_writebyte(m_device_serial[j][7]));
            OW_CHECK(onewire_writebyte(0xBE));
            
            OW_CHECK(onewire_m601z_readtemp(temp));

            if (CRC8(temp, 9) == 0)
            {
                char buf[8];
                int16_t *st = (int16_t*)temp;
                float tempf = (float)(*st)/256 + 40;
                snprintf(buf, 8, "%.4f", tempf);
                NRF_LOG_INFO("  T(%d): %s (%d)", j+1, buf, n);
                
                m_m601z_packet_helper.p_templist->id_temp[j].temp[0] = temp[0];
                m_m601z_packet_helper.p_templist->id_temp[j].temp[1] = temp[1];
                
                continue;
            }
            else
            {
                NRF_LOG_WARNING("    BAD CRC: %s", hexstr(temp, 9));
            }

            fail:
            m_m601z_packet_helper.p_templist->id_temp[j].temp[0] = 0xff;
            m_m601z_packet_helper.p_templist->id_temp[j].temp[1] = 0xff;
        }

        if (cdc_acm_port_open())
        {
            simple_crc((uint8_t *)&p_current_m601z_packet->type, &m_m601z_packet_helper.p_crc[0], &m_m601z_packet_helper.p_crc[1]);
            cdc_acm_send_packet((uint8_t *)p_current_m601z_packet, m_m601z_packet_helper.packet_size);
        }
        

        if (ble_nus_tx_running())
        {
            // uint16_t packet_size = m_m601z_packet_helper.packet_size;
            // ble_nus_comm_send((uint8_t *)p_current_m601z_packet, &packet_size);
            ble_nus_tx_buf_t* buf = ble_nus_tx_alloc();
            if (buf) 
            {
                uint16_t size = m_device_count * sizeof(ow_m601z_id_temp_t);
                buf->type = 1;
                buf->seq = n % 256; // this doesn't matter
                memcpy(buf->data, &m_m601z_packet_helper.p_templist->id_temp[0], size); 
                buf->len = size + 2;
                ble_nus_tx_send(buf);
            }
            else
            {
                NRF_LOG_INFO("nus tx no available buffer?");
            }
        }
//        else
//        {
//            NRF_LOG_INFO("nus tx not running");
//        }
        
        p_current_m601z_packet = next_m601z_packet();
    }
}

void owuart_freertos_init(void)
{
  BaseType_t xReturned = xTaskCreate(owuart_task,
                                     "owuart",
                                     TSK_OWUART_STACK_SIZE,
                                     NULL,
                                     TSK_OWUART_PRIORITY,
                                     &m_owuart_thread);
  if (xReturned != pdPASS)
  {
    NRF_LOG_ERROR("owuart task not created.");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  else
  {
    NRF_LOG_INFO("owuart task created.");
  }
}

void onewire_probe(void)
{
    int ret;
    // tx pin
    nrf_gpio_pin_set(OWUART_TX_PIN_NUMBER);
    
    nrf_gpio_cfg(OWUART_TX_PIN_NUMBER,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,       // standard 0 disconnect 1, open drain
                 NRF_GPIO_PIN_NOSENSE);   // only affect sleep wakeup
    // rx pin
    nrf_gpio_cfg_input(OWUART_RX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
    
    nrfx_uart_init(&m_owuart, &fast_config, NULL);
    
    ret = onewire_search(OW_DeviceROMCodes);
    if (ret < 0)
    {
        NRF_LOG_WARNING("onewire probe failed");
        return;
    }

    m_device_count = ret;
    NRF_LOG_INFO("%d onewire %s found", m_device_count, m_device_count == 1 ? "device" : "devices");
    // NRF_LOG_FLUSH();
    
    for (int i = 0; i < ret; i++)
    {
        bits2bytes(&OW_DeviceROMCodes[i * 64], m_device_serial[i]);
        NRF_LOG_INFO("(%d) SN: %s", i + 1, hexstr(m_device_serial[i], 8));
        // NRF_LOG_FLUSH();    // this is important!
        vTaskDelay(1);
    }
}

static sens_packet_t * next_m601z_packet(void)
{
    static bool initialized = false;
    static sens_packet_t * p_packet_pool[SENS_PACKET_POOL_SIZE] = {0};
    static int next_modulo = 0;

    if (!initialized)
    {
        // init m_packet helper
        sens_init_ow_m601z_packet(&m_m601z_packet_helper, NULL);

        // alloc mem
        for (int i = 0; i < SENS_PACKET_POOL_SIZE; i++)
        {
            p_packet_pool[i] = pvPortMalloc(m_m601z_packet_helper.packet_size);
            APP_ERROR_CHECK_BOOL(p_packet_pool[i] != NULL);
        }

        initialized = true;

        NRF_LOG_INFO("m601z packets init, len: %d, size: %d",
            m_m601z_packet_helper.payload_len,
            m_m601z_packet_helper.packet_size);
    }

    // init next packet
    sens_init_ow_m601z_packet(&m_m601z_packet_helper, p_packet_pool[next_modulo]);
    sens_packet_t * next = p_packet_pool[next_modulo];
    next_modulo = (next_modulo + 1) % SENS_PACKET_POOL_SIZE;
    return next;
}
