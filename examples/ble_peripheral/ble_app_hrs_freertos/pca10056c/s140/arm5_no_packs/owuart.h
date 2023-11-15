#

/**************************************************************************************************
** File:        Z511WireComm.h
** Description: This file contains all of the defines and function prototypes for Z511WireComm.c
**		and is used with the "Using a Z8051 UART to Implement a 1-Wire Master with Multiple 
**		Slaves" Application Note (AN0346).
** 
**
** Copyright 2012 Zilog Inc. ALL RIGHTS RESERVED.
*
***************************************************************************************************
* The source code in this file was written by an authorized Zilog employee or a licensed 
* consultant. The source code has been verified to the fullest extent possible. 
*
* Permission to use this code is granted on a royalty-free basis. However, users are cautioned to
* authenticate the code contained herein. 
* 
* ZILOG DOES NOT GUARANTEE THE VERACITY OF THIS SOFTWARE; ANY SOFTWARE CONTAINED HEREIN IS
* PROVIDED "AS IS." NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
* IMPLIED WARRANTIES OF FITNESS FOR PARTICULAR PURPOSE OR MERCHANTABILITY. IN NO EVENT WILL ZILOG
* BE LIABLE FOR ANY SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES OR ANY LIABILITY IN TORT,
* NEGLIGENCE, OR OTHER LIABILITY INCURRED AS A RESULT OF THE USE OF THE SOFTWARE, EVEN IF ZILOG 
* HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES. ZILOG ALSO DOES NOT WARRANT THAT THE USE 
* OF THE SOFTWARE, OR OF ANY INFORMATION CONTAINED THEREIN WILL NOT INFRINGE ANY PATENT, 
* COPYRIGHT, OR TRADEMARK OF ANY THIRD PERSON OR ENTITY.

* THE SOFTWARE IS NOT FAULT-TOLERANT AND IS NOT DESIGNED, MANUFACTURED OR INTENDED FOR USE IN 
* CONJUNCTION WITH ON-LINE CONTROL EQUIPMENT, IN HAZARDOUS ENVIRONMENTS, IN APPLICATIONS 
* REQUIRING FAIL-SAFE PERFORMANCE, OR WHERE THE FAILURE OF THE SOFTWARE COULD LEAD DIRECTLY TO 
* DEATH, PERSONAL INJURY OR SEVERE PHYSICAL OR ENVIRONMENTAL DAMAGE (ALL OF THE FOREGOING, 
* "HIGH RISK ACTIVITIES"). ZILOG SPECIFICALLY DISCLAIMS ANY EXPRESS OR IMPLIED WARRANTY TO HIGH 
* RISK ACTIVITIES.
*
**************************************************************************************************/

#ifndef __ONEWIRE_UART_H__
#define __ONEWIRE_UART_H__

#include <stdint.h>
                                                // if two pins are externally connected directly,
                                                // it doesn't matter which one is tx or rx.
                                                // if two pins are connected via a resistor, then
                                                // the pin drives 1-wire data line should be tx and 
                                                // the other one is rx, which connects to 1-wire data
                                                // line via a resistor, possibly a few hundreds or thousands
                                                // of ohms.
                                                
#define OWUART_TX_PIN_NUMBER            0       // M-DQ-P0.00
#define OWUART_RX_PIN_NUMBER            4       // P0.04
#define QWUART_INT_PIN                  31      // M-INT-P0.31 (not used yet)



#define TSK_OWUART_STACK_SIZE           (256)	// the unit is word, 4-bytes
#define TSK_OWUART_PRIORITY             (1)

/** this function should be called before freertos started */
void onewire_probe(void);

void owuart_freertos_init(void);

#endif // __ONEWIRE_UART_H__

////////////////////////////////////////////////////////////////////////////////////////End of File
