/*
 * usb_fsm.h
 *
 *  Created on: 24 ene. 2019
 *      Author: miguelvp
 */

#ifndef USB_FSM_H_
#define USB_FSM_H_

#include "fsm.h"
#include "shareData.h"

#define BUFFER_UART1	64
#define BUFFER_UART2	64
#define BUFFER_UART3	128

fsm_t* init_usb(pilePointers_t *data);

void newData (uint8_t* Buf, uint32_t len);

#endif /* USB_FSM_H_ */
