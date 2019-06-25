/*
 * micro.h
 *
 *  Created on: 15 ene. 2019
 *      Author: miguelvp
 */

#ifndef MICRO_FSM_H_
#define MICRO_FSM_H_

#include "fsm.h"
#include "shareData.h"

#define VIN_LENGTH	17
#define HEADER_STN	13

fsm_t* init_micro(car *coche);

void cambiaLED(leds color);
void enciendeLED(leds color);
void apagaLED(leds color);

#endif /* MICRO_FSM_H_ */
