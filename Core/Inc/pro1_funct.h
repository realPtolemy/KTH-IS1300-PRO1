/*
 * pro1_funct.h
 *
 *  Created on: Dec 8, 2023
 *      Author: Love Mitteregger
 */
#include "stdint.h"

#ifndef INC_IS1300_H_
#define INC_IS1300_H_

// System Initialization
void system_init(void);

// LED lights
void stageReg(void);
void latchReg(void);
void setReg(void);
void traffic_NS(uint8_t status);
void traffic_EW(uint8_t status);
void pedestrian_N(uint8_t status);
void pedestrian_W(uint8_t status);
void pedestrianPending_N(void);
void pedestrianPending_W(void);

// Tasks

#endif /* INC_IS1300_H_ */
